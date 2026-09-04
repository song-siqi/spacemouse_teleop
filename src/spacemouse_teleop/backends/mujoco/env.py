from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

from spacemouse_teleop.backends.mujoco.constants import (
    END_EFFECTOR_XARM_GRIPPER,
    GRIPPER_ACTUATOR_NAMES,
    GRIPPER_GUARD_GEOM_NAMES,
    GRIPPER_JOINT_LIMIT_RAD,
    GRIPPER_JOINT_NAMES,
    GRIPPER_PAD_GEOM_NAMES,
    HOME_QPOS,
    JOINT_NAMES,
)
from spacemouse_teleop.backends.mujoco.controller import DampedLeastSquaresIk
from spacemouse_teleop.backends.mujoco.math_utils import (
    quat_from_rotvec,
    quat_multiply,
    quat_normalize,
)
from spacemouse_teleop.backends.mujoco.model_source import (
    DEFAULT_END_EFFECTOR,
    GRIPPER_GUARD_COLLISION_BIT,
    default_model_path,
)
from spacemouse_teleop.spacemouse.command import TeleopCommand

Vector = Tuple[float, ...]
KINEMATIC_LATERAL_GUARD_PENETRATION_LIMIT_M = 0.002
KINEMATIC_VERTICAL_GUARD_NORMAL_Z_MIN = 0.7


@dataclass(frozen=True)
class _GuardContact:
    guard_geom_id: int
    object_geom_id: int
    distance_m: float
    normal_z_abs: float


@dataclass(frozen=True)
class MujocoObservation:
    joint_pos: Vector
    target_joint_pos: Vector
    ee_pos: Vector
    target_ee_pos: Vector
    ee_quat: Vector
    target_ee_quat: Vector
    cube_pos: Vector
    cube_quat: Vector
    gripper_closedness: float
    target_gripper_closedness: float
    gripper_joint_pos: Vector
    ik_error_norm: float
    timestamp: float

    def to_dict(self) -> dict:
        return asdict(self)


class XArm6TableCubeEnv:
    def __init__(
        self,
        model_path: Optional[Path] = None,
        end_effector: str = DEFAULT_END_EFFECTOR,
        joint_names: Sequence[str] = JOINT_NAMES,
        site_name: str = "eef",
        cube_body_name: str = "cube",
        control_hz: float = 60.0,
        target_mode: str = "velocity",
        arm_control_mode: str = "kinematic",
        ik_position_gain: float = 1.0,
        ik_orientation_gain: float = 0.6,
        max_ee_angular_speed_radps: Optional[float] = 0.25,
        workspace_min: Sequence[float] = (0.10, -0.45, 0.755),
        workspace_max: Sequence[float] = (0.85, 0.45, 1.25),
    ) -> None:
        self.mujoco = _require_mujoco()
        if target_mode not in ("velocity", "integrated"):
            raise ValueError("target_mode must be 'velocity' or 'integrated'")
        if arm_control_mode not in ("kinematic", "actuator"):
            raise ValueError("arm_control_mode must be 'kinematic' or 'actuator'")
        self.end_effector = end_effector
        self.model_path = (
            Path(model_path) if model_path else default_model_path(end_effector)
        )
        self.model = self.mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = self.mujoco.MjData(self.model)
        self.joint_names = tuple(joint_names)
        self.control_dt = 1.0 / float(control_hz)
        self.target_mode = target_mode
        self.arm_control_mode = arm_control_mode
        self.max_ee_angular_speed_radps = (
            None
            if max_ee_angular_speed_radps is None
            or max_ee_angular_speed_radps <= 0.0
            else float(max_ee_angular_speed_radps)
        )
        self.workspace_min = np.asarray(workspace_min, dtype=float)
        self.workspace_max = np.asarray(workspace_max, dtype=float)

        self.site_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_SITE, site_name
        )
        self.cube_body_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_BODY, cube_body_name
        )
        if self.site_id < 0:
            raise RuntimeError(f"MuJoCo site not found: {site_name}")
        if self.cube_body_id < 0:
            raise RuntimeError(f"MuJoCo body not found: {cube_body_name}")
        self.gripper_guard_geom_ids = frozenset(
            geom_id
            for geom_id in (
                self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_GEOM, name
                )
                for name in GRIPPER_GUARD_GEOM_NAMES
            )
            if geom_id >= 0
        )

        self.joint_ids = np.array(
            [
                self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name
                )
                for name in self.joint_names
            ],
            dtype=int,
        )
        self.qpos_ids = np.array(self.model.jnt_qposadr[self.joint_ids], dtype=int)
        self.dof_ids = np.array(self.model.jnt_dofadr[self.joint_ids], dtype=int)
        self.actuator_ids = np.array(
            [
                self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_pos"
                )
                for name in self.joint_names
            ],
            dtype=int,
        )
        if np.any(self.actuator_ids < 0):
            raise RuntimeError("MuJoCo xArm6 position actuators are incomplete")

        self.gripper_joint_ids = self._joint_ids(GRIPPER_JOINT_NAMES)
        self.has_gripper = len(self.gripper_joint_ids) == len(GRIPPER_JOINT_NAMES)
        self.gripper_qpos_ids = np.array(
            self.model.jnt_qposadr[self.gripper_joint_ids], dtype=int
        )
        self.gripper_dof_ids = np.array(
            self.model.jnt_dofadr[self.gripper_joint_ids], dtype=int
        )
        self.gripper_actuator_ids = np.array(
            [
                self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
                for name in GRIPPER_ACTUATOR_NAMES
            ],
            dtype=int,
        )
        self.has_gripper_actuators = len(self.gripper_actuator_ids) > 0 and not np.any(
            self.gripper_actuator_ids < 0
        )
        if self.end_effector == END_EFFECTOR_XARM_GRIPPER and (
            not self.has_gripper or not self.has_gripper_actuators
        ):
            raise RuntimeError("MuJoCo xArm gripper joints are incomplete")

        self.ik = DampedLeastSquaresIk(
            self.model,
            self.joint_names,
            site_name=site_name,
            position_gain=ik_position_gain,
            orientation_gain=ik_orientation_gain,
        )
        self.target_pos = np.zeros(3)
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.target_qpos = np.asarray(HOME_QPOS, dtype=float)
        self.target_gripper_closedness = 0.0
        self._last_gripper_command_direction = 0
        self.last_ik_error_norm = 0.0
        self.last_kinematic_guard_blocked = False

    def reset(self, qpos: Sequence[float] = HOME_QPOS) -> MujocoObservation:
        self.mujoco.mj_resetData(self.model, self.data)
        self.target_qpos = np.asarray(qpos, dtype=float)
        self.data.qpos[self.qpos_ids] = self.target_qpos
        self.data.ctrl[self.actuator_ids] = self.target_qpos
        self.target_gripper_closedness = 0.0
        self._last_gripper_command_direction = 0
        self.last_kinematic_guard_blocked = False
        self._set_gripper_qpos_from_target()
        self._apply_gripper_target()
        self.mujoco.mj_forward(self.model, self.data)
        self.target_pos = np.array(self.data.site_xpos[self.site_id], dtype=float)
        self.target_quat = self._site_quat()
        self.last_ik_error_norm = 0.0
        return self.observe()

    def step_command(self, command: TeleopCommand) -> MujocoObservation:
        dt = command.dt if command.dt > 0.0 else self.control_dt
        previous_qpos = np.array(self.data.qpos[self.qpos_ids], dtype=float)
        current_pos = np.array(self.data.site_xpos[self.site_id], dtype=float)
        current_quat = self._site_quat()
        if self.target_mode == "velocity":
            self.target_pos = current_pos
            self.target_quat = current_quat

        if command.enabled:
            anchor_pos = self.target_pos
            anchor_quat = self.target_quat
            if self.target_mode == "velocity":
                anchor_pos = current_pos
                anchor_quat = current_quat

            self.target_pos = np.clip(
                anchor_pos + np.asarray(command.delta_pos_m, dtype=float),
                self.workspace_min,
                self.workspace_max,
            )
            delta_rot = _limit_vector_norm(
                command.delta_rot_rad,
                None
                if self.max_ee_angular_speed_radps is None
                else self.max_ee_angular_speed_radps * dt,
            )
            delta_quat = quat_from_rotvec(delta_rot)
            self.target_quat = quat_multiply(delta_quat, anchor_quat)

        if self.has_gripper:
            self._update_gripper_target(command)
            self._apply_gripper_target()

        result = self.ik.solve(self.data, self.target_pos, self.target_quat, dt=dt)
        self.target_qpos = result.qpos_target
        self.last_ik_error_norm = result.error_norm
        self.data.ctrl[self.actuator_ids] = self.target_qpos
        if self.arm_control_mode == "kinematic":
            applied_qpos, commanded_qvel, guard_blocked = self._step_kinematic_arm(
                dt, previous_qpos, self.target_qpos
            )
            self.last_kinematic_guard_blocked = guard_blocked
            self.data.qpos[self.qpos_ids] = applied_qpos
            self.data.qvel[self.dof_ids] = commanded_qvel
            self.data.ctrl[self.actuator_ids] = applied_qpos
            self._apply_gripper_target()
            self.mujoco.mj_forward(self.model, self.data)
        else:
            self.last_kinematic_guard_blocked = False
            self._apply_gripper_target()
            self.step_physics(dt)
        return self.observe()

    def step_physics(self, dt: Optional[float] = None) -> None:
        target_dt = self.control_dt if dt is None or dt <= 0.0 else dt
        nsteps = max(1, int(round(target_dt / float(self.model.opt.timestep))))
        for _ in range(nsteps):
            self._apply_gripper_target()
            self.mujoco.mj_step(self.model, self.data)
        self._apply_gripper_target()
        self.mujoco.mj_forward(self.model, self.data)

    def _step_kinematic_arm(
        self,
        dt: float,
        start_qpos: np.ndarray,
        target_qpos: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, bool]:
        target_dt = self.control_dt if dt <= 0.0 else dt
        nsteps = max(1, int(round(target_dt / float(self.model.opt.timestep))))
        qpos_delta = target_qpos - start_qpos
        qvel = qpos_delta / target_dt
        zero_qvel = np.zeros_like(qvel)
        accepted_qpos = np.array(start_qpos, dtype=float)
        guard_blocked = False
        for index in range(nsteps):
            if not guard_blocked:
                self.data.qpos[self.qpos_ids] = accepted_qpos
                self.data.qvel[self.dof_ids] = qvel
                self.data.ctrl[self.actuator_ids] = accepted_qpos
                self._apply_gripper_target()
                self.mujoco.mj_forward(self.model, self.data)
                previous_contacts = self._guard_object_contacts()

                alpha = float(index + 1) / float(nsteps)
                candidate_qpos = start_qpos + alpha * qpos_delta
                self.data.qpos[self.qpos_ids] = candidate_qpos
                self.data.qvel[self.dof_ids] = qvel
                self.data.ctrl[self.actuator_ids] = candidate_qpos
                self._apply_gripper_target()
                self.mujoco.mj_forward(self.model, self.data)
                candidate_contacts = self._guard_object_contacts()
                guard_blocked = _has_blocking_guard_penetration(
                    previous_contacts, candidate_contacts
                )
                if not guard_blocked:
                    accepted_qpos = candidate_qpos

            if guard_blocked:
                self.data.qpos[self.qpos_ids] = accepted_qpos
                self.data.qvel[self.dof_ids] = zero_qvel
                self.data.ctrl[self.actuator_ids] = accepted_qpos
            else:
                self.data.qpos[self.qpos_ids] = accepted_qpos
                self.data.qvel[self.dof_ids] = qvel
                self.data.ctrl[self.actuator_ids] = accepted_qpos
            self._apply_gripper_target()
            self.mujoco.mj_step(self.model, self.data)
        return accepted_qpos, zero_qvel if guard_blocked else qvel, guard_blocked

    def _guard_object_contacts(self) -> Tuple[_GuardContact, ...]:
        if not self.gripper_guard_geom_ids:
            return ()
        contacts = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            guard_geom_ids = set(geom_ids).intersection(
                self.gripper_guard_geom_ids
            )
            if not guard_geom_ids:
                continue
            guard_geom_id = next(iter(guard_geom_ids))
            object_geom_id = (
                geom_ids[1] if geom_ids[0] == guard_geom_id else geom_ids[0]
            )
            if not (
                int(self.model.geom_conaffinity[object_geom_id])
                & GRIPPER_GUARD_COLLISION_BIT
            ):
                continue
            contacts.append(
                _GuardContact(
                    guard_geom_id=guard_geom_id,
                    object_geom_id=object_geom_id,
                    distance_m=float(contact.dist),
                    normal_z_abs=abs(float(contact.frame[2])),
                )
            )
        return tuple(contacts)

    def set_gripper_collision_debug(self, enabled: bool) -> None:
        alpha = 0.35 if enabled else 0.0
        for geom_names, rgb in (
            (GRIPPER_PAD_GEOM_NAMES, (0.05, 0.65, 0.20)),
            (GRIPPER_GUARD_GEOM_NAMES, (0.15, 0.55, 0.85)),
        ):
            for geom_name in geom_names:
                geom_id = self.mujoco.mj_name2id(
                    self.model, self.mujoco.mjtObj.mjOBJ_GEOM, geom_name
                )
                if geom_id >= 0:
                    self.model.geom_rgba[geom_id] = (*rgb, alpha)

    def observe(self) -> MujocoObservation:
        ee_quat = self._site_quat()
        cube_quat = np.array(self.data.xquat[self.cube_body_id], dtype=float)
        return MujocoObservation(
            joint_pos=tuple(float(v) for v in self.data.qpos[self.qpos_ids]),
            target_joint_pos=tuple(float(v) for v in self.target_qpos),
            ee_pos=tuple(float(v) for v in self.data.site_xpos[self.site_id]),
            target_ee_pos=tuple(float(v) for v in self.target_pos),
            ee_quat=tuple(float(v) for v in ee_quat),
            target_ee_quat=tuple(float(v) for v in self.target_quat),
            cube_pos=tuple(float(v) for v in self.data.xpos[self.cube_body_id]),
            cube_quat=tuple(float(v) for v in cube_quat),
            gripper_closedness=self._observed_gripper_closedness(),
            target_gripper_closedness=float(self.target_gripper_closedness),
            gripper_joint_pos=tuple(
                float(v) for v in self.data.qpos[self.gripper_qpos_ids]
            ),
            ik_error_norm=float(self.last_ik_error_norm),
            timestamp=float(self.data.time),
        )

    def _site_quat(self) -> np.ndarray:
        quat = np.zeros(4)
        self.mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.site_id])
        return quat_normalize(quat)

    def _joint_ids(self, names: Sequence[str]) -> np.ndarray:
        ids = [
            self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in names
        ]
        return np.array([joint_id for joint_id in ids if joint_id >= 0], dtype=int)

    def _apply_gripper_target(self) -> None:
        if not self.has_gripper:
            return
        lower, upper = GRIPPER_JOINT_LIMIT_RAD
        joint_pos = lower + (upper - lower) * self.target_gripper_closedness
        if self.has_gripper_actuators:
            self.data.ctrl[self.gripper_actuator_ids] = joint_pos

    def _update_gripper_target(self, command: TeleopCommand) -> None:
        if command.gripper_intent == "open":
            self.target_gripper_closedness = 0.0
            self._last_gripper_command_direction = -1
            return
        if command.gripper_intent == "close":
            self.target_gripper_closedness = 1.0
            self._last_gripper_command_direction = 1
            return
        if command.gripper_intent != "hold":
            raise ValueError(f"unknown gripper intent: {command.gripper_intent}")

        delta = float(command.delta_gripper)
        if delta != 0.0:
            direction = 1 if delta > 0.0 else -1
            if direction != self._last_gripper_command_direction:
                self.target_gripper_closedness = self._observed_gripper_closedness()
            self.target_gripper_closedness = _clamp01(
                self.target_gripper_closedness + delta
            )
            self._last_gripper_command_direction = direction
            return

        self._last_gripper_command_direction = 0
        if command.gripper is not None:
            self.target_gripper_closedness = _clamp01(command.gripper)

    def _set_gripper_qpos_from_target(self) -> None:
        if not self.has_gripper:
            return
        lower, upper = GRIPPER_JOINT_LIMIT_RAD
        joint_pos = lower + (upper - lower) * self.target_gripper_closedness
        self.data.qpos[self.gripper_qpos_ids] = joint_pos
        self.data.qvel[self.gripper_dof_ids] = 0.0

    def _observed_gripper_closedness(self) -> float:
        if not self.has_gripper or len(self.gripper_qpos_ids) == 0:
            return 0.0
        lower, upper = GRIPPER_JOINT_LIMIT_RAD
        span = upper - lower
        if span <= 0.0:
            return 0.0
        return _clamp01(
            (float(self.data.qpos[self.gripper_qpos_ids[0]]) - lower) / span
        )


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _limit_vector_norm(
    vector: Sequence[float], limit: Optional[float]
) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    if limit is None or limit <= 0.0:
        return value
    norm = float(np.linalg.norm(value))
    if norm <= limit or norm <= 1e-12:
        return value
    return value * (limit / norm)


def _has_blocking_guard_penetration(
    previous_contacts: Sequence[_GuardContact],
    candidate_contacts: Sequence[_GuardContact],
) -> bool:
    previous_distances = {}
    for contact in previous_contacts:
        contact_key = (contact.guard_geom_id, contact.object_geom_id)
        previous_distances[contact_key] = min(
            previous_distances.get(contact_key, float("inf")),
            contact.distance_m,
        )

    for contact in candidate_contacts:
        penetration_limit = (
            0.0
            if contact.normal_z_abs >= KINEMATIC_VERTICAL_GUARD_NORMAL_Z_MIN
            else KINEMATIC_LATERAL_GUARD_PENETRATION_LIMIT_M
        )
        if contact.distance_m >= -penetration_limit:
            continue
        previous_distance = previous_distances.get(
            (contact.guard_geom_id, contact.object_geom_id)
        )
        if (
            previous_distance is None
            or contact.distance_m < previous_distance - 1e-7
        ):
            return True
    return False


def _require_mujoco():
    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError(
            "MuJoCo is not installed. Run: uv pip install -e '.[sim]'"
        ) from exc
    return mujoco
