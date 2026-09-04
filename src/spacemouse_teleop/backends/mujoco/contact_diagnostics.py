from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

from spacemouse_teleop.backends.mujoco.constants import (
    GRIPPER_GUARD_GEOM_NAMES,
    GRIPPER_PAD_GEOM_NAMES,
)


@dataclass(frozen=True)
class ContactForce:
    geom1: str
    geom2: str
    position_m: Tuple[float, float, float]
    normal_xyz: Tuple[float, float, float]
    distance_m: float
    normal_force_n: float
    tangential_force_n: float
    torsional_torque_nm: float

    @property
    def pair_name(self) -> str:
        return " <-> ".join(sorted((self.geom1, self.geom2)))


@dataclass(frozen=True)
class ContactSnapshot:
    phase: str
    timestamp: float
    object_body_name: str
    object_geom_names: Tuple[str, ...]
    ee_pos_m: Tuple[float, float, float]
    object_pos_m: Tuple[float, float, float]
    gripper_pad_positions_m: Tuple[
        Tuple[float, float, float], Tuple[float, float, float]
    ]
    ee_linear_speed_mps: float
    ee_angular_speed_radps: float
    object_linear_speed_mps: float
    object_angular_speed_radps: float
    relative_angular_speed_radps: float
    object_kinetic_energy_j: float
    ee_quat_wxyz: Tuple[float, float, float, float]
    object_quat_wxyz: Tuple[float, float, float, float]
    contacts: Tuple[ContactForce, ...]

    def to_dict(self) -> dict:
        return asdict(self)

    @property
    def cube_pos_m(self) -> Tuple[float, float, float]:
        return self.object_pos_m

    @property
    def cube_linear_speed_mps(self) -> float:
        return self.object_linear_speed_mps

    @property
    def cube_angular_speed_radps(self) -> float:
        return self.object_angular_speed_radps

    @property
    def cube_kinetic_energy_j(self) -> float:
        return self.object_kinetic_energy_j

    @property
    def cube_quat_wxyz(self) -> Tuple[float, float, float, float]:
        return self.object_quat_wxyz


@dataclass(frozen=True)
class RoleContactSummary:
    contact_fraction: float
    contact_count: int
    max_normal_force_n: float
    max_tangential_force_n: float
    max_torsional_torque_nm: float
    max_penetration_m: float


@dataclass(frozen=True)
class PhaseContactSummary:
    phase: str
    sample_count: int
    all_contacts: RoleContactSummary
    pads: RoleContactSummary
    left_pad: RoleContactSummary
    right_pad: RoleContactSummary
    guards: RoleContactSummary
    table: RoleContactSummary
    max_ee_angular_speed_radps: float
    max_object_linear_speed_mps: float
    max_object_angular_speed_radps: float
    max_relative_angular_speed_radps: float
    net_ee_rotation_rad: float
    net_object_rotation_rad: float
    relative_rotation_drift_rad: float
    max_object_kinetic_energy_j: float

    @property
    def object_contact_fraction(self) -> float:
        return self.all_contacts.contact_fraction

    @property
    def cube_contact_fraction(self) -> float:
        return self.object_contact_fraction

    @property
    def pad_contact_fraction(self) -> float:
        return self.pads.contact_fraction

    @property
    def max_cube_linear_speed_mps(self) -> float:
        return self.max_object_linear_speed_mps

    @property
    def max_cube_angular_speed_radps(self) -> float:
        return self.max_object_angular_speed_radps

    @property
    def net_cube_rotation_rad(self) -> float:
        return self.net_object_rotation_rad

    @property
    def max_cube_kinetic_energy_j(self) -> float:
        return self.max_object_kinetic_energy_j

    @property
    def max_normal_force_n(self) -> float:
        return self.all_contacts.max_normal_force_n

    @property
    def max_tangential_force_n(self) -> float:
        return self.all_contacts.max_tangential_force_n

    @property
    def max_torsional_torque_nm(self) -> float:
        return self.all_contacts.max_torsional_torque_nm

    @property
    def max_penetration_m(self) -> float:
        return self.all_contacts.max_penetration_m


def capture_contact_snapshot(
    env,
    phase: str,
    *,
    object_body_name: str = "cube",
    object_geom_names: Optional[Sequence[str]] = None,
) -> ContactSnapshot:
    object_body_id = env.mujoco.mj_name2id(
        env.model, env.mujoco.mjtObj.mjOBJ_BODY, object_body_name
    )
    if object_body_id < 0:
        raise RuntimeError(f"MuJoCo object body not found: {object_body_name}")
    object_geom_ids = _object_geom_ids(
        env, object_body_id, object_geom_names
    )
    object_geom_id_set = frozenset(object_geom_ids)
    resolved_object_geom_names = tuple(
        _geom_name(env, geom_id) for geom_id in object_geom_ids
    )
    ee_velocity = _object_velocity(
        env, env.mujoco.mjtObj.mjOBJ_SITE, env.site_id
    )
    object_velocity = _object_velocity(
        env, env.mujoco.mjtObj.mjOBJ_BODY, object_body_id
    )
    ee_quat = _site_quat(env)
    object_quat = _normalize_quat(env.data.xquat[object_body_id])
    pad_positions = tuple(
        _geom_position(env, name)
        for name in ("left_finger_pad_collision", "right_finger_pad_collision")
    )
    contacts = []
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        geom1_id = int(contact.geom1)
        geom2_id = int(contact.geom2)
        if not object_geom_id_set.intersection((geom1_id, geom2_id)):
            continue
        geom1 = _geom_name(env, geom1_id)
        geom2 = _geom_name(env, geom2_id)

        wrench = np.zeros(6, dtype=float)
        env.mujoco.mj_contactForce(env.model, env.data, index, wrench)
        contacts.append(
            ContactForce(
                geom1=geom1,
                geom2=geom2,
                position_m=tuple(float(value) for value in contact.pos),
                normal_xyz=tuple(float(value) for value in contact.frame[:3]),
                distance_m=float(contact.dist),
                normal_force_n=abs(float(wrench[0])),
                tangential_force_n=float(np.linalg.norm(wrench[1:3])),
                torsional_torque_nm=abs(float(wrench[3])),
            )
        )

    return ContactSnapshot(
        phase=phase,
        timestamp=float(env.data.time),
        object_body_name=object_body_name,
        object_geom_names=resolved_object_geom_names,
        ee_pos_m=tuple(float(value) for value in env.data.site_xpos[env.site_id]),
        object_pos_m=tuple(
            float(value) for value in env.data.xpos[object_body_id]
        ),
        gripper_pad_positions_m=pad_positions,
        ee_linear_speed_mps=float(np.linalg.norm(ee_velocity[3:6])),
        ee_angular_speed_radps=float(np.linalg.norm(ee_velocity[0:3])),
        object_linear_speed_mps=float(np.linalg.norm(object_velocity[3:6])),
        object_angular_speed_radps=float(np.linalg.norm(object_velocity[0:3])),
        relative_angular_speed_radps=float(
            np.linalg.norm(object_velocity[0:3] - ee_velocity[0:3])
        ),
        object_kinetic_energy_j=_body_kinetic_energy(env, object_body_id),
        ee_quat_wxyz=tuple(float(value) for value in ee_quat),
        object_quat_wxyz=tuple(float(value) for value in object_quat),
        contacts=tuple(contacts),
    )


def summarize_contact_snapshots(
    snapshots: Iterable[ContactSnapshot],
) -> Tuple[PhaseContactSummary, ...]:
    grouped: Dict[str, list[ContactSnapshot]] = {}
    for snapshot in snapshots:
        grouped.setdefault(snapshot.phase, []).append(snapshot)

    summaries = []
    for phase, samples in grouped.items():
        sample_count = len(samples)
        first = samples[0]
        last = samples[-1]
        first_relative_quat = _quat_multiply(
            _quat_conjugate(first.ee_quat_wxyz), first.object_quat_wxyz
        )
        last_relative_quat = _quat_multiply(
            _quat_conjugate(last.ee_quat_wxyz), last.object_quat_wxyz
        )
        left_pad_name, right_pad_name = GRIPPER_PAD_GEOM_NAMES
        pad_names = frozenset(GRIPPER_PAD_GEOM_NAMES)
        guard_names = frozenset(GRIPPER_GUARD_GEOM_NAMES)
        summaries.append(
            PhaseContactSummary(
                phase=phase,
                sample_count=sample_count,
                all_contacts=_summarize_role(samples, lambda contact: True),
                pads=_summarize_role(
                    samples,
                    lambda contact, names=pad_names: _contact_involves_any(
                        contact, names
                    ),
                ),
                left_pad=_summarize_role(
                    samples,
                    lambda contact, name=left_pad_name: _contact_involves(
                        contact, name
                    ),
                ),
                right_pad=_summarize_role(
                    samples,
                    lambda contact, name=right_pad_name: _contact_involves(
                        contact, name
                    ),
                ),
                guards=_summarize_role(
                    samples,
                    lambda contact, names=guard_names: _contact_involves_any(
                        contact, names
                    ),
                ),
                table=_summarize_role(
                    samples,
                    lambda contact: _contact_involves(contact, "table"),
                ),
                max_ee_angular_speed_radps=max(
                    sample.ee_angular_speed_radps for sample in samples
                ),
                max_object_linear_speed_mps=max(
                    sample.object_linear_speed_mps for sample in samples
                ),
                max_object_angular_speed_radps=max(
                    sample.object_angular_speed_radps for sample in samples
                ),
                max_relative_angular_speed_radps=max(
                    sample.relative_angular_speed_radps for sample in samples
                ),
                net_ee_rotation_rad=_quat_distance(
                    first.ee_quat_wxyz, last.ee_quat_wxyz
                ),
                net_object_rotation_rad=_quat_distance(
                    first.object_quat_wxyz, last.object_quat_wxyz
                ),
                relative_rotation_drift_rad=_quat_distance(
                    first_relative_quat, last_relative_quat
                ),
                max_object_kinetic_energy_j=max(
                    sample.object_kinetic_energy_j for sample in samples
                ),
            )
        )
    return tuple(summaries)


def summarize_contact_pairs(
    snapshots: Iterable[ContactSnapshot],
) -> Dict[str, dict]:
    pairs: Dict[str, dict] = {}
    for snapshot in snapshots:
        for contact in snapshot.contacts:
            stats = pairs.setdefault(
                contact.pair_name,
                {
                    "samples": 0,
                    "max_normal_force_n": 0.0,
                    "max_tangential_force_n": 0.0,
                    "max_torsional_torque_nm": 0.0,
                    "max_penetration_m": 0.0,
                },
            )
            stats["samples"] += 1
            stats["max_normal_force_n"] = max(
                stats["max_normal_force_n"], contact.normal_force_n
            )
            stats["max_tangential_force_n"] = max(
                stats["max_tangential_force_n"], contact.tangential_force_n
            )
            stats["max_torsional_torque_nm"] = max(
                stats["max_torsional_torque_nm"], contact.torsional_torque_nm
            )
            stats["max_penetration_m"] = max(
                stats["max_penetration_m"], max(0.0, -contact.distance_m)
            )
    return pairs


def _summarize_role(
    samples: Sequence[ContactSnapshot],
    predicate: Callable[[ContactForce], bool],
) -> RoleContactSummary:
    contacts_by_sample = [
        tuple(contact for contact in sample.contacts if predicate(contact))
        for sample in samples
    ]
    contacts = [contact for group in contacts_by_sample for contact in group]
    return RoleContactSummary(
        contact_fraction=sum(bool(group) for group in contacts_by_sample)
        / len(samples),
        contact_count=len(contacts),
        max_normal_force_n=max(
            (contact.normal_force_n for contact in contacts), default=0.0
        ),
        max_tangential_force_n=max(
            (contact.tangential_force_n for contact in contacts), default=0.0
        ),
        max_torsional_torque_nm=max(
            (contact.torsional_torque_nm for contact in contacts), default=0.0
        ),
        max_penetration_m=max(
            (max(0.0, -contact.distance_m) for contact in contacts), default=0.0
        ),
    )


def _contact_involves(contact: ContactForce, geom_name: str) -> bool:
    return geom_name in (contact.geom1, contact.geom2)


def _contact_involves_any(
    contact: ContactForce, geom_names: frozenset[str]
) -> bool:
    return bool(geom_names.intersection((contact.geom1, contact.geom2)))


def _geom_name(env, geom_id: int) -> str:
    name = env.mujoco.mj_id2name(
        env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_id
    )
    return str(name) if name is not None else f"geom_{geom_id}"


def _geom_position(env, geom_name: str) -> Tuple[float, float, float]:
    geom_id = env.mujoco.mj_name2id(
        env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_name
    )
    if geom_id < 0:
        return (0.0, 0.0, 0.0)
    return tuple(float(value) for value in env.data.geom_xpos[geom_id])


def _object_velocity(env, object_type, object_id: int) -> np.ndarray:
    velocity = np.zeros(6, dtype=float)
    env.mujoco.mj_objectVelocity(
        env.model, env.data, object_type, object_id, velocity, 0
    )
    return velocity


def _site_quat(env) -> np.ndarray:
    quat = np.zeros(4, dtype=float)
    env.mujoco.mju_mat2Quat(quat, env.data.site_xmat[env.site_id])
    return _normalize_quat(quat)


def _normalize_quat(quat) -> np.ndarray:
    value = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return value / norm


def _quat_conjugate(quat) -> np.ndarray:
    value = _normalize_quat(quat)
    return np.array([value[0], -value[1], -value[2], -value[3]])


def _quat_multiply(first, second) -> np.ndarray:
    w1, x1, y1, z1 = _normalize_quat(first)
    w2, x2, y2, z2 = _normalize_quat(second)
    return _normalize_quat(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    )


def _quat_distance(first, second) -> float:
    dot = abs(float(np.dot(_normalize_quat(first), _normalize_quat(second))))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _object_geom_ids(
    env,
    object_body_id: int,
    object_geom_names: Optional[Sequence[str]],
) -> Tuple[int, ...]:
    if object_geom_names is None:
        geom_ids = tuple(
            geom_id
            for geom_id in range(env.model.ngeom)
            if _body_is_descendant(
                env.model, int(env.model.geom_bodyid[geom_id]), object_body_id
            )
        )
    else:
        geom_ids = tuple(
            env.mujoco.mj_name2id(
                env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            for geom_name in object_geom_names
        )
        missing = [
            name
            for name, geom_id in zip(object_geom_names, geom_ids, strict=True)
            if geom_id < 0
        ]
        if missing:
            raise RuntimeError(
                f"MuJoCo object geoms not found: {', '.join(missing)}"
            )
        outside_body = [
            _geom_name(env, geom_id)
            for geom_id in geom_ids
            if not _body_is_descendant(
                env.model, int(env.model.geom_bodyid[geom_id]), object_body_id
            )
        ]
        if outside_body:
            raise RuntimeError(
                "MuJoCo object geoms are outside the tracked body subtree: "
                + ", ".join(outside_body)
            )
    if not geom_ids:
        body_name = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_BODY, object_body_id
        )
        raise RuntimeError(f"MuJoCo object body has no geoms: {body_name}")
    return geom_ids


def _body_is_descendant(model, body_id: int, root_body_id: int) -> bool:
    current = body_id
    while current > 0:
        if current == root_body_id:
            return True
        current = int(model.body_parentid[current])
    return root_body_id == 0 and current == 0


def _body_kinetic_energy(env, object_body_id: int) -> float:
    dof_ids = []
    for body_id in range(env.model.nbody):
        if not _body_is_descendant(env.model, body_id, object_body_id):
            continue
        dof_adr = int(env.model.body_dofadr[body_id])
        dof_num = int(env.model.body_dofnum[body_id])
        if dof_adr >= 0:
            dof_ids.extend(range(dof_adr, dof_adr + dof_num))
    if not dof_ids:
        return 0.0

    dof_ids_array = np.asarray(dof_ids, dtype=int)
    mass_matrix = np.zeros((env.model.nv, env.model.nv), dtype=float)
    try:
        env.mujoco.mj_fullM(env.model, env.data, mass_matrix)
    except TypeError:
        compressed_mass = getattr(env.data, "qM", None)
        if compressed_mass is None:
            compressed_mass = env.data.M
        env.mujoco.mj_fullM(env.model, mass_matrix, compressed_mass)
    velocity = np.asarray(env.data.qvel[dof_ids_array], dtype=float)
    inertia = mass_matrix[np.ix_(dof_ids_array, dof_ids_array)]
    energy = 0.5 * float(velocity @ inertia @ velocity)
    return max(0.0, energy) if not np.isnan(energy) else 0.0
