from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from spacemouse_teleop.backends.mujoco.constants import JOINT_VELOCITY_LIMIT_RADPS
from spacemouse_teleop.backends.mujoco.math_utils import orientation_error


@dataclass(frozen=True)
class IkResult:
    qpos_target: np.ndarray
    position_error: np.ndarray
    orientation_error: np.ndarray
    joint_step: np.ndarray

    @property
    def error_norm(self) -> float:
        return float(
            np.linalg.norm(
                np.concatenate((self.position_error, self.orientation_error))
            )
        )


class DampedLeastSquaresIk:
    def __init__(
        self,
        model,
        joint_names: Sequence[str],
        site_name: str = "eef",
        damping: float = 0.04,
        position_gain: float = 1.0,
        orientation_gain: float = 0.6,
        joint_velocity_limit_radps: float = JOINT_VELOCITY_LIMIT_RADPS,
    ) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise RuntimeError(f"MuJoCo site not found: {site_name}")

        self.joint_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in joint_names
            ],
            dtype=int,
        )
        if np.any(self.joint_ids < 0):
            missing = [
                name for name, joint_id in zip(joint_names, self.joint_ids) if joint_id < 0
            ]
            raise RuntimeError(f"MuJoCo joints not found: {missing}")

        self.qpos_ids = np.array(model.jnt_qposadr[self.joint_ids], dtype=int)
        self.dof_ids = np.array(model.jnt_dofadr[self.joint_ids], dtype=int)
        self.joint_ranges = np.array(model.jnt_range[self.joint_ids], dtype=float)
        self.damping = float(damping)
        self.position_gain = float(position_gain)
        self.orientation_gain = float(orientation_gain)
        self.joint_velocity_limit_radps = float(joint_velocity_limit_radps)

    def solve(self, data, target_pos, target_quat, dt: float) -> IkResult:
        current_pos = np.array(data.site_xpos[self.site_id], dtype=float)
        current_quat = _site_quat(self.mujoco, data, self.site_id)

        pos_error = self.position_gain * (np.asarray(target_pos, dtype=float) - current_pos)
        rot_error = self.orientation_gain * orientation_error(target_quat, current_quat)

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        self.mujoco.mj_jacSite(self.model, data, jacp, jacr, self.site_id)
        jacobian = np.vstack((jacp[:, self.dof_ids], jacr[:, self.dof_ids]))
        error = np.concatenate((pos_error, rot_error))

        regularizer = (self.damping**2) * np.eye(6)
        joint_step = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + regularizer, error
        )

        max_step = max(0.0, dt) * self.joint_velocity_limit_radps
        if max_step > 0.0:
            joint_step = np.clip(joint_step, -max_step, max_step)

        qpos = np.array(data.qpos[self.qpos_ids], dtype=float)
        qpos_target = np.clip(
            qpos + joint_step,
            self.joint_ranges[:, 0],
            self.joint_ranges[:, 1],
        )
        return IkResult(qpos_target, pos_error, rot_error, joint_step)


def _site_quat(mujoco, data, site_id: int) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat
