"""Generic dynamics backend. Runtime control never teleports joints or objects."""
from __future__ import annotations
import math
import numpy as np
from robot_teleop.core import (CartesianTwist, CartesianPoseTarget, JointPositionTarget,
                              GripperCommand, GripperState, Pose, RobotState, Stamp)
from .assembly import build_model
from .controller import DampedLeastSquaresIk
from .math_utils import quat_from_rotvec, quat_multiply, orientation_error


class MujocoBackend:
    def __init__(self, robot="xarm6", tool="xarm_gripper", control_hz=60., *, model=None, binding=None):
        import mujoco
        if not math.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be positive")
        self.mj = mujoco
        self.model, self.binding = build_model(robot, tool) if model is None else (model, binding)
        self.data = mujoco.MjData(self.model)
        self.dt = 1. / control_hz
        self.servo_horizon = 1. / 60.
        b = self.binding
        self.joints = self._ids(mujoco.mjtObj.mjOBJ_JOINT, b.joints)
        self.qids = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.actuators = self._ids(mujoco.mjtObj.mjOBJ_ACTUATOR, b.actuators)
        self.tool_actuators = self._ids(mujoco.mjtObj.mjOBJ_ACTUATOR, b.gripper.actuators)
        self.tool_joints = self._ids(mujoco.mjtObj.mjOBJ_JOINT, b.gripper.joints)
        self.driver = self._ids(mujoco.mjtObj.mjOBJ_JOINT, (b.gripper.driver,))[0]
        self.site = self._ids(mujoco.mjtObj.mjOBJ_SITE, (b.tcp,))[0]
        self.object_bodies = {int(self.model.jnt_bodyid[j]) for j in range(self.model.njnt)
                              if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE}
        self.ik = DampedLeastSquaresIk(self.model, b.joints, b.tcp, orientation_gain=1.,
            joint_velocity_limit_radps=b.velocities, home=b.home, posture_gain=.02 if len(b.joints) == 7 else 0.)
        self.epoch = -1
        self.paused = False
        self.renderer = None
        self.reset()

    def _ids(self, kind, names):
        ids = np.array([self.mj.mj_name2id(self.model, kind, name) for name in names], dtype=int)
        if np.any(ids < 0):
            raise ValueError(f"Missing binding: {names}")
        return ids

    def capabilities(self):
        return frozenset((CartesianTwist, CartesianPoseTarget, JointPositionTarget, GripperCommand))

    def connect(self):
        return None

    def reset(self):
        self.epoch += 1
        self.mj.mj_resetData(self.model, self.data)
        self.data.qpos[self.qids] = self.binding.home
        self.data.ctrl[self.actuators] = self.binding.home
        # Default linkage coordinates are the upstream open configuration.
        self.data.ctrl[self.tool_actuators] = self.binding.gripper.control_range[0]
        self.tool_target = self.binding.gripper.control_range[0]
        self.mj.mj_forward(self.model, self.data)
        self.scheduled_time = 0.
        self.command = None
        self.deadline = self.stamp()
        self.tool_intent = "hold"
        self.target = self.observe().tcp
        return self.observe()

    def stamp(self):
        return Stamp(float(self.data.time), "simulation", self.epoch)

    def observe(self):
        q = np.zeros(4)
        self.mj.mju_mat2Quat(q, self.data.site_xmat[self.site])
        tool = self.binding.gripper
        native = float(self.data.qpos[self.model.jnt_qposadr[self.driver]])
        normalized = float(np.clip((native-tool.driver_range[0]) / (tool.driver_range[1]-tool.driver_range[0]), 0, 1))
        return RobotState(self.stamp(), self.binding.joints, tuple(self.data.qpos[self.qids]),
            tuple(self.data.qvel[self.dofs]), Pose(tuple(self.data.site_xpos[self.site]), tuple(q)),
            GripperState(normalized, tuple(self.data.qpos[self.model.jnt_qposadr[self.tool_joints]]), "rad", self.tool_intent),
            "paused" if self.paused else "running")

    def scene_state(self):
        return {self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_BODY, int(self.model.jnt_bodyid[j])):
                Pose(tuple(self.data.xpos[self.model.jnt_bodyid[j]]), tuple(self.data.xquat[self.model.jnt_bodyid[j]]))
                for j in range(self.model.njnt) if self.model.jnt_type[j] == self.mj.mjtJoint.mjJNT_FREE}

    def submit(self, command, deadline):
        if type(command) not in self.capabilities():
            raise ValueError("Unsupported command")
        if deadline.remaining(self.stamp()) <= 0:
            raise ValueError("Expired command")
        if isinstance(command, GripperCommand):
            self._gripper(command)
            self.tool_deadline = deadline
            return
        if isinstance(command, (CartesianTwist, CartesianPoseTarget)):
            frame = command.frame if isinstance(command, CartesianTwist) else command.pose.frame
            if frame != "world":
                raise ValueError("MuJoCo Cartesian commands require world frame")
        if isinstance(command, JointPositionTarget):
            if command.names != self.binding.joints:
                raise ValueError("Joint names/order must match binding")
            ranges = self.model.jnt_range[self.joints]
            if np.any(np.asarray(command.positions) < ranges[:, 0]) or np.any(np.asarray(command.positions) > ranges[:, 1]):
                raise ValueError("Joint target outside limits")
        self.command, self.deadline = command, deadline

    def _gripper(self, command):
        value = {"open": 0., "close": 1.}.get(command.intent, command.position)
        if command.intent == "hold":
            value = self.observe().gripper.normalized_position
        lo, hi = self.binding.gripper.control_range
        self.tool_target = lo + value * (hi - lo)
        if command.intent == "hold":
            self.data.ctrl[self.tool_actuators] = self.tool_target
        self.tool_intent = command.intent

    def stop(self):
        self.command = None
        self.target = self.observe().tcp
        self.data.ctrl[self.actuators] = self.data.qpos[self.qids]
        self._gripper(GripperCommand())

    def advance(self, dt=None, *, on_step=None):
        dt = self.dt if dt is None else dt
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("advance duration must be finite and positive")
        if self.paused:
            return self.observe()
        self.scheduled_time += dt
        h = float(self.model.opt.timestep)
        while self.data.time + h <= self.scheduled_time + 1e-10:
            now = self.stamp()
            if self.command is not None and self.deadline.remaining(now) <= 0:
                self.stop()
            if self.tool_intent != "hold" and self.tool_deadline.remaining(now) <= 0:
                self._gripper(GripperCommand())
            tool = self.binding.gripper
            step = tool.normalized_speed * (tool.control_range[1] - tool.control_range[0]) * h
            self.data.ctrl[self.tool_actuators] += np.clip(
                self.tool_target - self.data.ctrl[self.tool_actuators], -step, step)
            current = self.observe().tcp
            if isinstance(self.command, JointPositionTarget):
                delta = np.asarray(self.command.positions) - self.data.qpos[self.qids]
                self.data.ctrl[self.actuators] = self.data.qpos[self.qids] + np.clip(delta, -np.array(self.binding.velocities)*self.servo_horizon, np.array(self.binding.velocities)*self.servo_horizon)
            else:
                if isinstance(self.command, CartesianTwist):
                    # Reset accumulated error on reversal so retreat is immediate.
                    v = np.asarray(self.command.linear)
                    error = np.asarray(self.target.position) - current.position
                    error = np.where(error * v < 0, 0., error)
                    pos = np.asarray(current.position) + error + v * h
                    quat = quat_multiply(quat_from_rotvec(np.asarray(self.command.angular)*h), self.target.quaternion)
                elif isinstance(self.command, CartesianPoseTarget):
                    pos, quat = self.command.pose.position, self.command.pose.quaternion
                else:
                    pos, quat = self.target.position, self.target.quaternion
                error = np.asarray(pos) - current.position
                error *= min(1., .025 / max(np.linalg.norm(error), 1e-12))
                rot = orientation_error(quat, current.quaternion)
                rot *= min(1., .15 / max(np.linalg.norm(rot), 1e-12))
                quat = quat_multiply(quat_from_rotvec(rot), current.quaternion)
                # Limit downward loading at shell contacts by projecting the
                # actuator target, never the measured pose. Lateral pushing stays
                # compliant; retreat is not projected away.
                delta = np.concatenate((error, rot))
                for contact in self.data.contact[:self.data.ncon]:
                    a, b = int(contact.geom1), int(contact.geom2)
                    shell_a = self.model.geom_contype[a] == 4
                    shell_b = self.model.geom_contype[b] == 4
                    if shell_a == shell_b or contact.dist > -.0001:
                        continue
                    other = b if shell_a else a
                    if int(self.model.geom_bodyid[other]) not in self.object_bodies or not (self.model.geom_conaffinity[other] & 4):
                        continue
                    normal = np.asarray(contact.frame[:3]) * (1 if shell_a else -1)
                    if abs(normal[2]) < .7:
                        continue
                    lever = np.asarray(contact.pos) - current.position
                    axis = np.concatenate((normal, np.cross(lever, normal)))
                    inward = float(delta @ axis)
                    if inward > 0:
                        delta -= axis * inward / float(axis @ axis)
                error, rot = delta[:3], delta[3:]
                quat = quat_multiply(quat_from_rotvec(rot), current.quaternion)
                self.target = Pose(tuple(np.asarray(current.position) + error), tuple(quat))
                target = self.ik.solve(self.data, self.target.position, self.target.quaternion, self.servo_horizon)
                self.data.ctrl[self.actuators] = target.qpos_target
            self.mj.mj_step(self.model, self.data)
            if on_step is not None:
                on_step(self)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self.stop()
                raise RuntimeError("Nonfinite simulation state")
        self.mj.mj_forward(self.model, self.data)
        return self.observe()

    def render(self, camera="overview", width=640, height=480):
        if self.renderer is not None and self._render_size != (width, height):
            self.renderer.close()
            self.renderer = None
        if self.renderer is None:
            self.renderer = self.mj.Renderer(self.model, height=height, width=width)
            self._render_size = (width, height)
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render().copy()

    def set_collision_debug(self, enabled):
        for g in range(self.model.ngeom):
            role = int(self.model.geom_contype[g])
            if role in (2,4):
                self.model.geom_rgba[g] = (*( (.05,.65,.2) if role == 2 else (.15,.55,.85)), .35 if enabled else 0.)
                self.model.geom_group[g] = 1 if enabled else 3

    def close(self):
        self.stop()
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
