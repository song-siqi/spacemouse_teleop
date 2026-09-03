# MuJoCo Backend

This backend should consume the same `TeleopCommand` as the real xArm backend.

The intended flow is:

```text
TeleopCommand.delta_pos_m + delta_rot_rad
  -> target end-effector pose integrator
  -> IK / operational-space controller
  -> MuJoCo xArm6 controls
```

The MuJoCo backend should not depend on ROS2 for the first prototype. It should expose the same recorder hooks as the xArm backend:

- raw SpaceMouse input
- filtered TeleopCommand
- target end-effector pose
- target joint position
- observed end-effector pose
- observed joint position

## Current Prototype

There does not appear to be a published UFACTORY xArm6 MJCF in the official
GitHub organization. UFACTORY's public simulation support is ROS/Gazebo and
`uf-gym`'s PyBullet/panda-gym path. The xArm6 MuJoCo scene is therefore
generated locally from the official xArm6 xacro/URDF source:

```text
third_party/xarm_ros2 xArm6 xacro/URDF
  -> MuJoCo URDF compiler
  -> generated MJCF
  -> local actuator/site/table/cube additions
```

`ensure_official_xarm6_table_cube_mjcf()` writes the generated model under
`.generated/mujoco/`. The generated scene contains:

- fixed xArm6 base on a table
- official xArm6 visual/collision meshes and inertial values
- MuJoCo `gravcomp=1` on the arm bodies for powered-servo hold behavior
- six hinge joints named `joint1` through `joint6`
- one hidden `eef` site at the gripper grasp center
- one free cube body on the table
- six joint position actuators named `joint*_pos`
- one `gripper_pos` actuator when using the xArm gripper
- fixed viewer cameras named `rear_side`, `overview`, `front`, `side`, and `top`

The controller mirrors the real xArm ROS2 stack at the semantic boundary:

```text
TeleopCommand EE delta
  -> MuJoCo target EE pose
  -> damped least-squares IK
  -> joint position target
  -> MuJoCo position actuator
```

In `xarm_ros2`, MoveIt Servo publishes `JointTrajectory` position targets to `/xarm6_traj_controller/joint_trajectory`, and the real hardware backend eventually calls `set_servo_angle_j`. The MuJoCo backend uses the same position-target idea without ROS2.

The default MuJoCo target mode is `velocity`: each SpaceMouse delta is applied
from the observed EE pose for that control frame. If the input goes to zero, the
target immediately returns to the observed pose, so the robot does not keep
chasing a stale accumulated target. `--target-mode integrated` is available for
experiments that intentionally want a persistent target-pose integrator.

The default arm control mode is `kinematic`: IK joint targets are applied
directly to the MuJoCo arm so teleop response is not dominated by a soft
simulated joint servo. `--arm-control-mode actuator` is available when testing
the lower-bandwidth MuJoCo position actuators themselves.

The controller exposes separate translational and rotational weights:
`--position-gain` and `--orientation-gain`. Lower them when the SpaceMouse feels
too aggressive without changing the device-to-command mapping.

For gripper/cube contact, the generated model disables the base and knuckle
mesh collisions but keeps the distal finger mesh collisions active under
`left_finger_mesh_collision` and `right_finger_mesh_collision`. Invisible
fingertip pad boxes add stable high-friction contact without showing separate
green collision geometry in the viewer. Finger and pad collisions are masked to
interact with the cube but not the table, while the cube still collides with the
table as usual.

The generated model sets a smaller MuJoCo timestep, Newton solver, and no-slip
iterations, then gives table/cube contacts stiff `solref`/`solimp` values to
reduce visible tabletop penetration. Gripper contacts use `condim=3` so they
provide pinch friction without torsional or rolling constraints that can feel
like artificial glue.

Kinematic arm execution still writes the arm pose directly for responsive
teleop, but it now interpolates those writes across MuJoCo substeps and provides
the corresponding joint velocities. This gives the contact solver the tangential
motion it needs for frictional lift tests.

The generated scene has several fixed cameras. Start with a chosen view:

```bash
python scripts/mujoco_teleop.py --backend pyspacemouse --viewer --camera rear_side
```

Show several views at once inside the viewer:

```bash
python scripts/mujoco_teleop.py --backend pyspacemouse --viewer --camera rear_side --multiview
python scripts/mujoco_teleop.py --backend pyspacemouse --viewer --camera rear_side --multiview --multiview-cameras overview,front,side,top --multiview-layout grid
```

To debug the model independently from SpaceMouse input:

```bash
python scripts/mujoco_diagnose.py --force-regenerate --duration 5
python scripts/mujoco_response_probe.py --target-mode velocity --arm-control-mode kinematic --position-gain 0.6 --orientation-gain 0.35
python scripts/mujoco_response_probe.py --target-mode integrated --arm-control-mode actuator
```
