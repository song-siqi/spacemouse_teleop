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
src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2 xArm6 xacro/URDF
  -> MuJoCo URDF compiler
  -> generated MJCF
  -> local actuator/site/table/cube additions
```

The package-local `assets/xarm_ros2` directory is a minimal copied subset of
the upstream UFACTORY `xarm_ros2` model assets. The ignored
`third_party/xarm_ros2` checkout is only a reference for auditing or refreshing
that subset; the simulator should not require it at runtime.

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

TeleopCommand gripper intent
  -> MuJoCo backend open/close/hold target policy
  -> xArm gripper actuator target
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
The MuJoCo backend also limits EE target rotation to `0.25 rad/s` by default.
This keeps the kinematic controller inside the stable grasp-contact range while
leaving the shared SpaceMouse command unchanged. Pass
`--max-ee-angular-speed 0` to disable the limit for stress tests.

For gripper/object contact, the generated model keeps the official gripper mesh
for rendering but disables its complex mesh collision. Invisible fingertip pad
boxes provide the high-friction grasp surfaces. Low-friction primitive guards
cover the palm, outer knuckles, and finger backs so those visible surfaces do
not pass through manipulated objects. Scene, pad, and guard geoms use collision
bits `1`, `2`, and `4`; a dynamic manipulation object opts into all three with
`contype=1` and `conaffinity=7`. Pad and guard affinity is zero, so they do not
collide with themselves, one another, or the table.

The generated model borrows the table/object friction scale from
`MingqianW/embodied-ai-xarm`: table `1.0 0.01 0.001` and cube
`1.2 0.01 0.001`. Local manipulation regressions use fingertip pads
`2.0 0.005 0.0005` and guards `0.2 0.001 0.0001`. The scene keeps our smaller
MuJoCo timestep, Newton solver, no-slip
iterations, and stiff table/cube `solref`/`solimp` values to reduce visible
tabletop penetration. The pad geoms themselves use `condim=4`,
`solref="0.003 1"`, and `solimp="0.95 0.99 0.001"`; no object-name-specific
contact pair is required. Guards use `condim=3`, `solref="0.01 1"`, and
`solimp="0.9 0.95 0.001"` for softer low-friction push contact. Guard priority
is `2` versus the object's `1`, ensuring these guard parameters win MuJoCo's
dynamic contact-pair combination; pads remain at priority `3`.

The official gripper linkage is driven with `kp=20`, `kv=3`, and a `-4 4`
actuator force range. This is enough to lift the cube without the very large
residual pinch forces produced by the untuned linkage.

Kinematic arm execution still writes the arm pose directly for responsive
teleop, but it now interpolates those writes across MuJoCo substeps and provides
the corresponding joint velocities. This gives the contact solver the
tangential motion it needs for frictional lift tests. Each candidate substep is
also checked for contact between a guard and any geom that accepts collision bit
`4`. A contact with `|normal_z| >= 0.7` stops at the previous valid arm pose when
penetration would deepen. Other contacts may compress the guard by up to `2 mm`,
allowing the gripper to push a free object across the tabletop without permitting
unbounded overlap. Motion away from contact remains accepted.

The generated scene has several fixed cameras. Start with a chosen view:

```bash
SPACEMOUSE_TELEOP_CAMERA=rear_side ./scripts/run_mujoco_spacemouse.sh
```

Show several views at once inside the viewer:

```bash
SPACEMOUSE_TELEOP_MULTIVIEW=1 ./scripts/run_mujoco_spacemouse.sh
SPACEMOUSE_TELEOP_MULTIVIEW_CAMERAS=overview,front,side,top SPACEMOUSE_TELEOP_MULTIVIEW_LAYOUT=grid ./scripts/run_mujoco_spacemouse.sh
```

Show the pad primitives in translucent green and guard primitives in blue:

```bash
SPACEMOUSE_TELEOP_SHOW_COLLISION_GEOMS=1 ./scripts/run_mujoco_spacemouse.sh
```

To debug the model independently from SpaceMouse input:

```bash
python scripts/mujoco_diagnose.py --force-regenerate --duration 5
python scripts/mujoco_contact_diagnostics.py --log logs/mujoco_contact.jsonl
python scripts/mujoco_contact_diagnostics.py --model custom.xml --object-body payload --object-geom payload_collision
python scripts/mujoco_gripper_top_press_probe.py
python scripts/mujoco_response_probe.py --target-mode velocity --arm-control-mode kinematic --position-gain 0.6 --orientation-gain 0.35
python scripts/mujoco_response_probe.py --target-mode integrated --arm-control-mode actuator
```

Run the complete simulation regression suite without a physical SpaceMouse:

```bash
python -m unittest discover -s tests -v
```

The tests feed synthetic commands directly to the backend; reader coverage is
mocked and does not open a HID device.

## Physics Regression Contract

The automated suite treats the following limits as the baseline contract for
future geometry or contact tuning:

- A `120 mm` EE lift raises the object by at least `80 mm`, with both pads in
  sustained contact.
- A grasp held in free space for `2 s` drifts by less than `5 mm` and `5 deg`.
- Grasp rotation stays within `10 deg` of the EE, below `2 mm` penetration and
  `80 N` peak normal force.
- A top guard contact stays below `0.5 mm` penetration; object/table penetration
  stays below `2 mm`, and retreat remains possible.
- A lateral guard push moves the object by at least `40 mm`, stays below
  `2.5 mm` penetration, releases on retreat, and drags the object less than
  `5 mm` afterward.
- Renamed box and cylinder bodies exercise the same guard behavior, proving the
  kinematic constraint is selected by collision mask rather than geom name.
- Critical kinematic scenarios run at `30`, `60`, and `120 Hz`; actuator mode
  is checked for finite state, bounded velocity, and at most `3 mm` penetration.
