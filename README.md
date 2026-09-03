# SpaceMouse Teleop

This repo is organized around one reusable SpaceMouse command layer and separate execution backends for the real xArm6 stack and MuJoCo.

```text
SpaceMouse device
  -> src/spacemouse_teleop/spacemouse
  -> TeleopCommand
  -> src/spacemouse_teleop/backends/xarm_ros2
  -> src/spacemouse_teleop/backends/mujoco
```

Top-level layout:

```text
configs/                         shared launch/runtime config
scripts/                         direct debug entrypoints
src/spacemouse_teleop/spacemouse SpaceMouse reader, mapping, filters, command contract
src/spacemouse_teleop/backends   execution adapters, one subdirectory per backend
src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2
                                 repo-local xArm6/gripper simulation asset subset
src/spacemouse_teleop/recording  lightweight debug/data logging helpers
third_party/xarm_ros2/           read-only upstream reference clone
```

The cloned `third_party/xarm_ros2` directory is a read-only reference. Its SpaceMouse path maps `sensor_msgs/Joy` into `geometry_msgs/TwistStamped` on `/servo_server/delta_twist_cmds`, then MoveIt Servo converts that Cartesian command into joint trajectory commands for the xArm controller. The top-level `.gitignore` excludes this local clone so the main project does not accidentally vendor the upstream repository; add it deliberately as a submodule later if we want to pin an exact upstream revision.

MuJoCo runtime generation does not depend on the ignored `third_party` checkout.
The required xArm6/gripper xacro, mesh, and yaml subset is copied into
`src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2`, with the upstream
license kept next to those files. Use `third_party/xarm_ros2` only as an audit
and refresh reference.

If the directory is missing, recreate the local reference with:

```bash
git clone https://github.com/xArm-Developer/xarm_ros2.git third_party/xarm_ros2
```

For the real robot path, the xArm ROS2 controller updates at `150 Hz` and the hardware backend ultimately sends joint position commands through the SDK call `set_servo_angle_j`. The MuJoCo backend follows the same high-level split: SpaceMouse emits Cartesian EE commands, while the backend converts them into joint position targets.

## SpaceMouse Layer

The SpaceMouse layer is intentionally backend-neutral:

```text
pyspacemouse/mock reader
  -> RawSpaceMouseState
  -> TeleopCore mapping/deadzone/mode buttons/scaling/filtering
  -> TeleopCommand
```

The default debug loop samples the SpaceMouse and produces commands at `60 Hz`. Use `--hz` to change this target rate. `--print-rate` only controls terminal output frequency; it does not change the command sampling rate.

The current default xArm6 config maps normalized device axes directly into Cartesian end-effector velocity commands in `link_base`:

```text
x/y/z -> linear x/y/z
roll/pitch/yaw -> angular x/y/z
button 0 -> open gripper while held
button 1 -> close gripper while held
```

The SpaceMouse gripper command is expressed as a discrete operator intent:

```text
gripper_intent = open
gripper_intent = close
gripper_intent = hold
```

Button presses do not make the SpaceMouse layer maintain an absolute gripper
target or velocity ramp. Each backend/controller decides what target the intent
means for its hardware or simulator. The MuJoCo backend maps `open` and `close`
to immediate endpoint targets and leaves `hold` at the current controller target.

Defaults in `configs/spacemouse_xarm6.json`:

```text
deadzone: 0.08
filter_alpha: 0.35
linear_scale_mps: 0.12
angular_scale_radps: 0.8
max_linear_speed_mps: 0.2
max_angular_speed_radps: 1.0
timeout_s: 0.25
```

## Quick Checks

Mock mode verifies the command pipeline without a SpaceMouse or device dependencies:

```bash
cd /Users/song-siqi/Projects/spacemouse_teleop
UV_CACHE_DIR=.uv-cache uv run --no-project python scripts/spacemouse_probe.py --backend mock --duration 3
UV_CACHE_DIR=.uv-cache uv run --no-project python scripts/spacemouse_debug.py --backend mock --duration 3 --log logs/mock_debug.jsonl
```

For the real device on macOS, install the native HID library first:

```bash
brew install hidapi
```

Then create a local uv environment with the hardware Python dependencies and run:

```bash
cd /Users/song-siqi/Projects/spacemouse_teleop
UV_CACHE_DIR=.uv-cache uv venv
source .venv/bin/activate
UV_CACHE_DIR=.uv-cache uv pip install -e '.[hardware]'
python scripts/spacemouse_diagnose.py --skip-open
python scripts/spacemouse_diagnose.py --max-index 3
python scripts/spacemouse_probe.py --backend pyspacemouse --axis-convention ros --hz 60 --print-rate 10
python scripts/spacemouse_debug.py --backend pyspacemouse --axis-convention ros --hz 60 --config configs/spacemouse_xarm6.json --print-rate 10 --log logs/spacemouse_debug.jsonl
```

The reader will automatically try the usual Homebrew paths. For a custom install location:

```bash
export SPACEMOUSE_HIDAPI_DYLIB=/path/to/libhidapi.dylib
```

On Apple Silicon Macs, if `SpaceMouseCompact found` appears but opening still fails, try the patched `easyhid` package:

```bash
UV_CACHE_DIR=.uv-cache uv pip install --force-reinstall git+https://github.com/bglopez/python-easyhid.git
```

The real-device reader accepts the same pyspacemouse selection knobs:

```bash
python scripts/spacemouse_probe.py --backend pyspacemouse --device SpaceMouseCompact --device-index 0 --axis-convention ros
```

## MuJoCo Prototype

Install the simulator dependencies:

```bash
cd /Users/song-siqi/Projects/spacemouse_teleop
source .venv/bin/activate
UV_CACHE_DIR=.uv-cache uv pip install -e '.[sim,hardware]'
```

Headless smoke test with mock SpaceMouse input:

```bash
SPACEMOUSE_TELEOP_BACKEND=mock SPACEMOUSE_TELEOP_VIEWER=0 ./scripts/run_mujoco_spacemouse.sh --duration 5
```

Real SpaceMouse teleop with the MuJoCo viewer:

```bash
./scripts/run_mujoco_spacemouse.sh
```

The launcher uses `.venv/bin/mjpython` automatically when opening the MuJoCo
viewer on macOS. It still forwards extra CLI flags, so one-off runs can override
the defaults:

```bash
SPACEMOUSE_TELEOP_BACKEND=mock SPACEMOUSE_TELEOP_VIEWER=0 ./scripts/run_mujoco_spacemouse.sh --duration 2
SPACEMOUSE_TELEOP_CAMERA=overview SPACEMOUSE_TELEOP_MULTIVIEW=0 ./scripts/run_mujoco_spacemouse.sh
```

I did not find a published UFACTORY xArm6 MJCF in the official GitHub organization. The official public simulation support is ROS/Gazebo in `xarm_ros2` and PyBullet/panda-gym in `uf-gym`; the curated MuJoCo Menagerie currently has xArm7 MJCF, but not xArm6.

For xArm6, this repo generates a local MuJoCo scene from the official xArm6 xacro/URDF source and then adds only the MuJoCo-specific control/scene pieces:

```text
official xArm6 xacro/URDF
  -> MuJoCo compiler
  -> .generated/mujoco/xarm6_table_cube.xarm_gripper.official_derived.xml
  -> local TCP eef site, joint position actuators, gripper, table, cube, cameras
```

The "official xArm6 xacro/URDF" input is the repo-local MuJoCo asset subset
under `src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2`, not the ignored
reference clone under `third_party`.

This matters because loading URDF directly into MuJoCo produces robot joints but no actuator controls. The generated tabletop scene has a fixed xArm6, six arm `joint*_pos` position actuators, a `gripper_pos` actuator, an `eef` site, and a free cube body for teleop tests.
It also includes fixed cameras named `rear_side`, `overview`, `front`, `side`, and `top`.
With `--multiview`, the MuJoCo viewer overlays live thumbnails from multiple
cameras, defaulting to `front,side,top` while the main viewer uses `--camera`.

By default the generated model now attaches the official UFACTORY xArm Gripper
with `add_gripper=true`. The MuJoCo adapter maps normalized closedness to the
official gripper actuator target:

```text
gripper_pos_target_rad = 0.85 * gripper
```

The observed `drive_joint` may stop before the target when fingertip contacts
block the gripper, which is the desired behavior for cube interaction.

For MuJoCo teleop, the `eef` site is attached to the gripper base near the
midpoint between the fingertip pads. The official `link_tcp` body is still kept
in the imported model, but the tabletop cube task uses the grasp-center site for
control so the visible marker does not sit below the fingers near the table. To
compare against the previous bare-arm scene, pass:

```bash
python scripts/mujoco_teleop.py --backend mock --end-effector none --list-cameras
```

For cube interaction, the generated gripper keeps the official mesh for visual
appearance. The base and knuckle mesh collisions are disabled to avoid
outer-shell popping, but the visible distal finger collision meshes are active
as `left_finger_mesh_collision` and `right_finger_mesh_collision`. Two
transparent fingertip pad boxes named `left_finger_pad_collision` and
`right_finger_pad_collision` remain as high-friction contact helpers, and the
non-drive gripper joints are tied to `drive_joint` with MuJoCo equality
constraints. Finger and pad contacts are masked to interact with the cube but
not the tabletop, which keeps low grasp attempts from being blocked by the
finger shell touching the table first. The `eef` marker is hidden in the viewer
and placed at the grasp center so it does not contact or visually occlude the
cube.

The generated scene uses friction values adapted from the tuned
`MingqianW/embodied-ai-xarm` MuJoCo task scene: table `1.0 0.01 0.001`, cube
`1.2 0.01 0.001`, distal finger meshes `1.2 0.01 0.001`, and fingertip pads
`2.0 0.02 0.002`. It keeps our stiffer tabletop contact solver
(`timestep=0.001`, Newton solver, extra no-slip iterations, stiff table/cube
`solref`/`solimp`) to reduce visible tabletop penetration. Gripper contacts use
`condim=3`, so the pinch relies on sliding friction without extra
torsional/rolling friction that can make the cube feel glued to the fingers.

In kinematic arm mode, MuJoCo receives interpolated joint positions plus the
matching joint velocities. This lets contact friction see that the gripper is
moving upward, which is required for lifting a pinched cube instead of simply
sliding past it.

Probe gripper/cube contact without the SpaceMouse:

```bash
python scripts/mujoco_gripper_contact_probe.py
python scripts/mujoco_gripper_lift_probe.py
python scripts/mujoco_gripper_press_probe.py
```

The generated arm and gripper bodies use MuJoCo `gravcomp=1`, so a zero-command hold test behaves like a powered xArm servo stack instead of an unpowered arm sagging under gravity.

Check model hold behavior separately from SpaceMouse input:

```bash
python scripts/mujoco_diagnose.py --force-regenerate --duration 5
```

Check command response separately from SpaceMouse input:

```bash
python scripts/mujoco_response_probe.py --target-mode velocity --move-duration 0.5 --hold-duration 1.0
python scripts/mujoco_response_probe.py --target-mode integrated --arm-control-mode actuator --move-duration 0.5 --hold-duration 1.0
```

List available viewer cameras:

```bash
python scripts/mujoco_teleop.py --backend mock --list-cameras
```

Choose the thumbnail cameras or layout:

```bash
SPACEMOUSE_TELEOP_MULTIVIEW_CAMERAS=overview,front,side,top SPACEMOUSE_TELEOP_MULTIVIEW_LAYOUT=grid ./scripts/run_mujoco_spacemouse.sh
SPACEMOUSE_TELEOP_MULTIVIEW_CAMERAS=front,side,top SPACEMOUSE_TELEOP_MULTIVIEW_LAYOUT=column ./scripts/run_mujoco_spacemouse.sh
```

The MuJoCo teleop default uses `--target-mode velocity`, which applies each
delta command from the observed end-effector pose. This matches MoveIt Servo's
live twist-command behavior better than accumulating a long-lived pose target.
The default `--arm-control-mode kinematic` executes IK joint targets directly for
responsive teleop; `--arm-control-mode actuator` keeps the softer MuJoCo
position-actuator dynamics for experiments that need them.
MuJoCo also exposes `--position-gain` and `--orientation-gain` so translation and
rotation can be weighted independently in the IK controller.

## Current Command Contract

`TeleopCommand` is the canonical command emitted by the SpaceMouse layer:

```text
linear_vel_mps:  vx, vy, vz
angular_vel_radps: wx, wy, wz
delta_pos_m: linear velocity integrated over dt
delta_rot_rad: angular velocity integrated over dt
gripper: optional legacy/backend-internal target closedness
gripper_intent: open, close, or hold
delta_gripper: optional legacy/backend-internal closedness delta
gripper_velocity: optional legacy/backend-internal closedness velocity
enabled: whether the command should be executed
frame: command frame, default link_base
```

`gripper` is optional and treated as a legacy/backend-internal absolute target.
`delta_gripper` and `gripper_velocity` are also retained for compatibility. The
live SpaceMouse path uses `gripper_intent` so the wrapper reports operator intent
instead of doing controller state integration. The recorder intentionally logs
raw input, filtered command, and backend observations so dataset action encoding
can stay independent from the SpaceMouse device.

For the real xArm ROS2 backend, the backend should turn `open` or `close` into
its controller target, then map that target to official xArm Gripper commands:

```text
/xarm_gripper/gripper_action position = 0.86 * gripper
/xarm/set_gripper_position pos = 850 * (1 - gripper)
```

The service position is reversed because the xArm API defines `0` pulse as
closed and `850` pulse as fully open. Keeping this conversion inside the backend
lets logs and dataset actions stay backend-neutral.
