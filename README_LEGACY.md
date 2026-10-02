# Historical SpaceMouse Teleop Documentation

This document describes the pre-refactor code and historical hardware evidence.
Use README.md for current package names, entry points, and acceptance status.

The basic SpaceMouse -> `TeleopCommand` -> real xArm6 + official gripper demo
has been verified on hardware, including automatic gripper opening before reset.
Full tool commissioning and fault-stop validation remain open. See
[PROGRESS_PLAN.md](PROGRESS_PLAN.md) for current evidence, agreed behavior and
next-stage discussion. The existing MuJoCo pipeline remains separate; additional
simulation checks are paused at the user's request.

The collection route now records **raw v3 native streams**: three independently
encoded 640×480 RGB videos (requested **60 fps per camera**) and per-frame indexes,
every passive TCP 30000 robot report
with its original bytes, independent gripper/auxiliary feedback, and recurring
host clock observations. It creates no paired observations or training actions.
The existing SDK teleop backend is unchanged.

**The high-rate raw recording milestone is complete as of 2026-09-11.**
The existing **211 non-simulator tests passed**; two live sessions then produced
**three complete/kept episodes with 32,403 frames**, including two consecutive
recordings in one session. Full video, index, original-packet and checksum checks
passed, with no gaps or invalid states in the saved ranges. Cameras measured
**59.65/59.65/59.76 fps**, robot reports **246.56–246.72 Hz**, and gripper feedback
about **19.97 Hz**; arm sends remained about **60 Hz during recording**.
Raw episodes plus control records from their capture windows used about
**89–90 MB per captured minute** (decimal MB; session waiting logs are extra).

The intended demonstration interval is between the two Space presses, recorded
as `start_time/end_time`. Raw pre/post-roll remains available. Finalization is
outside that interval; its occasional scheduling stalls are tracked as a later
runtime improvement and do not block this recording milestone. Clock calibration
and aligned observations are planned for a separate offline processing stage
**within this repository**; that stage is not implemented yet.
See [raw v3 validation](docs/collection_raw_v3_validation.md),
[the collection guide](docs/xarm_collection_setup.md) and
[repository/data boundaries](docs/repository_maintenance.md).

The [timestamp and latency report](docs/collection_timing_report.md)
is historical evidence, including a separate static report rate of 246.5 Hz.
This is not a verified measurement refresh rate. Raw v3 preserves timing evidence;
device sampling events and cross-device clock calibration remain unresolved.

This repo is organized around one reusable SpaceMouse command layer and separate execution backends for the real xArm6 stack and MuJoCo.

```text
SpaceMouse device
  -> src/spacemouse_teleop/spacemouse
  -> TeleopCommand
  -> backends/xarm_sdk     real xArm6 (basic hardware demo verified)
  -> backends/xarm6_camera composition: existing SDK + three RGB views + raw episodes
  -> backends/mujoco       existing simulator
  -> backends/xarm_ros2    future ROS2 adapter/reference
```

Top-level layout:

```text
configs/                         independent real/demo/simulation configs
scripts/                         direct debug entrypoints
src/spacemouse_teleop/spacemouse SpaceMouse reader, mapping, filters, command contract
src/spacemouse_teleop/backends   execution adapters, one subdirectory per backend
src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2
                                 repo-local xArm6/gripper simulation asset subset
src/spacemouse_teleop/sensors    independent RealSense RGB acquisition
src/spacemouse_teleop/recording  control logs, native streams, clock evidence and integrity checks
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

For the ROS2 reference path, the xArm ROS2 controller updates at `150 Hz` and the hardware backend ultimately sends joint position commands through the SDK call `set_servo_angle_j`. The MuJoCo backend follows the same high-level split: SpaceMouse emits Cartesian EE commands, while the backend converts them into joint position targets.

## Real xArm6 on Ubuntu (SDK)

The basic real-robot demo is working. On 2026-09-08 the user verified the full
sequence: automatic gripper opening, joint reset, SpaceMouse teleop, gripper
control, and normal exit. That dated demo run recorded 3405 arm commands over 56.73 s
at approximately 60 Hz, with no reported arm/gripper errors. The dated 2026-09-08
review passed 97 relevant offline tests. Later raw v3 runs observed brief simultaneous
arm/gripper activity; broader combined-motion testing, tool calibration and physical
fault-stop measurements remain open. See
[the progress plan](PROGRESS_PLAN.md) for the next discussion and
[the validation record](docs/xarm_sdk_validation.md) for dated evidence.

### Run the verified demo

Use the project environment and the independent demo configuration:

```bash
cd /home/siqis3/workspace/spacemouse_teleop
./scripts/run_xarm_demo.sh               # default: 60 s after reset, including neutral wait
./scripts/run_xarm_demo.sh --duration 0  # no session time limit; Ctrl-C exits
```

Run one command at a time. Every live reset opens the gripper first, releasing
anything held from the previous trajectory, then returns the arm to its initial
joints. The startup sequence is:

1. Connect and check robot status, tool configuration, and input health.
2. Open the gripper and confirm its measured position, target and stopped state.
3. Reset the arm to `[0, -60, -30, 0, 90, 0]` degrees and confirm it has settled.
4. Wait for the SpaceMouse knob to be centered and both buttons released, then
   start teleop automatically, without an extra enable button.

If the device starts completely idle with no HID report, briefly move and release
the knob once. During teleop, centering the knob immediately clears the arm
velocity command. Hold button 0 to open the gripper; hold button 1 to close it.
Release or hold both buttons to hold its current position. The same held direction
sends its endpoint once; release or reversal replaces that target without waiting
for the old endpoint. Exit attempts a current-position hold and does not
intentionally open the gripper. The next reset is what opens it automatically.

Each run keeps a separate JSONL log, and `logs/xarm_demo_latest.jsonl` points to
the latest run. Full operating details are in [the demo guide](docs/xarm_demo_setup.md).

### Current demo parameters

| Setting | Value |
| --- | --- |
| Robot / input | xArm6 at `192.168.1.209`; SpaceMouse Compact, ROS axis convention |
| Arm translation / rotation limits | 30 mm/s / 0.15 rad/s |
| Arm acceleration limits | 50 mm/s² / 0.2 rad/s² |
| Additional software TCP position box | Disabled explicitly in the demo |
| Gripper endpoints / device speed | Open 850, close 230 pulse; SDK speed parameter 1000 |
| Startup gripper opening | Timeout 10 s; tolerance 15 pulse; stopped for 0.2 s |
| Joint reset | 0.1 rad/s, 0.2 rad/s²; tolerance 0.005 rad; settled for 0.3 s; timeout 120 s |
| Arm / HID / gripper polling | 60 / 250 / 20 Hz |
| Input / arm feedback timeout | 0.25 s |
| Maximum arm velocity command duration | 0.2 s |
| Demo session duration | 60 s by default; `--duration 0` removes only this session limit |

The opening, joint-reset and teleop timers are separate: the 120 s reset timer
starts after opening is confirmed; the teleop timer starts after joint reset.
An opening failure prevents joint motion. A reset failure prevents teleop.

The tactile sensor on the fingers prevents full closure to 0. The demo's close
limit of 230 is provisional and still needs calibration to the installation;
it is not a force limit. The `speed_pulse_s` field is passed unchanged to the
SDK speed register; despite the legacy field name, SDK 1.18.4 documents its speed
argument in r/min. Speed 1000 has now been used in real runs.

The demo uses TCP offset `[0,0,0,0,0,0]` and payload/center of mass `[0.82,0,0,0]`
from the trial setup. Zero TCP offset means the commanded point is the flange,
not the tactile contact surface. Formal tool calibration remains unfinished.
The demo has no additional software TCP box; speed, feedback and tool checks
still apply. The reset does not perform scene collision-path planning.

### Configuration ownership

| Entry point | Configuration |
| --- | --- |
| `scripts/run_xarm_demo.sh` | `configs/spacemouse_xarm6_demo.json` |
| SDK teleop / diagnosis default | `configs/spacemouse_xarm6_real.json` |
| `scripts/run_xarm_collection.sh` | `configs/spacemouse_xarm6_collection.json`, referencing the complete demo robot config |
| MuJoCo shell launcher | `configs/spacemouse_xarm6_mujoco.json` |
| Direct MuJoCo CLI / generic debug default | `configs/spacemouse_xarm6.json` |

Each real JSON declares its own `teleop`, `reader`, `robot`, `gripper` and `site`
sections. It does not inherit or merge another config or read the simulation
launcher's `SPACEMOUSE_TELEOP_CONFIG`. `--config` selects a complete real config;
`--ip` overrides its robot IP. Simulation loaders reject the nested real config.
The existing two simulation defaults are preserved. Default paths are resolved
relative to the editable checkout, independently of the terminal directory.

The formal real template still has unconfirmed site settings set to `null` and
rejects live startup before opening devices. Before using it, fill in the actual
TCP offset, payload and center of mass, initial joints, speed/acceleration limits,
and workspace bounds (unless explicitly disabled). Confirm gripper model,
baudrate, speed and endpoints too. Read-only diagnosis and dry-run work with
unconfigured site fields. Matching controller readback does not calibrate a tool.

### Environment, diagnosis and dry-run

This machine uses Ubuntu 20.04.6 and project Python 3.12.14, with independent
hardware/SDK and simulation environments. Install the real stack with:

```bash
sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
uv venv --python 3.12  # only when .venv does not exist
uv pip install --python .venv/bin/python -e '.[hardware,xarm]' -c requirements/xarm-validated.txt
```

The validated combination includes PySpaceMouse 2.1.0 and xArm SDK 1.18.4. Existing
udev rules on this machine have been verified after replug; do not run teleop
with sudo. A new machine needs its own matching HID access rule. The real route
uses HID and Ethernet; ROS2, spacenavd, 3Dconnexion software and CUDA are not
prerequisites. Real installation does not require MuJoCo; simulation does not
require the SDK/HID extras.

```bash
.venv/bin/python scripts/spacemouse_diagnose.py --max-index 1
.venv/bin/spacemouse-xarm-diagnose --config configs/spacemouse_xarm6_demo.json --gripper
.venv/bin/spacemouse-xarm-teleop --config configs/spacemouse_xarm6_demo.json --dry-run --backend mock --duration 3 --log logs/xarm_preview.jsonl
```

Diagnosis uses audited read-only queries without automatic baud changes, warning
clearing or enabling. Dry-run does not connect to the robot and skips automatic
opening and joint reset. Mock input is restricted to dry-run. After formal site
commissioning, the direct live entry is:

```bash
.venv/bin/spacemouse-xarm-teleop --config configs/spacemouse_xarm6_real.json
```

### Execution, logging and remaining limits

`TeleopCommand` remains in m/s and rad/s. The SDK converts linear velocity to
mm/s and uses base-frame Cartesian velocity in Mode 5; joint reset uses Mode 4.
MuJoCo continues to consume command increments. Mode-changed state 5 is accepted
at startup when otherwise healthy, and remains a fault during active control.
Faults stop and exit; the application does not clear errors, reconnect, or resume
automatically. Arm velocity commands expire on the controller if not refreshed.
Near a feedback deadline, reset briefly waits for a newer report, logging
`homing_waiting_feedback`; the 0.25 s feedback timeout remains in effect.

The gripper uses a separate socket and worker. Its position target has no
controller-side expiry: if the process or connection fails before a hold is
accepted, the endpoint can still complete. The 2026-09-08 demo verified sequential
operation and startup; the 2026-09-11 raw recordings added brief arm-command /
gripper-moving overlap. Broader combined-motion and full physical fault-stop
validation remain open.

Asynchronous JSONL logs contain config snapshots, raw input, common commands,
actual send/acknowledgement times, robot and gripper feedback, startup stages and
exit reasons. Console `goal_seq` counts acknowledged gripper target writes;
`device_target` reads the device's target register. In the verified runs, open
850 read back as 840, while close 230 read back as 230. New writes appear in the
next feedback poll. This demo log remains a control/debug log. The independent
collection entry below adds RGB and episode recording; no tactile capture or
success/failure labels are inferred from its keep/discard controls.

### Regression environments and next work

The historical 97-test review covers startup order, gripper faults, input
freshness, configuration isolation, reset beyond 30 s with slow reports, and true
feedback timeouts. The historical collection route added alignment, storage,
coordinator, camera and export tests; the 2026-09-09 combined run passed **190**
relevant tests; the 2026-09-10 video implementation increased this to **215**.
See the progress document for the breakdown and evidence.
Reproduce the real-backend/input checks with:

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_xarm*.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_input_safety.py' -v
```

Earlier simulation baseline/regression and headless smoke checks passed, but the
latest implementation has not been rerun through the full simulator suite since
the user paused simulation work. The existing workflow remains:

```bash
uv venv .venv-sim --python .venv/bin/python  # only when absent
uv pip install --python .venv-sim/bin/python -e '.[sim]' -c requirements/sim-validated.txt
.venv-sim/bin/python -m unittest discover -s tests -v
SPACEMOUSE_TELEOP_BACKEND=mock SPACEMOUSE_TELEOP_VIEWER=0 PYTHON_BIN=.venv-sim/bin/python ./scripts/run_mujoco_spacemouse.sh --duration 5
```

SDK codec checks skip in the sim-only environment. Tests do not connect to a
physical robot. See [PROGRESS_PLAN.md](PROGRESS_PLAN.md) for remaining demo
acceptance checks. Raw v3 native recording and storage integrity have passed the
current live recording acceptance. R1/R2, formal tool commissioning, expanded
operation checks and finalization scheduling remain separate follow-ups.

## Three-camera native collection (raw v3)

Install collection dependencies in the existing project environment:

```bash
uv pip install --python .venv/bin/python -e '.[hardware,xarm,collection]' -c requirements/collection-validated.txt
```

The schema-v2 collection config binds `wrist`, `fixed_1` and `fixed_2` by serial,
requests RGB 640×480 at 60 fps per camera, and records each native TCP 30000
report plus independent gripper feedback. Each source has its own timing and
count; there is no 20 Hz save grid or joint/TCP polling fallback.

```bash
# Existing live open-gripper/reset/teleop workflow.
./scripts/run_xarm_collection.sh --task pick_place

# Real capture with read-only robot access: no activation, reset or motion.
./scripts/run_xarm_collection.sh --task readonly_check --observe-only --headless --auto-record --duration 60 --output logs/collection_checks

# No hardware: three synthetic RGB streams, ~250 Hz reports and gripper.
./scripts/run_xarm_collection.sh --task mock_check --dry-run --headless --auto-record --duration 60 --output logs/collection_mock
```

After reset, center the SpaceMouse and release both buttons to enter teleop.
In the focused preview: **Space** starts/ends capture, **K** keeps, **X** marks
discarded without deleting data, **N** prepares the next episode, and **Q** exits.
SpaceMouse buttons retain gripper control. Wait for the tail buffer, writer drain
and integrity validation before K/X/N. Recording uses 0.5 seconds of pre/post-roll;
reset and fault boundaries prevent data from crossing discontinuities.
The two Space times define the demonstration interval; buffer frames remain raw
context for the planned offline processing stage. After ending a recording, center the SpaceMouse
and wait for validation before continuing the episode workflow. Earlier runs had
scheduling stalls during finalization outside the saved data; this remains a
runtime follow-up, not a missing recording capability.

Three independent FFmpeg processes encode during capture, with no PNG staging:
H.264, CRF 18, veryfast, yuv420p, two threads per camera, no B frames, fragmented MP4.
Video PTS locates saved frames; original sampling times and metadata are in each
frame index. No frame duplication, camera matching or time correction is applied.
System FFmpeg/ffprobe/libx264 are required and checked before motion startup.

```bash
.venv/bin/spacemouse-collection-check /absolute/path/to/episodes/episode_000001 --require-kept
```

A complete episode has fully decoded videos, consistent frame indexes and packet
offsets, and validated checksums. Gaps, clock uncertainty and operator disposition
remain separate. Failures retain available files with an incomplete marker.
Copy the whole session to retain its control, event and clock evidence.

Old configs are rejected with a migration message: start from schema v2 and
explicitly configure capture, queues, buffering and source timeout. Existing v1/v2
data remain readable/checkable with unchanged semantics; normal-capture v2 archives
still support `spacemouse-collection-finalize`. This is not a v3 recovery command.

The previous exporters and collection-export CLI were removed during the raw v3
transition. A new **offline data processing stage will be implemented in this
repository**, using raw v3 as its input. Planned work includes clock interpretation,
frame selection, state interpolation, aligned observations and quality reporting;
output formats and any later training/action definitions remain to be specified.

This stage is planned, not yet implemented. It will have a separate entry point,
processing configuration and versioned output directory, preserving raw files and
the completed collection workflow. Processing dependencies will remain separate
from hardware/teleop requirements. See [the C6 plan](PROGRESS_PLAN.md#c6--本仓库离线数据处理计划中尚未实现),
[setup](docs/xarm_collection_setup.md), [raw v3 format](docs/collection_format.md)
and [acceptance evidence](docs/collection_raw_v3_validation.md).

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

The SDK real-robot entry point loads its own complete config described above.
The following are generic debug defaults in `configs/spacemouse_xarm6.json`:

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

The simulator and all automated physics checks can run without a SpaceMouse or
the hardware Python dependencies. Create a Python 3.10+ environment and install
only the simulator extra:

```bash
cd /Users/song-siqi/Projects/spacemouse_teleop
UV_CACHE_DIR=.uv-cache uv venv --python 3.12
source .venv/bin/activate
UV_CACHE_DIR=.uv-cache uv pip install -e '.[sim]'
```

Run the complete hardware-free test suite:

```bash
python -m unittest discover -s tests -v
```

These tests use synthetic `TeleopCommand` values and mocked reader objects. They
do not open HID devices or require a connected SpaceMouse. The MuJoCo coverage
includes grasp, lift, hold, rotation, top contact, lateral push and retreat,
renamed box/cylinder objects, `30/60/120 Hz`, and actuator-mode stability.

Run a headless teleop smoke test with mock SpaceMouse input:

```bash
SPACEMOUSE_TELEOP_BACKEND=mock SPACEMOUSE_TELEOP_VIEWER=0 ./scripts/run_mujoco_spacemouse.sh --duration 5
```

For real SpaceMouse input, additionally install the hardware extra, then launch
the MuJoCo viewer:

```bash
UV_CACHE_DIR=.uv-cache uv pip install -e '.[hardware]'
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

For manipulation-object interaction, the generated gripper keeps the official
mesh for visual appearance but disables its complex mesh collision. Two
transparent boxes named
`left_finger_pad_collision` and `right_finger_pad_collision` provide the
high-friction fingertip grasp surfaces. Separate low-friction primitive guards
cover the palm, outer knuckles, and finger backs so the visible shell cannot pass
through manipulated objects. Scene, pad, and guard geoms use collision bits
`1`, `2`, and `4`; a dynamic object opts into all three with `contype=1` and
`conaffinity=7`. Pads and guards use zero affinity, so they do not collide with
one another, themselves, or the tabletop. This keeps shell protection from
competing with the grasp contacts. The non-drive gripper joints remain tied to
`drive_joint` with MuJoCo equality constraints. The `eef` marker is hidden in
the viewer and placed at the grasp center so it does not contact or visually
occlude the object.

The official linkage has substantial mechanical advantage, so its MuJoCo
position actuator is intentionally force-limited to `-4 4` with `kp=20` and
`kv=3`. This keeps a held object constrained without maintaining unrealistically
large fingertip forces after the close command.

The table/object friction scale is adapted from the tuned
`MingqianW/embodied-ai-xarm` MuJoCo task scene: table `1.0 0.01 0.001` and cube
`1.2 0.01 0.001`. Local manipulation regressions use fingertip pads
`2.0 0.005 0.0005` and outer guards `0.2 0.001 0.0001`. The scene keeps our
stiffer tabletop contact solver
(`timestep=0.001`, Newton solver, extra no-slip iterations, stiff table/cube
`solref`/`solimp`) to reduce visible tabletop penetration. Pad geoms directly
use `condim=4`, `solref="0.003 1"`, and `solimp="0.95 0.99 0.001"`, so the same
grasp behavior applies to every correctly masked object without a name-specific
contact pair. Guards use `condim=3` and softer contact settings for low-friction
pushing. Their priority is higher than an opted-in object's priority so MuJoCo
uses the guard friction instead of taking the larger object coefficient. Table
contacts keep their existing settings so grasp behavior can be compared without
changing the rest of the scene at once.

In kinematic arm mode, MuJoCo receives interpolated joint positions plus the
matching joint velocities. This lets contact friction see that the gripper is
moving upward, which is required for lifting a pinched object instead of simply
sliding past it. Before accepting each interpolated pose, the backend also
checks every guard contact whose counterpart opts into the guard collision bit.
A contact with `|normal_z| >= 0.7` that would deepen penetration is held at the
last valid pose. Other contacts allow up to `2 mm` of compliant guard compression
so they can transmit a pushing force, then apply the same limit if the object
cannot keep up. Retreating motion and any motion that reduces penetration remain
available. This extra check is needed because a directly written kinematic arm
cannot otherwise be stopped by contact forces.

Probe gripper/object contact without the SpaceMouse:

```bash
python scripts/mujoco_contact_diagnostics.py --log logs/mujoco_contact_baseline.jsonl
python scripts/mujoco_gripper_contact_probe.py
python scripts/mujoco_gripper_lift_probe.py
python scripts/mujoco_gripper_press_probe.py
python scripts/mujoco_gripper_top_press_probe.py
```

`mujoco_gripper_top_press_probe.py` reproduces an open-gripper approach from
above and verifies both shell/cube and cube/table penetration limits.

`mujoco_contact_diagnostics.py` runs approach, close, lift, tool-axis rotation,
hold, outer-shell push, and retreat phases. Its per-phase report splits contact
fraction, force, torque, and penetration into left pad, right pad, guard, and
table roles. Use `--object-body` and repeatable `--object-geom` options for
models that do not call the manipulated body `cube`.

Show the otherwise hidden pad and guard primitives in the viewer:

```bash
SPACEMOUSE_TELEOP_SHOW_COLLISION_GEOMS=1 ./scripts/run_mujoco_spacemouse.sh
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
The backend additionally limits EE target rotation to `0.25 rad/s` by default;
with the default orientation gain this keeps grasped-cube rotation inside the
stable contact range. Override it with `--max-ee-angular-speed`, or set
`SPACEMOUSE_TELEOP_MAX_EE_ANGULAR_SPEED=0` in the launcher environment to
disable the limit. This is a MuJoCo controller policy: the SpaceMouse command
format and the real-robot backend remain unchanged.

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
