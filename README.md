# Robot Teleop

Single-arm teleoperation and native-stream recording across real robots and simulation.
Python package: `robot_teleop`; distribution and CLI: `robot-teleop`.
The repository name remains `spacemouse_teleop`.

## Local Simulation

No SpaceMouse, robot, or RealSense is needed for these commands:

```sh
uv sync --extra sim --extra recording --extra xarm
uv run robot-teleop diagnose --config configs/xarm_sim.json --output .generated/validation/xarm --render
uv run robot-teleop diagnose --config configs/fr3_sim.json --output .generated/validation/fr3 --render
uv run robot-teleop diagnose --config configs/fr3_sim.json --scenario collision --output .generated/validation/fr3-collision
uv run robot-teleop collect --config configs/fr3_sim.json --duration 3 --output .generated/fr3-episode
uv run robot-teleop check .generated/fr3-episode
uv run python -m unittest discover -s tests -v
```

Each collection output directory must be new; existing recordings are never overwritten.
MuJoCo is pinned to 3.12.0. Rendering requires a working OS graphics context.
`uv.lock` records the tested dependency resolution. Use `configs/fr3_sim_rgb.json`
for three independently sampled cameras (60/20/15 Hz with 30 Hz control).
SDK tests use fake transports, not networked hardware. The SDK creates its own
log directory under the user's home; sandboxed tests can set `HOME` to a temporary directory.

Interactive local simulation on macOS:

```sh
uv run mjpython -m robot_teleop.cli.main teleop --config configs/fr3_sim.json --input keyboard --viewer --duration 300
```

Keyboard: W/S = X, A/D = Y, R/F = Z; I/K, J/L, U/O = rotation;
`[` opens, `]` closes, space stops, Q exits. Motion expires when key repeat stops.
Other platforms can use `python` instead of `mjpython`.

## Support Matrix

| Robot / Tool | Backend | Current Validation |
| --- | --- | --- |
| xArm6 / official gripper | MuJoCo dynamics | Grasp/hold/rotation and shell contact at 30/60/120 Hz |
| FR3 / Robotiq 2F85 | MuJoCo dynamics | Grasp/hold/rotation and shell contact at 30/60/120 Hz |
| xArm6 / official gripper | xArm SDK | Migrated lifecycle, translation, pose move; fake-SDK regression |
| xArm6 / official gripper | MuJoCo kinematic | Legacy debug environment and regression tests only |

Offline regression: **296 tests passed**. Collision cases include renamed box and
cylinder objects, top press, lateral push, retreat, and a tilted tool. Diagnostics
sample contact peaks at every physics step. This is a bounded regression suite,
not a guarantee for arbitrary objects or approach speeds. See
[migration and acceptance status](docs/refactor_status.md). Historical real-robot
evidence is not a post-refactor hardware certification. No FR3 real driver or
second simulator is provided.

## Structure

```text
src/robot_teleop/
  core/                    SI commands, state, clocks, protocol
  robots/                  engine-independent robot definitions
  end_effectors/           tool definitions
  backends/real/xarm_sdk/   transport, safety, raw report/source adapters
  backends/sim/mujoco/      bindings, assembly, dynamics, diagnostics, assets
  inputs/                  SpaceMouse mapping, keyboard, scripted input
  sensors/                 independent camera acquisition
  runtime/                 experiment and episode lifecycle
  recording/               raw v4, version dispatch, historical formats
  cli/                     unified and specialized entry implementations
```

`CartesianTwist`, `CartesianPoseTarget`, `JointPositionTarget`, and `GripperCommand`
are the public command types. Units are metres, radians, seconds, and `wxyz`
quaternions. Frames are explicit. Backends reject unsupported commands.
Simulation timestamps carry an epoch; reset increments it and invalidates leases.
The SDK uses host monotonic deadlines and retains its own safety checks.
Gripper position is normalized open=0 / closed=1, not calibrated aperture or force.
Hold captures the measured position, not the previous endpoint target.
Simulation gripper targets are slew-limited to avoid instantaneous endpoint jumps.

Experiments use JSON with six explicit sections: `robot`, `end_effector`,
`backend`, `setup`, `input`, `recording`. Each section may be an object or a path
relative to its declaring experiment file. Static bindings validate supported
robot/tool combinations. The current simulation setup is the packaged tabletop.

MuJoCo uses DLS IK, bounded targets, finite-force position actuators and gravity
compensation. Runtime dynamics does not write joint/object poses. The Robotiq
model retains upstream tendon/equality/linkage definitions. Menagerie assets are
vendored at a fixed commit with licenses and [provenance](src/robot_teleop/backends/sim/mujoco/assets/menagerie/PROVENANCE.md).
FR3 mounting is idealized, not a calibrated real adapter.
Inward pads carry high friction; their backing and external shell carry low
friction. A contact-aware target projection limits downward shell loading without
rewriting the measured pose. `--collision-debug --viewer` shows contact proxies.

## Real Robot Routes

The current xArm-specific adaptation is documented in
[the adaptation record](docs/xarm_main_adaptation.md). Demo starts at the current
pose, H opens the gripper and homes, and its default duration is unlimited.
Collection retains automatic opening/homing followed by explicit Space recording.
The production wrist/fixed_1/fixed_2 camera rates are 30/60/60 fps; tool TCP offset
is `[0,0,172,0,0,0]` mm/rad and payload/COM is
`[0.991,14.5,-3.23,61.24]` kg/mm. These settings are checked, not written to the robot.

```sh
./scripts/run_xarm_demo.sh
./scripts/run_xarm_translation.sh
./scripts/run_xarm_collection.sh --task pick_place
# Unified equivalent of the demo, including terminal H:
uv run robot-teleop teleop --backend xarm_sdk --profile demo --config configs/spacemouse_xarm6_demo.json
```


Install only the required extras: `spacemouse`, `xarm`, `realsense`, `recording`.
The `sim` extra does not install RealSense. Review site settings before any live
command. Live configuration remains deliberately explicit; no simulation defaults
are inherited by the SDK.

```sh
# Offline only: synthetic cameras and report packets, no hardware connection.
uv run robot-teleop collect --backend xarm_sdk --task mock_check --dry-run --headless --auto-record --duration 3 --output .generated/mock
# Parse and preview real commands without connecting.
uv run robot-teleop teleop --backend xarm_sdk --config configs/spacemouse_xarm6_demo.json --input mock --dry-run --duration 2
```

Base real teleop, collection and translation preserve automatic preparation and
neutral-input checks; the demo profile uses explicit terminal H for homing. `move-to-pose` retains its explicit SDK prechecks and dry-run route.
`configs/xarm_real.json` references the SDK and collection configurations. The
real command route still uses its specialized lifecycle, rather than the generic
simulation runner; this boundary is intentional and remains migration work.
Read the [historical real setup guide](docs/xarm_collection_setup.md) for site
constraints; its old package/CLI names need the mapping below.

## Recording

New sim and SDK collection writes **raw v4**. Sources declare their schema,
units, frame, clock, required status and paths. State and command streams remain
separate; video is MP4 plus frame indexes, native packets are binary plus indexed
offsets and checksums. Source timestamps and host arrival evidence are retained.
Video PTS is ordinal/fps, not a fitted physical capture timestamp.

Each new xArm episode preserves exact Space start/end, end reason and capture
ranges. It is fully checked in a bounded validation subprocess before complete,
including native xArm packet semantics. While it finalizes, source ingestion and
control logging continue; keep/discard and the next reset wait for completion.
Every invalid response is retained but cannot refresh the valid-source timeout.

The generic writer supports arbitrary joint counts and 0..N cameras. SDK camera
configuration schema 3 supports arbitrary roles; historical schema 2 retains its
three-role validation. Episode buffers use the session clock; reset splits epochs.
Bounded queues and writer faults mark captures incomplete rather than silently
dropping data. Keep/discard does not repair incomplete captures.

`check` dispatches v1/v2/v3/v4 without rewriting files. `finalize` archives eligible
historical v2 captures; it does not repair failed captures. No online alignment,
training actions, or training-format export is performed.

## Migration

| Previous | Current |
| --- | --- |
| `spacemouse_teleop` | `robot_teleop` |
| `spacemouse-mujoco-teleop` | `robot-teleop teleop --config configs/xarm_sim.json` |
| MuJoCo lift/contact probes | `robot-teleop diagnose --config ...` |
| `spacemouse-xarm-teleop` | `robot-teleop teleop --backend xarm_sdk` |
| translation script | `robot-teleop teleop --backend xarm_sdk --profile translation` |
| `spacemouse-xarm-collect` | `robot-teleop collect --backend xarm_sdk` |
| collection checker/finalizer | `robot-teleop check` / `robot-teleop finalize` |
| pose-movement script | `robot-teleop move-to-pose` |

Historical operational detail remains in [README_LEGACY.md](README_LEGACY.md).
Its hardware measurements describe the pre-refactor code, not this branch's
current hardware validation status.

## Offline Processing and Replay

The raw-to-LeRobot v3 pipeline supports configurable fps and joint/TCP absolute or delta actions.
Use the isolated processing environment and `scripts/run_processing.sh`; see
[processing, data semantics and replay instructions](docs/offline_processing.md).
One xArm joint-delta episode has completed hardware replay at 1x after first-state preparation; TCP replay and broader scenarios remain unvalidated.

Replay supports explicit `--prepare-start` to move slowly to the episode first joint state before starting its clock.
Gripper preparation requires a separate `--prepare-gripper`; see the bounded hardware validation record in the processing guide.
