# Refactor Acceptance Status

Implementation baseline: `origin/dev/sim/mujoco` at `09ec8d8`, fast-forwarded to
`origin/dev/real/xarm6` at `4b63902`. Work is on `dev/sim/mujoco`.

## Implemented

- Package/directory migration, lazy unified entry, engine-independent contracts.
- Explicit six/seven-joint bindings and structured FR3/Robotiq attachment.
- Dynamics-only generic backend with bounded targets, per-joint limits,
  redundant posture preference, measured-position gripper hold, accumulated time.
- Source-native raw v4 writer/checker, binary evidence, independent encoding,
  session-clock pre/post buffers, epoch reset, keep/discard and SDK source adapter.
- R1 input readiness gating and R2 feedback-limited velocity leases.
- Historical regression code and v1/v2/v3 readers retained.

## Offline Evidence (2026-10-02)

The initial full run found missing OpenCV and SDK test dependencies. After
installing those, the historical video hash test exposed disagreement between
the OpenCV-bundled decoder and system FFmpeg. Indexed reads now use the same
FFmpeg decoder as archival validation; the original hash assertion passes.

The full offline suite passes 296 tests. Both robots pass scripted
lift/hold/rotation and box contact regressions at 30/60/120 Hz, renamed cylinder
contacts at 60 Hz, and a 0.1 rad tilted top press at 60 Hz. The tests include four
seconds of blocked top-press targeting followed by retreat. Zero shell contacts
never count as a successful top-press test. An AST regression prohibits joint
pose/velocity assignments outside reset in the public dynamics backend.

Final 60 Hz measurements, using per-physics-step contact peak sampling:

| Metric | xArm6 | FR3 / 2F85 |
| --- | ---: | ---: |
| Object lift | 113.52 mm | 119.69 mm |
| Hold relative slip | 0.0041 mm | 0.0006 mm |
| Rotation relative drift | 0.210 degrees | 0.114 degrees |
| Peak individual pad normal force | 23.51 N | 23.23 N |
| Straight top-press shell penetration | 0.208 mm | 0.148 mm |
| Side-push displacement | 150.94 mm | 83.19 mm |
| Side-push shell penetration | 0.068 mm | 1.280 mm |
| Additional displacement during retreat | 0.160 mm | 2.031 mm |

Both side-push cases finish without shell contact after retreat. The FR3 side
trajectory also has pad contacts, explicitly reported in the diagnostics; it is
not evidence of a perfectly isolated shell-only push. These are simulation
measurements, not real gripper calibration.

Sampling only at the input/control rate initially missed an 80.04 N Robotiq
closing transient. Diagnostics now sample every physics step; normalized target
slewing resolves the transient without relaxing force limits. The xArm dynamics
shell uses a firmer contact solver response for tilted impacts; the legacy
kinematic configuration is unchanged. No arm pose rollback is used.

The installed wheel was loaded outside the repository with socket connections
disabled and a fresh generated-model cache. Both the six- and seven-joint models
compiled and advanced successfully. The README installation command was executed
and its resolved dependencies are recorded in `uv.lock`.

Generated evidence lives under `.generated/validation/` and is not committed as
source. It includes logs, raw v4 examples, and offscreen views. Native rendering
on macOS requires access to the OS graphics service.

Evidence paths relative to the repository:

- `.generated/validation/{xarm,fr3}/`: grasp JSONL, summary and two rendered views.
- `.generated/validation/{xarm,fr3}-collision/`: collision summary.
- `.generated/validation/xarm-episode-final/`: six-joint, zero-camera raw v4.
- `.generated/validation/fr3-rgb-final/`: seven-joint raw v4, three cameras with
  independent 60/20/15 Hz rates, 30 Hz control, real FFmpeg encoding.
- `.generated/validation/real-mock/` and `zero-camera/`: SDK-source mock captures.

Tests cover zero/one/three cameras, variable joint counts, consecutive episodes,
pre/post buffers, epoch reset, queue overflow, encoder/disk faults, duplicate/gap
samples, binary corruption, source-clock reversal, and historical v1-v3 reading.
The R1/R2 regressions use fake input/transport only. Hardware, SpaceMouse and
RealSense tests are explicitly excluded and still require a site revisit.

## Limits

- SDK dispatch currently retains the specialized real runtime/config lifecycle;
  it does not yet consume every experiment section through the generic sim runner.
- The legacy kinematic environment remains an internal regression fixture with
  its historical cube observation. The public dynamics state has no cube fields.
- Keyboard, scripted and SpaceMouse adapters are available through the generic
  sim entry. The SpaceMouse adapter has not been exercised with a physical device.
- Setup assembly currently supports the registered tabletop/ideal-mount setups;
  arbitrary calibrated mounting transforms and payloads are not implemented.
- The generic raw v4 checker checks integrity and source ordering, not physical
  clock synchronization or robot protocol semantics. Original xArm packet bytes
  and native fields are retained for offline inspection.
