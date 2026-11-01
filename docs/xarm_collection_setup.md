# xArm6 native RGB/state collection (raw v4)

This entry point composes the existing SpaceMouse/xArm6/official-gripper teleop with independent native recording. Motion, reset and gripper behavior remain owned by `backends/xarm_sdk`. The collection environment needs no simulator or training packages. GelSight remains outside this capture.

## Environment and configuration

```bash
uv sync --frozen --extra spacemouse --extra xarm --extra realsense --extra recording
```

FFmpeg and ffprobe with libx264 must be available before startup. Preflight runs before hardware motion initialization; missing tools fail explicitly. The implementation supports the installed FFmpeg 4.2.7. There is no PNG fallback.

Use `configs/spacemouse_xarm6_collection.json`, schema version 2:

| Setting | Default / meaning |
| --- | --- |
| `robot_config` | Existing independent `spacemouse_xarm6_demo.json`; unchanged motion settings |
| Cameras | RGB 640×480; wrist 30 fps, fixed views 60 fps |
| Wrist serial | `246322303938` |
| Fixed 1 serial (side) | `151422251501` |
| Fixed 2 serial (scene) | `231622300915` |
| `robot_state_source` | `tcp30000`, passive native reports with device timestamps |
| `pre_roll_s / post_roll_s` | 0.5 / 0.5 seconds |
| `source_timeout_s` | 1 second without a valid required-source response |
| `clock_interval_s` | 1 second between host clock brackets |
| `queues.video / states / session` | 64 per camera / 4096 / 4096 accepted items |
| `min_free_bytes` | 10 GiB reserve |
| `visual_storage` | native_video, H.264 CRF 18, veryfast, two threads per camera, GOP 30 |

The camera bindings were updated on 2026-10-22 to match the replacement devices
and station roles in `new_bridge/config.json`. The wrist currently enumerates at
USB 2.1 and advertises at most 30 fps for 640×480 RGB, so its configured rate is
30 fps. To restore wrist capture at 60 fps, connect it through a working USB 3
path, verify the advertised mode and update its `fps`. Close other camera consumers
before collection; an active bridge can cause `Device or resource busy`.

Each camera's `fps` configures capture and ordinal video PTS; it does not define a training sample rate. Historical read-only and live recording tests with the previous cameras at 60 fps measured approximately 59.65/59.65/59.76 fps; these do not validate the replacement devices. Use per-frame source times. GOP 30 remains unchanged (about 1 second for the wrist and 0.5 seconds for the fixed views at the configured rates). Three frame counts may differ. State reports are recorded whenever returned, with no polling-rate enforcement. Live gripper feedback retains its existing worker cadence; observe-only requests it at 20 Hz independently.

Old schema/version or production keys `sample_hz`, `robot_poll_hz`, `quality` (age/skew pairing thresholds), `export_fps`, and old storage modes are rejected with a migration message. Start from the shipped schema-v2 file and copy camera bindings, output path and robot config reference; then explicitly set queues, buffering and timeout values. Do not just relabel an old config. Existing data retain their format/version meaning.

## Verify without moving the robot

```bash
# Synthetic input + three cameras + ~250 Hz reports + independent gripper.
./scripts/run_xarm_collection.sh --task mock_check --dry-run --headless --auto-record --duration 60 --output logs/collection_mock

# Real cameras, passive native reports and read-only gripper/identity queries.
# This mode has no HID, activation, homing or motion calls.
./scripts/run_xarm_collection.sh --task readonly_check --observe-only --headless --auto-record --duration 60 --output logs/collection_checks
```

Both finite test modes automatically start and keep one successfully completed episode. Allow the configured tail and video validation to finish. Check the printed exit status and the episode path:

```bash
.venv/bin/robot-teleop check /absolute/path/to/episodes/episode_000001 --require-kept
```

The command fully decodes three videos and verifies native source/index/binary counts, checksums and shared clock evidence. It does not create a dataset derivative.

## Live teleop and recording

```bash
./scripts/run_xarm_collection.sh --task pick_place
```

The live command retains the established preparation: open gripper and confirm, reset joints, wait for centered SpaceMouse with released buttons, then enter teleop. Briefly push/release the knob if the preview is waiting for an input report.

With preview focused:

| Key | Operation |
| --- | --- |
| Space | Start or end the current recording |
| K | Mark finalized episode kept |
| X | Mark finalized episode discarded; retain files |
| N | Prepare the next episode through the existing open/reset sequence |
| Q | Exit through the existing robot stop path |

SpaceMouse buttons still operate the gripper. The preview shows accepted/written frame counts per camera, robot-report count, state and fault information. After stop, wait through **postroll** and **finalizing** before K/X/N. Encoding already runs during capture; finalization drains queues and checks files. Teleop remains available during this phase, but prior runs showed occasional scheduling stalls. For the current workflow, center the SpaceMouse after ending capture and wait for validation; finalization scheduling is a later optimization outside the demonstration interval. A recording-integrity fault ends capture and preserves data; existing robot/input fault handling retains its original behavior.

The intended demonstration window is the interval between the two Space presses,
recorded by `meta.json.start_time/end_time` in host monotonic time. The raw files
retain the configured pre/post-roll as context; no online trimming or pairing is
added. The planned offline processor in this repository will select that interval
after resolving source clocks.
A video frame index or PTS is not a substitute for those time boundaries.

Pre-roll cannot include preparation or a preceding discontinuity. Normal end saves the tail; a fault ends immediately without collecting across recovery. Quitting an active episode leaves it incomplete. Q stops the robot before waiting for acquisition/writer cleanup.

## Interpret the artifacts

The new real entry writes raw v4. Each episode declares its sources in `meta.json`.
Camera videos and indexes are `<role>.mp4` / `<role>.jsonl`; robot evidence is
`robot.bin` / `robot.jsonl`, with `gripper.jsonl`, `report.jsonl` and `commands.jsonl`.
Each index retains native fields in `values`, alongside `source_timestamp`,
`host_received_at`, `session_time`, source/session epochs and sequence. Robot
microsecond integers, frame metadata and gripper query intervals remain unchanged.

`start_time/end_time` are the two Space times; requested and actual capture ranges,
end reason, accepted/written counts, queue peaks and lifecycle events are preserved.
Video PTS is ordinal, not an exposure timestamp. No clock fitting, latency
subtraction, aligned observations or training actions are performed.

Every episode is drained and validated before `complete`: full video decoding,
indexes/counts/PTS, checksums, xArm original bytes versus decoded values, and shared
clock/session evidence. Validation runs in an owned subprocess with a deadline of
`max(120 seconds, 2 * recording duration)` and cannot block the motion loop or the
source-event consumer. K/X/N wait for finalization; failures retain incomplete data.
After ending recording, center the knob and allow finalization to finish.

SDK feedback-limited sends can return `waiting_for_feedback`. Preview counters
separate successful sends and waits; 60 Hz loop rate does not assert 60 actual
sends per second. Native TCP 30000 capture remains independent of that control
feedback cadence. Invalid samples are saved but do not refresh source health.

Keep the whole session: the episode links to session/control/events/clock evidence.
The new reader/checker also reads existing raw v3 in place; no old recording is
migrated, rewritten or reinterpreted with the current station calibration.

## Failure and legacy data

Queue overflow, encoder/write failure or a stopped required source creates an incomplete recording and preserves available files. Individual frame gaps or invalid responses are diagnosed without a training-quality filter. Complete means storage validation passed; it is separate from kept/discarded and from synchronization quality. Interrupted fragmented video can retain readable fragments but never receives an automatic complete marker.

Old v1 PNG and v2 video data remain readable/checkable with their original semantics. Existing v2 archives with normal-capture evidence can be recovered:

```bash
.venv/bin/robot-teleop finalize /absolute/path/to/v2/episode_000001
```

This command does not migrate v1, repair capture gaps or finalize raw v3/v4. See [legacy format semantics](collection_format_v1_v2.md).

## Validation scope

See [the main-architecture adaptation record](xarm_main_adaptation.md) for current
software validation and read-only replay of historical real data. The earlier
[raw v3 validation](collection_raw_v3_validation.md) documents pre-migration
hardware behavior; it does not certify the newly adapted entry on hardware.

Current station bindings are wrist/fixed_1/fixed_2 at 30/60/60 fps. Earlier 60 fps
measurements used different camera bindings. Full input sessions and generated
replay artifacts are local data, not Git content. Replay never commands a robot
and does not constitute a new physical demonstration.

The output filesystem must retain 10 GiB free. On 2026-11-01 the current `/home`
filesystem had less than this, so production preflight correctly refused capture.
Free space or choose an adequately sized `--output` filesystem before live use;
the production reserve is not automatically lowered.

Offline alignment and training export remain planned in this repository, with
separate configuration and outputs; they are not implemented in this migration.
