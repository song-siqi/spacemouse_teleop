# xArm6 native RGB/state collection (raw v3)

This entry point composes the existing SpaceMouse/xArm6/official-gripper teleop with independent native recording. Motion, reset and gripper behavior remain owned by `backends/xarm_sdk`. The collection environment needs no simulator or training packages. GelSight remains outside this capture.

## Environment and configuration

```bash
uv pip install --python .venv/bin/python -e '.[hardware,xarm,collection]' -c requirements/collection-validated.txt
```

FFmpeg and ffprobe with libx264 must be available before startup. Preflight runs before hardware motion initialization; missing tools fail explicitly. The implementation supports the installed FFmpeg 4.2.7. There is no PNG fallback.

Use `configs/spacemouse_xarm6_collection.json`, schema version 2:

| Setting | Default / meaning |
| --- | --- |
| `robot_config` | Existing independent `spacemouse_xarm6_demo.json`; unchanged motion settings |
| Cameras | RGB 640×480, requested 60 fps per camera |
| Wrist serial | `215122252880` |
| Fixed 1 serial | `311322303615` |
| Fixed 2 serial | `311322303806` |
| `robot_state_source` | `tcp30000`, passive native reports with device timestamps |
| `pre_roll_s / post_roll_s` | 0.5 / 0.5 seconds |
| `source_timeout_s` | 1 second without a valid required-source response |
| `clock_interval_s` | 1 second between host clock brackets |
| `queues.video / states / session` | 64 per camera / 4096 / 4096 accepted items |
| `min_free_bytes` | 10 GiB reserve |
| `visual_storage` | native_video, H.264 CRF 18, veryfast, two threads per camera, GOP 30 |

Each camera's `fps` configures capture and ordinal video PTS; it does not define a training sample rate. Read-only and live recording tests measured approximately 59.65/59.65/59.76 fps; 60 is the requested mode, not a promise of exact 1/60-second sampling. Use per-frame source times. GOP 30 remains unchanged (about 0.5 seconds at the configured rate). Three frame counts may differ. State reports are recorded whenever returned, with no polling-rate enforcement. Live gripper feedback retains its existing worker cadence; observe-only requests it at 20 Hz independently.

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
.venv/bin/spacemouse-collection-check /absolute/path/to/episodes/episode_000001 --require-kept
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

See [the full raw v3 contract](collection_format.md). There are three MP4 videos, three frame JSONL indexes, native robot JSONL plus exact binary packets, independent gripper/auxiliary JSONL, and session control/events/clock evidence.

Video PTS is an ordinal locator, not real trajectory time. Use original camera timestamps/domains, frame metadata, robot device timestamps, host receive times and repeated host clock brackets as evidence. Global-time metadata does not establish hardware sync, and the robot's unknown epoch/latency is not corrected.

The collection entry does not create aligned observations or training actions.
Offline alignment and data processing are planned within this repository, with
separate configuration, commands and derived output. The raw reader/checker
already exists; the full processing pipeline and its CLI are not implemented yet.
20 Hz observations, quality rules and output formats will be specified separately;
delta, gripper mapping and action definitions remain later decisions. Keep the
entire session and all raw files unchanged.

## Failure and legacy data

Queue overflow, encoder/write failure or a stopped required source creates an incomplete recording and preserves available files. Individual frame gaps or invalid responses are diagnosed without a training-quality filter. Complete means storage validation passed; it is separate from kept/discarded and from synchronization quality. Interrupted fragmented video can retain readable fragments but never receives an automatic complete marker.

Old v1 PNG and v2 video data remain readable/checkable with their original semantics. Existing v2 archives with normal-capture evidence can be recovered:

```bash
.venv/bin/spacemouse-collection-finalize /absolute/path/to/v2/episode_000001
```

This command does not migrate v1, repair capture gaps or finalize raw v3. See [legacy format semantics](collection_format_v1_v2.md).

## Acceptance scope

See [raw v3 validation](collection_raw_v3_validation.md) for this implementation's tests and recordings. Historical [query-frequency](collection_frequency_validation.md), [motion](collection_motion_validation.md) and [v2 video](collection_video_field_validation.md) experiments remain evidence for their original workloads. The [timing report](collection_timing_report.md) preserves the earlier experiments and now identifies their historical scope.

The high-rate raw recording milestone is complete as of **2026-09-11**. Existing
211-test non-simulator results and the 60-second read-only baseline are retained.
Two live sessions added three complete/kept episodes (32,403 frames), including
two consecutive recordings. Full video, frame index, native packet and checksum
checks passed with no gaps or invalid states in the saved intervals. Control sends
remained about 60 Hz during recording. See the dated validation report for exact
windows and counts.

The live episodes plus control records from their capture windows used about
**89–90 MB per captured minute** (decimal MB); full-session waiting and reset logs
are additional. The older static read-only result was 74.06 MB/minute. Neither is
a universal capacity guarantee. No temporary PNGs are written; retain the 10 GiB
reserve and measure longer/new scenes before budgeting a batch.

Finalization is outside the selected demonstration interval. Previous 146/172 ms
send gaps and camera frame-number gaps occurred after capture had closed; the
latest centered-knob trial had no gaps and a 27.29 ms maximum send interval during
finalization. These observations do not change raw integrity checks or prove a
runtime fix. Finalization scheduling, N/X coverage in this configuration and
long-duration stress tests remain separate follow-ups, not blockers for the
completed raw recording milestone.

Device measurement timing and cross-device clock calibration remain unresolved;
acquisition integrity is not synchronized observation quality. The next planned
stage is offline data processing in this repository: establish clock mappings,
select frames/interpolate states, report alignment quality and write separate
versioned outputs. This documentation update adds no processing implementation.
See [C6 in the progress plan](../PROGRESS_PLAN.md#c6--本仓库离线数据处理计划中尚未实现).
