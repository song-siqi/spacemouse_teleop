# Collection raw format v3

raw v3 records independent source streams and their timing evidence. The raw capture format has no observation grid, camera pairing, interpolation, derived action or training export. An offline processing stage for these streams is planned within this repository; its derived output contract has not yet been defined or implemented. Collection configuration uses `schema_version: 2`; saved sessions and episodes use `format_version: 3`.

The old meanings remain documented in [the v1/v2 reference](collection_format_v1_v2.md). Existing data are readable/checkable; v2 archive recovery remains available. No old dataset is migrated or deleted. Historical timing evidence remains in [collection_timing_report.md](collection_timing_report.md).

## Files

```text
session_<UTC>_<id>/
  session.json
  control.jsonl
  events.jsonl
  clock_anchors.jsonl
  episodes/episode_000001/
    meta.json
    videos/{wrist,fixed_1,fixed_2}.mp4
    frames/{wrist,fixed_1,fixed_2}.jsonl
    robot_states.jsonl
    robot_reports.bin
    gripper_states.jsonl
    auxiliary_states.jsonl
```

Keep the whole session when moving an episode: its metadata links to shared evidence using `../../session.json`, `../../control.jsonl`, `../../events.jsonl` and `../../clock_anchors.jsonl`. Empty auxiliary streams are allowed. Three nonempty videos, robot reports and gripper responses are required for a complete recording. No PNG staging or `observations.jsonl` is created.

## Independent image streams

The shipped collection configuration requests **640×480 RGB at 60 fps for each of the three cameras**. This is a configurable capture rate, not a fixed-rate observation grid or a format-version change. Existing raw v3 recordings retain the rates stored in their own metadata.

Each camera has a bounded queue, writer thread and FFmpeg process. RGB frames are owned copies from the camera; publishers never wait for encoding or disk IO. Each accepted frame is written once in its stream order. Cameras may have different frame counts, rates and start/end times.

Default encoding: H.264/libx264, CRF 18, veryfast, yuv420p, two threads per camera, no B frames, GOP 30, original resolution, no resampling or padding. Fragmented MP4 uses `frag_keyframe+empty_moov+default_base_moof`. Fragmentation reduces finalization dependence, but only decoding establishes what survived an interruption. H.264 is lossy; completeness describes frame coverage. See [FFmpeg fragmentation](https://ffmpeg.org/ffmpeg-formats.html#Fragmentation).

Each frame-index row contains:

| Field | Meaning |
| --- | --- |
| `file`, `video_frame_index`, `video_pts_s` | Episode-relative MP4, zero-based display index and ordinal PTS |
| `role`, `serial`, `width`, `height`, `capture_fps` | Actual source binding and stream configuration |
| `sequence`, `frame_id` | Local capture sequence and camera-provided frame number |
| `received_at` | Host monotonic seconds immediately after the SDK frame wait returns; not exposure time |
| `device_timestamp_ms`, `timestamp_domain` | Unmodified SDK timestamp in ms and its SDK domain |
| `clock_id`, `clock_epoch` | Device identity and source clock segment |
| `frame_metadata` | Supported metadata raw integers, availability, units/event definitions and read errors |
| `camera_info` | Camera firmware/identity and actual global-time option observations |

`video_pts_s = video_frame_index / configured_camera_fps` locates a frame in the video (currently `index / 60` with the default config). It is **not the sampling timestamp**. A video with frame gaps plays those saved frames consecutively; playback duration does not measure trajectory duration. Device frame gaps and duplicates are explicit events. Repeated SDK returns of the same device frame are diagnosed without fabricating another image.

Metadata names: `frame_timestamp`, `sensor_timestamp`, `actual_exposure`, `time_of_arrival`, `backend_timestamp`, `actual_fps`. Unsupported fields contain `available: false, raw: null`, not zero. Device readout/exposure times and exposure duration use microseconds; actual FPS uses fps × 1000. Arrival/backend values retain raw integers with locally unverified units marked unknown. These definitions follow [the pinned RealSense header](https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/include/librealsense2/h/rs_frame.h); SDK global time is a software mapping into OS time, not hardware synchronization across devices.

## Robot reports and original bytes

A dedicated receive-only TCP 30000 connection publishes every complete report. It has no motion initialization or send operation. Split and coalesced reads are handled; reports completed in one receive batch retain the same `received_at` and `receive_batch`. They are never redistributed onto an invented time grid.

`robot_reports.bin` concatenates exact complete received packets, with no SDK padding. Each `robot_states.jsonl` row contains `packet_offset`, `packet_bytes`, `layout`, sequence, receive batch/time, source/clock identity, `device_timestamp_raw` (integer), `device_timestamp_unit: "us"`, decoded values, validity and invalid-field names. Unknown layouts stop capture explicitly; complete supported packets preceding an unknown packet are retained. A truncated/unknown tail is retained as hex in the source-fault event, separate from indexed complete reports.

Current decoder supports 784 and 804 bytes. The locally verified 784-byte prefix and regression packets define the parsing contract. The original timestamp is big-endian uint64 at offset 4; actual joints are little-endian floats at 116; TCP pose is at 472. Six joints are decoded; the seventh protocol slot remains in the binary. The longer layout's unverified tail is not interpreted. [Official port documentation](https://docs.supportarticle.ufactory.cc/zhHans/support_articles/developer/firmware/data-description-of-tcp-port.html) covers different firmware revisions; accepting these sizes does not establish all firmware compatibility.

`values` includes:

- `joints_rad`, joint velocity/acceleration, actual current and estimated joint torque.
- `tcp_mm_rotvec_rad`: base-frame configured TCP `[x,y,z,rx,ry,rz]`, translation in mm and a rotation vector in rad. It is **not** the old `tcp_mm_rad` RPY field.
- Actual TCP speed/acceleration and estimated TCP wrench.
- Separately named `target_*` joint/TCP fields, plus state, mode and command count.
- Raw monitor information only when `monitor_device_type != 0`; its unit verification is false. Type zero means unavailable and never supplies gripper position.

Per-field units and interpretation limits are stored in `robot_report_semantics`. Nonfinite decoded values become JSON null with `valid: false`; original bytes remain unchanged. An isolated invalid response is recorded and diagnosed, rather than removed by a training-quality filter. Prolonged absence of valid required-source responses triggers source timeout.

The historical static result was approximately 246.5 Hz; the three 2026-09-11 live episodes measured 246.56–246.72 Hz. These are report arrival frequencies, not commanded collection rates or verified measurement refresh rates. Device epoch, timestamp event, fixed clock offset and transport latency remain unknown.

## Gripper and auxiliary feedback

`gripper_states.jsonl` records each actual query completion, including actual `position_pulse`, `controller_target_pulse`, motion/status/error fields when available, sequence, source, validity and `query_started_at → query_finished_at` with host `received_at`. Failed queries keep the interval, error and invalid status. These reads can contain multiple SDK operations and do not claim an atomic hardware measurement.

Live capture observes the existing gripper worker; mock/observe-only use an independent read-only 20 Hz query worker. Pulse is the SDK raw unit, not calibrated physical opening or force. The motion and gripper backend remains unchanged.

`auxiliary_states.jsonl` contains newly returned existing control feedback: running state, mode, errors, tool parameters, etc. Its kind is `report`; native TCP 30000 rows have kind `robot`. Auxiliary feedback retains its own source, sequence and receive time. A repeated cached control snapshot is not published as new auxiliary feedback.

`control.jsonl` preserves existing session input, command, execution and lifecycle records. Control commands express intent/execution requests and are not derived training actions.

## Clock evidence

All receive/query/control times refer to host monotonic seconds in the session. The clock recorder writes an anchor at session start/end and every second by default:

```text
sequence, reason, clock_id="host_realtime", clock_epoch,
monotonic_before_ns, realtime_ns, monotonic_after_ns, read_interval_ns
```

The two monotonic reads bracket the realtime read. An observed offset change beyond the recorded 5 ms threshold opens a new host mapping epoch and emits an event, without changing source timestamps. This is a diagnostic threshold, not an accuracy bound.

Camera frame metadata pairs device-clock evidence with SDK-mapped timestamps on the same frame. Robot device time plus receive time is a one-way arrival observation. `clock_id` and `clock_epoch` identify each source segment; frame-number resets, device-time regressions and camera timestamp-domain changes emit boundary events and end an active episode immediately. Host wall-clock changes preserve monotonic recording and all anchors.

No fitted mapping, latency subtraction, fixed offset or invented uncertainty is recorded. `clock_mapping_status` is `evidence_only_not_calibrated`; unknown offset uncertainty is null. Later synchronization is planned in this repository's separate offline processing stage. Its mappings, uncertainty and processing provenance must be recorded with derived outputs while preserving the original raw evidence; that stage is not implemented yet.

## Boundaries and integrity

Space records an operation start/end. Each episode requests 0.5 s pre-roll and 0.5 s post-roll by default; actual saved ranges are stored independently for each source. Available pre-roll can be shorter during startup or following a boundary. Preparation start/end, reset, disconnect and previous capture closure prevent pre-roll from crossing discontinuities.

The agreed demonstration interval is the operator's `start_time/end_time` (the two
Space presses) on the host monotonic clock. Pre/post-roll remains in raw storage
as optional context; the planned in-repository offline processor will select the
desired interval after establishing source-clock mappings. Finalization occurs after capture closure
and is not part of that demonstration. Frame gaps observed only after capture
closure do not imply missing frames in the saved episode. Gaps inside a selected
or saved window still retain their original diagnostics; this convention does
not relax completion checks or make an interrupted episode complete.

Lifecycle: `idle → recording → postroll → finalizing → review/fault → idle`. K/X applies disposition after finalization; N can prepare the next episode only after review. Normal stop continues collecting the tail, drains accepted queues, closes encoders, decodes every video, checks counts/PTS/indexes and original packet references, fsyncs files and atomically writes a complete marker.

Faults/disconnection cut capture immediately with no tail. Queue overflow, stopped required source, encoder failure, disk failure and session-log/clock failure invalidate integrity. Acquired files remain present. A handled failure also records best-effort decoded video-prefix counts and a checksum inventory of stopped writers; those diagnostics never promote an incomplete episode. A process kill or disk-full error may leave only the initial `recording` metadata; missing `complete` always means uncommitted. Fragmentation cannot restore an unflushed frame, index or report.

`meta.json` records operation times (`start_time/end_time`), requested capture window, actual `capture_start_time/capture_end_time`, per-source ranges/counts, queue high-water marks, discontinuity summaries, errors, configuration, provenance and a size/SHA-256 manifest of the ten raw stream files. Session close writes checksums of shared control/events/clock files in `session.json`. A complete episode depends on its shared session evidence.

`complete` means all accepted data were written and validated; it does not imply no camera gaps, calibrated timing, or training suitability. `disposition` is independently pending/kept/discarded; X never deletes files, and K cannot repair an incomplete episode.

## Reading and checking

`recording/raw.py` provides read-only `check_episode()`, native `iter_source()`, and v1/v2 validation used by the archive recovery path. `recording/visual.py` provides video/frame inspection. The check command never changes the dataset:

```bash
.venv/bin/spacemouse-collection-check /absolute/path/to/episodes/episode_000001
```

Checks validate paths, source ordering, capture windows, decoded packet consistency, video dimensions/counts/PTS, indexes, manifests and clock evidence. Gaps/invalid responses are reported, not used to generate observations or actions. Use `--require-kept` when operator selection matters.

`spacemouse-collection-finalize` remains **v2-only** for archives with normal-capture evidence. It does not promote interrupted raw v3 recordings. The previous training exporters and their CLI were removed during the raw v3 transition. New offline processing is planned in this repository against raw v3; it has no implemented processing/export CLI yet. A stale editable-install command can be removed by reinstalling the project.

Configuration migration and actual acceptance limits are described in [the setup guide](xarm_collection_setup.md) and [raw v3 validation](collection_raw_v3_validation.md).
