# Legacy collection raw formats: v1 PNG and v2 indexed video

This document preserves historical storage semantics. New recording uses [raw v3](collection_format.md). Old online configuration and training export commands are no longer supported; existing data and v2 archive recovery remain supported.

证据保存约定：本页结论随仓库提交；`logs/`、`data/` 和外部绝对路径仅为历史/本机证据定位，不随 clone 提供，也不保证历史原始文件仍在本机。

`CollectionRecorder` records three required RGB views and absolute robot states.
The recorder never controls the robot. Capture and robot publishers only add data
to bounded buffers; a separate scheduler selects observations and a separate
writer uses three fixed PNG workers and writes JSON. Only one observation is
submitted to the image workers at a time; its index follows all three completed images.
In v2, those PNGs are temporary; an independent archive thread runs three FFmpeg
processes after normal capture completion. It never uses the control-log writer
for encoding or video validation. A recording fault stops the affected episode
and leaves motion handling to the existing teleop lifecycle.

## Directory and lifecycle

```text
<output>/<task_slug>/session_<UTC>_<id>/
  session.json
  control.jsonl
  episodes/episode_000001/
    meta.json
    observations.jsonl
    raw_states.jsonl
    videos/wrist.mp4
    videos/fixed_1.mp4
    videos/fixed_2.mp4
    _staging/                  # only during capture/archive, or after a failure
      images/{role}/000000.png
      videos/{role}.mp4
```

`format_version: 2` uses the structure above. Version 1 remains supported and
stores `images/{role}/000000.png` directly. Existing PNG episodes are never
automatically migrated or deleted. A persistent empty `.archive.lock` coordinates
offline and online finalizers; it is not a data stream.

`task_slug` only names the directory; metadata retains the original task text.
`session.json` contains a UTC/monotonic anchor, configuration, quality limits,
software/hardware metadata supplied by the caller, and requested sample rate.
`control.jsonl` includes the complete session, including preparation and reset.
It is separate from observations and must not be treated as measured state.

`meta.json` is written before image work begins, initially with `status: recording`.
Ending an episode drains accepted events, flushes/fsyncs/closes its data streams,
and checks references. For v1 it then commits `complete`. For v2, the state remains
`finalizing` while a separate archive task checks every staging PNG and native
reference, writes `capture_complete: true` with `capture_sha256` for both JSONL
streams, then encodes the videos. An externally callable finalizer requires that
normal-capture marker; only the recorder can initiate its initial validation.

The videos remain in `_staging/videos` until full decode, dimensions, frame counts,
PTS, continuous observation indexes and native references pass. The task renames
all three videos into `videos/`, fsyncs that directory, and atomically replaces
`meta.json` with `status: complete`. **Only after that durable marker** may staging
PNGs be removed. A crash between individual video renames leaves an uncommitted
episode and intact PNGs; retry re-encodes all three views. A cleanup failure leaves
complete videos and JSON, sets `archive.cleanup_pending`, and records a warning.
Successful archival never changes the JSONL streams.

A missing, `recording`, `finalizing`, or `incomplete` status has no successful integrity marker.
Zero observations, source faults, writer failure, robot/input faults during an
active recording and interrupted recordings cannot be repaired by video encoding.
Process termination or a full disk may leave initial metadata instead of a final
reason; absence of `complete` remains authoritative. Checks also verify recorded
JSON/video SHA-256 values, so a subsequently damaged file is not accepted merely
because metadata once said complete.

`disposition` is separately `pending`, `kept`, or `discarded`. A complete episode
requires explicit keep/discard before another starts. Keeping an incomplete
episode does not repair its integrity. Discard is a metadata operation and does
not delete videos, staging images or states. Each new episode starts its own grid and index at
zero; no derived action may cross an episode boundary.

The UI-facing states are `idle`, `recording`, `finalizing`, `review`, and `fault`.
`start_episode()` returns an id or `None`; failures appear in `status().error`.
`stop_episode()` and `disposition()` are asynchronous. Wait for `review`/`fault`
and then `idle`, respectively. Space/K/X/N cannot advance while archiving; motion
and session logging continue. One episode may archive at a time; a new recording
requires completion/failure review and explicit K/X first. `close()` runs after
motion shutdown and drains accepted writes. An active recording at shutdown becomes
incomplete; a previously normally ended episode can finish archiving. Its total
deadline is `max(120, 2*(end_time-start_time))` seconds from archive submission,
including source validation, encoding and full decode. Exit waits for the remaining
archive deadline (plus bounded child termination), separately from the existing
30-second writer drain. Timed-out FFmpeg children are terminated and reaped.

## Time and source semantics

The observation grid is `start_time + index / sample_hz`, default 20 Hz. The
scheduler selects the newest available source with `received_at <= timestamp`.
It retains history rather than using a future latest cache. A scheduler tick
that is at least one full grid period late faults the episode instead of
compressing or silently skipping time. Smaller jitter is recorded separately.

All `timestamp`, `received_at`, and query interval fields use host monotonic
seconds. A source's `received_at` means response/frame arrival, not a claim about
the device's internal measurement instant. `device_timestamp_ms` and its
`timestamp_domain` are retained independently; do not subtract them from host
time without a measured clock mapping. Camera skew is explicitly the spread of
selected **host receive times**, not proven exposure synchronization.

Source sequences are local acquisition counters. Robot state sequence is unique
per `kind` for the whole session, including repeated robot backend recreation.
Camera sequence is local per role; frame id is the device frame number. Repeated
source sequence/frame id does not refresh the cached timestamp. New observations
may reuse still-valid sources, but must retain their original ids and timestamps;
metadata reports that reuse. Static numerical values with new acquisition
sequences are allowed, and are not proof of stale device measurements.

Thresholds are explicit capture configuration: `max_camera_age_s`,
`max_robot_age_s`, `max_gripper_age_s`, `max_camera_skew_s`. Module defaults are
provisional engineering values (0.10, 0.10, 0.15, 0.05 seconds respectively), not
hardware validation results. No automatic threshold relaxation or resampling
occurs. Every selected required source must pass the configured limits.

## Native state rows

Each `raw_states.jsonl` row is a direct acquired source record:

```json
{
  "kind": "joints",
  "sequence": 125,
  "received_at": 3456.712,
  "query_started_at": 3456.710,
  "query_finished_at": 3456.712,
  "values": {"joints_rad": [0, -1, -0.5, 0, 1.5, 0]},
  "valid": true
}
```

Supported kinds and values:

| Kind | Values and meaning |
| --- | --- |
| `joints` | `joints_rad`: six absolute joint angles in joint order, radians |
| `tcp` | `tcp_mm_rad`: base-frame `[x,y,z,roll,pitch,yaw]`, mm/mm/mm/rad/rad/rad; TCP definition comes from captured robot config |
| `gripper` | Direct worker/channel status including required measured `position_pulse`; preserve available controller target, motion/state and error fields under their original names |
| `report` | Original low-frequency feedback fields; optional, with their own actual acquisition sequence and age |

Joint/TCP queries have separate intervals. A gripper status query can internally
read multiple registers and is not an atomic snapshot. The locally assigned
state `sequence` is unrelated to a gripper **command** `target_sequence`.
Unavailable optional values remain absent or explicitly unavailable; never fill
zeros. Non-finite required values invalidate recording; diagnostic raw evidence
uses JSON `null` plus `valid: false` and an error instead of invalid JSON NaNs.

Native rows are written at acquisition rate during recording, plus one causal
predecessor per available source for the first observation. This predecessor can
precede episode start and does not create an extra training step. The key
`(kind, sequence)` uniquely identifies a native row. Raw metadata retains actual
source times, so a 30 Hz poll or 5 Hz report is not relabeled as 20 Hz measurement.

## Observation rows

`observations.jsonl` contains one row per saved grid point:

| Field | Meaning |
| --- | --- |
| `index` | Consecutive integer starting at zero |
| `timestamp`, `relative_time_s` | Scheduled host grid time and time from episode start |
| `scheduler_lag_s` | Actual selection time minus scheduled time |
| `images` | Required `wrist`, `fixed_1`, `fixed_2` image records |
| `joints_rad`, `tcp_mm_rad` | Selected absolute values, with native units above |
| `gripper` | Copy of selected gripper source's values |
| `sources` | Separate native references for joints, TCP and gripper |
| `camera_skew_s` | Max minus min selected camera host receive times |
| `valid` | `true` for saved rows; a quality failure ends the episode |

Each image record contains `file` relative to its episode directory, `role`,
`serial`, `sequence`, `frame_id`, `received_at`, `device_timestamp_ms`,
`timestamp_domain`, `width`, `height`, `capture_fps`, and `age_s`. All remain
unchanged in v2. Its only image-reference changes are:

```json
{"file": "videos/wrist.mp4", "video_frame_index": 37, "video_pts_s": 1.85}
```

These fields are added to the existing camera provenance, not a replacement for
it. At observation index `k`, every role references video frame `k` (zero based)
and `video_pts_s = k / sample_hz`. Each video has exactly N frames for N observation
rows, including the final raw observation. Video duration is N/sample_hz while
the grid's first-to-last span is (N-1)/sample_hz. PTS locates an image within a
video; it does not replace host/device timestamps or imply hardware synchronization.

V1 and v2 staging PNGs preserve RGB losslessly. V2 final H.264/yuv420p video is
lossy. Resolution and selected frames are preserved, with no interpolation,
dropping or insertion to conceal gaps. Only the selected 20 Hz images are stored;
unsaved intermediate 30 fps images cannot later be recovered. Three PNG files
must finish writing before their corresponding observation row is appended.

Each `sources[kind]` contains `sequence`, `received_at`, `age_s`, and query interval
fields when present. Its key must exist in `raw_states.jsonl`, and observation
values must equal that source's raw values. No online training `state` or `actions`
vector is stored; no delta or normalized/binary gripper target overwrites raw state.

## Episode metadata and quality evidence

Metadata includes `format_version`, `episode_id`, task, sample rate, required roles,
status, disposition, start/end/reason, saved `frame_count`, config, caller metadata,
quality limits, units/frame definition, and paths to session metadata/control log.
`frame_count` counts full observations, not individual RGB files.

Statistics contain raw counts per kind, native host response frequencies and
interval distributions, source age distributions, receive-time camera skew,
scheduler lag, reused-source counts/fractions, and session queue high-water/drop
counts. Native `device_update_hz` remains null: host responses alone cannot prove
sensor update rate. Hardware frequency acceptance belongs to the separate C0/C1
measurement report.

Disk reserve checks run at pre-motion open and in the writer, which refreshes a
free-space cache even while idle and rechecks on episode begin. `start_episode()`
uses only that cache and memory; it never calls the filesystem from the motion
loop. A reserve breach discovered after begin marks the episode incomplete.

The queue reserves one slot for episode finalization. A single deferred finish
fence protects even an unexpectedly fully occupied queue without raising into
control; new enqueues are rejected until earlier accepted work drains.
On queue overflow, stale or
missing required input, invalid state, image/write failure, or disk reserve breach,
the affected episode becomes incomplete. Publishers never wait for image encoding
or disk access and never raise a recording exception into robot callbacks.
A fatal writer error prevents further episodes; transient source faults permit a
new segment only after explicit disposition and fresh-source checks. Inspect the
recording status rather than assuming motion continuation implies recording.

Offline exports must require **both** `status: complete` and `disposition: kept`,
validate references and exact grid spacing, and generate labels only within the
retained continuous episode. A default 20 Hz next-state action and an explicitly
resampled 10 Hz action are different dataset versions.

## Visual storage configuration and recovery

The collection-only field `visual_storage` accepts `{"mode":"png"}` or
`{"mode":"video_after_episode"}`. Missing configuration defaults to v1 PNG;
the shipped collection config explicitly opts into v2. It is not passed to the
SDK motion configuration. Video mode requires positive even image dimensions.
Before robot connection/reset, preflight requires local `ffmpeg`, `ffprobe`, and
`libx264`; failure exits without silently falling back to PNG. No LeRobot, PyTorch
or training packages are required in the teleop environment.

Fixed v2 settings: MP4, libx264, CRF 18, medium, yuv420p, GOP/keyint_min 40,
scene-cut keyframes disabled, two encoder threads per role, faststart. Frame rate
is the episode's `sample_hz`. Metadata records these settings, executable paths,
FFmpeg/ffprobe versions, decoded counts, dimensions, video hashes/bytes, archive
runtime, temporary PNG bytes and remaining cleanup state. Final integrity checks
use full RGB decode rather than trusting a container frame-count header.

The normal collection still requires 10 GiB available. Until archive cleanup,
budget approximately **1.4 GB of temporary PNG per minute**, plus MP4 and JSON/log
space. `temporary_visual_peak_bytes` reports PNG+MP4 logical file bytes; it excludes
JSON, control logs and filesystem allocation overhead. It is a capacity calculation,
not a high-frequency measurement of allocated disk blocks. `cleanup_pending: true`
means that space has not been confirmed released.

```bash
.venv/bin/spacemouse-collection-finalize /absolute/path/to/episode_000001
```

This retries a v2 archive with `capture_complete: true` or revalidates a completed
v2 episode before cleaning its remaining staging files. It preserves disposition,
refuses concurrent finalization, rejects v1/missing normal-completion evidence,
and does not fill gaps or resume capture. A discarded but intact episode may be
archived/cleaned; it remains discarded.

`VisualReader` supports either a PNG path or a video path plus frame index.
`spacemouse-collection-check` fully decodes v2 videos and reports `image_count`
(image frames across views) separately from `video_file_count` (three). Training exports have been removed. VisualReader remains for raw image access and v2 archive validation; no actions or paired training samples are generated.
