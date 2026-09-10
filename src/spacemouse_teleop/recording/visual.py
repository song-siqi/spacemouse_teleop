"""PNG and indexed MP4 access. Hardware and training packages are never imported."""
from __future__ import annotations
import hashlib
import json
import math
import os
import selectors
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import zlib
from fractions import Fraction
from pathlib import Path

VIDEO_SETTINGS = {"mode": "video_after_episode", "container": "mp4", "codec": "h264",
                  "encoder": "libx264", "crf": 18, "preset": "medium", "pix_fmt": "yuv420p",
                  "gop": 40, "keyint_min": 40, "scene_cut": 0, "threads_per_camera": 2}


def storage_config(value=None):
    if value is None:
        return {"mode": "png"}
    if not isinstance(value, dict) or set(value) != {"mode"} or value["mode"] not in ("png", "video_after_episode"):
        raise ValueError("visual_storage must contain mode: png or video_after_episode")
    return dict(value)


class VideoError(ValueError):
    pass


class VideoRunner:
    """Stream process output with a shared deadline/cancellation and spooled stderr."""
    def __init__(self, deadline=None, cancel=None):
        self.deadline = deadline if deadline is not None else time.monotonic() + 120
        self.cancel = cancel if cancel is not None else threading.Event()

    def check(self):
        if self.cancel.is_set():
            raise VideoError("video archive cancelled")
        if time.monotonic() >= self.deadline:
            raise VideoError("video archive timed out")

    def run(self, args, on_line=None):
        self.check()
        output = []
        output_size = 0
        with tempfile.TemporaryFile() as errors:
            try:
                process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                           stderr=errors)
            except OSError as exc:
                raise VideoError(f"Cannot start {args[0]}: {exc}") from exc
            try:
                pending = b""
                with selectors.DefaultSelector() as selector:
                    selector.register(process.stdout, selectors.EVENT_READ)
                    done = False
                    while not done:
                        self.check()
                        for key, _ in selector.select(.1):
                            chunk = os.read(key.fileobj.fileno(), 65536)
                            if not chunk:
                                done = True
                                break
                            pending += chunk
                            if len(pending) > 2 * 1024**2:
                                raise VideoError("Unexpectedly large video process output line")
                            while b"\n" in pending:
                                line, pending = pending.split(b"\n", 1)
                                text = line.decode('utf-8', errors='replace').strip()
                                if on_line:
                                    on_line(text)
                                else:
                                    output_size += len(line)
                                    if output_size > 2 * 1024**2:
                                        raise VideoError("Unexpectedly large video metadata")
                                    output.append(text)
                        # Poll even when the process has closed stdout but has not exited.
                    if pending:
                        text = pending.decode('utf-8', errors='replace').strip()
                        if on_line: on_line(text)
                        else: output.append(text)
                    while process.poll() is None:
                        self.check()
                        time.sleep(.02)
                if process.returncode:
                    errors.seek(0, os.SEEK_END)
                    errors.seek(max(0, errors.tell()-4096))
                    raise VideoError(f"{Path(args[0]).name} failed ({process.returncode}): "
                                     + errors.read().decode('utf-8', errors='replace'))
                return "\n".join(output)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                process.stdout.close()


def video_preflight(runner=None):
    runner = runner or VideoRunner(time.monotonic()+15)
    ffmpeg, ffprobe = shutil.which('ffmpeg'), shutil.which('ffprobe')
    if not ffmpeg or not ffprobe:
        raise VideoError("Video mode requires ffmpeg and ffprobe; PNG fallback is disabled")
    encoders = runner.run([ffmpeg, '-hide_banner', '-encoders'])
    if not any('libx264' in line.split() for line in encoders.splitlines()):
        raise VideoError("FFmpeg libx264 encoder is unavailable")
    return {"ffmpeg": ffmpeg, "ffprobe": ffprobe,
            "ffmpeg_version": runner.run([ffmpeg, '-version']).splitlines()[0],
            "ffprobe_version": runner.run([ffprobe, '-version']).splitlines()[0]}


def file_sha256(path, runner=None):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while chunk := stream.read(1024**2):
            if runner: runner.check()
            digest.update(chunk)
    return digest.hexdigest()


def safe_file(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("Image references must be relative paths within the episode")
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f"Missing image or path outside episode: {relative}")
    return path


def png_size(path):
    """Check chunk integrity and a complete compressed image without image dependencies."""
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"Invalid PNG: {path.name}")
    cursor, size, compressed, ended = 8, None, bytearray(), False
    while cursor + 12 <= len(data):
        length = struct.unpack_from(">I", data, cursor)[0]
        kind = data[cursor + 4:cursor + 8]
        end = cursor + 12 + length
        if end > len(data):
            raise ValueError(f"Truncated PNG: {path.name}")
        payload = data[cursor + 8:cursor + 8 + length]
        crc = struct.unpack_from(">I", data, cursor + 8 + length)[0]
        if zlib.crc32(kind + payload) & 0xffffffff != crc:
            raise ValueError(f"PNG checksum mismatch: {path.name}")
        if kind == b"IHDR":
            if size is not None or cursor != 8 or length != 13:
                raise ValueError(f"Invalid PNG header: {path.name}")
            width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", payload)
            if not width or not height or compression or filtering or interlace not in (0, 1):
                raise ValueError(f"Invalid PNG dimensions/encoding: {path.name}")
            size = (width, height)
        elif kind == b"IDAT":
            compressed.extend(payload)
        elif kind == b"IEND":
            ended = length == 0 and end == len(data)
            break
        cursor = end
    if size is None or not ended or not compressed:
        raise ValueError(f"Incomplete PNG: {path.name}")
    try:
        decoder = zlib.decompressobj()
        pixels = decoder.decompress(compressed)
        if not pixels or not decoder.eof or decoder.unused_data:
            raise ValueError(f"Incomplete PNG pixels: {path.name}")
    except zlib.error as exc:
        raise ValueError(f"Invalid PNG pixels: {path.name}") from exc
    return size



def video_reference(frame, role, index, fps):
    if frame.get('file') != f'videos/{role}.mp4':
        raise VideoError(f"Unexpected video path for {role}")
    if (isinstance(frame.get('video_frame_index'), bool) or not isinstance(frame.get('video_frame_index'), int)
            or frame.get('video_frame_index') != index):
        raise VideoError(f"Video frame index mismatch: {role} observation {index}")
    pts = frame.get('video_pts_s')
    if isinstance(pts, bool) or not isinstance(pts, (int, float)) or not math.isfinite(pts) or abs(pts-index/fps) > 1e-6:
        raise VideoError(f"Video PTS mapping mismatch: {role} observation {index}")


def validate_video(path, count, fps, width, height, *, runner=None, progress=None, hashes=False):
    runner = runner or VideoRunner(time.monotonic()+max(120, 2*count/fps))
    probe = json.loads(runner.run(['ffprobe', '-v', 'error', '-show_streams', '-of', 'json', str(path)]))
    streams = probe.get('streams', [])
    if len(streams) != 1:
        raise VideoError("Expected exactly one video stream and no audio")
    stream = streams[0]
    if (stream.get('codec_type') != 'video' or stream.get('codec_name') != 'h264'
            or stream.get('pix_fmt') != 'yuv420p'
            or (stream.get('width'), stream.get('height')) != (width, height)
            or abs(float(Fraction(stream.get('avg_frame_rate', '0/1')))-fps) > 1e-6):
        raise VideoError(f"Video stream parameters differ from observation metadata: {path}")
    # Inspect decoded source PTS before RGB conversion: the rawvideo encoder's
    # output time base could otherwise round a small timestamp error away.
    source_count = 0
    def source_line(value):
        nonlocal source_count
        if not value:
            return
        fields = dict(part.split('=', 1) for part in value.split('|') if '=' in part)
        if 'width' not in fields or 'height' not in fields:
            raise VideoError('Incomplete source frame metadata')
        try:
            pts = float(fields['best_effort_timestamp_time'])
            size = (int(fields['width']), int(fields['height']))
        except (ValueError, KeyError) as exc:
            raise VideoError('Missing or invalid source frame PTS') from exc
        if (source_count >= count or not math.isfinite(pts)
                or abs(pts-source_count/fps) > 1e-6 or size != (width, height)):
            raise VideoError(f'Source video PTS/dimensions mismatch at {source_count}: {path}')
        source_count += 1
    runner.run(['ffprobe','-v','error','-threads','2','-select_streams','v:0',
                '-show_entries','frame=best_effort_timestamp_time,width,height',
                '-of','compact=p=0:nk=0',str(path)], source_line)
    if source_count != count:
        raise VideoError(f'Source video frame count mismatch: {source_count} != {count}')
    seen, time_base, decoded_size = 0, None, None
    checksums = []
    def line(value):
        nonlocal seen, time_base, decoded_size
        if value.startswith('#tb 0:'):
            time_base = Fraction(value.split(':', 1)[1].strip())
        elif value.startswith('#dimensions 0:'):
            decoded_size = tuple(map(int, value.split(':',1)[1].strip().split('x')))
        elif value and not value.startswith('#'):
            fields = [v.strip() for v in value.split(',')]
            if len(fields) != 6 or time_base is None:
                raise VideoError("Invalid decoded frame record")
            timestamp = int(fields[2])*time_base
            if (seen >= count or abs(float(timestamp)-seen/fps) > 1e-6
                    or int(fields[4]) != width*height*3):
                raise VideoError(f"Decoded frame gap, extra frame or invalid PTS at {seen}: {path}")
            if hashes: checksums.append(fields[5])
            seen += 1
            if progress: progress(seen)
    runner.run(['ffmpeg','-nostdin','-v','error','-xerror','-threads','2','-copyts','-i',str(path),
                '-map','0:v:0','-an','-vsync','0','-pix_fmt','rgb24','-threads','2','-f','framemd5','pipe:1'], line)
    if seen != count or decoded_size != (width, height):
        raise VideoError(f"Decoded frame count/dimensions mismatch: {seen} != {count}: {path}")
    result = {"frames": seen, "width": width, "height": height, "fps": fps,
              "bytes": Path(path).stat().st_size, "sha256": file_sha256(path, runner)}
    if hashes: result['rgb_md5'] = checksums
    return result


class VisualReader:
    """Context-managed image access; arrays/decoders are only imported on use."""
    def __init__(self, root):
        self.root = Path(root).resolve()
        self._captures = {}

    def read_rgb(self, frame):
        import cv2
        path = safe_file(self.root, frame['file'])
        if 'video_frame_index' not in frame:
            bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        else:
            index = frame['video_frame_index']
            if isinstance(index, bool) or not isinstance(index, int) or index < 0:
                raise VideoError("Invalid video frame index")
            if path not in self._captures:
                options = [cv2.CAP_PROP_N_THREADS, 2] if hasattr(cv2, 'CAP_PROP_N_THREADS') else []
                capture = cv2.VideoCapture(str(path), cv2.CAP_FFMPEG, options)
                if not capture.isOpened():
                    capture.release()
                    raise VideoError(f"Could not open video: {path}")
                self._captures[path] = capture
            capture = self._captures[path]
            if int(capture.get(cv2.CAP_PROP_POS_FRAMES)) != index:
                if not capture.set(cv2.CAP_PROP_POS_FRAMES, index):
                    raise VideoError(f"Could not seek video frame {index}")
            ok, bgr = capture.read()
            if not ok: bgr = None
        if bgr is None or bgr.shape != (frame['height'], frame['width'], 3):
            raise VideoError("Decoded image missing or dimensions differ")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def export_png(self, frame, destination):
        if 'video_frame_index' not in frame:
            shutil.copyfile(safe_file(self.root, frame['file']), destination)
        else:
            import cv2
            rgb = self.read_rgb(frame)
            if not cv2.imwrite(str(destination), rgb[:,:,::-1], [cv2.IMWRITE_PNG_COMPRESSION,1]):
                raise VideoError(f"Cannot write exported PNG: {destination}")

    def close(self):
        for capture in self._captures.values(): capture.release()
        self._captures.clear()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()
