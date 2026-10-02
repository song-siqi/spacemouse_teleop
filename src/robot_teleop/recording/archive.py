"""Post-episode video archival. Only caller-owned episode files are modified."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import shutil
import threading
import time
import uuid

from .visual import VIDEO_SETTINGS, VideoError, VideoRunner, file_sha256, video_preflight, validate_video

ROLES = ('wrist', 'fixed_1', 'fixed_2')


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, allow_nan=False, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
        sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(descriptor)
    finally: os.close(descriptor)


def archive_timeout(meta):
    duration = float(meta['end_time'])-float(meta['start_time'])
    if not math.isfinite(duration) or duration < 0:
        raise VideoError('Invalid completed capture duration')
    return max(120., 2*duration)


def _owned(root, relative):
    """Reject symlinks for every mutable archive path, including existing parents."""
    path = root / relative
    if not path.is_relative_to(root):
        raise VideoError('Archive path outside episode')
    for part in (path, *path.parents):
        if part == root.parent: break
        if part.is_symlink():
            raise VideoError(f'Archive path cannot be a symlink: {part}')
    return path


@contextmanager
def episode_lock(root):
    import fcntl
    path = _owned(root, '.archive.lock')
    with path.open('a') as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc: raise VideoError('Episode archive is already running') from exc
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)


def _cleanup(root, meta):
    staging = _owned(root, '_staging')
    if staging.exists():
        # Do not traverse symlinks; deletion must remain confined to this episode.
        if any(p.is_symlink() for p in staging.rglob('*')):
            raise VideoError('Staging cleanup refused a symlink')
        shutil.rmtree(staging)
        sync_directory(root)
    meta['archive']['cleanup_pending'] = False
    meta['archive'].pop('cleanup_warning', None)


def _eligible(meta, capture_ready=False):
    if not isinstance(meta, dict):
        raise VideoError('Episode metadata must be an object')
    if meta.get('format_version') != 2 or meta.get('visual_storage', {}).get('mode') != 'video_after_episode':
        raise VideoError('Finalize only supports raw v2 video episodes; existing PNG datasets are not migrated')
    already_complete = meta.get('status') == 'complete'
    if not already_complete and meta.get('capture_complete') is not True and not (
            capture_ready and meta.get('status') == 'finalizing'):
        raise VideoError('Capture did not complete normally; refusing to repair a fault/interrupted episode')
    if already_complete and meta.get('capture_complete') is not True:
        raise VideoError('Complete video lacks capture completion evidence')
    return already_complete


def finalize_episode(path, *, progress=None, runner=None, tools=None, _capture_ready=False):
    """Archive v2 or retry an intact, normally ended v2 capture; never repair capture gaps."""
    from .raw import _load_episode, _read_json
    original = Path(path).absolute()
    if original.is_symlink(): raise VideoError('Episode cannot be a symlink')
    root = original.resolve()
    if not root.is_dir(): raise VideoError('Episode directory does not exist')
    meta = _read_json(_owned(root, 'meta.json'))
    if meta.get('format_version') == 4:
        from .v4 import check_episode
        report = check_episode(root)
        if not report['valid']:
            raise VideoError('Raw v4 integrity check failed: ' + '; '.join(report['errors']))
        return meta
    # Reject unsupported/fault captures without creating a lock or changing old datasets.
    _eligible(_read_json(_owned(root, 'meta.json')), _capture_ready)
    with episode_lock(root):
        meta = _read_json(_owned(root, 'meta.json'))
        already_complete = _eligible(meta, _capture_ready)
        runner = runner or VideoRunner(time.monotonic()+archive_timeout(meta))
        started = time.monotonic()
        meta.setdefault('archive', {})
        def update(phase, role=None, frames=0):
            runner.check()
            if progress:
                progress({'phase': phase, 'role': role, 'frames': frames,
                          'total_frames': meta.get('frame_count',0),
                          'elapsed_s': time.monotonic()-started,
                          'free_bytes': shutil.disk_usage(root).free if role is None else None})
        committed = already_complete
        try:
            tools = tools or video_preflight(runner)
            if already_complete:
                update('validating')
                _load_episode(root, _archive=True, _runner=runner,
                              _progress=lambda role,n: update('validating',role,n))
            else:
                update('checking_capture')
                for filename in ('observations.jsonl','raw_states.jsonl'):
                    _owned(root,filename)
                _owned(root, '_staging/images')
                for role in ROLES:
                    _owned(root, f'_staging/images/{role}')
                _, _, rows, _ = _load_episode(root, _archive=True, _staged=True, _runner=runner)
                count, fps = len(rows), float(meta['sample_hz'])
                input_paths = [root/f'_staging/images/{role}/{i:06d}.png' for role in ROLES for i in range(count)]
                # Verify no unindexed/missing staging frames, and no symbolic source substitutions.
                for role in ROLES:
                    folder = root/f'_staging/images/{role}'
                    if {p.name for p in folder.iterdir()} != {f'{i:06d}.png' for i in range(count)}:
                        raise VideoError(f'Unexpected staging PNG list: {role}')
                for image in input_paths:
                    if image.is_symlink(): raise VideoError('Staging PNG cannot be a symlink')
                capture_hashes = {name:file_sha256(root/name,runner) for name in ('observations.jsonl','raw_states.jsonl')}
                if meta.get('capture_sha256') and meta['capture_sha256'] != capture_hashes:
                    raise VideoError('Capture JSON changed since normal completion')
                png_bytes = sum(p.stat().st_size for p in input_paths)
                meta.update(capture_complete=True, capture_sha256=capture_hashes, status='finalizing')
                meta['visual_storage'] = {**VIDEO_SETTINGS, 'fps':fps, **tools}
                meta['archive'].update(phase='encoding',error=None,cleanup_pending=True,
                                       timeout_s=archive_timeout(meta))
                atomic_json(root/'meta.json', meta)
                temp = _owned(root, '_staging/videos')
                temp.mkdir(exist_ok=True)
                update('encoding')
                def encode(role):
                    target = _owned(root, f'_staging/videos/{role}.mp4')
                    def line(value):
                        if value.startswith('frame='):
                            update('encoding',role,int(value.split('=',1)[1]))
                    runner.run([tools['ffmpeg'],'-nostdin','-y','-hide_banner','-loglevel','error',
                                '-threads','2','-framerate',str(fps),'-start_number','0','-i',
                                str(root/f'_staging/images/{role}/%06d.png'),'-frames:v',str(count),
                                '-vsync','0','-an','-c:v','libx264','-preset','medium','-crf','18',
                                '-pix_fmt','yuv420p','-g','40','-keyint_min','40','-sc_threshold','0',
                                '-threads','2','-movflags','+faststart','-progress','pipe:1',str(target)],line)
                    with target.open('rb') as stream: os.fsync(stream.fileno())
                    return target
                with ThreadPoolExecutor(max_workers=3,thread_name_prefix='collection-video-encode') as pool:
                    futures = {pool.submit(encode,role):role for role in ROLES}
                    try:
                        for future in as_completed(futures): future.result()
                    except BaseException:
                        runner.cancel.set()
                        raise
                update('validating')
                checks = {}
                for role in ROLES:
                    image = rows[0]['images'][role]
                    checks[role] = validate_video(temp/f'{role}.mp4', count, fps, image['width'], image['height'],
                                                  runner=runner,progress=lambda n,role=role:update('validating',role,n))
                if {name:file_sha256(root/name,runner) for name in capture_hashes} != capture_hashes:
                    raise VideoError('Capture JSON changed while archiving')
                final = _owned(root,'videos')
                final.mkdir(exist_ok=True)
                update('committing')
                for role in ROLES:
                    runner.check()
                    destination = _owned(root,f'videos/{role}.mp4')
                    (temp/f'{role}.mp4').replace(destination)
                sync_directory(final)
                video_bytes = sum(v['bytes'] for v in checks.values())
                meta['archive'].update(phase='complete',error=None,videos=checks,
                    elapsed_s=time.monotonic()-started,png_bytes=png_bytes,video_bytes=video_bytes,
                    temporary_visual_peak_bytes=png_bytes+video_bytes,
                    video_bytes_per_minute=video_bytes*60/(count/fps),
                    capacity_note='Visual bytes only; excludes JSON/control logs. Temporary peak is PNG + MP4.')
                meta['status'] = 'complete'
                atomic_json(root/'meta.json',meta)
                committed = True
            # Deletion is strictly after a durable complete marker. Its failure is not data corruption.
            try:
                update('cleaning')
                _cleanup(root,meta)
            except Exception as exc:
                meta['archive'].update(cleanup_pending=True,cleanup_warning=str(exc))
            atomic_json(root/'meta.json',meta)
            if progress:
                progress({'phase':'complete','frames':meta['frame_count'],'total_frames':meta['frame_count'],
                          'cleanup_pending':meta['archive']['cleanup_pending']})
            return meta
        except Exception as exc:
            if not committed:
                meta['status'] = 'incomplete'
                meta['archive'].update(phase='failed',error=str(exc),cleanup_pending=True)
                try: atomic_json(root/'meta.json',meta)
                except OSError: pass  # An uncommitted marker remains authoritative on a full disk.
            raise


class ArchiveJob:
    """One background archive, with a bounded lifetime and a small progress snapshot."""
    def __init__(self, path, meta, on_done, tools=None):
        self.path, self.on_done, self.tools = Path(path), on_done, tools
        self.runner = VideoRunner(time.monotonic()+archive_timeout(meta))
        self._lock = threading.Lock()
        self._progress = {'phase':'checking_capture','frames':0,'total_frames':meta['frame_count']}
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self._run,name='collection-video-archive',daemon=True)

    @property
    def remaining(self): return max(0.,self.runner.deadline-time.monotonic())
    def start(self): self.thread.start()
    def cancel(self): self.runner.cancel.set()
    def snapshot(self):
        with self._lock: return dict(self._progress)
    def _update(self, progress):
        with self._lock: self._progress = progress
    def _run(self):
        result, error = None, None
        try:
            result = finalize_episode(self.path, runner=self.runner, tools=self.tools,
                                      progress=self._update,_capture_ready=True)
        except Exception as exc:
            error = str(exc)
            try: result = json.loads((self.path/'meta.json').read_text())
            except (OSError,ValueError): pass
        finally:
            try: self.on_done(result,error)
            finally: self.finished.set()
