"""Raw v2 archival tests, using tiny real encodes and injected storage/process faults."""
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch, MagicMock

import cv2
import numpy as np

from test_collection_export import fixture, read_rows, write_json, write_rows
from test_collection_recording import state, frame, wait_until
from spacemouse_teleop.recording import archive
from spacemouse_teleop.recording.collection import CollectionRecorder, ROLES
from spacemouse_teleop.recording.export import check_episode, export_episode
from spacemouse_teleop.recording.visual import (VideoError, VideoRunner, VisualReader, file_sha256,
                                               storage_config, validate_video, video_preflight)


def video_fixture(root, count=5):
    rows, raw = fixture(root, count)
    (root / '_staging').mkdir()
    (root / 'images').rename(root / '_staging/images')
    for row in rows:
        index = row['index']
        for n, role in enumerate(ROLES):
            rgb = np.zeros((24, 32, 3), np.uint8)
            rgb[:, :, n] = (index * 7 + 40) % 256
            rgb[2:12, index % 20:index % 20 + 8, :] = 240
            path = root / f'_staging/images/{role}/{index:06d}.png'
            assert cv2.imwrite(str(path), rgb[:, :, ::-1])
            row['images'][role].update(file=f'videos/{role}.mp4', video_frame_index=index,
                                      video_pts_s=index/20, width=32, height=24)
    write_rows(root/'observations.jsonl', rows)
    meta = json.loads((root/'meta.json').read_text())
    meta.update(format_version=2, capture_complete=True, status='finalizing',
                visual_storage={'mode':'video_after_episode'}, archive={})
    write_json(root/'meta.json', meta)
    return rows, raw


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'FFmpeg integration requires system tools')
class VideoArchiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.root = self.base/'episode'
        self.rows, self.raw = video_fixture(self.root)

    def meta(self): return json.loads((self.root/'meta.json').read_text())

    def test_three_videos_preserve_all_raw_rows_and_terminal_observation(self):
        before = {p:file_sha256(self.root/p) for p in ('observations.jsonl','raw_states.jsonl')}
        meta = archive.finalize_episode(self.root)
        self.assertEqual(meta['status'], 'complete')
        self.assertTrue(meta['capture_complete'])
        self.assertFalse((self.root/'_staging').exists())
        self.assertEqual(before, {p:file_sha256(self.root/p) for p in before})
        report = check_episode(self.root, require_kept=True)
        self.assertTrue(report['valid'],report)
        self.assertEqual((report['sample_count'],report['image_count'],report['video_file_count']), (5,15,3))
        self.assertEqual(read_rows(self.root/'observations.jsonl')[-1], self.rows[-1])
        self.assertNotIn('actions',self.rows[-1])
        for hz, expected in ((20,4),(10,2)):
            dest = self.base/f'light{hz}'
            export_episode(self.root,dest,format='light',fps=hz,fixed_view='fixed_2')
            exported=read_rows(dest/'train.jsonl')
            self.assertEqual(len(exported),expected)
            self.assertEqual(exported[-1]['actions'][:6],self.rows[-1]['joints_rad'])
            self.assertTrue((dest/exported[0]['image']).is_file())
        export_episode(self.root,self.base/'legacy',format='legacy',fps=10,fixed_view='fixed_1')
        export_episode(self.root,self.base/'derived',format='derived',fps=10,gripper_open=850,gripper_close=230)
        self.assertEqual(len(read_rows(self.base/'derived/derived.jsonl')),2)

    def test_random_index_matches_full_decode_including_gop_boundary(self):
        shutil.rmtree(self.root)
        rows,_ = video_fixture(self.root,45)
        archive.finalize_episode(self.root)
        with VisualReader(self.root) as reader:
            indices=list(range(45)); random.Random(12).shuffle(indices)
            for role in ROLES:
                checks=validate_video(self.root/f'videos/{role}.mp4',45,20,32,24,hashes=True)
                for i in indices+[0,39,40,41,44,0]:
                    image=reader.read_rgb(rows[i]['images'][role])
                    self.assertEqual(hashlib.md5(image.tobytes()).hexdigest(), checks['rgb_md5'][i])

    def test_missing_png_refuses_archive_and_preserves_other_images(self):
        (self.root/'_staging/images/wrist/000002.png').unlink()
        with self.assertRaisesRegex(ValueError,'Missing image'):
            archive.finalize_episode(self.root)
        self.assertEqual(self.meta()['status'],'incomplete')
        self.assertEqual(len(list(self.root.rglob('*.png'))),14)
        self.assertFalse(check_episode(self.root)['valid'])

    def test_index_gap_and_source_mismatch_cannot_be_repaired(self):
        for change in ('gap','source','float_index'):
            with self.subTest(change=change):
                rows=copy.deepcopy(self.rows)
                if change=='gap': rows[2]['index']=3
                elif change=='source': rows[2]['joints_rad'][0]=999
                else: rows[2]['images']['wrist']['video_frame_index']=2.0
                write_rows(self.root/'observations.jsonl',rows)
                with self.assertRaises(ValueError): archive.finalize_episode(self.root)
                self.assertNotEqual(self.meta()['status'],'complete')
                self.assertEqual(len(list(self.root.rglob('*.png'))),15)

    def test_fault_and_v1_episodes_are_never_promoted(self):
        for changes in ({'capture_complete':False,'status':'incomplete'},
                        {'format_version':1,'status':'complete'}):
            with self.subTest(changes=changes):
                meta=self.meta(); meta.update(changes); write_json(self.root/'meta.json',meta)
                before={str(p.relative_to(self.root)):file_sha256(p) for p in self.root.rglob('*') if p.is_file()}
                with self.assertRaises(ValueError): archive.finalize_episode(self.root)
                after={str(p.relative_to(self.root)):file_sha256(p) for p in self.root.rglob('*') if p.is_file()}
                self.assertEqual(before,after)
                self.assertEqual(len(list(self.root.rglob('*.png'))),15)

    def test_encode_failure_is_retryable_only_with_capture_marker(self):
        original=VideoRunner.run
        def fail(runner,args,on_line=None):
            if '-c:v' in args: raise VideoError('injected encoding failure')
            return original(runner,args,on_line)
        with patch.object(VideoRunner,'run',fail), self.assertRaisesRegex(ValueError,'encoding failure'):
            archive.finalize_episode(self.root)
        self.assertTrue(self.meta()['capture_complete'])
        self.assertEqual(self.meta()['status'],'incomplete')
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertEqual(archive.finalize_episode(self.root)['status'],'complete')

    def test_cleanup_failure_keeps_complete_data_then_retries_cleanup(self):
        with patch.object(archive.shutil,'rmtree',side_effect=OSError('cleanup denied')):
            meta=archive.finalize_episode(self.root)
        self.assertEqual(meta['status'],'complete')
        self.assertTrue(meta['archive']['cleanup_pending'])
        self.assertIn('cleanup denied',meta['archive']['cleanup_warning'])
        self.assertTrue(check_episode(self.root)['valid'])
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertFalse(archive.finalize_episode(self.root)['archive']['cleanup_pending'])
        self.assertFalse((self.root/'_staging').exists())

    def test_corrupt_final_video_prevents_cleanup_retry(self):
        with patch.object(archive.shutil,'rmtree',side_effect=OSError('denied')):
            archive.finalize_episode(self.root)
        (self.root/'videos/fixed_1.mp4').write_bytes(b'broken MP4')
        with self.assertRaises(ValueError): archive.finalize_episode(self.root)
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertFalse(check_episode(self.root)['valid'])

    def test_invalid_encoded_video_never_commits(self):
        with patch.object(archive,'validate_video',side_effect=VideoError('damaged frames')):
            with self.assertRaisesRegex(ValueError,'damaged frames'): archive.finalize_episode(self.root)
        self.assertEqual(self.meta()['status'],'incomplete')
        self.assertFalse((self.root/'videos').exists())
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)

    def test_disk_failure_at_commit_preserves_staging_and_retry_succeeds(self):
        original=archive.atomic_json
        def fail(path,meta):
            if meta['status']=='complete': raise OSError('disk full at commit')
            return original(path,meta)
        with patch.object(archive,'atomic_json',fail), self.assertRaisesRegex(OSError,'disk full'):
            archive.finalize_episode(self.root)
        self.assertEqual(self.meta()['status'],'incomplete')
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertEqual(archive.finalize_episode(self.root)['status'],'complete')

    def test_interruption_between_video_renames_is_retryable(self):
        original=Path.replace
        def interrupt(path,target):
            if str(target).endswith('/videos/fixed_1.mp4') and '/_staging/' not in str(target):
                raise KeyboardInterrupt()
            return original(path,target)
        with patch.object(Path,'replace',interrupt), self.assertRaises(KeyboardInterrupt):
            archive.finalize_episode(self.root)
        self.assertEqual(self.meta()['status'],'finalizing')
        self.assertTrue(self.meta()['capture_complete'])
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertEqual(archive.finalize_episode(self.root)['status'],'complete')

    def test_source_pts_shift_is_detected_before_rgb_rounding(self):
        archive.finalize_episode(self.root)
        shifted=self.base/'shifted.mp4'
        VideoRunner().run(['ffmpeg','-v','error','-i',str(self.root/'videos/wrist.mp4'),
                           '-c','copy','-output_ts_offset','0.01',str(shifted)])
        with self.assertRaisesRegex(ValueError,'Source video PTS'):
            validate_video(shifted,5,20,32,24)

    def test_interruption_after_commit_keeps_complete_and_retry_cleans(self):
        with patch.object(archive,'_cleanup',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt): archive.finalize_episode(self.root)
        self.assertEqual(self.meta()['status'],'complete')
        self.assertTrue(self.meta()['archive']['cleanup_pending'])
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)
        self.assertFalse(archive.finalize_episode(self.root)['archive']['cleanup_pending'])

    def test_expired_archive_preserves_staging(self):
        with self.assertRaisesRegex(ValueError,'timed out'):
            archive.finalize_episode(self.root,runner=VideoRunner(time.monotonic()-1))
        self.assertNotEqual(self.meta()['status'],'complete')
        self.assertEqual(len(list(self.root.rglob('*.png'))),15)

    def test_capture_mutation_after_failed_archive_rejected(self):
        with patch.object(archive,'validate_video',side_effect=VideoError('fault')):
            with self.assertRaises(ValueError): archive.finalize_episode(self.root)
        with (self.root/'observations.jsonl').open('a') as stream: stream.write('\n')
        with self.assertRaisesRegex(ValueError,'JSON changed'): archive.finalize_episode(self.root)

    def test_concurrent_finalize_and_symlink_staging_are_refused(self):
        with archive.episode_lock(self.root), self.assertRaisesRegex(ValueError,'already running'):
            archive.finalize_episode(self.root)
        original=self.root/'_staging/images/wrist/000000.png'
        external=self.base/'outside.png'; original.rename(external); original.symlink_to(external)
        with self.assertRaisesRegex(ValueError,'outside episode|symlink'): archive.finalize_episode(self.root)
        self.assertTrue(external.is_file())


class VideoProcessTests(unittest.TestCase):
    def test_process_timeout_reaps_child(self):
        created=[]
        popen=subprocess.Popen
        def start(*a,**kw):
            process=popen(*a,**kw); created.append(process); return process
        with patch('spacemouse_teleop.recording.visual.subprocess.Popen',start):
            with self.assertRaisesRegex(ValueError,'timed out'):
                VideoRunner(time.monotonic()+.1).run([sys.executable,'-c','import time; time.sleep(30)'])
        self.assertTrue(created)
        self.assertTrue(all(p.poll() is not None for p in created))

    def test_missing_tool_has_no_png_fallback(self):
        with patch('spacemouse_teleop.recording.visual.shutil.which',return_value=None):
            with self.assertRaisesRegex(ValueError,'fallback is disabled'): video_preflight()
        self.assertEqual(storage_config(),{'mode':'png'})
        with self.assertRaises(ValueError): storage_config({'mode':'guess'})

    def test_quit_stops_backend_before_archive_or_source_cleanup(self):
        from spacemouse_teleop.cli.xarm_collect import main
        calls=[]
        recorder=MagicMock()
        recorder.metadata={}
        recorder.status.return_value={'fatal_error':None}
        recorder.stop_episode.side_effect=lambda *a,**kw:calls.append('stop_episode')
        recorder.close.side_effect=lambda:calls.append('archive_drain')
        backend=MagicMock()
        def stop(): calls.append('robot_stop'); return []
        backend.close.side_effect=stop
        cameras=MagicMock(); cameras.latest_frames.return_value={k:None for k in ROLES}
        cameras.close.side_effect=lambda:calls.append('camera_join')
        with contextlib.ExitStack() as stack:
            for name,value in [('CollectionRecorder',recorder),('XArmCameraBackend',backend),
                               ('CameraGroup',cameras),('StatePoller',MagicMock()),
                               ('Preview',MagicMock()),('LatestInput',MagicMock())]:
                stack.enter_context(patch('spacemouse_teleop.cli.xarm_collect.'+name,return_value=value))
            stack.enter_context(patch('spacemouse_teleop.cli.xarm_collect.run_loop',side_effect=KeyboardInterrupt))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            self.assertEqual(main(['--task','quit_order']),0)
        self.assertLess(calls.index('robot_stop'),calls.index('camera_join'))
        self.assertLess(calls.index('robot_stop'),calls.index('archive_drain'))
        recorder.stop_episode.assert_called_once_with('interrupted',incomplete=True)

    def test_preflight_failure_precedes_robot_connection(self):
        from spacemouse_teleop.cli.xarm_collect import main
        with patch.object(CollectionRecorder,'preflight',side_effect=VideoError('missing encoder')), \
             patch('spacemouse_teleop.cli.xarm_collect.XArmCameraBackend') as backend, \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(['--task','preflight_test']),1)
        backend.assert_not_called()


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'Requires FFmpeg')
class VideoRecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.rec=CollectionRecorder(self.tmp.name,'video',{},min_free_bytes=0,
                                    visual_storage={'mode':'video_after_episode'})
        self.rec.open(); self.addCleanup(self.rec.close)
        self.producing=threading.Event(); self.producing.set()
        def feed():
            sequence=0
            while self.producing.is_set():
                sequence+=1; now=time.monotonic()
                for kind in ('joints','tcp','gripper','report'):
                    self.rec.publish_state(state(kind,sequence,now))
                for role in ROLES:
                    sample=frame(role,sequence,now); sample.update(rgb=np.zeros((4,4,3),np.uint8),height=4)
                    self.rec.publish_frame(sample)
                time.sleep(.02)
        self.producer=threading.Thread(target=feed); self.producer.start()
        def stop(): self.producing.clear(); self.producer.join(2)
        self.addCleanup(stop)
        wait_until(lambda:len(self.rec._frames['wrist'])>0)

    def test_slow_archive_keeps_log_writer_live_and_blocks_episode_controls(self):
        entered=threading.Event(); release=threading.Event()
        self.addCleanup(release.set)
        original=archive.finalize_episode
        def slow(*args,**kw):
            entered.set()
            if not release.wait(5): raise VideoError('test release timeout')
            return original(*args,**kw)
        with patch.object(archive,'finalize_episode',slow):
            self.assertIsNotNone(self.rec.start_episode())
            wait_until(lambda:self.rec.status()['frame_count']>=2)
            self.rec.stop_episode()
            self.assertTrue(entered.wait(2))
            self.assertEqual(self.rec.status()['state'],'finalizing')
            self.assertIsNone(self.rec.start_episode())
            self.assertFalse(self.rec.disposition('kept'))
            from spacemouse_teleop.backends.xarm6_camera.backend import XArmCameraBackend
            from spacemouse_teleop.backends.xarm6_camera.coordinator import CollectionCoordinator
            sends=[]
            motion=XArmCameraBackend(None,self.rec)
            motion.inner=types.SimpleNamespace(send=lambda value:sends.append(value))
            coordinator=CollectionCoordinator(self.rec)
            for i in range(70):
                self.assertIsNone(coordinator.poll(True))
                motion.send(i)
                self.rec.write({'motion_tick':i}); time.sleep(.017)
            self.assertEqual(sends,list(range(70)))
            text=(self.rec.session_path/'control.jsonl').read_text()
            self.assertIn('motion_tick',text)
            self.assertFalse(self.rec.status()['fatal_error'])
            self.assertEqual(self.rec.status()['dropped_events'],0)
            release.set()
            wait_until(lambda:self.rec.status()['state']=='review',10)
        self.rec.close()
        self.assertEqual(len(read_rows(self.rec.session_path/'control.jsonl')),70)
        self.assertTrue(check_episode(self.rec.status()['path'])['valid'])

    def test_archive_fault_can_be_discarded_then_new_episode_archives(self):
        with patch.object(archive,'validate_video',side_effect=VideoError('encoder validation fault')):
            self.rec.start_episode()
            wait_until(lambda:self.rec.status()['frame_count']>=2)
            self.rec.stop_episode()
            wait_until(lambda:self.rec.status()['state']=='fault',10)
        first=Path(self.rec.status()['path'])
        self.assertTrue(list(first.rglob('*.png')))
        self.assertTrue(self.rec.disposition('discarded'))
        wait_until(lambda:self.rec.status()['state']=='idle')
        self.assertIsNotNone(self.rec.start_episode())
        wait_until(lambda:self.rec.status()['frame_count']>=2)
        self.rec.stop_episode(); self.rec.close()
        self.assertTrue(self.rec.status()['complete'],self.rec.status())
        self.assertFalse(self.rec._archive_job.thread.is_alive())
        self.assertEqual(json.loads((first/'meta.json').read_text())['disposition'],'discarded')

    def test_quit_during_recording_does_not_start_archive(self):
        with patch.object(archive,'finalize_episode') as finalize:
            self.rec.start_episode()
            wait_until(lambda:self.rec.status()['frame_count']>=2)
            self.rec.close()
            finalize.assert_not_called()
        meta=json.loads((Path(self.rec.status()['path'])/'meta.json').read_text())
        self.assertEqual(meta['status'],'incomplete')
        self.assertFalse(meta['capture_complete'])
        with self.assertRaisesRegex(ValueError,'did not complete normally'):
            archive.finalize_episode(self.rec.status()['path'])


if __name__=='__main__': unittest.main()
