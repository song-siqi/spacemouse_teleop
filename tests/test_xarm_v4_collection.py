"""Actual SDK-source recorder integration without hardware, using real packet bytes."""
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from robot_teleop.backends.real.xarm_sdk.collection import XArmCollectionRecorder
from robot_teleop.backends.real.xarm_sdk.report import decode_report
from robot_teleop.recording.raw import check_episode
from robot_teleop.recording.native_writer import StreamingVideo
from robot_teleop.recording.validation import validate_before_commit


def wait(check, timeout=12):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if check():
            return
        time.sleep(.005)
    raise AssertionError('Condition did not become true')


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.now = 10.
        self.packet = Path(__file__).with_name('fixtures').joinpath('xarm30000_784.bin').read_bytes()

    def recorder(self, cameras=0, **kwargs):
        specs=[dict(role=r,serial='s-'+r,width=32,height=24,fps=f)
               for r,f in list(zip(('wrist','fixed_1','fixed_2'),(10,20,20)))[:cameras]]
        rec=XArmCollectionRecorder(self.tmp.name,'integration',{}, cameras=specs,
            pre_roll_s=.1,post_roll_s=.1,min_free_bytes=0,clock=lambda:self.now,
            video_queue_size=16,state_queue_size=128,**kwargs)
        rec.open(); self.addCleanup(rec.close)
        return rec

    def feed(self, rec, at, seq, bad_gripper=False):
        packet=bytearray(self.packet);struct.pack_into('>Q',packet,4,268858225408+seq*4000)
        sample=decode_report(packet)
        sample.update(kind='robot',source='tcp30000',sequence=seq,received_at=at,
                      clock_epoch=0,clock_id='robot_tcp30000',receive_batch=seq,packet=bytes(packet))
        rec.publish_state(sample)
        rec.publish_state(dict(kind='gripper',sequence=seq,received_at=at,clock_epoch=0,
            valid=not bad_gripper,source='existing_gripper_worker',device_sequence=None,
            query_started_at=at-.004,query_finished_at=at,
            values={} if bad_gripper else {'position_pulse':600,'controller_target_pulse':840,'motion_state':'stopped'},
            error='SDK code 1' if bad_gripper else None))
        for spec in rec.cameras:
            rec.publish_frame(dict(**{k:spec[k] for k in ('role','serial','width','height')},
                sequence=seq,frame_id=seq+100,received_at=at,device_timestamp_ms=at*1000,
                clock_epoch=0,timestamp_domain='timestamp_domain.global_time',capture_fps=spec['fps'],
                frame_metadata={'sensor_timestamp':{'available':True,'raw':123400+seq,'unit':'us'}},
                rgb=np.full((24,32,3),seq*30%256,np.uint8)))
        rec.session.flush(wait_finalization=False)

    def start(self,rec):
        self.feed(rec,self.now-.05,0)
        self.assertIsNotNone(rec.start_episode())
        wait(lambda:rec.session.active is not None and rec.session.state=='recording')

    def finish(self,rec,seq=1,reason='user'):
        self.feed(rec,self.now+.03,seq)
        self.now+=.05;rec.stop_episode(reason)
        self.feed(rec,self.now+.09,seq+1)
        self.now+=.11
        wait(lambda:rec.status()['state'] in ('review','fault'))
        return rec.session.last

    def test_boundaries_native_metadata_settings_and_consecutive_records(self):
        settings=dict(crf=23,preset='ultrafast',gop=7,threads_per_camera=1)
        seen=[]
        class Encoder(StreamingVideo):
            def __init__(self,path,spec,config):
                seen.append(dict(config));super().__init__(path,spec,config)
        rec=self.recorder(3,visual_storage=settings,video_factory=Encoder)
        self.start(rec);episode=self.finish(rec)
        self.assertTrue(check_episode(episode.path)['valid'],episode.meta)
        self.assertEqual(episode.meta['start_time'],10.)
        self.assertEqual(episode.meta['end_time'],10.05)
        self.assertEqual(episode.meta['end_reason'],'user')
        self.assertLess(episode.meta['capture_start_time'],episode.meta['start_time'])
        self.assertGreater(episode.meta['capture_end_time'],episode.meta['end_time'])
        self.assertEqual(episode.meta['queue_capacities']['wrist'],16)
        self.assertEqual(episode.meta['queue_capacities']['robot'],128)
        self.assertTrue(all(c['crf']==23 and c['gop']==7 and c['threads_per_camera']==1 for c in seen))
        frames=rows(episode.path/'wrist.jsonl')
        self.assertEqual([r['values']['frame_id'] for r in frames],[100,101,102])
        self.assertEqual([r['values']['frame_metadata']['sensor_timestamp']['raw'] for r in frames],[123400,123401,123402])
        self.assertEqual([r['video_frame_index'] for r in frames],[0,1,2])
        self.assertEqual(episode.meta['counts']['robot'],3)
        self.assertTrue(rec.disposition('kept'));rec.session.flush()
        self.assertEqual(episode.meta['disposition'],'kept')
        # A reset clears the prebuffer and increments only the session epoch.
        self.now+=.2;rec.mark_boundary('preparation_start');rec.session.flush()
        self.now+=.2;self.feed(rec,self.now-.02,3)
        self.assertIsNotNone(rec.start_episode());wait(lambda:rec.session.active is not None)
        second=self.finish(rec,4)
        self.assertEqual(second.meta['epoch'],1)
        self.assertTrue(all(r['session_time']>=10.36 for r in rows(second.path/'robot.jsonl')))
        self.assertTrue(check_episode(second.path)['valid'],second.meta)
        self.assertTrue(rec.disposition('discarded'));rec.session.flush()
        self.assertTrue((second.path/'robot.bin').exists())
        self.assertEqual(second.meta['disposition'],'discarded')

    def test_invalid_gripper_does_not_refresh_health_and_can_recover(self):
        rec=self.recorder();self.start(rec)
        self.now=11.1;self.feed(rec,self.now,1,bad_gripper=True)
        wait(lambda:rec.status()['state']=='fault')
        failed=rec.session.last
        self.assertEqual(failed.meta['status'],'incomplete')
        self.assertIn('gripper',failed.meta['end_reason'])
        self.assertFalse(check_episode(failed.path)['valid'])
        invalid=rows(failed.path/'gripper.jsonl')[-1]
        self.assertFalse(invalid['valid'])
        self.assertEqual(invalid['values']['error'],'SDK code 1')
        self.assertEqual(rec.latest['gripper'],9.95)
        self.assertTrue(rec.disposition('discarded'));rec.session.flush()
        self.now=11.2;self.feed(rec,self.now,2)
        self.assertIsNotNone(rec.start_episode());wait(lambda:rec.session.active is not None)
        successful=self.finish(rec,3)
        self.assertTrue(check_episode(successful.path)['valid'],successful.meta)

    def test_slow_validation_does_not_block_sources_or_session_log(self):
        rec=self.recorder();self.start(rec)
        entered,release=threading.Event(),threading.Event()
        from robot_teleop.recording import validation
        real=validation.validate_before_commit
        def slow(*args):
            entered.set();release.wait(5);return real(*args)
        with patch.object(validation,'validate_before_commit',side_effect=slow):
            self.now+=.05;rec.stop_episode();self.now+=.11
            self.assertTrue(entered.wait(3))
            try:
                for i in range(1,20):self.feed(rec,self.now+i*.001,i)
                rec.write({'event':'still_teleop','timestamp':self.now})
                rec.write({'execution':{'waiting_for_feedback':True},'timestamp':self.now})
                rec.log.flush();rec.session.flush(wait_finalization=False)
                self.assertTrue(any(r.get('event')=='still_teleop' for r in rows(rec.session_path/'control.jsonl')))
                self.assertEqual(rec.control_stats['waiting_for_feedback'],1)
                self.assertFalse(rec.disposition('kept'))
                self.assertIsNone(rec.start_episode())
                self.assertGreater(len(rec.session.history),0)
                self.assertEqual(rec.status()['state'],'finalizing')
            finally:release.set()
            wait(lambda:rec.status()['state']=='review')
        self.assertEqual(rec.session.last.meta['counts']['robot'],1)
        self.assertTrue(check_episode(rec.session.last.path)['valid'])

    def test_validator_failure_retains_evidence_and_never_commits(self):
        rec=self.recorder();self.start(rec)
        with patch('robot_teleop.recording.validation.validate_before_commit',return_value={'valid':False,'errors':['corrupt video']}):
            episode=self.finish(rec)
        self.assertEqual(episode.meta['status'],'incomplete')
        self.assertIn('corrupt video',' '.join(episode.meta['errors']))
        self.assertTrue((episode.path/'robot.bin').is_file())
        self.assertFalse(check_episode(episode.path)['valid'])

    def test_native_field_tamper_is_rejected_even_after_rehash(self):
        rec=self.recorder();self.start(rec);episode=self.finish(rec)
        index=episode.path/'robot.jsonl';samples=rows(index)
        samples[0]['values']['values']['joints_rad'][0]+=1
        index.write_text(''.join(json.dumps(r)+'\n' for r in samples))
        meta=episode.meta;meta['checksums']['robot.jsonl']=hashlib.sha256(index.read_bytes()).hexdigest()
        (episode.path/'meta.json').write_text(json.dumps(meta))
        report=check_episode(episode.path)
        self.assertFalse(report['valid']);self.assertIn('original packet',' '.join(report['errors']))

    def test_nonmatching_camera_is_recording_fault_not_motion_call(self):
        rec=self.recorder(1)
        rec.publish_frame(dict(role='wrist',serial='wrong',received_at=10.,rgb=np.zeros((24,32,3),np.uint8)))
        self.assertIn('identity',rec.error)
        self.assertIsNone(rec.start_episode())

    def test_source_reset_cuts_episode_and_cannot_leak_prebuffer(self):
        rec=self.recorder();self.start(rec)
        self.now=10.04
        rec.publish_event(dict(event='source_clock_reset',source='robot',received_at=self.now,boundary=True))
        wait(lambda:rec.status()['state']=='fault')
        self.assertEqual(rec.session.last.meta['status'],'incomplete')
        self.assertEqual(rec.session.last.meta['end_reason'],'source_clock_reset')
        self.assertFalse(rec.latest)

    def test_shared_session_evidence_is_checked_and_never_rewritten(self):
        rec=self.recorder();self.start(rec);episode=self.finish(rec)
        rec.close()
        self.assertTrue(check_episode(episode.path)['valid'])
        clocks=rec.session_path/'clock_anchors.jsonl'
        clocks.write_text(clocks.read_text()+'{}\n')
        self.assertFalse(check_episode(episode.path)['valid'])

    def test_log_flush_failure_drains_writers_and_cannot_complete(self):
        rec=self.recorder();self.start(rec)
        with patch.object(rec.session,'prepare_validation',side_effect=RuntimeError('disk full')):
            episode=self.finish(rec)
        self.assertEqual(episode.meta['status'],'incomplete')
        self.assertTrue(all(not t.is_alive() for t in episode.threads))
        self.assertIn('disk full',' '.join(episode.meta['errors']))

    def test_quit_while_recording_retains_incomplete_and_stops_threads(self):
        rec=self.recorder();self.start(rec)
        self.now=10.03;self.feed(rec,self.now,1)
        rec.close();episode=rec.session.last
        self.assertEqual(episode.meta['status'],'incomplete')
        self.assertFalse(check_episode(episode.path)['valid'])
        self.assertTrue(all(not t.is_alive() for t in episode.threads))
        self.assertFalse(rec.session.thread.is_alive())

    def test_encoder_failure_preserves_raw_state_and_is_disposable(self):
        class FailingEncoder:
            def __init__(self,*args):pass
            def write(self,rgb):raise OSError('encoder failed')
            def abort(self):pass
        rec=self.recorder(1,video_factory=FailingEncoder)
        self.start(rec)
        wait(lambda:rec.status()['state']=='fault')
        self.assertEqual(rec.session.last.meta['status'],'incomplete')
        self.assertTrue((rec.session.last.path/'robot.bin').is_file())
        self.assertTrue(rec.disposition('discarded'))


    def test_invalid_operation_bounds_cannot_pass_integrity_check(self):
        rec=self.recorder();self.start(rec);episode=self.finish(rec)
        meta=episode.meta
        meta['end_time']=meta['capture_requested_end_time']+1
        (episode.path/'meta.json').write_text(json.dumps(meta))
        report=check_episode(episode.path)
        self.assertFalse(report['valid'])
        self.assertIn('Operation interval',' '.join(report['errors']))



class ValidatorProcessTests(unittest.TestCase):
    def test_timeout_terminates_owned_validator_process(self):
        real=subprocess.Popen;processes=[]
        def launch(*args,**kwargs):
            process=real([sys.executable,'-c','import time; time.sleep(60)'],**kwargs)
            processes.append(process);return process
        with patch('robot_teleop.recording.validation.subprocess.Popen',side_effect=launch):
            with self.assertRaises(subprocess.TimeoutExpired):
                validate_before_commit('/unused',time.monotonic()+.15)
        self.assertIsNotNone(processes[0].poll())
