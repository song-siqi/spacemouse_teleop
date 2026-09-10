import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from spacemouse_teleop.cli.xarm_frequency import (
    analyze_directory, build_result, camera_overlap, change_evidence,
    compare_control_logs, main, measurement_fingerprint, profile, query_pair_summary,
    summarize, summarize_control_log,
)


def sample(kind, sequence, timestamp, value=0., valid=True):
    values = {'joints': {'joints_rad': [value]*6}, 'tcp': {'tcp_mm_rad': [value]*6},
              'gripper': {'position_pulse': value, 'received_at': timestamp, 'target_sequence': sequence},
              'report': {'joints_rad': [value]*6, 'tcp_mm_rad': [value]*6, 'received_at': timestamp, 'sequence': sequence}}[kind]
    return {'kind':kind,'sequence':sequence,'received_at':timestamp,'query_started_at':timestamp-.001,
            'query_finished_at':timestamp,'values':values,'valid':valid}


def control(timestamp, sequence, position, gripper_time=None, speed=1.):
    return {'raw':{'timestamp':timestamp}, 'execution':{'send_started_at':timestamp+.001,
            'ack_received_at':timestamp+.002,'speeds_mm_s_rad_s':[speed,0,0,0,0,0],'return_code':0},
            'feedback':{'sequence':sequence,'received_at':10.+sequence*.2,'joints_rad':[position]*6,
                        'tcp_mm_rad':[position]*6,'error_code':0},
            'gripper':{'received_at':gripper_time if gripper_time is not None else 10.,
                       'position_pulse':100,'target_sequence':1,'error_code':0}}


def write_rows(path, rows):
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


class CollectionFrequencyTest(unittest.TestCase):
    def test_measurement_fingerprint_ignores_timestamps_sequence_targets_and_status(self):
        for kind in ('report','gripper','joints','tcp'):
            rows=[sample(kind,1,10.),sample(kind,2,10.05)]
            rows[1]['values'].update(controller_target_pulse=850,status_code=2,mode=5)
            self.assertEqual(measurement_fingerprint(rows[0]),measurement_fingerprint(rows[1]))
            self.assertEqual(change_evidence(rows,kind)['value_changes'],0)

    def test_small_numeric_noise_is_not_claimed_as_internal_refresh_proof(self):
        rows=[sample('joints',i,10.+i*.05,1e-8*i) for i in range(10)]
        result=build_result(rows,10.,10.5,20.,0.)
        self.assertEqual(result['sources']['joints']['value_changes'],9)
        self.assertEqual(result['sources']['joints']['material_change_count'],0)
        self.assertFalse(result['sources']['joints']['nontrivial_motion_span_observed'])
        self.assertEqual(result['device_update_conclusion'],'insufficient_evidence')
        self.assertIsNone(result['device_update_rate_hz'])

    def test_static_values_do_not_reject_response_rate(self):
        rows=[sample(kind,i,10.+i*.05) for i in range(1200) for kind in ('joints','tcp','gripper')]
        result=build_result(rows,10.,70.,20.,20.)
        self.assertTrue(result['supports_response_rate'])
        self.assertTrue(result['acceptance_duration_met'])
        self.assertEqual(result['device_update_conclusion'],'insufficient_evidence')
        self.assertEqual(result['sources']['gripper']['value_changes'],0)

    def test_material_change_intervals_use_measurements_only(self):
        rows=[sample('gripper',i,10.+i*.05,100+10*(i//2)) for i in range(8)]
        evidence=change_evidence(rows,'gripper')
        self.assertEqual(evidence['value_changes'],3)
        self.assertEqual(evidence['material_change_count'],3)
        self.assertAlmostEqual(evidence['value_change_interval_s']['median'],.1)
        self.assertTrue(evidence['nontrivial_motion_span_observed'])

    def test_pair_rtt_skew_and_multiregister_gripper_span(self):
        joints=sample('joints',1,10.01)
        joints['query_started_at']=10.
        tcp=sample('tcp',1,10.03)
        tcp['query_started_at']=10.02
        gripper=sample('gripper',1,10.05)
        gripper['query_started_at']=10.01
        result=query_pair_summary([joints,tcp,gripper],10.,11.)
        self.assertEqual(result['matched_pairs'],1)
        self.assertAlmostEqual(result['joint_tcp_pair_rtt_s']['median'],.03)
        self.assertAlmostEqual(result['joint_tcp_receive_skew_s']['median'],.02)
        self.assertAlmostEqual(result['gripper_status_query_span_s']['median'],.04)
        self.assertIn('not atomic',result['gripper_timing_note'])

    def test_camera_comparison_uses_only_real_overlap_and_marks_short_run(self):
        samples=[sample('joints',i,float(i)) for i in range(120)]
        diagnostic={'measured_at_monotonic':80.,'duration_s':70.}
        result=camera_overlap(samples,70.,115.,diagnostic)
        self.assertEqual(result['overlap_interval'],[70.,80.])
        self.assertEqual(result['overlap_duration_s'],10.)
        self.assertFalse(result['acceptance_duration_met'])
        self.assertEqual(result['sources_during_overlap']['joints']['responses'],10)
        no_overlap=camera_overlap(samples,90.,115.,diagnostic)
        self.assertIsNone(no_overlap['overlap_interval'])

    def test_future_tail_is_preserved_but_excluded_from_phase_counts(self):
        rows=[sample('joints',0,9.99),sample('joints',1,10.02),sample('joints',2,10.12)]
        result=build_result(rows,10.,10.1,20.,0.)
        self.assertEqual(result['sources']['joints']['responses'],1)
        self.assertEqual(result['outside_window_samples'],2)
        self.assertEqual(result['sources']['joints']['grid_missing_source_count'],0)

    def test_control_logs_deduplicate_feedback_and_gripper_by_their_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'control.jsonl'
            rows=[control(10.+i/60,1 if i<6 else 2,0.,10. if i<3 else 10.05) for i in range(12)]
            write_rows(path,rows)
            result=summarize_control_log(path,10.,10.4)
            self.assertEqual(result['control_rows'],12)
            self.assertAlmostEqual(result['command_hz'],60.)
            self.assertAlmostEqual(result['command_ack_rtt_s']['median'],.001)
            self.assertEqual(result['deduplicated_sources']['report']['responses'],1) # second report is exactly window end
            self.assertEqual(result['deduplicated_sources']['gripper']['responses'],2)
            self.assertEqual(result['deduplicated_sources']['gripper']['value_changes'],0)
            comparison=compare_control_logs(path,path,10.,10.4)
            self.assertTrue(comparison['comparison_available'])
            self.assertAlmostEqual(comparison['difference_s']['command_ack_rtt_s']['median'],0.)

    def test_motion_matched_control_adds_evidence_but_not_internal_timing_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'control.jsonl'
            write_rows(path,[control(10.+i/60,1,i*.01) for i in range(30)])
            rows=[sample(kind,i,10.+i*.05,i*.02) for i in range(10) for kind in ('joints','tcp')]
            result=build_result(rows,10.,10.5,20.,0.,control_log=path)
            self.assertIn('internal_timing_unverified',result['device_update_conclusion'])
            self.assertIsNone(result['device_update_rate_hz'])

    def test_offline_reanalysis_preserves_raw_files_and_old_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'old'
            root.mkdir()
            rows=[{'event':'profile_start','timestamp':10.,'poll_hz':20.,'gripper_hz':20.},
                  sample('gripper',1,10.01),sample('gripper',2,10.06)]
            write_rows(root/'phase_0.jsonl',rows)
            (root/'summary.json').write_text(json.dumps([{'duration_s':.1,'failures':[]}]))
            before={p.name:p.read_bytes() for p in root.iterdir()}
            result=analyze_directory(root)
            self.assertFalse(result[0]['historical_tail_drain_verified'])
            self.assertEqual(result[0]['sources']['gripper']['value_changes'],0)
            with patch('spacemouse_teleop.cli.xarm_frequency.ReportReceiver',side_effect=AssertionError('hardware accessed')):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(['--analyze',str(root),'--output',str(Path(directory)/'new')]),0)
            self.assertEqual(before,{p.name:p.read_bytes() for p in root.iterdir()})

    def test_profile_drains_producer_close_tail_before_file_close(self):
        class Clock:
            now=10.
            def monotonic(self): return self.now
            def sleep(self,duration): self.now+=duration
        clock=Clock()
        class Feedback:
            sequence=1
            received_at=10.
            def to_dict(self): return sample('report',1,10.)['values']
        class Reports:
            def __init__(self,*a): pass
            def start(self): pass
            def wait_first(self,*a): return Feedback()
            def snapshot(self): return Feedback()
            def close(self): pass
        class Poller:
            def __init__(self,ip,publish,error,**kw): self.publish=publish
            def start(self): self.publish(sample('joints',1,10.001))
            def close(self): self.publish(sample('tcp',1,clock.now+.001))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'phase.jsonl'
            with patch('spacemouse_teleop.cli.xarm_frequency.time',clock), patch('spacemouse_teleop.cli.xarm_frequency.ReportReceiver',Reports), patch('spacemouse_teleop.cli.xarm_frequency.StatePoller',Poller):
                result=profile('unused',.05,.01,20.,0.,path)
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(any(row.get('kind')=='tcp' for row in rows))
            self.assertEqual(rows[-1]['event'],'profile_end')
            self.assertEqual(result['sources']['tcp']['responses'],0)
            self.assertFalse(result['response_failures'])

    def test_failed_response_returns_nonzero_cli_and_short_duration_is_labelled(self):
        with tempfile.TemporaryDirectory() as directory:
            result=build_result([sample('joints',1,10.01,valid=False)],10.,10.1,20.,0.)
            with patch('spacemouse_teleop.cli.xarm_frequency.profile',return_value=result):
                output=io.StringIO()
                with contextlib.redirect_stdout(output):
                    status=main(['--duration','.1','--rates','20','--output',str(Path(directory)/'output')])
            self.assertEqual(status,1)
            self.assertIn('not an acceptance run',output.getvalue())
            self.assertTrue(result['response_failures'])
            late_failure=build_result([sample('joints',1,10.2,valid=False)],10.,10.1,20.,0.)
            self.assertTrue(late_failure['response_failures'])  # Tail failures also fail the diagnostic.


if __name__=='__main__':
    unittest.main()
