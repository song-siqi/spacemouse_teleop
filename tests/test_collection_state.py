import time
import unittest

from robot_teleop.backends.real.xarm_sdk.state import StatePoller, ObservedChannel
from robot_teleop.cli.xarm_frequency import summarize


class Channel:
    def query(self, name):
        return [0.] * (7 if name == 'get_joint_pos' else 6)

    def gripper_status(self):
        return {'position_pulse': 300., 'received_at': time.monotonic()}


class CollectionStateTests(unittest.TestCase):
    def test_query_units_and_readonly_constructor(self):
        opened, samples = [], []
        class Fake(Channel):
            def __init__(self, ip, timeout, writable):
                opened.append(writable)
            def close(self):
                pass
        poller = StatePoller('fake', samples.append, self.fail, channel_factory=Fake)
        poller.start()
        time.sleep(.07)
        poller.close()
        self.assertEqual(opened, [False])
        self.assertEqual(samples[0]['values']['joints_rad'], [0.]*6)
        self.assertEqual(samples[1]['values']['tcp_mm_rad'], [0.]*6)
        self.assertLessEqual(samples[0]['query_started_at'], samples[0]['received_at'])

    def test_invalid_sample_published_as_invalid(self):
        samples, errors = [], []
        class Bad(Channel):
            def query(self, name):
                return [float('nan')]*7
        poller = StatePoller('fake', samples.append, errors.append)
        poller._query(Bad(), 'joints')
        self.assertFalse(samples[0]['valid'])
        self.assertEqual(len(errors), 1)

    def test_observer_does_not_change_call_on_record_failure(self):
        def fail(record):
            raise OSError('disk full')
        observed = ObservedChannel(Channel(), fail, lambda: 1)
        self.assertEqual(observed.gripper_status()['position_pulse'], 300.)

    def test_failed_gripper_query_keeps_interval_and_original_exception(self):
        samples = []
        failure = TimeoutError("gripper timeout")
        class Bad(Channel):
            def gripper_status(self): raise failure
        observed = ObservedChannel(Bad(), samples.append, lambda: 1)
        with self.assertRaises(TimeoutError) as caught:
            observed.gripper_status()
        self.assertIs(caught.exception, failure)
        self.assertFalse(samples[0]["valid"])
        self.assertLessEqual(samples[0]["query_started_at"], samples[0]["query_finished_at"])
        self.assertEqual(samples[0]["query_finished_at"], samples[0]["received_at"])

    def test_equal_values_are_new_responses_but_not_motion_evidence(self):
        rows = [{'kind':'joints','sequence':i,'valid':True,'values':{'joints_rad':[0.]*6},
                 'received_at':i*.05, 'query_started_at':i*.05-.001,'query_finished_at':i*.05}
                for i in range(21)]
        result = summarize(rows, 0., 1.)['joints']
        self.assertAlmostEqual(result['response_hz'], 20.)
        self.assertEqual(result['value_changes'], 0)


if __name__ == '__main__':
    unittest.main()
