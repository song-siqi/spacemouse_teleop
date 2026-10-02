import queue
import threading
import unittest
from robot_teleop.runtime.collection.preview import Preview
from robot_teleop.runtime.collection.coordinator import CollectionCoordinator

class Process:
    def __init__(self, code): self.exitcode=code
    def is_alive(self): return False
    def join(self, timeout=None): pass

class PreviewExitTests(unittest.TestCase):
    def test_normal_gui_exit_signals_even_when_key_queue_is_full(self):
        events=queue.Queue(1); events.put('toggle')
        preview=Preview(lambda:{},lambda:{},events)
        self.addCleanup(preview.close)
        preview.process=Process(0)
        preview._feed()
        self.assertTrue(preview.quit_event.is_set())

    def test_gui_crash_faults_recording_without_signalling_motion_exit(self):
        errors=[]
        preview=Preview(lambda:{},lambda:{},queue.Queue(),on_fault=errors.append)
        self.addCleanup(preview.close)
        preview.process=Process(1)
        preview._feed()
        self.assertFalse(preview.quit_event.is_set())
        self.assertEqual(len(errors),1)

    def test_exit_event_has_priority_over_prepare_and_record_requests(self):
        class Recorder:
            def status(self): return {'state':'idle'}
        coordinator=CollectionCoordinator(Recorder())
        coordinator.events.put('prepare_next')
        coordinator.events.put('toggle')
        coordinator.exit_event=threading.Event()
        coordinator.exit_event.set()
        with self.assertRaises(KeyboardInterrupt): coordinator.poll(True)

if __name__=='__main__': unittest.main()
