import contextlib
import io
import types
import unittest
from unittest.mock import patch
from robot_teleop.inputs.spacemouse.polling import LatestInput
from robot_teleop.backends.real.xarm_sdk.safety import TeleopFault
from robot_teleop.cli.xarm_teleop import run_loop
import test_xarm_runtime
from test_xarm_config_safety import live_config, feedback
from test_input_safety import raw


class ReadinessTests(unittest.TestCase):
    def test_none_reports_cannot_refresh_readiness_timeout(self):
        source = LatestInput(live_config(),reader=object())
        source.started_at = 1.
        source.polled_at = 1.24
        self.assertEqual(source.snapshot(1.1), (None,"hold"))
        with self.assertRaisesRegex(TeleopFault,"initial report"):
            source.snapshot(1.26)

    def test_no_preparation_before_initial_report(self):
        config = live_config()
        config.robot["homing"]["enabled"] = True
        now = [10.]
        calls = []
        backend = types.SimpleNamespace(home=lambda *args:calls.append("home"))
        source = types.SimpleNamespace(snapshot=lambda now:(None,"hold"))
        recorder = types.SimpleNamespace(write=lambda row:None)
        with patch("robot_teleop.cli.xarm_teleop.time.monotonic",lambda:now[0]), patch("robot_teleop.cli.xarm_teleop.time.sleep",lambda dt:now.__setitem__(0,now[0]+dt)), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(TeleopFault,"readiness"):
                run_loop(config,source,backend,recorder)
        self.assertEqual(calls,[])

    def test_preparation_rechecks_source(self):
        config = live_config()
        config.robot["homing"]["enabled"] = True
        states = iter(((raw(10.),"hold"),(None,"hold")))
        source = types.SimpleNamespace(snapshot=lambda now:next(states))
        backend = types.SimpleNamespace(home=lambda recorder,guard:guard())
        recorder = types.SimpleNamespace(write=lambda row:None)
        with patch("robot_teleop.cli.xarm_teleop.time.monotonic",return_value=10.), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(TeleopFault,"not ready"):
                run_loop(config,source,backend,recorder)

    def test_feedback_lease_caps_send_and_waits_near_expiry(self):
        for age in (.15,.22):
            backend, channel = test_xarm_runtime.BackendTest().make_backend()
            backend.reports.snapshot = lambda:feedback(10.-age)
            with patch("robot_teleop.backends.real.xarm_sdk.backend.time.monotonic",return_value=10.):
                sent = backend.send((1,0,0,0,0,0),10.2)
            if age == .15:
                self.assertAlmostEqual(sent["duration_s"],.05)
            else:
                self.assertTrue(sent["waiting_for_feedback"])
                self.assertEqual(channel.calls,[])
            backend.close()
