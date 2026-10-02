import json
import math
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from robot_teleop.backends.real.xarm_sdk.config import DEFAULT_CONFIG, XArmConfig
from robot_teleop.backends.real.xarm_sdk.safety import RobotFeedback, TeleopFault, VelocityLimiter, check_feedback, check_site
from robot_teleop.cli.common import load_config
from robot_teleop.cli.xarm_teleop import build_parser
from robot_teleop.inputs.spacemouse import TeleopCore
from robot_teleop.inputs.spacemouse.command import RawSpaceMouseState, TeleopCommand

ROOT = Path(__file__).resolve().parents[1]


def command(t=10., v=(.02, 0, 0), w=(0, 0, 0), **kwargs):
    return TeleopCommand(v, w, (99, 99, 99), (99, 99, 99), True, 'link_base', 1 / 60, t, **kwargs)


def feedback(t=10., **kwargs):
    value = RobotFeedback(t, 1, 2, 5, 0, 0, (0.,) * 6, (0.,) * 6, (0.,) * 6, (1., 0., 0., 0.), 6, 6, 63, 63, 'v2.5.5')
    return replace(value, **kwargs)


def live_config():
    config = XArmConfig.load()
    config.robot["homing"]["enabled"] = False
    config.site.update(expected_tcp_offset_mm_rad=[0.] * 6, expected_payload_kg_com_mm=[1., 0, 0, 0],
                       initial_joints_rad=[0.] * 6, workspace_min_m=[-.1] * 3, workspace_max_m=[.1] * 3,
                       max_linear_speed_mps=.03, max_angular_speed_radps=.15,
                       max_linear_accel_mps2=.3, max_angular_accel_radps2=1.)
    config.gripper.update(model='xarm_gripper', baudrate=2000000, speed_pulse_s=500)
    return config


class ConfigTest(unittest.TestCase):
    def test_site_fields_are_unset_and_live_is_rejected(self):
        config = XArmConfig.load()
        self.assertTrue(config.missing_site_settings())
        with self.assertRaisesRegex(ValueError, 'site.expected_tcp_offset'):
            config.require_live_settings()
        live_config().require_live_settings()

    def test_real_defaults_ignore_sim_environment(self):
        with patch.dict(os.environ, {'SPACEMOUSE_TELEOP_CONFIG': '/missing/sim.json'}):
            self.assertEqual(Path(build_parser().parse_args([]).config), DEFAULT_CONFIG)
            self.assertEqual(XArmConfig.load().robot['ip'], '192.168.1.209')
        self.assertEqual(XArmConfig.load(ip='127.0.0.1').robot['ip'], '127.0.0.1')

    def test_real_and_sim_settings_are_independent_in_both_directions(self):
        sim_path = ROOT / 'configs/spacemouse_xarm6_mujoco.json'
        state = RawSpaceMouseState(1, 0, 0, 0, 0, 0, timestamp=1)
        sim_before = TeleopCore(load_config(sim_path)).process(state)
        real_before = TeleopCore(XArmConfig.load().teleop).process(state)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'real.json'
            data = json.loads(DEFAULT_CONFIG.read_text())
            data['teleop']['linear_scale_mps'] *= .5
            path.write_text(json.dumps(data))
            real_after = TeleopCore(XArmConfig.load(path).teleop).process(state)
            self.assertNotEqual(real_before.linear_vel_mps, real_after.linear_vel_mps)
            self.assertEqual(sim_before, TeleopCore(load_config(sim_path)).process(state))
            sim_data = json.loads(sim_path.read_text())
            sim_data['linear_scale_mps'] *= .5
            path.write_text(json.dumps(sim_data))
            self.assertNotEqual(sim_before.linear_vel_mps, TeleopCore(load_config(path)).process(state).linear_vel_mps)
            self.assertEqual(real_before, TeleopCore(XArmConfig.load().teleop).process(state))
        self.assertFalse(hasattr(load_config(sim_path), 'robot'))

    def test_real_loader_rejects_sim_file_and_missing_explicit_input_setting(self):
        with self.assertRaises(ValueError):
            XArmConfig.load(ROOT / 'configs/spacemouse_xarm6_mujoco.json')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            data = json.loads(DEFAULT_CONFIG.read_text())
            del data['teleop']['filter_alpha']
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'filter_alpha'):
                XArmConfig.load(path)

    def test_nan_or_impossible_deadline_is_rejected(self):
        for key, value in (('socket_timeout_s', float('nan')), ('command_duration_s', 0.), ('command_duration_s', 9.), ('socket_timeout_s', 1.)):
            data = json.loads(DEFAULT_CONFIG.read_text())
            data['robot'][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'bad.json'
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    XArmConfig.load(path)


class LimiterTest(unittest.TestCase):
    def test_units_and_rotation_use_velocity_not_delta(self):
        limiter = VelocityLimiter(XArmConfig.load())
        speeds = limiter.convert(command(v=(.01, -.02, 0), w=(.1, 0, -.05)), 10., .2)
        self.assertEqual(speeds, (10., -20., 0., .1, 0., -.05))

    def test_vector_norm_limit_applies_to_combined_axes(self):
        limiter = VelocityLimiter(XArmConfig.load())
        speeds = limiter.convert(command(v=(10, 10, 10), w=(1, 1, 1)), 10, .2)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in speeds[:3])), 30.)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in speeds[3:])), .15)

    def test_acceleration_is_bounded_and_stop_has_no_slew_tail(self):
        config = live_config()
        limiter = VelocityLimiter(config)
        first = limiter.convert(command(), 10., .2, feedback())
        self.assertAlmostEqual(first[0], 5.)
        second = limiter.convert(command(t=10.02), 10.02, .2, feedback(10.02))
        self.assertAlmostEqual(second[0] - first[0], 6.)
        stopped = limiter.convert(replace(command(t=10.04), enabled=False), 10.04, .2)
        self.assertEqual(stopped, (0.,) * 6)

    def test_explicit_neutral_stops_filtered_command_immediately(self):
        limiter = VelocityLimiter(live_config())
        limiter.convert(command(), 10., .2)
        self.assertEqual(limiter.convert(command(t=10.01), 10.01, .2, neutral=True), (0.,) * 6)

    def test_boundary_prevents_outward_motion_but_allows_retreat(self):
        config = live_config()
        for pos, velocity, allowed in ((101, .02, False), (101, -.02, True), (-101, -.02, False), (-101, .02, True)):
            limiter = VelocityLimiter(config)
            result = limiter.convert(command(v=(velocity, 0, 0)), 10., .2, feedback(tcp_mm_rad=(pos, 0, 0, 0, 0, 0)))
            self.assertEqual(result[0] != 0, allowed)

    def test_boundary_accounts_for_lease_feedback_age_and_braking(self):
        config = live_config()
        limiter = VelocityLimiter(config)
        pose = feedback(t=9.9, tcp_mm_rad=(99.9, 0, 0, 0, 0, 0))
        speeds = limiter.convert(command(), 10., .2, pose)
        v = speeds[0] / 1000
        self.assertLessEqual(v * .3 + v * v / (2 * .3), .0001 + 1e-12)

    def test_invalid_frame_time_and_numeric_input_fail(self):
        for bad in (replace(command(), frame='tool'), replace(command(), timestamp=0), command(v=(float('nan'), 0, 0))):
            with self.assertRaises(TeleopFault):
                VelocityLimiter(XArmConfig.load()).convert(bad, 10., .2)

    def test_feedback_freshness_fault_mode_and_motor_masks(self):
        for bad in (feedback(9.), feedback(error_code=1), feedback(warn_code=1), feedback(mode=0), feedback(motor_enable=0), feedback(axis=7), feedback(tcp_mm_rad=(float('nan'),) * 6)):
            with self.assertRaises(TeleopFault):
                check_feedback(bad, 10., .25, running=True)
        check_feedback(feedback(), 10., .25, running=True)

    def test_mode_changed_is_allowed_only_for_checked_startup(self):
        check_feedback(feedback(state=5, mode=0), 10., .25, running=False)
        with self.assertRaises(TeleopFault):
            check_feedback(feedback(state=5), 10., .25, running=True)
        with self.assertRaises(TeleopFault):
            check_feedback(feedback(state=5, error_code=1), 10., .25, running=False)
        for state in (1, 3, 6):
            with self.assertRaises(TeleopFault):
                check_feedback(feedback(state=state), 10., .25, running=False)

    def test_site_confirmation_does_not_reposition_robot(self):
        check_site(live_config(), feedback())
        for bad in (feedback(joints_rad=(1.,) * 6), feedback(payload_kg_com_mm=(2., 0, 0, 0)), feedback(tcp_offset_mm_rad=(10.,) * 6)):
            with self.assertRaises(TeleopFault):
                check_site(live_config(), bad)

class ConfigBoundaryTest(unittest.TestCase):
    def test_sim_loader_rejects_real_execution_settings(self):
        with self.assertRaisesRegex(ValueError, 'real xArm config'):
            load_config(DEFAULT_CONFIG)


class OptionalWorkspaceTest(unittest.TestCase):
    def test_workspace_requires_explicit_opt_out_but_other_site_fields_remain_required(self):
        config = live_config()
        config.site.update(workspace_min_m=None, workspace_max_m=None)
        with self.assertRaisesRegex(ValueError, 'workspace_min_m'):
            config.require_live_settings()
        config.site['workspace_limits_enabled'] = False
        config.require_live_settings()
        config.site['expected_tcp_offset_mm_rad'] = None
        with self.assertRaisesRegex(ValueError, 'expected_tcp_offset'):
            config.require_live_settings()

    def test_disabled_box_does_not_clip_teleop_or_home_but_keeps_speed_and_tool_checks(self):
        from robot_teleop.backends.real.xarm_sdk.safety import check_homing_site
        config = live_config()
        config.site['workspace_limits_enabled'] = False
        config.site['max_linear_accel_mps2'] = None
        far = feedback(tcp_mm_rad=(500, -400, 600, 0, 0, 0))
        check_homing_site(config, far)
        speeds = VelocityLimiter(config).convert(command(v=(1, 0, 0)), 10., .2, far)
        self.assertAlmostEqual(speeds[0], 30.)
        with self.assertRaises(TeleopFault):
            check_homing_site(config, replace(far, payload_kg_com_mm=(2, 0, 0, 0)))

    def test_workspace_switch_must_be_boolean(self):
        data = json.loads(DEFAULT_CONFIG.read_text())
        data['site']['workspace_limits_enabled'] = 'false'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'workspace_limits_enabled'):
                XArmConfig.load(path)
