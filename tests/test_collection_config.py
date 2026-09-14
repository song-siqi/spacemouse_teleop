import json
from pathlib import Path
import tempfile
import unittest
from spacemouse_teleop.backends.xarm6_camera.config import CollectionConfig, DEFAULT_CONFIG
from spacemouse_teleop.backends.xarm_sdk.config import XArmConfig

class CollectionConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.data=json.loads(DEFAULT_CONFIG.read_text())
        self.data['robot_config']=str(DEFAULT_CONFIG.parent/'spacemouse_xarm6_demo.json')
        self.path=Path(self.tmp.name)/'config.json'
    def load(self):
        self.path.write_text(json.dumps(self.data))
        return CollectionConfig.load(self.path)
    def test_native_config_does_not_change_motion_config(self):
        self.data['pre_roll_s']=.2
        self.assertEqual(self.load().robot, XArmConfig.load(self.data['robot_config']))
        self.assertEqual(self.load().data['visual_storage']['mode'],'native_video')
    def test_legacy_schema_and_online_alignment_settings_require_migration(self):
        self.data['schema_version']=1
        with self.assertRaisesRegex(ValueError,'Migrate'): self.load()
        self.data['schema_version']=2; self.data['sample_hz']=20
        with self.assertRaisesRegex(ValueError,'alignment'): self.load()
    def test_passive_source_has_no_silent_fallback(self):
        self.data['robot_state_source']='query'
        with self.assertRaises(ValueError): self.load()
    def test_camera_roles_and_binding(self):
        self.data['cameras'][1]['serial']=self.data['cameras'][0]['serial']
        with self.assertRaises(ValueError): self.load()
    def test_live_rejects_unbound_camera(self):
        self.data['cameras'][0]['serial']=None
        with self.assertRaises(ValueError): self.load().require_live()
    def test_buffers_timeouts_and_queue_sizes(self):
        for key,value in [('pre_roll_s',-.1),('post_roll_s',float('nan')),('source_timeout_s',0),('clock_interval_s',True)]:
            old=self.data[key];self.data[key]=value
            with self.assertRaises(ValueError):self.load()
            self.data[key]=old
        self.data['queues']['video']=4
        with self.assertRaisesRegex(ValueError,'pre-roll'):self.load()
    def test_encoder_checks_precede_hardware(self):
        self.data['cameras'][0]['height']=479
        with self.assertRaises(ValueError):self.load()
        self.data['cameras'][0]['height']=480
        self.data['visual_storage']['mode']='video_after_episode'
        with self.assertRaises(ValueError):self.load()
    def test_ip_override_does_not_write_robot_file(self):
        path=Path(self.data['robot_config']); before=path.read_bytes()
        self.load()
        loaded=CollectionConfig.load(self.path,ip='192.0.2.1')
        self.assertEqual(loaded.robot.robot['ip'],'192.0.2.1')
        self.assertEqual(path.read_bytes(),before)
