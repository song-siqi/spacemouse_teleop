import json
from pathlib import Path
import tempfile
import unittest
from spacemouse_teleop.backends.xarm6_camera.config import CollectionConfig, DEFAULT_CONFIG
from spacemouse_teleop.backends.xarm_sdk.config import XArmConfig

class CollectionConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.data=json.loads(DEFAULT_CONFIG.read_text())
        self.data['robot_config']=str(DEFAULT_CONFIG.parent/'spacemouse_xarm6_demo.json')
        self.path=Path(self.temp.name)/'collection.json'
    def load(self):
        self.path.write_text(json.dumps(self.data))
        return CollectionConfig.load(self.path)
    def test_input_mapping_comes_only_from_referenced_real_config(self):
        loaded=self.load()
        real=XArmConfig.load(self.data['robot_config'])
        self.assertEqual(loaded.robot,real)
        self.data['sample_hz']=10
        self.data['quality']['max_camera_age_s']=.2
        self.assertEqual(self.load().robot,real)
    def test_camera_roles_and_serials_are_unambiguous(self):
        self.data['cameras'][1]['serial']=self.data['cameras'][0]['serial']
        with self.assertRaises(ValueError): self.load()
    def test_live_rejects_unbound_camera(self):
        self.data['cameras'][0]['serial']=None
        config=self.load()
        with self.assertRaises(ValueError): config.require_live()
    def test_collection_document_is_not_a_real_or_sim_input_config(self):
        self.load()
        with self.assertRaises((ValueError,KeyError,TypeError)): XArmConfig.load(self.path)
    def test_nonfinite_quality_and_rate_rejected(self):
        self.data['sample_hz']=float('nan')
        with self.assertRaises(ValueError): self.load()
        self.data['sample_hz']=20
        self.data['quality']['max_robot_age_s']=-1
        with self.assertRaises(ValueError): self.load()
    def test_old_config_defaults_to_png_and_project_default_selects_video(self):
        self.assertEqual(self.load().data['visual_storage']['mode'],'video_after_episode')
        self.data.pop('visual_storage')
        self.assertEqual(self.load().data['visual_storage']['mode'],'png')
    def test_video_rejects_invalid_mode_and_odd_dimensions_before_connect(self):
        self.data['visual_storage']={'mode':'invalid'}
        with self.assertRaises(ValueError): self.load()
        self.data['visual_storage']={'mode':'video_after_episode'}
        self.data['cameras'][0]['height']=479
        with self.assertRaises(ValueError): self.load()
    def test_ip_override_does_not_write_referenced_file(self):
        real=Path(self.data['robot_config']); previous=real.read_bytes()
        self.load()
        loaded=CollectionConfig.load(self.path,ip='192.0.2.1')
        self.assertEqual(loaded.robot.robot['ip'],'192.0.2.1')
        self.assertEqual(real.read_bytes(),previous)

if __name__=='__main__': unittest.main()
