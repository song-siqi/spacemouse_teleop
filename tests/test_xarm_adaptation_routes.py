"""Public routes retain the dedicated real lifecycle and production bindings."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from robot_teleop.cli.main import main
from robot_teleop.runtime.experiment import load_experiment
from robot_teleop.backends.real.xarm_sdk.collection_config import CollectionConfig


class AdaptationRoutesTests(unittest.TestCase):
    def test_demo_and_translation_profiles_use_specialized_entrypoints(self):
        for profile in ('demo','translation'):
            with self.subTest(profile=profile), patch('robot_teleop.cli.main.importlib.import_module') as load:
                load.return_value.main.return_value=0
                self.assertEqual(main(['teleop','--backend','xarm_sdk','--profile',profile,'--dry-run']),0)
                load.assert_called_once_with('robot_teleop.cli.xarm_'+profile)
                self.assertEqual(load.return_value.main.call_args.args[0],['--dry-run'])

    def test_experiment_and_shell_collection_select_same_production_config(self):
        root=Path(__file__).resolve().parents[1]
        experiment=load_experiment(root/'configs/xarm_real.json')
        production=root/'configs/spacemouse_xarm6_collection.json'
        self.assertEqual(Path(experiment['recording']['config']),production)
        config=CollectionConfig.load(production)
        self.assertEqual([c['fps'] for c in config.data['cameras']],[30,60,60])
        self.assertEqual([c['serial'] for c in config.data['cameras']],['246322303938','151422251501','231622300915'])
        self.assertEqual(config.data['min_free_bytes'],10*1024**3)
        self.assertEqual(config.robot.site['expected_tcp_offset_mm_rad'],[0,0,172,0,0,0])
        self.assertEqual(config.robot.site['expected_payload_kg_com_mm'],[.991,14.5,-3.23,61.24])

    def test_unified_demo_dry_run_has_no_hardware_connections(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            with patch('socket.create_connection',side_effect=AssertionError('No network in dry-run')):
                self.assertEqual(main(['teleop','--backend','xarm_sdk','--profile','demo','--input','mock',
                    '--dry-run','--duration','.03','--log',str(Path(tmp)/'demo.jsonl')]),0)
