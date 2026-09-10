from __future__ import annotations
import json
import math
from dataclasses import dataclass
from pathlib import Path
from spacemouse_teleop.recording.visual import storage_config
from spacemouse_teleop.backends.xarm_sdk.config import XArmConfig

DEFAULT_CONFIG = Path(__file__).resolve().parents[4] / 'configs' / 'spacemouse_xarm6_collection.json'
ROLES = ('wrist', 'fixed_1', 'fixed_2')

@dataclass(frozen=True)
class CollectionConfig:
    path: Path
    data: dict
    robot: XArmConfig

    @classmethod
    def load(cls, path=DEFAULT_CONFIG, ip=None):
        path = Path(path).expanduser().resolve()
        data = json.loads(path.read_text())
        if not isinstance(data, dict) or data.get('schema_version') != 1:
            raise ValueError('Expected collection schema_version 1')
        real_path = (path.parent / data['robot_config']).resolve()
        robot = XArmConfig.load(real_path, ip=ip)
        cameras = data['cameras']
        if len(cameras) != 3 or {c['role'] for c in cameras} != set(ROLES):
            raise ValueError('Exactly wrist, fixed_1 and fixed_2 cameras are required')
        serials = [c.get('serial') for c in cameras if c.get('serial')]
        if len(set(serials)) != len(serials):
            raise ValueError('Camera serial numbers must be unique')
        for camera in cameras:
            for field in ('width', 'height', 'fps'):
                if not isinstance(camera[field], int) or isinstance(camera[field], bool) or camera[field] <= 0:
                    raise ValueError(f'camera.{field} must be a positive integer')
        for field in ('sample_hz', 'robot_poll_hz'):
            value = data[field]
            if not isinstance(value, (float,int)) or isinstance(value,bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{field} must be finite and positive')
        for field in ('max_camera_age_s','max_robot_age_s','max_gripper_age_s','max_camera_skew_s'):
            value = data['quality'][field]
            if not isinstance(value,(float,int)) or isinstance(value,bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'quality.{field} must be finite and positive')
        for field in ('queue_size','min_free_bytes'):
            value = data[field]
            if not isinstance(value,int) or isinstance(value,bool) or value <= 0:
                raise ValueError(f'{field} must be a positive integer')
        data['visual_storage'] = storage_config(data.get('visual_storage'))
        if data['visual_storage']['mode'] == 'video_after_episode':
            if any(c['width'] % 2 or c['height'] % 2 for c in cameras):
                raise ValueError('H.264 yuv420p requires even camera dimensions; resizing is disabled')
        data['robot_config'] = str(real_path)
        data['output_root'] = str((path.parent / data['output_root']).resolve())
        return cls(path, data, robot)

    def require_live(self):
        self.robot.require_live_settings()
        if not self.robot.gripper['enabled']:
            raise ValueError('Collection requires enabled actual gripper feedback')
        if any(not isinstance(c.get('serial'),str) or not c['serial'].strip() for c in self.data['cameras']):
            raise ValueError('Bind all three camera serials using camera preview before live collection')
