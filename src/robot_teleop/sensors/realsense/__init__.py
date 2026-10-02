"""RGB-only RealSense capture, independent of the robot control backend."""

from .capture import CameraGroup, MockCameraGroup, REQUIRED_ROLES, enumerate_devices

__all__ = ["CameraGroup", "MockCameraGroup", "REQUIRED_ROLES", "enumerate_devices"]
