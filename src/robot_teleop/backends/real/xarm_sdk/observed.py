"""Motion delegation and xArm-native observational taps."""
from __future__ import annotations
import itertools
from robot_teleop.backends.real.xarm_sdk.backend import XArmBackend
from robot_teleop.backends.real.xarm_sdk.transport import SDKChannel
from robot_teleop.backends.real.xarm_sdk.state import ObservedChannel

class ObservedXArmBackend:
    def __init__(self, config, recorder, backend_factory=XArmBackend, channel_factory=SDKChannel):
        self.config, self.recorder = config, recorder
        self.backend_factory, self.channel_factory = backend_factory, channel_factory
        self.inner = None
        self._used = False
        self._gripper_sequence = itertools.count(1)
        self._report_sequence = itertools.count(1)
        self._report_key = None

    def __getattr__(self, name):
        if self.inner is None:
            raise AttributeError(name)
        return getattr(self.inner, name)

    def _channel(self, *args, **kwargs):
        return ObservedChannel(self.channel_factory(*args, **kwargs),
                               self.recorder.publish_state, lambda: next(self._gripper_sequence))

    def connect(self):
        if self.inner is not None:
            raise RuntimeError('Motion backend is already connected')
        self.inner = self.backend_factory(self.config, channel_factory=self._channel)
        self._report_key = None
        return self.inner.connect()

    def home(self, recorder, input_guard=None):
        if self._used:
            errors = self.close()
            if errors:
                raise RuntimeError('Previous motion backend cleanup failed: ' + '; '.join(errors))
            self.connect()
        self._used = True
        return self.inner.home(recorder, input_guard)

    def activate(self):
        self._used = True
        return self.inner.activate()

    def feedback(self):
        feedback = self.inner.feedback()
        key = (id(self.inner), feedback.sequence)
        if key != self._report_key:
            self._report_key = key
            self.recorder.publish_state({'kind':'report','sequence':next(self._report_sequence),
                'received_at':feedback.received_at, 'valid':True, 'source':'existing_control_feedback',
                'control_report_sequence':feedback.sequence, 'values':feedback.to_dict()})
        return feedback

    def send(self, *args, **kwargs):
        return self.inner.send(*args, **kwargs)

    def close(self):
        if self.inner is None:
            return []
        inner, self.inner = self.inner, None
        return inner.close()
