"""Low-speed joint feedback homing with finite leases; no queued position move."""
from __future__ import annotations

import math
from .safety import TeleopFault


class JointHomer:
    def __init__(self, config):
        self.config = config
        self.settings = config.robot['homing']
        self.target = tuple(config.site['initial_joints_rad'])
        self.previous = (0.,) * 6
        self.last_time = None

    def step(self, feedback, now):
        r, s = self.config.robot, self.settings
        dt = 1 / r['control_hz'] if self.last_time is None else now - self.last_time
        if not 0 < dt <= r['feedback_timeout_s']:
            raise TeleopFault('Homing control loop stalled')
        errors = tuple(t - p for t, p in zip(self.target, feedback.joints_rad))
        if len(errors) != 6 or not all(math.isfinite(e) for e in errors):
            raise TeleopFault('Invalid homing joint feedback')
        at_target = all(abs(e) <= s['tolerance_rad'] for e in errors)
        lease = r['command_duration_s'] + max(0., now - feedback.received_at)
        result = []
        for error, previous in zip(errors, self.previous):
            remaining = max(0., abs(error) - s['tolerance_rad'] * .5)
            cap = min(s['speed_radps'], remaining / lease,
                      math.sqrt(2 * s['accel_radps2'] * remaining))
            target = math.copysign(cap, error)
            change = max(-s['accel_radps2'] * dt, min(s['accel_radps2'] * dt, target - previous))
            velocity = previous + change
            if abs(error) <= s['tolerance_rad'] or velocity * error <= 0:
                velocity = 0.
            else:
                velocity = math.copysign(min(abs(velocity), cap), error)
            result.append(velocity)
        self.previous, self.last_time = tuple(result), now
        return self.previous, at_target
