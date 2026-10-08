"""Causal planar velocity estimates from timestamped odometry poses."""

from collections import deque
from math import atan2, cos, hypot, isfinite, sin


class PoseVelocityEstimator:
    """Differentiate a short pose window; restart after resets or large gaps."""

    def __init__(self, window=0.15, min_interval=0.05, max_gap=1.0,
                 max_linear_speed=10.0, max_angular_speed=6.0):
        if not 0 < min_interval <= window <= max_gap:
            raise ValueError('Require 0 < min_interval <= window <= max_gap')
        if max_linear_speed <= 0 or max_angular_speed <= 0:
            raise ValueError('Reset thresholds must be positive')
        self.window = window
        self.min_interval = min_interval
        self.max_gap = max_gap
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.samples = deque()
        self.frame = None

    def reset(self):
        self.samples.clear()
        self.frame = None

    def update(self, stamp, x, y, yaw, frame):
        """Return (body vx, body vy, yaw rate), or None until initialized.

        Time is the message header time, never callback arrival time. Duplicate
        samples are ignored. Clock/frame changes and implausible pose jumps
        start a new window rather than producing velocity spikes. Body axes
        use the midpoint heading of the differentiated interval.
        """
        if not all(isfinite(v) for v in (stamp, x, y, yaw)):
            self.reset()
            return None
        if frame != self.frame:
            self.reset()
            self.frame = frame
        if self.samples:
            previous = self.samples[-1]
            dt = stamp - previous[0]
            if dt == 0:
                return None
            dyaw = atan2(sin(yaw - previous[3]), cos(yaw - previous[3]))
            if (dt < 0 or dt > self.max_gap or
                    hypot(x - previous[1], y - previous[2]) / dt >
                    self.max_linear_speed or
                    abs(dyaw / dt) > self.max_angular_speed):
                self.reset()
                self.frame = frame
            else:
                yaw = previous[3] + dyaw
        self.samples.append((stamp, x, y, yaw))
        while len(self.samples) > 2 and self.samples[1][0] <= stamp - self.window:
            self.samples.popleft()
        first = self.samples[0]
        dt = stamp - first[0]
        if dt < self.min_interval:
            return None
        dx = (x - first[1]) / dt
        dy = (y - first[2]) / dt
        heading = (yaw + first[3]) / 2.0
        return (cos(heading) * dx + sin(heading) * dy,
                -sin(heading) * dx + cos(heading) * dy,
                (yaw - first[3]) / dt)
