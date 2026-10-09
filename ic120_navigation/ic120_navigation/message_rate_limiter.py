"""Limit derived ROS publications using source simulation timestamps."""
from math import isfinite


class StampRateLimiter:
    def __init__(self, frequency=0.0):
        if not isfinite(frequency) or frequency < 0:
            raise ValueError('publish_frequency must be finite and nonnegative')
        self.interval = 1.0 / frequency if frequency else 0.0
        self.last_stamp = None

    def reset(self):
        self.last_stamp = None

    def allow(self, stamp):
        if not isfinite(stamp):
            return False
        if (self.interval == 0 or self.last_stamp is None or stamp < self.last_stamp or
                stamp - self.last_stamp + 1e-9 >= self.interval):
            self.last_stamp = stamp
            return True
        return False
