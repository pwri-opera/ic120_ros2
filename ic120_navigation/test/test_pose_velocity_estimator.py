"""Offline kinematic checks; no ROS nodes or driving commands are started."""

from math import cos, pi, sin

import pytest

from ic120_navigation.pose_velocity_estimator import PoseVelocityEstimator


@pytest.mark.parametrize('speed', [-1.0, 1.0])
def test_forward_and_reverse_in_rotated_frame(speed):
    estimator = PoseVelocityEstimator()
    heading = 1.2
    result = None
    for i in range(30):
        t = i * 0.02
        result = estimator.update(t, speed*t*cos(heading), speed*t*sin(heading),
                                  heading, 'odom')
    assert result == pytest.approx((speed, 0.0, 0.0), abs=1e-10)


def test_rotation_across_pi_uses_header_time():
    estimator = PoseVelocityEstimator()
    result = None
    for i in range(30):
        t = i * 0.02
        yaw = (3.1 + 0.6*t + pi) % (2*pi) - pi
        result = estimator.update(t, 0.0, 0.0, yaw, 'odom')
    assert result == pytest.approx((0.0, 0.0, 0.6), abs=1e-10)


def test_constant_turn_has_no_spurious_lateral_motion():
    estimator = PoseVelocityEstimator()
    result = None
    for i in range(30):
        t = i * 0.02
        result = estimator.update(t, sin(0.6*t)/0.6, (1-cos(0.6*t))/0.6,
                                  0.6*t, 'odom')
    assert result[0] == pytest.approx(1.0, abs=0.002)
    assert result[1] == pytest.approx(0.0, abs=1e-10)
    assert result[2] == pytest.approx(0.6, abs=1e-10)


def test_duplicate_clock_reset_and_gap_do_not_make_spikes():
    estimator = PoseVelocityEstimator()
    estimator.update(10.0, 0.0, 0.0, 0.0, 'odom')
    assert estimator.update(10.1, 0.1, 0.0, 0.0, 'odom')[0] == pytest.approx(1.0)
    assert estimator.update(10.1, 100.0, 0.0, 0.0, 'odom') is None
    assert estimator.update(10.2, 0.2, 0.0, 0.0, 'odom')[0] == pytest.approx(1.0)
    assert estimator.update(0.0, 0.0, 0.0, 0.0, 'odom') is None
    assert estimator.update(0.1, 0.1, 0.0, 0.0, 'odom')[0] == pytest.approx(1.0)
    assert estimator.update(5.0, 0.0, 0.0, 0.0, 'odom') is None


def test_pose_jump_frame_change_and_nonfinite_data_reset_history():
    estimator = PoseVelocityEstimator()
    estimator.update(0.0, 0.0, 0.0, 0.0, 'odom')
    assert estimator.update(0.1, 100.0, 0.0, 0.0, 'odom') is None
    assert estimator.update(0.2, 100.1, 0.0, 0.0, 'odom')[0] == pytest.approx(1.0)
    assert estimator.update(0.3, 0.0, 0.0, 0.0, 'new_odom') is None
    assert estimator.update(0.4, float('nan'), 0.0, 0.0, 'new_odom') is None
    assert estimator.update(0.5, 0.0, 0.0, 0.0, 'new_odom') is None
