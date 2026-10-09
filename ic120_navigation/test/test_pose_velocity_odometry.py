"""Exercise the ROS message adapter without an executor or DDS node."""

from math import pi, sin, cos
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from ic120_navigation.pose_velocity_estimator import PoseVelocityEstimator
from ic120_navigation.message_rate_limiter import StampRateLimiter
from ic120_navigation.pose_velocity_odometry import (
    PoseVelocityOdometry, planar_twist_covariance)


def test_separate_output_preserves_pose_and_original_zero_twist():
    published = []
    adapter = SimpleNamespace(
        estimator=PoseVelocityEstimator(), base_frame='test_tf/base_link',
        rate_limiter=StampRateLimiter(),
        twist_covariance=planar_twist_covariance(0.25, 0.04),
        publisher=SimpleNamespace(publish=published.append))
    inputs = []
    for i in range(20):
        msg = Odometry()
        msg.header.stamp.nanosec = i * 20000000
        msg.header.frame_id = 'test_tf/odom'
        msg.pose.pose.position.y = i * 0.02
        msg.pose.pose.orientation.z = sin(pi/4)
        msg.pose.pose.orientation.w = cos(pi/4)
        inputs.append(msg)
        PoseVelocityOdometry.on_odometry(adapter, msg)
    assert published
    assert published[-1].twist.twist.linear.x == pytest.approx(1.0)
    assert published[-1].twist.twist.linear.y == pytest.approx(0.0, abs=1e-10)
    assert published[-1].child_frame_id == 'test_tf/base_link'
    assert published[-1].header == inputs[-1].header
    assert published[-1].pose == inputs[-1].pose
    assert all(msg.twist.twist.linear.x == 0.0 for msg in inputs)
    assert all(all(v == 0.0 for v in msg.twist.covariance) for msg in inputs)
    assert published[-1].twist.covariance[0] > 0
    assert published[-1].twist.covariance[35] > 0
    assert published[-1].twist.covariance[14] > 1e3


def test_invalid_quaternion_does_not_publish_a_velocity():
    published = []
    adapter = SimpleNamespace(
        estimator=PoseVelocityEstimator(), base_frame='',
        rate_limiter=StampRateLimiter(),
        publisher=SimpleNamespace(publish=published.append))
    msg = Odometry()
    msg.header.frame_id = 'odom'
    PoseVelocityOdometry.on_odometry(adapter, msg)
    assert not published


def test_publication_limit_preserves_full_rate_velocity_estimation():
    published = []
    adapter = SimpleNamespace(estimator=PoseVelocityEstimator(), base_frame='base_link',
                              rate_limiter=StampRateLimiter(20),
                              twist_covariance=planar_twist_covariance(.25, .04),
                              publisher=SimpleNamespace(publish=published.append))
    for i in range(61):
        msg = Odometry()
        ns = round(i * 1e9 / 60)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
        msg.header.frame_id = 'odom'; msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.position.x = i / 60
        PoseVelocityOdometry.on_odometry(adapter, msg)
    assert 18 <= len(published) <= 20
    assert all(m.twist.twist.linear.x == pytest.approx(1, abs=1e-6) for m in published)
    assert adapter.estimator.samples[-1][0] == 1.0


@pytest.mark.parametrize('linear,angular', [
    (0.0, 0.04), (-1.0, 0.04), (0.25, 0.0),
    (float('nan'), 0.04), (0.25, float('inf'))])
def test_invalid_covariance_cannot_claim_precise_velocity(linear, angular):
    with pytest.raises(ValueError):
        planar_twist_covariance(linear, angular)
