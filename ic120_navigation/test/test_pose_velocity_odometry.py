"""Exercise the ROS message adapter without an executor or DDS node."""

from math import pi, sin, cos
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from ic120_navigation.pose_velocity_estimator import PoseVelocityEstimator
from ic120_navigation.pose_velocity_odometry import PoseVelocityOdometry


def test_separate_output_preserves_pose_and_original_zero_twist():
    published = []
    adapter = SimpleNamespace(
        estimator=PoseVelocityEstimator(), base_frame='test_tf/base_link',
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


def test_invalid_quaternion_does_not_publish_a_velocity():
    published = []
    adapter = SimpleNamespace(
        estimator=PoseVelocityEstimator(), base_frame='',
        publisher=SimpleNamespace(publish=published.append))
    msg = Odometry()
    msg.header.frame_id = 'odom'
    PoseVelocityOdometry.on_odometry(adapter, msg)
    assert not published
