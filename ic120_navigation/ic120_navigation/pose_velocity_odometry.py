"""Publish Nav2-only odometry with twist derived from the input pose."""

from copy import deepcopy
from math import atan2, isfinite, sqrt

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ic120_navigation.pose_velocity_estimator import PoseVelocityEstimator
from ic120_navigation.message_rate_limiter import StampRateLimiter


def planar_twist_covariance(linear_variance, angular_variance):
    """Describe estimated planar twist without asserting perfect certainty.

    These configurable variances are nominal assumptions, not fitted sensor
    noise. Unestimated axes have large variance so their zero values are not
    accidentally fused as precise measurements.
    """
    if not all(isfinite(v) and v > 0 for v in
               (linear_variance, angular_variance)):
        raise ValueError('Velocity variances must be finite and positive')
    covariance = [0.0] * 36
    for index in (14, 21, 28):
        covariance[index] = 1e6
    covariance[0] = covariance[7] = linear_variance
    covariance[35] = angular_variance
    return covariance


class PoseVelocityOdometry(Node):
    def __init__(self):
        super().__init__('pose_velocity_odometry')
        self.declare_parameter('input_topic', 'odom_pose')
        self.declare_parameter('output_topic', 'odom_nav')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('window', 0.15)
        self.declare_parameter('min_interval', 0.05)
        self.declare_parameter('max_gap', 1.0)
        self.declare_parameter('linear_velocity_variance', 0.25)
        self.declare_parameter('angular_velocity_variance', 0.04)
        self.declare_parameter('publish_frequency', 0.0)
        self.rate_limiter = StampRateLimiter(self.get_parameter('publish_frequency').value)
        self.twist_covariance = planar_twist_covariance(
            self.get_parameter('linear_velocity_variance').value,
            self.get_parameter('angular_velocity_variance').value)
        self.estimator = PoseVelocityEstimator(
            window=self.get_parameter('window').value,
            min_interval=self.get_parameter('min_interval').value,
            max_gap=self.get_parameter('max_gap').value)
        self.base_frame = self.get_parameter('base_frame').value
        # Latest pose only: do not accumulate a backlog of old feedback.
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 1)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self.on_odometry,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_odometry(self, msg):
        q = msg.pose.pose.orientation
        norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not isfinite(norm) or norm < 1e-9 or not msg.header.frame_id:
            self.estimator.reset()
            self.rate_limiter.reset()
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        position = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = (msg.header.frame_id, msg.child_frame_id)
        if frame != self.estimator.frame:
            self.rate_limiter.reset()
        # Estimate from every input; only copying/publishing is rate-limited.
        velocity = self.estimator.update(stamp, position.x, position.y, yaw, frame)
        if velocity is None:
            return
        if not self.rate_limiter.allow(stamp):
            return
        output = deepcopy(msg)
        if self.base_frame:
            output.child_frame_id = self.base_frame
        output.twist.twist.linear.x = velocity[0]
        output.twist.twist.linear.y = velocity[1]
        output.twist.twist.linear.z = 0.0
        output.twist.twist.angular.x = 0.0
        output.twist.twist.angular.y = 0.0
        output.twist.twist.angular.z = velocity[2]
        # Input covariance describes the simulator's specified zero twist,
        # not the differentiated estimate now used by both Nav2 and EKF.
        output.twist.covariance = self.twist_covariance
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVelocityOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
