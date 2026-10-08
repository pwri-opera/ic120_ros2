"""Publish Nav2-only odometry with twist derived from the input pose."""

from copy import deepcopy
from math import atan2, isfinite, sqrt

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ic120_navigation.pose_velocity_estimator import PoseVelocityEstimator


class PoseVelocityOdometry(Node):
    def __init__(self):
        super().__init__('pose_velocity_odometry')
        self.declare_parameter('input_topic', 'odom_pose')
        self.declare_parameter('output_topic', 'odom_nav')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('window', 0.15)
        self.declare_parameter('min_interval', 0.05)
        self.declare_parameter('max_gap', 1.0)
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
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        position = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        velocity = self.estimator.update(
            stamp, position.x, position.y, yaw,
            (msg.header.frame_id, msg.child_frame_id))
        if velocity is None:
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
        # Input twist covariance describes the specified zero twist, not this
        # estimate. Mark it unspecified; this topic is for Nav2, not EKF fusion.
        output.twist.covariance = [0.0] * 36
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVelocityOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
