"""Thin real-data ROS 2 adapter for Frob and global pose sources."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .adapters import (
    FrobAdapterConfig,
    odometry_to_motion,
    pose_message_to_measurement,
    validate_motion_timing,
)
from .ekf import NavigationEKF


class ResilientNavigationNode(Node):
    def __init__(self):
        super().__init__("resilient_navigation")
        defaults = FrobAdapterConfig()
        self.declare_parameter("odometry_topic", defaults.odometry_topic)
        self.declare_parameter("camera_pose_topic", defaults.external_pose_topic)
        self.declare_parameter("gnss_pose_topic", defaults.gnss_pose_topic)
        self.declare_parameter("output_topic", defaults.output_topic)
        self.declare_parameter("map_frame", defaults.map_frame)
        self.declare_parameter("base_frame", defaults.base_frame)
        self.declare_parameter("publish_tf", defaults.publish_tf)
        self._ekf = NavigationEKF(max_measurement_age=1.5)
        self._last_motion_time = None
        self._publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self._on_motion,
            20,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("camera_pose_topic").value,
            lambda msg: self._on_pose(msg, "camera"),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("gnss_pose_topic").value,
            lambda msg: self._on_pose(msg, "gnss"),
            10,
        )
        if self.get_parameter("publish_tf").value:
            self.get_logger().warning(
                "publish_tf зарезервирован, но отключён в прототипе: назначьте одного владельца map→odom перед включением"
            )
        self.get_logger().info(
            "Реальный адаптер использует только готовую объединённую одометрию Frob; raw /odom и IMU повторно не сливаются"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_motion(self, message: Odometry) -> None:
        measurement = odometry_to_motion(message, self._now_seconds())
        timing = validate_motion_timing(
            self._last_motion_time,
            measurement.measured_at,
            measurement.received_at,
        )
        if not timing.accepted:
            self.get_logger().warning(f"Одометрия отклонена по времени: {timing.reason}")
            return
        self._last_motion_time = timing.next_timestamp
        if timing.dt > 0.0:
            self._ekf.predict(measurement, timing.dt)
            self._publish(message.header.stamp)

    def _on_pose(self, message: PoseWithCovarianceStamped, source: str) -> None:
        measurement = pose_message_to_measurement(message, source, self._now_seconds())
        decision = self._ekf.update_pose(measurement, self._now_seconds())
        log = self.get_logger().info if decision.accepted else self.get_logger().warning
        log(f"{source}: {decision.reason}; поправка {decision.correction_m:.2f} м")
        if decision.accepted:
            self._publish(message.header.stamp)

    def _publish(self, stamp) -> None:
        state = self._ekf.snapshot()
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.get_parameter("map_frame").value
        message.child_frame_id = self.get_parameter("base_frame").value
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(state.yaw / 2.0)
        message.twist.twist.linear.x = state.speed
        covariance = [0.0] * 36
        covariance[0] = float(state.covariance[0, 0])
        covariance[1] = float(state.covariance[0, 1])
        covariance[6] = float(state.covariance[1, 0])
        covariance[7] = float(state.covariance[1, 1])
        covariance[35] = float(state.covariance[2, 2])
        message.pose.covariance = covariance
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ResilientNavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
