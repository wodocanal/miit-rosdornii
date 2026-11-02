"""ROS-message-shaped adapters kept free of rclpy imports for testability."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .measurements import MotionMeasurement, PoseMeasurement
from .world import wrap_angle


@dataclass(frozen=True)
class FrobAdapterConfig:
    odometry_topic: str = "/odometry/filtered"
    external_pose_topic: str = "/camera/global_pose"
    gnss_pose_topic: str = "/gnss/global_pose"
    output_topic: str = "/resilient_nav/odometry"
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_footprint"
    publish_tf: bool = False


@dataclass(frozen=True)
class MotionTimingDecision:
    accepted: bool
    dt: float
    next_timestamp: float | None
    reason: str


def validate_motion_timing(
    previous_timestamp: float | None,
    measured_at: float,
    received_at: float,
    max_age: float = 1.5,
    max_dt: float = 0.5,
) -> MotionTimingDecision:
    """Reject stale/non-monotonic motion without moving the time watermark."""
    if received_at - measured_at > max_age:
        return MotionTimingDecision(False, 0.0, previous_timestamp, "stale")
    if previous_timestamp is None:
        return MotionTimingDecision(True, 0.0, measured_at, "initial")
    if measured_at <= previous_timestamp:
        return MotionTimingDecision(False, 0.0, previous_timestamp, "non_increasing")
    return MotionTimingDecision(True, min(measured_at - previous_timestamp, max_dt), measured_at, "accepted")


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return wrap_angle(math.atan2(siny_cosp, cosy_cosp))


def odometry_to_motion(message, received_at: float | None = None) -> MotionMeasurement:
    measured_at = stamp_seconds(message.header.stamp)
    received = measured_at if received_at is None else float(received_at)
    covariance = message.twist.covariance
    speed_variance = float(covariance[0]) if covariance[0] > 0.0 else 0.04
    yaw_rate_variance = float(covariance[35]) if covariance[35] > 0.0 else 0.0025
    return MotionMeasurement(
        "frob_fused_odometry",
        measured_at,
        received,
        float(message.twist.twist.linear.x),
        float(message.twist.twist.angular.z),
        speed_variance,
        yaw_rate_variance,
    )


def pose_message_to_measurement(
    message,
    source: str,
    received_at: float | None = None,
) -> PoseMeasurement:
    measured_at = stamp_seconds(message.header.stamp)
    received = measured_at if received_at is None else float(received_at)
    pose = message.pose.pose
    source_covariance = message.pose.covariance
    xx = float(source_covariance[0]) if source_covariance[0] > 0.0 else 1.0
    yy = float(source_covariance[7]) if source_covariance[7] > 0.0 else 1.0
    yaw_var = float(source_covariance[35]) if source_covariance[35] > 0.0 else 0.25
    return PoseMeasurement(
        source,
        measured_at,
        received,
        float(pose.position.x),
        float(pose.position.y),
        quaternion_to_yaw(pose.orientation),
        (xx, 0.0, 0.0, 0.0, yy, 0.0, 0.0, 0.0, yaw_var),
    )
