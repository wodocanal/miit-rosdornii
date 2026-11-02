import math
from types import SimpleNamespace

import pytest
from catkin_pkg.package import parse_package

from gnss_resilient_demo.adapters import (
    FrobAdapterConfig,
    odometry_to_motion,
    pose_message_to_measurement,
    quaternion_to_yaw,
    validate_motion_timing,
)


def ns(**values):
    return SimpleNamespace(**values)


def stamp(seconds, nanoseconds):
    return ns(sec=seconds, nanosec=nanoseconds)


def test_quaternion_to_yaw_wraps_planar_orientation():
    half = math.pi / 4.0
    quaternion = ns(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    assert quaternion_to_yaw(quaternion) == pytest.approx(math.pi / 2.0)


def test_frob_odometry_converts_timestamp_twist_and_covariance():
    covariance = [0.0] * 36
    covariance[0], covariance[35] = 0.04, 0.0025
    message = ns(
        header=ns(stamp=stamp(12, 340_000_000)),
        twist=ns(
            twist=ns(linear=ns(x=1.7), angular=ns(z=-0.2)),
            covariance=covariance,
        ),
    )

    measurement = odometry_to_motion(message, received_at=12.5)

    assert measurement.measured_at == pytest.approx(12.34)
    assert measurement.received_at == pytest.approx(12.5)
    assert measurement.speed == pytest.approx(1.7)
    assert measurement.yaw_rate == pytest.approx(-0.2)
    assert measurement.speed_variance == pytest.approx(0.04)
    assert measurement.yaw_rate_variance == pytest.approx(0.0025)


def test_external_pose_maps_xy_yaw_covariance_without_truth_fields():
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[35] = 0.25, 0.36, 0.04
    half = 0.25
    message = ns(
        header=ns(stamp=stamp(5, 0)),
        pose=ns(
            pose=ns(
                position=ns(x=8.0, y=-3.0),
                orientation=ns(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
            ),
            covariance=covariance,
        ),
    )

    measurement = pose_message_to_measurement(message, "camera", received_at=5.2)

    assert (measurement.x, measurement.y, measurement.yaw) == pytest.approx((8.0, -3.0, 0.5))
    assert measurement.covariance == pytest.approx((0.25, 0, 0, 0, 0.36, 0, 0, 0, 0.04))
    assert not hasattr(measurement, "truth")


def test_adapter_defaults_use_only_fused_frob_odometry():
    config = FrobAdapterConfig()

    assert config.odometry_topic == "/odometry/filtered"
    assert config.external_pose_topic == "/camera/global_pose"
    assert config.publish_tf is False


def test_package_manifest_is_valid_ament_python_package():
    package = parse_package("src/gnss_resilient_demo/package.xml")

    assert package.name == "gnss_resilient_demo"
    assert any(item.tagname == "build_type" and item.content == "ament_python" for item in package.exports)


def test_motion_timing_rejects_out_of_order_without_advancing_watermark():
    rejected = validate_motion_timing(10.0, 9.0, received_at=10.1)
    accepted = validate_motion_timing(10.0, 10.1, received_at=10.1)

    assert rejected.accepted is False
    assert rejected.next_timestamp == 10.0
    assert accepted.accepted is True
    assert accepted.dt == pytest.approx(0.1)


def test_motion_timing_rejects_stale_arrival():
    decision = validate_motion_timing(4.9, 5.0, received_at=8.0, max_age=1.5)

    assert decision.accepted is False
    assert decision.reason == "stale"
