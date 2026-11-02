import math
from dataclasses import replace

import pytest

from gnss_resilient_demo.models import WorldSnapshot
from gnss_resilient_demo.sensors import SensorSuite
from gnss_resilient_demo.world import WorldModel


def snapshot(time=0.0, x=0.0, y=0.0, yaw=0.0, tunnel=False):
    return WorldSnapshot(time, x, y, yaw, 4.0, 0.1, False, tunnel, False)


def world_with(markers, obstacles=None):
    return WorldModel.from_dict(
        {
            "bounds": [-10, -10, 20, 20],
            "route": [[0, 0], [10, 0]],
            "initial_pose": [0, 0, 0],
            "vehicle": {"radius": 0.2},
            "obstacles": obstacles or [],
            "tunnels": [],
            "gnss_zones": [],
            "markers": markers,
            "beacons": [
                {"id": "B1", "x": -8, "y": -8},
                {"id": "B2", "x": 8, "y": -8},
                {"id": "B3", "x": 8, "y": 8},
                {"id": "B4", "x": -8, "y": 8},
            ],
        }
    )


def test_disabling_camera_does_not_change_gnss_random_sequence():
    world = world_with([])
    enabled = SensorSuite(seed=516, gnss_hz=10.0, camera_hz=10.0)
    disabled = SensorSuite(seed=516, gnss_hz=10.0, camera_hz=10.0)
    disabled.faults.camera.enabled = False
    enabled_values, disabled_values = [], []

    for index in range(6):
        state = snapshot(time=index * 0.1, x=index * 0.4)
        a = enabled.sample(state, world)
        b = disabled.sample(state, world)
        if a.gnss:
            enabled_values.append((a.gnss.x, a.gnss.y))
            disabled_values.append((b.gnss.x, b.gnss.y))

    assert enabled_values == disabled_values


def test_gnss_requires_manual_enable_and_world_availability():
    world = world_with([])
    sensors = SensorSuite(seed=1, gnss_hz=10.0)

    assert sensors.sample(snapshot(time=0.0, tunnel=True), world).gnss is None
    sensors.faults.gnss.enabled = False
    assert sensors.sample(snapshot(time=0.1, tunnel=False), world).gnss is None
    sensors.faults.gnss.enabled = True
    assert sensors.sample(snapshot(time=0.2, tunnel=False), world).gnss is not None


@pytest.mark.parametrize(
    "markers,obstacles,visible_id",
    [
        ([{"id": 1, "x": 5, "y": 0, "facing": math.pi}], [], 1),
        ([{"id": 2, "x": -5, "y": 0, "facing": 0}], [], None),
        ([{"id": 3, "x": 5, "y": 0, "facing": 0}], [], None),
        (
            [{"id": 4, "x": 5, "y": 0, "facing": math.pi}],
            [{"id": "wall", "rect": [2, -1, 1, 2]}],
            None,
        ),
        ([{"id": 5, "x": 15, "y": 0, "facing": math.pi}], [], None),
    ],
)
def test_camera_respects_fov_working_face_occlusion_and_range(markers, obstacles, visible_id):
    world = world_with(markers, obstacles)
    sensors = SensorSuite(seed=2, camera_hz=10.0)
    sensors.faults.camera.dropout = 0.0
    sensors.faults.camera.outlier_probability = 0.0
    sensors.faults.camera.max_range = 10.0

    packet = sensors.sample(snapshot(), world)

    ids = [item.marker_id for item in packet.camera]
    assert ids == ([] if visible_id is None else [visible_id])


def test_camera_does_not_republish_old_observation_when_marker_disappears():
    world = world_with([{"id": 7, "x": 5, "y": 0, "facing": math.pi}])
    sensors = SensorSuite(seed=3, camera_hz=10.0)
    sensors.faults.camera.dropout = 0.0
    first = sensors.sample(snapshot(time=0.0), world)
    sensors.faults.camera.hidden_ids.add(7)

    second = sensors.sample(snapshot(time=0.1), world)

    assert [item.marker_id for item in first.camera] == [7]
    assert second.camera == ()
    assert not hasattr(first.camera[0], "truth")
    assert not hasattr(first.gnss, "truth")


def test_uwb_uses_real_ranges_and_produces_a_position_with_three_beacons():
    world = world_with([])
    sensors = SensorSuite(seed=8, uwb_hz=10.0)
    sensors.faults.uwb.enabled = True
    sensors.faults.uwb.noise_std = 0.0
    sensors.faults.uwb.dropout = 0.0

    packet = sensors.sample(snapshot(x=1.5, y=-2.0), world)

    assert len(packet.uwb_ranges) == 4
    assert packet.uwb_pose is not None
    assert (packet.uwb_pose.x, packet.uwb_pose.y) == pytest.approx((1.5, -2.0), abs=1e-8)


def test_moving_a_beacon_changes_its_range_and_disabling_uwb_stops_updates():
    world = world_with([])
    sensors = SensorSuite(seed=9, uwb_hz=10.0)
    sensors.faults.uwb.enabled = True
    sensors.faults.uwb.noise_std = 0.0
    sensors.faults.uwb.dropout = 0.0
    first = sensors.sample(snapshot(time=0.0), world)
    world.move_beacon("B1", -2.0, -2.0)
    second = sensors.sample(snapshot(time=0.1), world)
    sensors.faults.uwb.enabled = False
    third = sensors.sample(snapshot(time=0.2), world)

    assert first.uwb_ranges[0].distance != second.uwb_ranges[0].distance
    assert third.uwb_ranges == ()
    assert third.uwb_pose is None


def test_motion_packet_marks_independently_disabled_channels_invalid():
    world = world_with([])
    sensors = SensorSuite(seed=10, motion_hz=10.0)
    sensors.faults.motion.odom_enabled = False
    sensors.faults.motion.imu_enabled = True

    imu_only = sensors.sample(snapshot(time=0.0), world).motion
    sensors.faults.motion.odom_enabled = True
    sensors.faults.motion.imu_enabled = False
    odom_only = sensors.sample(snapshot(time=0.1), world).motion

    assert imu_only.speed_valid is False
    assert imu_only.yaw_rate_valid is True
    assert odom_only.speed_valid is True
    assert odom_only.yaw_rate_valid is False
