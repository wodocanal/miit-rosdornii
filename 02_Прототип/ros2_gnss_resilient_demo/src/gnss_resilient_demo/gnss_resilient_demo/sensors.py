"""Deterministic sensor generators and independently controlled faults."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .measurements import (
    MarkerObservation,
    MotionMeasurement,
    PoseMeasurement,
    RangeMeasurement,
    SensorPacket,
    SensorStatus,
)
from .models import Vec2, WorldSnapshot
from .world import WorldModel, wrap_angle


@dataclass
class GnssFaults:
    enabled: bool = True
    automatic_enabled: bool = True
    noise_std: float = 0.7
    dropout: float = 0.0
    latency: float = 0.0
    jump_x: float = 0.0
    jump_y: float = 0.0
    ramp_rate_x: float = 0.0
    ramp_rate_y: float = 0.0
    automatic_jump_x: float = 0.0
    automatic_jump_y: float = 0.0
    automatic_ramp_rate_x: float = 0.0
    automatic_ramp_rate_y: float = 0.0


@dataclass
class CameraFaults:
    enabled: bool = True
    automatic_enabled: bool = True
    quality: float = 1.0
    dropout: float = 0.04
    latency: float = 0.0
    max_range: float = 10.0
    fov: float = math.radians(75.0)
    hidden_ids: set[int] = field(default_factory=set)
    automatic_hidden_ids: set[int] = field(default_factory=set)
    hide_all: bool = False
    automatic_hide_all: bool = False
    outlier_probability: float = 0.01


@dataclass
class MotionFaults:
    odom_enabled: bool = True
    imu_enabled: bool = True
    scale_error: float = 0.0
    slip: float = 0.0
    odom_noise_std: float = 0.03
    gyro_noise_std: float = 0.008
    gyro_bias: float = 0.0
    dropout: float = 0.0


@dataclass
class UwbFaults:
    enabled: bool = True
    automatic_enabled: bool = True
    noise_std: float = 0.12
    dropout: float = 0.0
    nlos_bias: float = 0.0
    # The demo map is about 130 m wide; infrastructure beacons are placed so
    # that at least three links cover the complete test route.
    max_range: float = 140.0
    disabled_ids: set[str] = field(default_factory=set)


@dataclass
class FaultControls:
    gnss: GnssFaults = field(default_factory=GnssFaults)
    camera: CameraFaults = field(default_factory=CameraFaults)
    motion: MotionFaults = field(default_factory=MotionFaults)
    uwb: UwbFaults = field(default_factory=UwbFaults)


class SensorSuite:
    """Generates measurement-only objects from world snapshots."""

    def __init__(
        self,
        seed: int = 516,
        motion_hz: float = 50.0,
        gnss_hz: float = 2.0,
        camera_hz: float = 5.0,
        uwb_hz: float = 5.0,
    ):
        self.seed = seed
        sequence = np.random.SeedSequence(seed)
        odom_seed, imu_seed, gnss_seed, camera_seed, uwb_seed = sequence.spawn(5)
        self._rng_odom = np.random.default_rng(odom_seed)
        self._rng_imu = np.random.default_rng(imu_seed)
        self._rng_gnss = np.random.default_rng(gnss_seed)
        self._rng_camera = np.random.default_rng(camera_seed)
        self._rng_uwb = np.random.default_rng(uwb_seed)
        self.periods = {
            "motion": 1.0 / motion_hz,
            "gnss": 1.0 / gnss_hz,
            "camera": 1.0 / camera_hz,
            "uwb": 1.0 / uwb_hz,
        }
        self.faults = FaultControls()
        self.reset()

    def reset(self) -> None:
        sequence = np.random.SeedSequence(self.seed)
        odom_seed, imu_seed, gnss_seed, camera_seed, uwb_seed = sequence.spawn(5)
        self._rng_odom = np.random.default_rng(odom_seed)
        self._rng_imu = np.random.default_rng(imu_seed)
        self._rng_gnss = np.random.default_rng(gnss_seed)
        self._rng_camera = np.random.default_rng(camera_seed)
        self._rng_uwb = np.random.default_rng(uwb_seed)
        self._next = {name: 0.0 for name in self.periods}
        self._last: dict[str, float | None] = {"motion": None, "gnss": None, "camera": None, "uwb": None}

    def _due(self, name: str, now: float) -> bool:
        if now + 1e-9 < self._next[name]:
            return False
        while self._next[name] <= now + 1e-9:
            self._next[name] += self.periods[name]
        return True

    def sample(self, truth: WorldSnapshot, world: WorldModel) -> SensorPacket:
        motion = self._sample_motion(truth) if self._due("motion", truth.sim_time) else None
        gnss = self._sample_gnss(truth) if self._due("gnss", truth.sim_time) else None
        camera = self._sample_camera(truth, world) if self._due("camera", truth.sim_time) else ()
        uwb_ranges, uwb_pose = self._sample_uwb(truth, world) if self._due("uwb", truth.sim_time) else ((), None)
        statuses = (
            SensorStatus(
                "Одометрия + IMU",
                motion is not None,
                self._last["motion"],
                "Прогноз движения" if motion else "Нет новых данных",
                "",
            ),
            SensorStatus(
                "GNSS",
                gnss is not None,
                self._last["gnss"],
                "Новое измерение" if gnss else "Недоступен" if truth.in_tunnel else "Нет новых данных",
                "Тоннель/зона блокировки" if truth.in_tunnel or truth.in_gnss_zone else "",
            ),
            SensorStatus(
                "Камера",
                bool(camera),
                self._last["camera"],
                "Привязка по маркеру" if camera else "Нет видимых ориентиров",
                "",
            ),
            SensorStatus(
                "Радиомаяки",
                bool(uwb_ranges),
                self._last["uwb"],
                f"Дальностей: {len(uwb_ranges)}" if uwb_ranges else "Нет новых данных",
                "Недостаточно маяков" if uwb_ranges and uwb_pose is None else "",
            ),
        )
        return SensorPacket(motion, gnss, camera, uwb_ranges, uwb_pose, statuses)

    def _sample_uwb(
        self, truth: WorldSnapshot, world: WorldModel
    ) -> tuple[tuple[RangeMeasurement, ...], PoseMeasurement | None]:
        faults = self.faults.uwb
        if not (faults.enabled and faults.automatic_enabled):
            return (), None
        variance = max(faults.noise_std**2, 1e-6)
        ranges: list[RangeMeasurement] = []
        positions: list[tuple[float, float]] = []
        for beacon in world.beacons:
            true_distance = math.hypot(beacon.x - truth.x, beacon.y - truth.y)
            if (
                beacon.id in faults.disabled_ids
                or true_distance > min(faults.max_range, beacon.coverage)
                or self._rng_uwb.random() < faults.dropout
            ):
                continue
            measured = max(0.0, true_distance + faults.nlos_bias + self._rng_uwb.normal(0.0, faults.noise_std))
            ranges.append(RangeMeasurement(beacon.id, truth.sim_time, truth.sim_time, measured, variance))
            positions.append((beacon.x, beacon.y))
        if ranges:
            self._last["uwb"] = truth.sim_time
        pose = self._trilaterate(ranges, positions, truth.sim_time, variance)
        return tuple(ranges), pose

    @staticmethod
    def _trilaterate(
        ranges: list[RangeMeasurement],
        positions: list[tuple[float, float]],
        sim_time: float,
        variance: float,
    ) -> PoseMeasurement | None:
        if len(ranges) < 3:
            return None
        x0, y0 = positions[0]
        r0 = ranges[0].distance
        matrix = []
        rhs = []
        for item, (x, y) in zip(ranges[1:], positions[1:]):
            matrix.append((2.0 * (x - x0), 2.0 * (y - y0)))
            rhs.append(r0 * r0 - item.distance * item.distance + x * x + y * y - x0 * x0 - y0 * y0)
        a = np.asarray(matrix, dtype=float)
        if np.linalg.matrix_rank(a) < 2:
            return None
        solution, _, _, singular = np.linalg.lstsq(a, np.asarray(rhs, dtype=float), rcond=None)
        if singular[-1] <= 1e-9 or singular[0] / singular[-1] > 1000.0:
            return None
        geometry_scale = max(1.0, float(singular[0] / singular[-1]))
        pose_variance = max(variance * geometry_scale, 1e-4)
        return PoseMeasurement(
            "uwb",
            sim_time,
            sim_time,
            float(solution[0]),
            float(solution[1]),
            None,
            (pose_variance, 0.0, 0.0, pose_variance),
        )

    def _sample_motion(self, truth: WorldSnapshot) -> MotionMeasurement | None:
        faults = self.faults.motion
        if not (faults.odom_enabled or faults.imu_enabled):
            return None
        if self._rng_odom.random() < faults.dropout:
            return None
        measured_speed = 0.0
        if faults.odom_enabled:
            measured_speed = truth.speed * (1.0 + faults.scale_error) * (1.0 - faults.slip)
            measured_speed += self._rng_odom.normal(0.0, faults.odom_noise_std)
        measured_rate = 0.0
        if faults.imu_enabled:
            measured_rate = truth.yaw_rate + faults.gyro_bias
            measured_rate += self._rng_imu.normal(0.0, faults.gyro_noise_std)
        self._last["motion"] = truth.sim_time
        return MotionMeasurement(
            "local_motion",
            truth.sim_time,
            truth.sim_time,
            measured_speed,
            measured_rate,
            max(faults.odom_noise_std**2, 1e-6),
            max(faults.gyro_noise_std**2, 1e-7),
            speed_valid=faults.odom_enabled,
            yaw_rate_valid=faults.imu_enabled,
        )

    def _sample_gnss(self, truth: WorldSnapshot) -> PoseMeasurement | None:
        faults = self.faults.gnss
        world_blocked = truth.in_tunnel or truth.in_gnss_zone
        if not (faults.enabled and faults.automatic_enabled) or world_blocked or self._rng_gnss.random() < faults.dropout:
            return None
        bias_x = faults.jump_x + faults.automatic_jump_x
        bias_y = faults.jump_y + faults.automatic_jump_y
        bias_x += (faults.ramp_rate_x + faults.automatic_ramp_rate_x) * truth.sim_time
        bias_y += (faults.ramp_rate_y + faults.automatic_ramp_rate_y) * truth.sim_time
        x = truth.x + bias_x + self._rng_gnss.normal(0.0, faults.noise_std)
        y = truth.y + bias_y + self._rng_gnss.normal(0.0, faults.noise_std)
        variance = max(faults.noise_std**2, 1e-4)
        self._last["gnss"] = truth.sim_time
        return PoseMeasurement(
            "gnss",
            truth.sim_time,
            truth.sim_time + faults.latency,
            x,
            y,
            None,
            (variance, 0.0, 0.0, variance),
        )

    def _sample_camera(self, truth: WorldSnapshot, world: WorldModel) -> tuple[MarkerObservation, ...]:
        faults = self.faults.camera
        if not (faults.enabled and faults.automatic_enabled) or faults.hide_all or faults.automatic_hide_all:
            return ()
        observations: list[MarkerObservation] = []
        rover = Vec2(truth.x, truth.y)
        for marker in world.markers:
            if marker.id in faults.hidden_ids or marker.id in faults.automatic_hidden_ids:
                continue
            dx, dy = marker.x - truth.x, marker.y - truth.y
            distance = math.hypot(dx, dy)
            bearing = wrap_angle(math.atan2(dy, dx) - truth.yaw)
            face_to_rover = math.atan2(truth.y - marker.y, truth.x - marker.x)
            face_error = abs(wrap_angle(face_to_rover - marker.facing))
            visible = (
                0.15 < distance <= faults.max_range
                and abs(bearing) <= faults.fov / 2.0
                and face_error <= math.pi / 2.0
                and world.has_line_of_sight(rover, Vec2(marker.x, marker.y))
            )
            if not visible or self._rng_camera.random() < faults.dropout:
                continue
            angular_penalty = 1.0 + face_error / (math.pi / 2.0)
            quality = max(0.05, min(1.0, faults.quality / angular_penalty))
            position_std = (0.04 + 0.012 * distance * distance) / quality
            yaw_std = (0.018 + 0.007 * distance) / quality
            noisy_range = distance + self._rng_camera.normal(0.0, position_std)
            noisy_bearing = bearing + self._rng_camera.normal(0.0, yaw_std)
            estimated_yaw = wrap_angle(truth.yaw + self._rng_camera.normal(0.0, yaw_std))
            estimated_x = marker.x - noisy_range * math.cos(estimated_yaw + noisy_bearing)
            estimated_y = marker.y - noisy_range * math.sin(estimated_yaw + noisy_bearing)
            if self._rng_camera.random() < faults.outlier_probability:
                estimated_x += self._rng_camera.normal(0.0, 4.0)
                estimated_y += self._rng_camera.normal(0.0, 4.0)
            received_at = truth.sim_time + faults.latency
            pose = PoseMeasurement(
                "camera",
                truth.sim_time,
                received_at,
                estimated_x,
                estimated_y,
                estimated_yaw,
                (
                    position_std**2,
                    0.0,
                    0.0,
                    0.0,
                    position_std**2,
                    0.0,
                    0.0,
                    0.0,
                    yaw_std**2,
                ),
                str(marker.id),
            )
            observations.append(
                MarkerObservation(
                    marker.id,
                    truth.sim_time,
                    received_at,
                    noisy_range,
                    noisy_bearing,
                    wrap_angle(marker.facing - estimated_yaw),
                    quality,
                    pose,
                )
            )
        if observations:
            self._last["camera"] = truth.sim_time
        return tuple(observations)
