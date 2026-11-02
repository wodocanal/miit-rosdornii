"""Transparent planar EKF with innovation gating and cautious reacquisition."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .measurements import MotionMeasurement, PoseMeasurement
from .world import wrap_angle


@dataclass(frozen=True)
class FilterSnapshot:
    time: float
    x: float
    y: float
    yaw: float
    speed: float
    gyro_bias: float
    covariance: np.ndarray


@dataclass(frozen=True)
class UpdateDecision:
    source: str
    measured_at: float
    accepted: bool
    reason: str
    mahalanobis: float | None
    correction_m: float
    recovery_count: int = 0
    innovation_m: float = 0.0
    covariance_multiplier: float = 1.0
    gate_threshold: float = 0.0


class NavigationEKF:
    """Five-state EKF. It accepts measurements only and has no truth input."""

    def __init__(
        self,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        max_measurement_age: float = 1.0,
        recovery_after: float = 6.0,
        recovery_required: int = 3,
    ):
        self.max_measurement_age = max_measurement_age
        self.recovery_after = recovery_after
        self.recovery_required = recovery_required
        self._initial = (initial_x, initial_y, initial_yaw)
        self.reset()

    def reset(self) -> FilterSnapshot:
        x, y, yaw = self._initial
        self._x = np.array([x, y, wrap_angle(yaw), 0.0, 0.0], dtype=float)
        self._p = np.diag([1.0, 1.0, 0.2, 1.0, 0.02])
        self._time = 0.0
        self._last_external_accepted = 0.0
        self._recovery: deque[tuple[str, float, np.ndarray, PoseMeasurement]] = deque(maxlen=8)
        return self.snapshot()

    def snapshot(self) -> FilterSnapshot:
        return FilterSnapshot(
            self._time,
            float(self._x[0]),
            float(self._x[1]),
            float(self._x[2]),
            float(self._x[3]),
            float(self._x[4]),
            self._p.copy(),
        )

    def predict(
        self,
        measurement: MotionMeasurement,
        dt: float,
        process_noise_multiplier: float = 1.0,
    ) -> FilterSnapshot:
        if dt <= 0.0:
            return self.snapshot()
        yaw = float(self._x[2])
        speed = float(measurement.speed) if measurement.speed_valid else float(self._x[3])
        corrected_rate = (
            float(measurement.yaw_rate) - float(self._x[4])
            if measurement.yaw_rate_valid
            else 0.0
        )
        self._x[0] += speed * math.cos(yaw) * dt
        self._x[1] += speed * math.sin(yaw) * dt
        self._x[2] = wrap_angle(yaw + corrected_rate * dt)
        self._x[3] = speed

        f = np.eye(5)
        f[0, 2] = -speed * math.sin(yaw) * dt
        f[1, 2] = speed * math.cos(yaw) * dt
        f[2, 4] = -dt
        f[0, 3] = math.cos(yaw) * dt
        f[1, 3] = math.sin(yaw) * dt
        q = np.diag(
            [
                0.025 * dt + (measurement.speed_variance if measurement.speed_valid else 0.5) * dt * dt,
                0.025 * dt + (measurement.speed_variance if measurement.speed_valid else 0.5) * dt * dt,
                0.004 * dt + (measurement.yaw_rate_variance if measurement.yaw_rate_valid else 0.05) * dt * dt,
                0.06 * dt + (measurement.speed_variance if measurement.speed_valid else 0.5),
                0.00008 * dt,
            ]
        ) * max(1.0, float(process_noise_multiplier))
        self._p = f @ self._p @ f.T + q
        self._p = (self._p + self._p.T) * 0.5
        self._time = max(self._time + dt, measurement.measured_at)
        return self.snapshot()

    def predict_without_motion(
        self,
        dt: float,
        process_noise_multiplier: float = 1.0,
    ) -> FilterSnapshot:
        """Coast on the last state while uncertainty grows without local data."""
        if dt <= 0.0:
            return self.snapshot()
        yaw = float(self._x[2])
        speed = float(self._x[3])
        self._x[0] += speed * math.cos(yaw) * dt
        self._x[1] += speed * math.sin(yaw) * dt
        f = np.eye(5)
        f[0, 2] = -speed * math.sin(yaw) * dt
        f[1, 2] = speed * math.cos(yaw) * dt
        f[0, 3] = math.cos(yaw) * dt
        f[1, 3] = math.sin(yaw) * dt
        multiplier = max(1.0, float(process_noise_multiplier))
        q = np.diag([0.12 * dt, 0.12 * dt, 0.05 * dt, 0.35 * dt, 0.0002 * dt]) * multiplier
        self._p = f @ self._p @ f.T + q
        self._p = (self._p + self._p.T) * 0.5
        self._time += dt
        return self.snapshot()

    def update_pose(
        self,
        measurement: PoseMeasurement,
        now: float,
        covariance_multiplier: float = 1.0,
        gate_multiplier: float = 1.0,
    ) -> UpdateDecision:
        age = max(now - measurement.measured_at, measurement.received_at - measurement.measured_at)
        if age > self.max_measurement_age + 1e-9:
            return UpdateDecision(
                measurement.source,
                measurement.measured_at,
                False,
                "Отклонено: устаревшее измерение",
                None,
                0.0,
            )
        has_yaw = measurement.yaw is not None
        indices = [0, 1, 2] if has_yaw else [0, 1]
        h = np.zeros((len(indices), 5))
        for row, index in enumerate(indices):
            h[row, index] = 1.0
        z_values = [measurement.x, measurement.y]
        if has_yaw:
            z_values.append(float(measurement.yaw))
        z = np.asarray(z_values, dtype=float)
        innovation = z - h @ self._x
        if has_yaw:
            innovation[2] = wrap_angle(float(innovation[2]))
        size = 3 if has_yaw else 2
        covariance_multiplier = max(1.0, float(covariance_multiplier))
        r = np.asarray(measurement.covariance, dtype=float).reshape(size, size) * covariance_multiplier
        s = h @ self._p @ h.T + r
        mahalanobis = float(innovation.T @ np.linalg.solve(s, innovation))
        innovation_m = float(np.linalg.norm(innovation[:2]))
        gate = (11.34 if has_yaw else 9.21) * max(0.1, float(gate_multiplier))
        if mahalanobis <= gate:
            return self._apply_update(
                measurement, z, h, r, innovation, mahalanobis, now, False,
                innovation_m, covariance_multiplier, gate,
            )
        recovery_count = self._track_recovery(measurement, now)
        if recovery_count >= self.recovery_required:
            inflated = r * 4.0
            decision = self._apply_update(
                measurement,
                z,
                h,
                inflated,
                innovation,
                mahalanobis,
                now,
                True,
                innovation_m,
                covariance_multiplier * 4.0,
                gate,
            )
            self._recovery.clear()
            return decision
        reason = (
            "Ожидание согласованной серии для восстановления"
            if recovery_count
            else "Отклонено: большое расхождение"
        )
        return UpdateDecision(
            measurement.source,
            measurement.measured_at,
            False,
            reason,
            mahalanobis,
            0.0,
            recovery_count,
            innovation_m,
            covariance_multiplier,
            gate,
        )

    def _track_recovery(self, measurement: PoseMeasurement, now: float) -> int:
        if now - self._last_external_accepted < self.recovery_after:
            self._recovery.clear()
            return 0
        offset = np.array([measurement.x - self._x[0], measurement.y - self._x[1]])
        self._recovery.append((measurement.source, measurement.measured_at, offset, measurement))
        recent = [item for item in self._recovery if item[0] == measurement.source]
        recent = recent[-self.recovery_required :]
        if len(recent) < 2:
            return len(recent)
        if recent[-1][1] - recent[0][1] > 3.0:
            return 1
        offsets = np.stack([item[2] for item in recent])
        spread = float(np.max(np.linalg.norm(offsets - offsets.mean(axis=0), axis=1)))
        if spread > 1.0:
            self._recovery.clear()
            self._recovery.append((measurement.source, measurement.measured_at, offset, measurement))
            return 1
        return len(recent)

    def _apply_update(
        self,
        measurement: PoseMeasurement,
        z: np.ndarray,
        h: np.ndarray,
        r: np.ndarray,
        innovation: np.ndarray,
        mahalanobis: float,
        now: float,
        recovery: bool,
        innovation_m: float,
        covariance_multiplier: float,
        gate_threshold: float,
    ) -> UpdateDecision:
        before = self._x[:2].copy()
        s = h @ self._p @ h.T + r
        k = self._p @ h.T @ np.linalg.inv(s)
        self._x = self._x + k @ innovation
        self._x[2] = wrap_angle(float(self._x[2]))
        identity = np.eye(5)
        residual_transform = identity - k @ h
        self._p = residual_transform @ self._p @ residual_transform.T + k @ r @ k.T
        self._p = (self._p + self._p.T) * 0.5
        self._time = max(self._time, now)
        self._last_external_accepted = now
        self._recovery.clear()
        correction = float(np.linalg.norm(self._x[:2] - before))
        return UpdateDecision(
            measurement.source,
            measurement.measured_at,
            True,
            "Восстановление по серии наблюдений" if recovery else "Измерение принято",
            mahalanobis,
            correction,
            self.recovery_required if recovery else 0,
            innovation_m,
            covariance_multiplier,
            gate_threshold,
        )
