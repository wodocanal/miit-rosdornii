"""Sensor messages crossing the truth/navigation trust boundary."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionMeasurement:
    source: str
    measured_at: float
    received_at: float
    speed: float
    yaw_rate: float
    speed_variance: float
    yaw_rate_variance: float
    speed_valid: bool = True
    yaw_rate_valid: bool = True


@dataclass(frozen=True)
class PoseMeasurement:
    source: str
    measured_at: float
    received_at: float
    x: float
    y: float
    yaw: float | None
    covariance: tuple[float, ...]
    reference_id: str = ""


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    measured_at: float
    received_at: float
    range: float
    bearing: float
    relative_yaw: float
    quality: float
    pose: PoseMeasurement


@dataclass(frozen=True)
class RangeMeasurement:
    beacon_id: str
    measured_at: float
    received_at: float
    distance: float
    variance: float


@dataclass(frozen=True)
class SensorStatus:
    name: str
    available: bool
    last_measured_at: float | None
    state: str
    reason: str = ""


@dataclass(frozen=True)
class SensorPacket:
    motion: MotionMeasurement | None
    gnss: PoseMeasurement | None
    camera: tuple[MarkerObservation, ...]
    uwb_ranges: tuple[RangeMeasurement, ...]
    uwb_pose: PoseMeasurement | None
    statuses: tuple[SensorStatus, ...]
