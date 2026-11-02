"""Online source reliability without access to simulation truth or fault flags."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Sequence

import numpy as np

from .ekf import UpdateDecision
from .measurements import PoseMeasurement


class ReliabilityState(str, Enum):
    NORMAL = "Норма"
    SUSPECT = "Подозрение"
    DEGRADED = "Ухудшен"
    REJECTED = "Отклонён"
    RECOVERING = "Восстановление"


@dataclass(frozen=True)
class UpdatePolicy:
    source: str
    covariance_multiplier: float
    gate_multiplier: float
    hard_reject: bool
    reason: str


@dataclass(frozen=True)
class ReliabilitySnapshot:
    source: str
    score: float
    state: ReliabilityState
    reason: str
    covariance_multiplier: float
    gate_multiplier: float
    recovery_count: int
    last_correction_m: float
    age: float | None
    available: bool


@dataclass
class _SourceRecord:
    score: float = 0.92
    reason: str = "Данные согласованы"
    last_seen: float | None = None
    last_correction_m: float = 0.0
    recovery_count: int = 0
    recovering: bool = False
    available: bool = True
    missing_since: float | None = None
    last_missing_penalty: float | None = None


class ReliabilityManager:
    """Turns sensor health and mutual consistency into EKF policies.

    The public API deliberately accepts measurements, the filter prediction and
    mapped context only.  Simulation truth and injected-fault state cannot cross
    this boundary.
    """

    SOURCES = ("local_motion", "gnss", "camera", "uwb")

    def __init__(self, recovery_required: int = 3, consistency_distance: float = 3.0):
        self.recovery_required = max(2, int(recovery_required))
        self.consistency_distance = float(consistency_distance)
        self._records = {source: _SourceRecord() for source in self.SOURCES}
        self._urban_gnss_risk = False

    @property
    def process_noise_multiplier(self) -> float:
        score = self._records["local_motion"].score
        return min(8.0, max(1.0, 1.0 / max(score * score, 0.04)))

    def reset(self) -> None:
        self._records = {source: _SourceRecord() for source in self.SOURCES}
        self._urban_gnss_risk = False

    def assess_batch(
        self,
        measurements: Sequence[PoseMeasurement],
        predicted_xy: tuple[float, float],
        now: float,
        urban_gnss_risk: bool = False,
    ) -> dict[str, UpdatePolicy]:
        """Assess one delivery batch before any measurement changes the EKF."""
        self._urban_gnss_risk = bool(urban_gnss_risk)
        grouped = self._group_positions(measurements)
        tolerances = self._source_tolerances(measurements)
        for source in grouped:
            record = self._record(source)
            record.last_seen = now
            record.available = True
            record.missing_since = None
            record.last_missing_penalty = None

        points = {"local_motion": np.asarray(predicted_xy, dtype=float), **grouped}
        consensus = self._consensus(points.values()) if len(points) >= 3 else None
        if consensus is not None:
            for source, point in points.items():
                distance = float(np.linalg.norm(point - consensus))
                if distance > tolerances.get(source, self.consistency_distance):
                    penalty = 0.38 if source == "gnss" and urban_gnss_risk else 0.28
                    record = self._record(source)
                    record.score = max(0.05, record.score - penalty)
                    record.reason = (
                        "Локальный прогноз расходится с внешними источниками"
                        if source == "local_motion"
                        else "Расхождение с независимыми источниками"
                    )
                    record.recovery_count = 0
                    record.recovering = False
                else:
                    self._note_consistent(source, urban_gnss_risk)

        if urban_gnss_risk:
            record = self._records["gnss"]
            if "Расхождение" not in record.reason:
                record.reason = "Плотная городская застройка: GNSS проверяется строже"

        return {source: self._policy(source) for source in grouped}

    def observe_decision(self, decision: UpdateDecision, now: float) -> None:
        record = self._record(decision.source)
        record.last_seen = now
        record.available = True
        if decision.accepted:
            gain = 0.035 if record.score >= 0.5 else 0.07
            record.score = min(0.98, record.score + gain)
            record.last_correction_m = decision.correction_m
            if record.recovering:
                if record.score >= 0.78 and record.recovery_count >= self.recovery_required + 2:
                    record.recovering = False
                    record.recovery_count = 0
                    record.reason = "Доверие восстановлено по серии данных"
                else:
                    record.reason = "Восстановление: данные снова согласованы"
            elif decision.innovation_m <= self.consistency_distance:
                record.reason = "Данные согласованы"
        else:
            record.score = max(0.05, record.score - 0.22)
            record.reason = decision.reason
            record.recovery_count = 0
            record.recovering = False

    def mark_availability(self, source: str, available: bool, now: float) -> None:
        record = self._record(source)
        record.available = bool(available)
        if available:
            record.last_seen = now
            record.missing_since = None
            record.last_missing_penalty = None
            return

        if record.missing_since is None:
            record.missing_since = now
        missing_for = max(0.0, now - record.missing_since)
        if missing_for > 1.0:
            record.reason = "Нет свежих данных"
            # Sensor sampling runs much faster than the UI.  Penalise at a
            # bounded cadence so a temporarily invisible marker does not lose
            # all trust in a single render frame.
            if record.last_missing_penalty is None or now - record.last_missing_penalty >= 0.5:
                # Missing data means the source is unavailable, not proven
                # faulty.  Keep enough prior trust to evaluate the first
                # returning measurement instead of locking the source out.
                record.score = max(0.52, record.score - 0.08)
                record.last_missing_penalty = now

    def snapshot(self, source: str, now: float) -> ReliabilitySnapshot:
        record = self._record(source)
        policy = self._policy(source)
        age = None if record.last_seen is None else max(0.0, now - record.last_seen)
        return ReliabilitySnapshot(
            source=source,
            score=record.score,
            state=self._state(record),
            reason=record.reason,
            covariance_multiplier=policy.covariance_multiplier,
            gate_multiplier=policy.gate_multiplier,
            recovery_count=record.recovery_count,
            last_correction_m=record.last_correction_m,
            age=age,
            available=record.available,
        )

    def snapshots(self, now: float) -> tuple[ReliabilitySnapshot, ...]:
        return tuple(self.snapshot(source, now) for source in self.SOURCES)

    def _policy(self, source: str) -> UpdatePolicy:
        record = self._record(source)
        covariance = min(25.0, max(1.0, 1.0 / max(record.score * record.score, 0.04)))
        gate = max(0.45, min(1.0, 0.70 + 0.30 * record.score))
        if source == "gnss" and self._urban_gnss_risk:
            covariance = max(2.0, covariance)
            gate = min(0.70, gate)
        state = self._state(record)
        hard_reject = state is ReliabilityState.REJECTED
        return UpdatePolicy(source, covariance, gate, hard_reject, record.reason)

    def _note_consistent(self, source: str, urban_gnss_risk: bool) -> None:
        record = self._record(source)
        if self._state(record) is ReliabilityState.REJECTED or record.recovering:
            record.recovery_count += 1
            if record.recovery_count >= self.recovery_required:
                record.recovering = True
                record.score = max(record.score, 0.32)
                record.reason = "Восстановление по серии согласованных данных"
            else:
                record.reason = f"Проверка восстановления: {record.recovery_count}/{self.recovery_required}"
            return
        record.score = min(0.98, record.score + 0.015)
        if source == "gnss" and urban_gnss_risk:
            record.reason = "Плотная городская застройка: GNSS проверяется строже"
        else:
            record.reason = "Данные согласованы"

    @staticmethod
    def _consensus(points: Iterable[np.ndarray]) -> np.ndarray:
        return np.median(np.stack(tuple(points)), axis=0)

    @staticmethod
    def _group_positions(measurements: Sequence[PoseMeasurement]) -> dict[str, np.ndarray]:
        grouped: dict[str, list[tuple[float, float]]] = {}
        for measurement in measurements:
            grouped.setdefault(measurement.source, []).append((measurement.x, measurement.y))
        return {
            source: np.mean(np.asarray(values, dtype=float), axis=0)
            for source, values in grouped.items()
        }

    def _source_tolerances(self, measurements: Sequence[PoseMeasurement]) -> dict[str, float]:
        tolerances: dict[str, float] = {}
        for measurement in measurements:
            covariance = measurement.covariance
            if len(covariance) >= 9:
                position_variance = max(float(covariance[0]), float(covariance[4]))
            elif len(covariance) >= 4:
                position_variance = max(float(covariance[0]), float(covariance[3]))
            else:
                position_variance = 0.0
            tolerance = max(self.consistency_distance, 3.0 * math.sqrt(max(0.0, position_variance)))
            tolerances[measurement.source] = max(tolerances.get(measurement.source, 0.0), tolerance)
        return tolerances

    def _record(self, source: str) -> _SourceRecord:
        if source not in self._records:
            self._records[source] = _SourceRecord()
        return self._records[source]

    @staticmethod
    def _state(record: _SourceRecord) -> ReliabilityState:
        if record.recovering:
            return ReliabilityState.RECOVERING
        if record.score >= 0.78:
            return ReliabilityState.NORMAL
        if record.score >= 0.52:
            return ReliabilityState.SUSPECT
        if record.score >= 0.28:
            return ReliabilityState.DEGRADED
        return ReliabilityState.REJECTED
