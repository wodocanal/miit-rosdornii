"""Fixed-step experiment orchestration, evaluation and presentation frames."""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np

from .ekf import FilterSnapshot, NavigationEKF, UpdateDecision
from .measurements import PoseMeasurement, SensorPacket
from .models import Control, Vec2, WorldSnapshot
from .reliability import ReliabilityManager, ReliabilitySnapshot
from .scenario import ExperimentEvent
from .sensors import FaultControls, SensorSuite
from .world import WorldModel, wrap_angle


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class NavigationMode(str, Enum):
    """What the dispatcher is allowed to receive from the vehicle."""

    GNSS_ONLY = "gnss_only"
    FUSION = "fusion"


@dataclass(frozen=True)
class DispatcherState:
    mode: NavigationMode
    pose: Pose2D | None
    source: str
    status: str
    fresh: bool
    age: float | None
    covariance: tuple[float, float, float] | None


@dataclass(frozen=True)
class MetricSnapshot:
    current_error: float
    rmse: float
    p95: float
    maximum: float
    heading_error_deg: float
    samples: int


@dataclass(frozen=True)
class TrackPoint:
    sim_time: float
    truth: Pose2D | None
    estimate: Pose2D
    odometry: Pose2D
    gnss: tuple[float, float] | None
    camera: tuple[float, float] | None
    dispatcher: Pose2D | None
    dispatcher_error: float | None
    position_error: float | None
    heading_error_deg: float | None


@dataclass(frozen=True)
class Frame:
    sim_time: float
    paused: bool
    speed_multiplier: int
    truth: WorldSnapshot | None
    estimate: FilterSnapshot
    dispatcher: DispatcherState
    odometry: Pose2D
    packet: SensorPacket
    decisions: tuple[UpdateDecision, ...]
    metrics: MetricSnapshot | None
    dispatcher_metrics: MetricSnapshot | None
    events: tuple[ExperimentEvent, ...]
    active_effects: tuple[str, ...]
    reliability: tuple[ReliabilitySnapshot, ...]
    urban_gnss_risk: bool
    history: tuple[TrackPoint, ...]


class ExperimentEngine:
    DT = 0.02

    def __init__(
        self,
        seed: int = 516,
        truth_available: bool = True,
        world_path: str | Path | None = None,
        navigation_mode: NavigationMode | str = NavigationMode.FUSION,
    ):
        self.seed = seed
        self.truth_available = truth_available
        self.world_path = Path(world_path) if world_path else Path(__file__).parents[1] / "config" / "world_fields.json"
        self.navigation_mode = NavigationMode(navigation_mode)
        self.paused = False
        self.speed_multiplier = 1
        self._wall_accumulator = 0.0
        self._control = Control(auto=True)
        self.reset()

    @property
    def sensors(self) -> SensorSuite:
        return self._sensors

    @property
    def world(self) -> WorldModel:
        return self._world

    @property
    def records(self) -> tuple[dict[str, object], ...]:
        return tuple(self._records)

    def reset(self, faults=None, beacon_positions: dict[str, tuple[float, float]] | None = None) -> Frame:
        self._world = WorldModel.from_json(self.world_path)
        for beacon_id, position in (beacon_positions or {}).items():
            self._world.move_beacon(beacon_id, *position)
        truth = self._world.reset()
        self._sensors = SensorSuite(self.seed)
        if faults is not None:
            self._sensors.faults = deepcopy(faults)
        self._apply_mode_source_defaults()
        self._ekf = NavigationEKF(initial_x=truth.x, initial_y=truth.y, initial_yaw=truth.yaw)
        self._reliability = ReliabilityManager()
        self._pending_external: list[PoseMeasurement] = []
        self._pending_gnss_dispatch: list[PoseMeasurement] = []
        self._events: list[ExperimentEvent] = []
        self._errors: list[float] = []
        self._error_sumsq = 0.0
        self._error_max = 0.0
        self._p95_cache = 0.0
        self._dispatcher_errors: list[float] = []
        self._dispatcher_error_sumsq = 0.0
        self._dispatcher_error_max = 0.0
        self._dispatcher_p95_cache = 0.0
        self._history: list[TrackPoint] = []
        self._step_index = 0
        self._records: list[dict[str, object]] = []
        self._odom = Pose2D(truth.x, truth.y, truth.yaw)
        self._wall_accumulator = 0.0
        empty_packet = self._sensors.sample(truth, self._world)
        estimate = self._ekf.snapshot()
        self._last_gnss_dispatch_pose: Pose2D | None = None
        self._last_gnss_dispatch_time: float | None = None
        dispatcher = self._dispatcher_state(truth.sim_time, empty_packet, estimate)
        self._last_frame = Frame(
            sim_time=0.0,
            paused=self.paused,
            speed_multiplier=self.speed_multiplier,
            truth=truth if self.truth_available else None,
            estimate=estimate,
            dispatcher=dispatcher,
            odometry=self._odom,
            packet=empty_packet,
            decisions=(),
            metrics=self._metrics(truth, estimate) if self.truth_available else None,
            dispatcher_metrics=self._dispatcher_metrics(truth, dispatcher) if self.truth_available else None,
            events=(),
            active_effects=(),
            reliability=self._reliability.snapshots(0.0),
            urban_gnss_risk=self._world.kind == "city",
            history=(),
        )
        return self._last_frame

    def set_navigation_mode(self, mode: NavigationMode | str) -> Frame:
        """Switch dispatcher behaviour and restart the deterministic experiment."""
        beacon_positions = {item.id: (item.x, item.y) for item in self._world.beacons}
        self.navigation_mode = NavigationMode(mode)
        return self.reset(faults=FaultControls(), beacon_positions=beacon_positions)

    def set_world(self, world_path: str | Path) -> Frame:
        path = Path(world_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        self.world_path = path
        return self.reset(faults=FaultControls())

    def _apply_mode_source_defaults(self) -> None:
        fusion = self.navigation_mode is NavigationMode.FUSION
        faults = self._sensors.faults
        faults.gnss.enabled = True
        faults.camera.enabled = fusion
        faults.motion.odom_enabled = fusion
        faults.motion.imu_enabled = fusion
        faults.uwb.enabled = fusion

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    def set_speed_multiplier(self, multiplier: int) -> None:
        if multiplier not in (1, 2, 5):
            raise ValueError("speed multiplier must be 1, 2 or 5")
        self.speed_multiplier = multiplier

    def advance_wall_time(self, wall_seconds: float) -> Frame:
        if self.paused or wall_seconds <= 0.0:
            return self.frame()
        self._wall_accumulator += wall_seconds * self.speed_multiplier
        while self._wall_accumulator + 1e-12 >= self.DT:
            self.step()
            self._wall_accumulator -= self.DT
        return self.frame()

    def step(self) -> Frame:
        if self.paused:
            return self.frame()
        truth = self._world.step(self.DT, self._control)
        packet = self._sensors.sample(truth, self._world)
        self._reliability.mark_availability("local_motion", packet.motion is not None, truth.sim_time)
        self._reliability.mark_availability("gnss", packet.gnss is not None, truth.sim_time)
        self._reliability.mark_availability("camera", bool(packet.camera), truth.sim_time)
        self._reliability.mark_availability("uwb", packet.uwb_pose is not None, truth.sim_time)
        if packet.motion is not None:
            estimate = self._ekf.predict(
                packet.motion,
                self.DT,
                process_noise_multiplier=self._reliability.process_noise_multiplier,
            )
            yaw = wrap_angle(self._odom.yaw + packet.motion.yaw_rate * self.DT)
            self._odom = Pose2D(
                self._odom.x + packet.motion.speed * math.cos(self._odom.yaw) * self.DT,
                self._odom.y + packet.motion.speed * math.sin(self._odom.yaw) * self.DT,
                yaw,
            )
        else:
            estimate = self._ekf.predict_without_motion(
                self.DT,
                process_noise_multiplier=max(2.0, self._reliability.process_noise_multiplier),
            )
        if packet.gnss is not None:
            self._pending_external.append(packet.gnss)
        self._pending_external.extend(item.pose for item in packet.camera)
        if packet.uwb_pose is not None:
            self._pending_external.append(packet.uwb_pose)
        ready = [item for item in self._pending_external if item.received_at <= truth.sim_time + 1e-9]
        self._pending_external = [item for item in self._pending_external if item.received_at > truth.sim_time + 1e-9]
        urban_gnss_risk = self._world.gnss_risk_at(Vec2(estimate.x, estimate.y))
        policies = self._reliability.assess_batch(
            ready,
            predicted_xy=(estimate.x, estimate.y),
            now=truth.sim_time,
            urban_gnss_risk=urban_gnss_risk,
        )
        decisions: list[UpdateDecision] = []
        for measurement in sorted(ready, key=lambda item: (item.received_at, item.measured_at)):
            policy = policies[measurement.source]
            if policy.hard_reject:
                decision = UpdateDecision(
                    source=measurement.source,
                    measured_at=measurement.measured_at,
                    accepted=False,
                    reason="Отклонено: низкое доверие, проверка восстановления",
                    mahalanobis=None,
                    correction_m=0.0,
                    recovery_count=self._reliability.snapshot(measurement.source, truth.sim_time).recovery_count,
                    innovation_m=math.hypot(measurement.x - estimate.x, measurement.y - estimate.y),
                    covariance_multiplier=policy.covariance_multiplier,
                    gate_threshold=0.0,
                )
            else:
                decision = self._ekf.update_pose(
                    measurement,
                    truth.sim_time,
                    covariance_multiplier=policy.covariance_multiplier,
                    gate_multiplier=policy.gate_multiplier,
                )
            decisions.append(decision)
            # A policy hold is already accounted for by assess_batch().  Feeding
            # it back as a fresh sensor failure would reset the recovery series.
            if not policy.hard_reject:
                self._reliability.observe_decision(decision, truth.sim_time)
            if not decision.accepted or decision.correction_m > 0.5:
                self._events.append(
                    ExperimentEvent(
                        truth.sim_time,
                        "algorithm",
                        f"{measurement.source}: {decision.reason}; поправка {decision.correction_m:.2f} м",
                    )
                )
        estimate = self._ekf.snapshot()
        dispatcher = self._dispatcher_state(truth.sim_time, packet, estimate)
        metrics = self._metrics(truth, estimate) if self.truth_available else None
        dispatcher_metrics = self._dispatcher_metrics(truth, dispatcher) if self.truth_available else None
        point = self._track_point(truth, estimate, packet, dispatcher, metrics)
        self._step_index += 1
        if self._step_index % 5 == 0:
            self._history.append(point)
            display_history = self._history
        else:
            display_history = [*self._history, point]
        self._append_record(truth, estimate, dispatcher, packet, decisions, metrics, dispatcher_metrics)
        active: set[str] = set()
        if not self._sensors.faults.gnss.enabled:
            active.add("Полное глушение GNSS")
        elif self._sensors.faults.gnss.dropout >= 0.5:
            active.add("Частичное глушение GNSS")
        if abs(self._sensors.faults.gnss.jump_x) + abs(self._sensors.faults.gnss.jump_y) > 0.0:
            active.add("Подмена координат GNSS")
        if truth.in_tunnel:
            active.add("GNSS недоступен в тоннеле")
        if truth.in_gnss_zone:
            active.add("GNSS ухудшен городской застройкой")
        self._last_frame = Frame(
            sim_time=truth.sim_time,
            paused=self.paused,
            speed_multiplier=self.speed_multiplier,
            truth=truth if self.truth_available else None,
            estimate=estimate,
            dispatcher=dispatcher,
            odometry=self._odom,
            packet=packet,
            decisions=tuple(decisions),
            metrics=metrics,
            dispatcher_metrics=dispatcher_metrics,
            events=tuple(self._events[-100:]),
            active_effects=tuple(sorted(active)),
            reliability=self._reliability.snapshots(truth.sim_time),
            urban_gnss_risk=urban_gnss_risk,
            history=tuple(display_history),
        )
        return self._last_frame

    def frame(self) -> Frame:
        if self._last_frame.paused != self.paused or self._last_frame.speed_multiplier != self.speed_multiplier:
            self._last_frame = Frame(**{
                **self._last_frame.__dict__,
                "paused": self.paused,
                "speed_multiplier": self.speed_multiplier,
            })
        return self._last_frame

    def _dispatcher_state(
        self,
        sim_time: float,
        packet: SensorPacket,
        estimate: FilterSnapshot,
    ) -> DispatcherState:
        if self.navigation_mode is NavigationMode.FUSION:
            covariance = estimate.covariance
            external = (
                self._reliability.snapshot(source, sim_time)
                for source in ("gnss", "camera", "uwb")
            )
            external_lost = all(
                not item.available
                and (
                    (item.age is None and sim_time > 1.0)
                    or (item.age is not None and item.age > 1.0)
                )
                for item in external
            )
            sigma = math.sqrt(max(float(covariance[0, 0]), float(covariance[1, 1])))
            if external_lost:
                status = "Нет внешней привязки"
            elif sigma > 4.0:
                status = "Низкая точность"
            else:
                status = "Позиция передаётся"
            return DispatcherState(
                self.navigation_mode,
                Pose2D(estimate.x, estimate.y, estimate.yaw),
                "Устойчивое слияние",
                status,
                not external_lost,
                0.0,
                (float(covariance[0, 0]), float(covariance[0, 1]), float(covariance[1, 1])),
            )

        if packet.gnss is not None:
            self._pending_gnss_dispatch.append(packet.gnss)
        ready = [item for item in self._pending_gnss_dispatch if item.received_at <= sim_time + 1e-9]
        self._pending_gnss_dispatch = [item for item in self._pending_gnss_dispatch if item.received_at > sim_time + 1e-9]
        if ready:
            latest = max(ready, key=lambda item: (item.received_at, item.measured_at))
            self._last_gnss_dispatch_pose = Pose2D(latest.x, latest.y, estimate.yaw)
            self._last_gnss_dispatch_time = latest.received_at
        age = None if self._last_gnss_dispatch_time is None else max(0.0, sim_time - self._last_gnss_dispatch_time)
        if self._last_gnss_dispatch_pose is None or age is None or age > 2.0:
            return DispatcherState(
                self.navigation_mode, None, "GNSS без проверки", "Нет координат GNSS", False, age, None
            )
        fresh = bool(ready)
        return DispatcherState(
            self.navigation_mode,
            self._last_gnss_dispatch_pose,
            "GNSS без проверки",
            "Точка GNSS получена" if fresh else "Ожидание GNSS",
            fresh,
            age,
            None,
        )

    def _metrics(self, truth: WorldSnapshot, estimate: FilterSnapshot) -> MetricSnapshot:
        error = math.hypot(estimate.x - truth.x, estimate.y - truth.y)
        self._errors.append(error)
        self._error_sumsq += error * error
        self._error_max = max(self._error_max, error)
        if len(self._errors) <= 10 or len(self._errors) % 10 == 0:
            self._p95_cache = float(np.percentile(self._errors, 95))
        heading = abs(math.degrees(wrap_angle(estimate.yaw - truth.yaw)))
        return MetricSnapshot(
            error,
            math.sqrt(self._error_sumsq / len(self._errors)),
            self._p95_cache,
            self._error_max,
            heading,
            len(self._errors),
        )

    def _dispatcher_metrics(
        self, truth: WorldSnapshot, dispatcher: DispatcherState
    ) -> MetricSnapshot | None:
        if dispatcher.pose is None:
            return None
        error = math.hypot(dispatcher.pose.x - truth.x, dispatcher.pose.y - truth.y)
        self._dispatcher_errors.append(error)
        self._dispatcher_error_sumsq += error * error
        self._dispatcher_error_max = max(self._dispatcher_error_max, error)
        if len(self._dispatcher_errors) <= 10 or len(self._dispatcher_errors) % 10 == 0:
            self._dispatcher_p95_cache = float(np.percentile(self._dispatcher_errors, 95))
        heading = abs(math.degrees(wrap_angle(dispatcher.pose.yaw - truth.yaw)))
        return MetricSnapshot(
            error,
            math.sqrt(self._dispatcher_error_sumsq / len(self._dispatcher_errors)),
            self._dispatcher_p95_cache,
            self._dispatcher_error_max,
            heading,
            len(self._dispatcher_errors),
        )

    def _track_point(
        self,
        truth: WorldSnapshot,
        estimate: FilterSnapshot,
        packet: SensorPacket,
        dispatcher: DispatcherState,
        metrics: MetricSnapshot | None,
    ) -> TrackPoint:
        truth_pose = Pose2D(truth.x, truth.y, truth.yaw) if self.truth_available else None
        gnss = (packet.gnss.x, packet.gnss.y) if packet.gnss else None
        camera = (packet.camera[0].pose.x, packet.camera[0].pose.y) if packet.camera else None
        dispatcher_error = (
            math.hypot(dispatcher.pose.x - truth.x, dispatcher.pose.y - truth.y)
            if self.truth_available and dispatcher.pose is not None
            else None
        )
        return TrackPoint(
            truth.sim_time,
            truth_pose,
            Pose2D(estimate.x, estimate.y, estimate.yaw),
            self._odom,
            gnss,
            camera,
            dispatcher.pose,
            dispatcher_error,
            metrics.current_error if metrics else None,
            metrics.heading_error_deg if metrics else None,
        )

    def _append_record(
        self,
        truth: WorldSnapshot,
        estimate: FilterSnapshot,
        dispatcher: DispatcherState,
        packet: SensorPacket,
        decisions: list[UpdateDecision],
        metrics: MetricSnapshot | None,
        dispatcher_metrics: MetricSnapshot | None,
    ) -> None:
        gnss = packet.gnss
        self._records.append(
            {
                "sim_time": truth.sim_time,
                "truth_x": truth.x if self.truth_available else "",
                "truth_y": truth.y if self.truth_available else "",
                "truth_yaw": truth.yaw if self.truth_available else "",
                "estimate_x": estimate.x,
                "estimate_y": estimate.y,
                "estimate_yaw": estimate.yaw,
                "estimate_speed": estimate.speed,
                "navigation_mode": self.navigation_mode.value,
                "dispatcher_x": dispatcher.pose.x if dispatcher.pose else "",
                "dispatcher_y": dispatcher.pose.y if dispatcher.pose else "",
                "dispatcher_source": dispatcher.source,
                "cov_xx": float(estimate.covariance[0, 0]),
                "cov_xy": float(estimate.covariance[0, 1]),
                "cov_yy": float(estimate.covariance[1, 1]),
                "odom_x": self._odom.x,
                "odom_y": self._odom.y,
                "gnss_x": gnss.x if gnss else "",
                "gnss_y": gnss.y if gnss else "",
                "gnss_measured_at": gnss.measured_at if gnss else "",
                "gnss_received_at": gnss.received_at if gnss else "",
                "camera_ids": ";".join(str(item.marker_id) for item in packet.camera),
                "position_error": dispatcher_metrics.current_error if dispatcher_metrics else "",
                "filter_position_error": metrics.current_error if metrics else "",
                "decisions": "; ".join(
                    f"{item.source}:{'accepted' if item.accepted else 'rejected'}:{item.reason}"
                    for item in decisions
                ),
            }
        )
