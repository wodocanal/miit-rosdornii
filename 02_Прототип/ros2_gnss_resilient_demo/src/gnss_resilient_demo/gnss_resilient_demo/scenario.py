"""Timed test effects kept outside the navigation algorithm."""

from __future__ import annotations

from dataclasses import dataclass

from .sensors import SensorSuite


@dataclass(frozen=True)
class ExperimentEvent:
    sim_time: float
    category: str
    text: str


@dataclass(frozen=True)
class ScenarioAction:
    at: float
    name: str
    values: tuple[float, ...] = ()


class MainScenario:
    """A repeatable sequence of injected effects; flags never reach the EKF."""

    ACTIONS = (
        ScenarioAction(0.02, "start"),
        ScenarioAction(10.0, "gnss_off"),
        ScenarioAction(20.0, "gnss_on"),
        ScenarioAction(24.0, "hide_some_markers"),
        ScenarioAction(30.0, "gnss_jump", (25.0, -18.0)),
        ScenarioAction(42.0, "gnss_jump_clear"),
        ScenarioAction(55.0, "camera_off"),
        ScenarioAction(70.0, "camera_on"),
        ScenarioAction(90.0, "gnss_ramp", (0.18, 0.08)),
        ScenarioAction(120.0, "gnss_ramp_clear"),
    )

    def __init__(self):
        self.enabled = False
        self.reset()

    def reset(self) -> None:
        self._index = 0
        self.active_effects: set[str] = set()

    def clear_automatic_effects(self, sensors: SensorSuite) -> None:
        sensors.faults.gnss.automatic_enabled = True
        sensors.faults.gnss.automatic_jump_x = 0.0
        sensors.faults.gnss.automatic_jump_y = 0.0
        sensors.faults.gnss.automatic_ramp_rate_x = 0.0
        sensors.faults.gnss.automatic_ramp_rate_y = 0.0
        sensors.faults.camera.automatic_enabled = True
        sensors.faults.camera.automatic_hide_all = False
        sensors.faults.camera.automatic_hidden_ids.clear()
        self.active_effects.clear()

    def advance(self, now: float, sensors: SensorSuite) -> tuple[ExperimentEvent, ...]:
        if not self.enabled:
            return ()
        events: list[ExperimentEvent] = []
        while self._index < len(self.ACTIONS) and self.ACTIONS[self._index].at <= now + 1e-9:
            action = self.ACTIONS[self._index]
            events.append(self._apply(action, sensors))
            self._index += 1
        return tuple(events)

    def _apply(self, action: ScenarioAction, sensors: SensorSuite) -> ExperimentEvent:
        gnss, camera = sensors.faults.gnss, sensors.faults.camera
        messages = {
            "start": "Сценарий защиты запущен",
            "gnss_off": "Введено воздействие: GNSS отключён",
            "gnss_on": "Воздействие снято: GNSS снова доступен",
            "hide_some_markers": "Введено воздействие: часть ориентиров скрыта",
            "gnss_jump": "Введено воздействие: ложный скачок GNSS",
            "gnss_jump_clear": "Воздействие снято: скачок GNSS",
            "camera_off": "Введено воздействие: все ориентиры камеры отключены",
            "camera_on": "Воздействие снято: камера снова доступна",
            "gnss_ramp": "Введено воздействие: плавное ложное смещение GNSS",
            "gnss_ramp_clear": "Воздействие снято: плавное смещение GNSS",
        }
        if action.name == "gnss_off":
            gnss.automatic_enabled = False
            self.active_effects.add("GNSS отключён сценарием")
        elif action.name == "gnss_on":
            gnss.automatic_enabled = True
            self.active_effects.discard("GNSS отключён сценарием")
        elif action.name == "hide_some_markers":
            camera.automatic_hidden_ids.update({21, 31})
            self.active_effects.add("Часть ориентиров скрыта")
        elif action.name == "gnss_jump":
            gnss.automatic_jump_x, gnss.automatic_jump_y = action.values
            self.active_effects.add("Скачок GNSS")
        elif action.name == "gnss_jump_clear":
            gnss.automatic_jump_x = gnss.automatic_jump_y = 0.0
            self.active_effects.discard("Скачок GNSS")
        elif action.name == "camera_off":
            camera.automatic_enabled = False
            self.active_effects.add("Камера отключена сценарием")
        elif action.name == "camera_on":
            camera.automatic_enabled = True
            self.active_effects.discard("Камера отключена сценарием")
        elif action.name == "gnss_ramp":
            gnss.automatic_ramp_rate_x, gnss.automatic_ramp_rate_y = action.values
            self.active_effects.add("Плавный спуфинг GNSS")
        elif action.name == "gnss_ramp_clear":
            gnss.automatic_ramp_rate_x = gnss.automatic_ramp_rate_y = 0.0
            self.active_effects.discard("Плавный спуфинг GNSS")
        return ExperimentEvent(action.at, "test", messages[action.name])
