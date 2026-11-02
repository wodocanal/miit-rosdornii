"""Two synchronized Russian-language Qt operator windows."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .engine import ExperimentEngine, Frame, NavigationMode
from .style import APP_STYLESHEET, COLORS
from .widgets import ErrorPlot, NavigationCanvas, WorldCanvas

_APP_HOLDER = None


class RangeWindow(QMainWindow):
    def __init__(self, engine: ExperimentEngine):
        super().__init__()
        self.engine = engine
        self.setWindowTitle("Полигон — испытатель")
        self.resize(960, 720)
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        toolbar = QHBoxLayout()
        title = QLabel("ПОЛИГОН / МОДЕЛЬ МИРА")
        title.setObjectName("kicker")
        toolbar.addWidget(title)
        toolbar.addStretch(1)
        self.clock_label = QLabel("T+000.0 с")
        self.clock_label.setObjectName("value")
        toolbar.addWidget(self.clock_label)
        layout.addLayout(toolbar)
        selectors = QHBoxLayout()
        selectors.addWidget(QLabel("Режим"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Обычный: GNSS", NavigationMode.GNSS_ONLY.value)
        self.mode_combo.addItem("Наше решение", NavigationMode.FUSION.value)
        self.mode_combo.setCurrentIndex(1 if engine.navigation_mode is NavigationMode.FUSION else 0)
        self.mode_combo.setMinimumWidth(230)
        selectors.addWidget(self.mode_combo)
        selectors.addWidget(QLabel("Карта"))
        self.map_combo = QComboBox()
        map_dir = Path(__file__).parents[1] / "config"
        for filename, title_text in (
            ("world_city.json", "Плотный город"),
            ("world_fields.json", "Поля и посёлок"),
            ("world_forest.json", "Лесная дорога"),
            ("world_tunnels.json", "Тоннели и развязка"),
        ):
            self.map_combo.addItem(title_text, str(map_dir / filename))
        current_path = str(engine.world_path.resolve())
        for index in range(self.map_combo.count()):
            if str(Path(self.map_combo.itemData(index)).resolve()) == current_path:
                self.map_combo.setCurrentIndex(index)
                break
        self.map_combo.setMinimumWidth(220)
        selectors.addWidget(self.map_combo)
        self.coverage_button = QPushButton("◉ Зоны UWB")
        self.coverage_button.setCheckable(True)
        self.coverage_button.setToolTip("Показать круги максимальной дальности каждого радиомаяка")
        selectors.addWidget(self.coverage_button)
        selectors.addStretch(1)
        layout.addLayout(selectors)

        controls = QHBoxLayout()
        self.start_button = QPushButton("▶ Старт")
        self.pause_button = QPushButton("Ⅱ Пауза")
        self.pause_button.setCheckable(True)
        self.reset_button = QPushButton("↺ Сброс")
        controls.addWidget(self.start_button)
        controls.addWidget(self.pause_button)
        controls.addWidget(self.reset_button)
        controls.addWidget(QLabel("Скорость"))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["×1", "×2", "×5"])
        controls.addWidget(self.speed_combo)
        controls.addStretch(1)
        layout.addLayout(controls)

        content = QSplitter(Qt.Horizontal)
        self.canvas = WorldCanvas(engine)
        self.coverage_button.toggled.connect(self.canvas.set_uwb_coverage)
        content.addWidget(self.canvas)
        content.addWidget(self._build_fault_panel())
        content.setSizes([650, 360])
        layout.addWidget(content, 1)
        self.effects_label = QLabel("Воздействия: нет")
        self.effects_label.setObjectName("muted")
        layout.addWidget(self.effects_label)
        self.setCentralWidget(root)
        self.apply_mode_visibility(engine.navigation_mode)

    def _build_fault_panel(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.gnss_group = self._gnss_group()
        self.camera_group = self._camera_group()
        self.motion_group = self._motion_group()
        self.uwb_group = self._uwb_group()
        layout.addWidget(self.gnss_group)
        layout.addWidget(self.camera_group)
        layout.addWidget(self.motion_group)
        layout.addWidget(self.uwb_group)
        note = QLabel("Упрощённая модель доступности и ошибок; физика распространения не моделируется.")
        note.setWordWrap(True)
        note.setObjectName("muted")
        layout.addWidget(note)
        layout.addStretch(1)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(280)
        return scroll

    def _gnss_group(self):
        faults = self.engine.sensors.faults.gnss
        group = QGroupBox("GNSS")
        layout = QVBoxLayout(group)
        self.gnss_enabled = QCheckBox("GNSS включён")
        self.gnss_enabled.setChecked(faults.enabled)
        self.gnss_partial_jam = QCheckBox("Частичное глушение")
        self.gnss_full_jam = QCheckBox("Полное глушение")
        self.gnss_spoof = QCheckBox("Подмена координат")
        self.gnss_jump = self.gnss_spoof
        # Availability is controlled by the unambiguous "full jamming" switch.
        # Keep this compatibility control for tests/adapters, but never expose it.
        self.gnss_enabled.hide()
        layout.addWidget(self.gnss_partial_jam)
        layout.addWidget(self.gnss_full_jam)
        layout.addWidget(self.gnss_spoof)
        self.gnss_enabled.toggled.connect(lambda value: setattr(self.engine.sensors.faults.gnss, "enabled", value))
        self.gnss_partial_jam.toggled.connect(self._set_gnss_partial_jam)
        self.gnss_full_jam.toggled.connect(self._set_gnss_full_jam)
        self.gnss_spoof.toggled.connect(lambda value: self._set_gnss_jump(value))
        return group

    def _set_gnss_partial_jam(self, enabled):
        faults = self.engine.sensors.faults.gnss
        faults.dropout = 0.65 if enabled else 0.0
        faults.noise_std = 3.0 if enabled else 0.7

    def _set_gnss_full_jam(self, enabled):
        self.engine.sensors.faults.gnss.enabled = not enabled
        self.gnss_enabled.blockSignals(True)
        self.gnss_enabled.setChecked(not enabled)
        self.gnss_enabled.blockSignals(False)

    def _set_gnss_jump(self, enabled):
        faults = self.engine.sensors.faults.gnss
        faults.jump_x, faults.jump_y = ((25.0, -18.0) if enabled else (0.0, 0.0))

    def _camera_group(self):
        faults = self.engine.sensors.faults.camera
        group = QGroupBox("Камера и ориентиры")
        layout = QVBoxLayout(group)
        self.camera_enabled = QCheckBox("Камера включена")
        self.camera_enabled.setChecked(faults.enabled)
        self.camera_degraded = QCheckBox("Помехи камере")
        self.camera_hide_all = QCheckBox("Скрыть ориентиры")
        self.camera_hide_all.setToolTip("Ручное воздействие: камера перестаёт получать наблюдения всех контрольных меток")
        self.camera_status_label = QLabel("Жёлтая метка — ориентир распознан сейчас")
        self.camera_status_label.setWordWrap(True)
        self.camera_status_label.setObjectName("muted")
        layout.addWidget(self.camera_enabled)
        layout.addWidget(self.camera_degraded)
        layout.addWidget(self.camera_hide_all)
        layout.addWidget(self.camera_status_label)
        self.camera_enabled.toggled.connect(lambda value: setattr(self.engine.sensors.faults.camera, "enabled", value))
        self.camera_degraded.toggled.connect(lambda value: setattr(self.engine.sensors.faults.camera, "quality", 0.3 if value else 1.0))
        self.camera_hide_all.toggled.connect(lambda value: setattr(self.engine.sensors.faults.camera, "hide_all", value))
        return group

    def _motion_group(self):
        faults = self.engine.sensors.faults.motion
        group = QGroupBox("Локальное движение")
        layout = QGridLayout(group)
        self.odom_enabled = QCheckBox("Одометрия")
        self.odom_enabled.setChecked(True)
        self.imu_enabled = QCheckBox("IMU")
        self.imu_enabled.setChecked(True)
        self.wheel_slip = QCheckBox("Проскальзывание колёс")
        self.gyro_drift = QCheckBox("Смещение курса IMU")
        layout.addWidget(self.odom_enabled, 0, 0); layout.addWidget(self.imu_enabled, 0, 1)
        layout.addWidget(self.wheel_slip, 1, 0, 1, 2)
        layout.addWidget(self.gyro_drift, 2, 0, 1, 2)
        self.odom_enabled.toggled.connect(lambda value: setattr(self.engine.sensors.faults.motion, "odom_enabled", value))
        self.imu_enabled.toggled.connect(lambda value: setattr(self.engine.sensors.faults.motion, "imu_enabled", value))
        self.wheel_slip.toggled.connect(
            lambda value: setattr(self.engine.sensors.faults.motion, "slip", 0.45 if value else 0.0)
        )
        self.gyro_drift.toggled.connect(
            lambda value: setattr(self.engine.sensors.faults.motion, "gyro_bias", 0.08 if value else 0.0)
        )
        return group

    def _uwb_group(self):
        faults = self.engine.sensors.faults.uwb
        group = QGroupBox("Радиомаяки UWB")
        layout = QGridLayout(group)
        self.uwb_enabled = QCheckBox("Использовать маяки")
        self.uwb_enabled.setChecked(faults.enabled)
        self.uwb_nlos = QCheckBox("NLOS: +2 м к дальности")
        self.uwb_nlos.setToolTip(
            "Добавить ко всем дальностям положительную ошибку 2 м — простая модель сигнала без прямой видимости."
        )
        self.uwb_links_label = QLabel(
            "Линия = свежее измерение дальности внутри круга покрытия. Для позиции нужны 3+ линии."
        )
        self.uwb_links_label.setWordWrap(True)
        self.uwb_links_label.setObjectName("muted")
        layout.addWidget(self.uwb_enabled, 0, 0, 1, 2)
        layout.addWidget(self.uwb_nlos, 1, 0, 1, 2)
        layout.addWidget(self.uwb_links_label, 2, 0, 1, 2)
        self.uwb_enabled.toggled.connect(lambda value: setattr(self.engine.sensors.faults.uwb, "enabled", value))
        self.uwb_nlos.toggled.connect(lambda value: setattr(self.engine.sensors.faults.uwb, "nlos_bias", 2.0 if value else 0.0))
        return group

    def update_frame(self, frame: Frame):
        self.clock_label.setText(f"T+{frame.sim_time:05.1f} с")
        self.canvas.set_frame(frame)
        recognized = self.canvas.recognized_marker_ids()
        marker_text = ", ".join(str(marker_id) for marker_id in sorted(recognized)) or "нет"
        self.camera_status_label.setText(
            f"Распознано: {marker_text}. Жёлтая метка и линия — активное наблюдение."
        )
        links = len(self.canvas.active_beacon_ids())
        position_state = "позиция вычисляется" if links >= 3 else "для позиции нужно минимум 3"
        self.uwb_links_label.setText(
            f"Связей: {links} — {position_state}. Линия есть только внутри зоны покрытия."
        )
        self.effects_label.setText("Воздействия: " + (", ".join(frame.active_effects) or "нет"))

    def apply_mode_visibility(self, mode: NavigationMode) -> None:
        fusion = mode is NavigationMode.FUSION
        self.camera_group.setVisible(fusion)
        self.motion_group.setVisible(fusion)
        self.uwb_group.setVisible(fusion)
        self.coverage_button.setVisible(fusion)
        if not fusion:
            self.coverage_button.setChecked(False)
        self._sync_source_controls()

    def _sync_source_controls(self) -> None:
        faults = self.engine.sensors.faults
        for widget, value in (
            (self.gnss_enabled, faults.gnss.enabled),
            (self.gnss_partial_jam, faults.gnss.dropout >= 0.5),
            (self.gnss_full_jam, not faults.gnss.enabled),
            (self.gnss_spoof, abs(faults.gnss.jump_x) + abs(faults.gnss.jump_y) > 0.0),
            (self.camera_enabled, faults.camera.enabled),
            (self.odom_enabled, faults.motion.odom_enabled),
            (self.imu_enabled, faults.motion.imu_enabled),
            (self.wheel_slip, faults.motion.slip > 0.0),
            (self.gyro_drift, abs(faults.motion.gyro_bias) > 0.0),
            (self.uwb_enabled, faults.uwb.enabled),
        ):
            widget.blockSignals(True)
            widget.setChecked(value)
            widget.blockSignals(False)

class NavigationWindow(QMainWindow):
    SOURCE_NAMES = ("Одо. + IMU", "GNSS", "Камера", "UWB")
    TRUST_SOURCES = (
        ("local_motion", "ОДОМ. + IMU"),
        ("gnss", "GNSS"),
        ("camera", "КАМЕРА"),
        ("uwb", "UWB"),
    )

    def __init__(self, engine: ExperimentEngine):
        super().__init__()
        self.engine = engine
        self._last_decisions = {}
        self.setWindowTitle("Диспетчер — транспорт на карте")
        self.resize(980, 720)
        root = QWidget()
        layout = QVBoxLayout(root)
        header = QGridLayout()
        label = QLabel("ДИСПЕТЧЕР")
        label.setObjectName("kicker")
        label.setWordWrap(True)
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        header.addWidget(label, 0, 0, 1, 2)
        self.mode_label = QLabel("НАШЕ РЕШЕНИЕ")
        self.mode_label.setObjectName("kicker")
        self.mode_label.setWordWrap(True)
        self.mode_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        header.addWidget(self.mode_label, 1, 0, 1, 2)
        self.quality_label = QLabel("КООРДИНАТЫ ПЕРЕДАЮТСЯ")
        self.quality_label.setObjectName("value")
        self.quality_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.quality_label.setWordWrap(True)
        self.quality_label.setMaximumWidth(360)
        self.quality_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        header.addWidget(self.quality_label, 1, 2)
        self.time_label = QLabel("T+000.0 с")
        self.time_label.setObjectName("value")
        self.time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.time_label.setMaximumWidth(220)
        header.addWidget(self.time_label, 0, 2)
        header.setColumnStretch(1, 1)
        layout.addLayout(header)
        layer_row = QGridLayout()
        self.layer_checks = {}
        for index, (key, text, color) in enumerate((
            ("truth", "·· Истина", COLORS["truth"]),
            ("gnss", "• GNSS", COLORS["red"]),
            ("odometry", "-- Одометрия", COLORS["orange"]),
            ("camera", "◆ Камера", COLORS["cyan"]),
            ("estimate", "━ Итоговая позиция", COLORS["green"]),
        )):
            check = QCheckBox(text)
            check.setChecked(self.canvas.layers[key] if hasattr(self, "canvas") else key == "estimate")
            check.setStyleSheet(f"color: {color};")
            self.layer_checks[key] = check
            layer_row.addWidget(check, index // 3, index % 3)
        layer_row.setColumnStretch(2, 1)
        layout.addLayout(layer_row)
        split = QSplitter(Qt.Horizontal)
        left = QWidget(); left_layout = QVBoxLayout(left); left_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas = NavigationCanvas(engine)
        self.error_plot = ErrorPlot()
        left_layout.addWidget(self.canvas, 1); left_layout.addWidget(self.error_plot)
        split.addWidget(left)
        split.addWidget(self._build_diagnostics())
        split.setSizes([590, 350])
        layout.addWidget(split, 1)
        self.camera_model_note = QLabel("Геометрическая модель ориентиров; распознавание изображений не моделируется.")
        self.camera_model_note.setObjectName("muted")
        self.camera_model_note.setWordWrap(True)
        self.camera_model_note.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        layout.addWidget(self.camera_model_note)
        self.spoofing_warning_label = QLabel(
            "Ограничение: плавная подмена GNSS может пройти контроль инноваций без независимой привязки."
        )
        self.spoofing_warning_label.setWordWrap(True)
        self.spoofing_warning_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.spoofing_warning_label.setStyleSheet(f"color: {COLORS['amber']};")
        layout.addWidget(self.spoofing_warning_label)
        self.setCentralWidget(root)
        for name, check in self.layer_checks.items():
            check.toggled.connect(lambda enabled, layer=name: self.canvas.set_layer(layer, enabled))

    def _build_diagnostics(self):
        panel = QWidget(); layout = QVBoxLayout(panel); layout.setContentsMargins(0, 0, 0, 0)
        panel.setMinimumWidth(410)
        self.metrics_panel = QFrame(); self.metrics_panel.setObjectName("card")
        grid = QGridLayout(self.metrics_panel)
        self.metric_labels = {}
        for row, (key, title) in enumerate((("current", "Сейчас"), ("rmse", "RMSE"), ("p95", "95%"), ("max", "Максимум"), ("heading", "Курс"))):
            grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—"); value.setObjectName("value"); grid.addWidget(value, row, 1)
            self.metric_labels[key] = value
        layout.addWidget(self.metrics_panel)
        self.no_truth_label = QLabel("РЕАЛЬНЫЙ РЕЖИМ\nИстина отсутствует — абсолютная ошибка не вычисляется")
        self.no_truth_label.setWordWrap(True)
        self.no_truth_label.setStyleSheet(f"color: {COLORS['amber']}; padding: 12px;")
        self.no_truth_label.hide()
        layout.addWidget(self.no_truth_label)
        layout.addWidget(QLabel("ДОВЕРИЕ ИСТОЧНИКОВ"))
        self.trust_panel = QFrame(); self.trust_panel.setObjectName("card")
        trust_layout = QVBoxLayout(self.trust_panel)
        trust_layout.setSpacing(8)
        self.trust_rows = {}
        for source, title in self.TRUST_SOURCES:
            container = QWidget()
            row_layout = QGridLayout(container)
            row_layout.setContentsMargins(4, 2, 4, 2)
            name = QLabel(title); name.setStyleSheet("font-weight: 700;")
            bar = QProgressBar(); bar.setRange(0, 100); bar.setValue(92); bar.setFormat("0.92")
            state = QLabel("Норма"); state.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            reason = QLabel("Данные согласованы"); reason.setObjectName("muted"); reason.setWordWrap(True)
            row_layout.addWidget(name, 0, 0)
            row_layout.addWidget(state, 0, 1)
            row_layout.addWidget(bar, 1, 0, 1, 2)
            row_layout.addWidget(reason, 2, 0, 1, 2)
            trust_layout.addWidget(container)
            self.trust_rows[source] = {
                "container": container,
                "bar": bar,
                "state": state,
                "reason": reason,
            }
        self.trust_note = QLabel("Доверие 0…1 — текущая надёжность данных, не заявленная точность.")
        self.trust_note.setObjectName("muted"); self.trust_note.setWordWrap(True)
        trust_layout.addWidget(self.trust_note)
        layout.addWidget(self.trust_panel)
        layout.addWidget(QLabel("ЖУРНАЛ СОБЫТИЙ"))
        self.event_list = QListWidget()
        self.event_list.setMaximumHeight(190)
        layout.addWidget(self.event_list, 1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setMinimumWidth(430)
        scroll.setWidget(panel)
        return scroll

    def update_frame(self, frame: Frame):
        self.time_label.setText(f"T+{frame.sim_time:05.1f} с")
        self.canvas.set_frame(frame); self.error_plot.set_frame(frame)
        sigma = math.sqrt(max(frame.estimate.covariance[0, 0], frame.estimate.covariance[1, 1]))
        gnss_only = frame.dispatcher.mode is NavigationMode.GNSS_ONLY
        self.mode_label.setText("ОБЫЧНЫЙ РЕЖИМ · ТОЛЬКО GNSS" if gnss_only else "НАШЕ РЕШЕНИЕ · СЛИЯНИЕ ДАННЫХ")
        self.quality_label.setText(frame.dispatcher.status.upper())
        bad = frame.dispatcher.pose is None or (not gnss_only and sigma > 4.0)
        self.quality_label.setStyleSheet(
            f"color: {COLORS['red'] if bad else COLORS['green']}; "
            'font-family: "DejaVu Sans Mono"; font-size: 38px; font-weight: 700;'
        )
        self._apply_mode_visibility(gnss_only)
        if frame.metrics is None:
            self.metrics_panel.hide(); self.no_truth_label.show()
        else:
            self.metrics_panel.show(); self.no_truth_label.hide()
            metrics = frame.dispatcher_metrics
            if metrics is None:
                for label in self.metric_labels.values():
                    label.setText("—")
            else:
                self.metric_labels["current"].setText(f"{metrics.current_error:.2f} м")
                self.metric_labels["rmse"].setText(f"{metrics.rmse:.2f} м")
                self.metric_labels["p95"].setText(f"{metrics.p95:.2f} м")
                self.metric_labels["max"].setText(f"{metrics.maximum:.2f} м")
                self.metric_labels["heading"].setText("—" if gnss_only else f"{metrics.heading_error_deg:.1f}°")
        trust = {item.source: item for item in frame.reliability}
        for source, _ in self.TRUST_SOURCES:
            widgets = self.trust_rows[source]
            widgets["container"].setVisible(not gnss_only or source == "gnss")
            if gnss_only and source == "gnss":
                visible = frame.dispatcher.pose is not None
                widgets["bar"].setValue(100 if visible else 0)
                widgets["bar"].setFormat("без оценки")
                widgets["state"].setText("Без проверки")
                widgets["reason"].setText(frame.dispatcher.status)
                widgets["reason"].show()
                continue
            item = trust[source]
            widgets["bar"].setValue(round(item.score * 100))
            widgets["bar"].setFormat(f"{item.score:.2f}")
            stale = not item.available and (item.age is None or item.age > 0.6)
            widgets["state"].setText("Нет данных" if stale else item.state.value)
            correction = f" · поправка {item.last_correction_m:.2f} м" if item.last_correction_m > 0.0 else ""
            widgets["reason"].setText(item.reason + correction)
            # Keep the overview compact: explanations expand automatically
            # only when a source needs the operator's attention.
            widgets["reason"].setVisible(stale or item.state.value != "Норма")
            color = COLORS["green"] if item.score >= 0.78 else COLORS["amber"] if item.score >= 0.4 else COLORS["red"]
            widgets["bar"].setStyleSheet(f"QProgressBar::chunk {{ background: {color}; }}")
        self.event_list.clear()
        if gnss_only:
            for effect in frame.active_effects:
                if "GNSS" in effect:
                    self.event_list.addItem(f"СЕЙЧАС  {effect}")
        else:
            for event in frame.events[-10:][::-1]:
                self.event_list.addItem(f"{event.sim_time:06.1f}  {event.text}")

    def _apply_mode_visibility(self, gnss_only: bool) -> None:
        for key in ("truth", "odometry", "camera"):
            self.layer_checks[key].setVisible(not gnss_only)
        self.layer_checks["gnss"].setVisible(not gnss_only)
        self.layer_checks["estimate"].setText(
            "━ GNSS диспетчеру" if gnss_only else "━ Передано диспетчеру"
        )
        self.camera_model_note.setVisible(not gnss_only)
        self.spoofing_warning_label.setVisible(not gnss_only)


class DemoController:
    def __init__(self, engine: ExperimentEngine, range_window: RangeWindow, navigation_window: NavigationWindow):
        self.engine = engine
        self.range_window = range_window
        self.navigation_window = navigation_window
        self.timer = QTimer()
        self.timer.setInterval(40)
        self.timer.timeout.connect(self._tick)
        self._last_wall = time.monotonic()
        range_window.start_button.clicked.connect(self.start)
        range_window.pause_button.toggled.connect(self._pause)
        range_window.reset_button.clicked.connect(self.reset)
        range_window.speed_combo.currentIndexChanged.connect(lambda index: engine.set_speed_multiplier((1, 2, 5)[index]))
        range_window.mode_combo.currentIndexChanged.connect(self._change_mode)
        range_window.map_combo.currentIndexChanged.connect(self._change_world)
        self._publish(engine.frame())

    def start(self):
        self.engine.set_paused(False)
        self.range_window.pause_button.setChecked(False)
        self._last_wall = time.monotonic()
        self.timer.start()

    def _pause(self, paused):
        self.engine.set_paused(paused)
        if not paused:
            self._last_wall = time.monotonic()

    def reset(self):
        frame = self.engine.reset()
        self.range_window.apply_mode_visibility(self.engine.navigation_mode)
        self._last_wall = time.monotonic()
        self._publish(frame)

    def _change_mode(self, index):
        mode = self.range_window.mode_combo.itemData(index)
        frame = self.engine.set_navigation_mode(mode)
        self.range_window.apply_mode_visibility(self.engine.navigation_mode)
        self._last_wall = time.monotonic()
        self._publish(frame)

    def _change_world(self, index):
        path = self.range_window.map_combo.itemData(index)
        frame = self.engine.set_world(path)
        self.range_window.apply_mode_visibility(self.engine.navigation_mode)
        self._last_wall = time.monotonic()
        self._publish(frame)

    def _tick(self):
        now = time.monotonic()
        elapsed = min(now - self._last_wall, 0.2)
        self._last_wall = now
        self._publish(self.engine.advance_wall_time(elapsed))

    def _publish(self, frame):
        self.range_window.update_frame(frame)
        self.navigation_window.update_frame(frame)


def create_windows(engine: ExperimentEngine | None = None):
    global _APP_HOLDER
    _APP_HOLDER = QApplication.instance() or QApplication([])
    engine = engine or ExperimentEngine()
    range_window = RangeWindow(engine)
    navigation_window = NavigationWindow(engine)
    controller = DemoController(engine, range_window, navigation_window)
    return range_window, navigation_window, controller


def run_gui():
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)
    range_window, navigation_window, controller = create_windows()
    screen = app.primaryScreen().availableGeometry()
    half = max(760, screen.width() // 2)
    # Large readable typography needs the full vertical workspace.  Both
    # windows retain scrollable side panels on smaller displays.
    height = screen.height()
    range_window.setGeometry(screen.x(), screen.y(), half, height)
    navigation_window.setGeometry(screen.x() + half, screen.y(), screen.width() - half, height)
    range_window.show(); navigation_window.show(); controller.start()
    return app.exec_()


def main():
    raise SystemExit(run_gui())


if __name__ == "__main__":
    main()
