import os
import math

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtGui import QImage, QPainter
from PyQt5.QtWidgets import QApplication

from gnss_resilient_demo.engine import ExperimentEngine
from gnss_resilient_demo.gui import create_windows
from gnss_resilient_demo.models import Marker
from gnss_resilient_demo.style import APP_STYLESHEET


def app():
    return QApplication.instance() or QApplication([])


def test_two_windows_construct_with_operator_titles():
    app()
    engine = ExperimentEngine(seed=10)

    range_window, navigation_window, controller = create_windows(engine)

    assert range_window.windowTitle() == "Полигон — испытатель"
    assert navigation_window.windowTitle() == "Диспетчер — транспорт на карте"
    assert controller.engine is engine
    range_window.close()
    navigation_window.close()


def test_fault_controls_change_sensor_configuration_not_algorithm_state():
    app()
    engine = ExperimentEngine(seed=11)
    before = engine.frame().estimate
    range_window, navigation_window, controller = create_windows(engine)

    range_window.gnss_enabled.setChecked(False)

    assert engine.sensors.faults.gnss.enabled is False
    assert engine.frame().estimate.x == before.x
    assert engine.frame().estimate.y == before.y
    range_window.close()
    navigation_window.close()


def test_navigation_layer_switch_updates_canvas():
    app()
    engine = ExperimentEngine(seed=12)
    range_window, navigation_window, controller = create_windows(engine)

    navigation_window.layer_checks["gnss"].setChecked(False)

    assert navigation_window.canvas.layers["gnss"] is False
    range_window.close()
    navigation_window.close()


def test_no_truth_mode_hides_absolute_metric_panel():
    app()
    engine = ExperimentEngine(seed=13, truth_available=False)
    range_window, navigation_window, controller = create_windows(engine)

    navigation_window.update_frame(engine.step())

    assert navigation_window.metrics_panel.isHidden()
    assert navigation_window.no_truth_label.isHidden() is False
    range_window.close()
    navigation_window.close()


def test_quality_copy_does_not_claim_spoofing_proof():
    app()
    engine = ExperimentEngine(seed=14)
    range_window, navigation_window, controller = create_windows(engine)

    navigation_window.update_frame(engine.step())

    assert "ДОСТОВЕРНА" not in navigation_window.quality_label.text()
    assert "ПОЗИЦИЯ" in navigation_window.quality_label.text()
    assert "плавная подмена gnss может пройти" in navigation_window.spoofing_warning_label.text().lower()
    range_window.close()
    navigation_window.close()


def test_mode_selector_resets_and_changes_dispatcher_behaviour():
    app()
    engine = ExperimentEngine(seed=15)
    range_window, navigation_window, controller = create_windows(engine)
    engine.step()

    range_window.mode_combo.setCurrentIndex(0)

    assert engine.navigation_mode.value == "gnss_only"
    assert engine.frame().sim_time == 0.0
    assert "ТОЛЬКО GNSS" in navigation_window.mode_label.text()
    range_window.close()
    navigation_window.close()


def test_uwb_can_be_disabled_from_range_window():
    app()
    engine = ExperimentEngine(seed=16)
    range_window, navigation_window, controller = create_windows(engine)

    range_window.uwb_enabled.setChecked(False)

    assert engine.sensors.faults.uwb.enabled is False
    range_window.close()
    navigation_window.close()


def test_gnss_only_diagnostics_do_not_claim_fusion_rejection():
    app()
    engine = ExperimentEngine(seed=17)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.mode_combo.setCurrentIndex(0)
    engine.sensors.faults.gnss.jump_x = 25.0
    for _ in range(30):
        frame = engine.step()

    navigation_window.update_frame(frame)

    assert navigation_window.trust_rows["gnss"]["state"].text() == "Без проверки"
    assert navigation_window.trust_rows["local_motion"]["container"].isHidden()
    assert navigation_window.trust_rows["camera"]["container"].isHidden()
    assert navigation_window.trust_rows["uwb"]["container"].isHidden()
    range_window.close()
    navigation_window.close()


def test_mode_hides_irrelevant_source_panels_and_shows_simple_gnss_faults():
    app()
    engine = ExperimentEngine(seed=18)
    range_window, navigation_window, controller = create_windows(engine)

    range_window.mode_combo.setCurrentIndex(0)

    assert range_window.camera_group.isHidden()
    assert range_window.motion_group.isHidden()
    assert range_window.uwb_group.isHidden()
    assert not range_window.gnss_group.isHidden()
    assert hasattr(range_window, "gnss_partial_jam")
    assert hasattr(range_window, "gnss_full_jam")
    assert hasattr(range_window, "gnss_spoof")
    assert range_window.gnss_enabled.isHidden()

    range_window.mode_combo.setCurrentIndex(1)

    assert not range_window.camera_group.isHidden()
    assert not range_window.motion_group.isHidden()
    assert not range_window.uwb_group.isHidden()
    range_window.close()
    navigation_window.close()


def test_ui_uses_large_text_and_large_primary_controls():
    application = app()
    application.setStyleSheet(APP_STYLESHEET)
    engine = ExperimentEngine(seed=31)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.show()
    application.processEvents()

    assert range_window.font().pixelSize() >= 28
    assert range_window.start_button.height() >= 58
    assert range_window.gnss_partial_jam.height() >= 58
    assert navigation_window.quality_label.font().pixelSize() >= 36
    assert navigation_window.trust_rows["gnss"]["bar"].font().pixelSize() >= 24
    assert navigation_window.event_list.font().pixelSize() >= 24
    range_window.close()
    navigation_window.close()


def test_full_jamming_is_the_only_visible_gnss_availability_switch():
    app()
    engine = ExperimentEngine(seed=32)
    range_window, navigation_window, controller = create_windows(engine)

    assert range_window.gnss_enabled.isHidden()
    range_window.gnss_full_jam.setChecked(True)

    assert engine.sensors.faults.gnss.enabled is False
    assert range_window.gnss_enabled.isChecked() is False
    range_window.close()
    navigation_window.close()


def test_operator_ui_has_no_manual_drive_or_automatic_fault_scenario_controls():
    app()
    engine = ExperimentEngine(seed=39)
    range_window, navigation_window, controller = create_windows(engine)

    assert not hasattr(range_window, "auto_drive")
    assert not hasattr(range_window, "scenario_button")
    assert not hasattr(range_window, "camera_range")
    assert not hasattr(range_window, "camera_fov")
    assert not hasattr(range_window, "odom_scale")
    assert not hasattr(range_window, "uwb_noise")
    assert hasattr(range_window, "wheel_slip")
    assert hasattr(range_window, "gyro_drift")
    range_window.close()
    navigation_window.close()


def test_dispatcher_shows_live_trust_score_state_and_reason():
    app()
    engine = ExperimentEngine(seed=40)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.gnss_spoof.setChecked(True)
    for _ in range(180):
        frame = engine.step()

    navigation_window.update_frame(frame)
    row = navigation_window.trust_rows["gnss"]

    assert row["bar"].value() < navigation_window.trust_rows["uwb"]["bar"].value()
    assert row["state"].text() in {"Подозрение", "Ухудшен", "Отклонён", "Восстановление"}
    assert row["reason"].text()
    assert "не заявленная точность" in navigation_window.trust_note.text().lower()
    range_window.close()
    navigation_window.close()


def test_map_selector_switches_between_four_presets():
    app()
    engine = ExperimentEngine(seed=19)
    range_window, navigation_window, controller = create_windows(engine)

    assert range_window.map_combo.count() == 4
    target = next(
        index for index in range(range_window.map_combo.count())
        if "Лес" in range_window.map_combo.itemText(index)
    )
    range_window.map_combo.setCurrentIndex(target)

    assert engine.world.kind == "forest"
    assert engine.frame().sim_time == 0.0
    range_window.close()
    navigation_window.close()


def test_coverage_button_and_disabled_sources_control_map_infrastructure():
    app()
    engine = ExperimentEngine(seed=20)
    range_window, navigation_window, controller = create_windows(engine)

    assert range_window.canvas.infrastructure_visibility() == {"markers": True, "beacons": True}
    assert range_window.canvas.show_uwb_coverage is False
    range_window.coverage_button.setChecked(True)
    assert range_window.canvas.show_uwb_coverage is True

    range_window.camera_enabled.setChecked(False)
    range_window.uwb_enabled.setChecked(False)
    assert range_window.canvas.infrastructure_visibility() == {"markers": False, "beacons": False}

    range_window.mode_combo.setCurrentIndex(0)
    assert range_window.canvas.infrastructure_visibility() == {"markers": False, "beacons": False}
    range_window.close()
    navigation_window.close()


def test_every_map_renders_to_an_image_without_painter_errors():
    app()
    engine = ExperimentEngine(seed=30)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.canvas.resize(720, 520)

    for index in range(range_window.map_combo.count()):
        range_window.map_combo.setCurrentIndex(index)
        image = QImage(720, 520, QImage.Format_ARGB32)
        painter = QPainter(image)
        range_window.canvas.render(painter)
        painter.end()
        assert not image.isNull()

    range_window.close()
    navigation_window.close()


def test_map_outlines_do_not_reuse_the_last_decoration_fill_color():
    app()
    engine = ExperimentEngine(seed=34)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.canvas.resize(720, 520)

    image = QImage(720, 520, QImage.Format_ARGB32)
    painter = QPainter(image)
    range_window.canvas.render(painter)
    painter.end()
    farm_center = range_window.canvas._point(25.0, 23.0)

    assert image.pixelColor(round(farm_center.x()), round(farm_center.y())).name() == "#c7ae8a"

    forest_index = next(
        index for index in range(range_window.map_combo.count())
        if "Лес" in range_window.map_combo.itemText(index)
    )
    range_window.map_combo.setCurrentIndex(forest_index)
    painter = QPainter(image)
    range_window.canvas.render(painter)
    painter.end()
    zone_center = range_window.canvas._point(50.0, 30.0)

    assert image.pixelColor(round(zone_center.x()), round(zone_center.y())).name() == "#727a52"
    range_window.close()
    navigation_window.close()


def test_camera_and_beacon_activity_does_not_flicker_between_sensor_ticks():
    app()
    engine = ExperimentEngine(seed=35)
    engine.world.markers = (Marker(999, 8.0, 8.0, math.pi, 0.6),)
    range_window, navigation_window, controller = create_windows(engine)

    initial_beacons = range_window.canvas.active_beacon_ids()
    range_window.canvas.set_frame(engine.step())
    held_beacons = range_window.canvas.active_beacon_ids()

    assert initial_beacons
    assert held_beacons == initial_beacons

    observed = None
    for _ in range(20):
        frame = engine.step()
        if frame.packet.camera:
            observed = frame
            break
    assert observed is not None
    range_window.canvas.set_frame(observed)
    recognized = range_window.canvas.recognized_marker_ids()
    range_window.canvas.resize(720, 520)
    image = QImage(720, 520, QImage.Format_ARGB32)
    painter = QPainter(image)
    range_window.canvas.render(painter)
    painter.end()
    marker_point = range_window.canvas._point(8.0, 8.0)
    recognized_colors = {
        image.pixelColor(x, y).name()
        for x in range(round(marker_point.x()) - 10, round(marker_point.x()) + 11)
        for y in range(round(marker_point.y()) - 10, round(marker_point.y()) + 11)
    }
    next_frame = engine.step()
    assert not next_frame.packet.camera
    range_window.canvas.set_frame(next_frame)

    assert recognized == {999}
    assert "#ffb547" in recognized_colors
    assert range_window.canvas.recognized_marker_ids() == {999}
    range_window.close()
    navigation_window.close()


def test_dispatcher_layout_still_fits_half_of_a_full_hd_screen_with_large_text():
    application = app()
    application.setStyleSheet(APP_STYLESHEET)
    engine = ExperimentEngine(seed=36)
    range_window, navigation_window, controller = create_windows(engine)

    navigation_window.resize(960, 780)
    navigation_window.show()
    application.processEvents()

    assert navigation_window.width() <= 960
    range_window.close()
    navigation_window.close()


def test_gnss_only_event_log_shows_impacts_not_unused_filter_decisions():
    app()
    engine = ExperimentEngine(seed=32)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.mode_combo.setCurrentIndex(0)
    range_window.gnss_spoof.setChecked(True)
    for _ in range(30):
        frame = engine.step()

    navigation_window.update_frame(frame)
    entries = [navigation_window.event_list.item(index).text() for index in range(navigation_window.event_list.count())]

    assert any("Подмена координат GNSS" in entry for entry in entries)
    assert all("Отклонено" not in entry for entry in entries)
    range_window.close()
    navigation_window.close()


def test_resetting_experiment_synchronizes_gnss_impact_switches():
    app()
    engine = ExperimentEngine(seed=33)
    range_window, navigation_window, controller = create_windows(engine)
    range_window.gnss_partial_jam.setChecked(True)
    range_window.gnss_full_jam.setChecked(True)
    range_window.gnss_spoof.setChecked(True)

    range_window.map_combo.setCurrentIndex(0)

    assert range_window.gnss_enabled.isChecked()
    assert not range_window.gnss_partial_jam.isChecked()
    assert not range_window.gnss_full_jam.isChecked()
    assert not range_window.gnss_spoof.isChecked()
    range_window.close()
    navigation_window.close()
