import csv
import json

import pytest

from gnss_resilient_demo.engine import ExperimentEngine, NavigationMode
from gnss_resilient_demo.recording import export_run


def run_steps(engine, count=300):
    values = []
    for _ in range(count):
        frame = engine.step()
        values.append((frame.estimate.x, frame.estimate.y, frame.metrics.current_error))
    return values


def test_replay_is_deterministic_across_instances_and_reset():
    first = ExperimentEngine(seed=516)
    second = ExperimentEngine(seed=516)

    original = run_steps(first)
    parallel = run_steps(second)
    first.reset()
    replay = run_steps(first)

    assert original == parallel == replay


def test_pause_speed_and_reset_use_fixed_simulation_time():
    engine = ExperimentEngine(seed=1)
    engine.set_paused(True)
    engine.advance_wall_time(1.0)
    assert engine.frame().sim_time == 0.0

    engine.set_paused(False)
    engine.set_speed_multiplier(2)
    engine.advance_wall_time(0.5)
    assert engine.frame().sim_time == pytest.approx(1.0)

    engine.reset()
    assert engine.frame().sim_time == 0.0


def test_no_automatic_faults_are_injected_during_the_run():
    engine = ExperimentEngine(seed=2)

    while engine.frame().sim_time < 40.0:
        frame = engine.step()

    assert not any("Введено воздействие" in event.text for event in frame.events)
    assert engine.sensors.faults.gnss.automatic_jump_x == 0.0
    assert engine.sensors.faults.gnss.automatic_ramp_rate_x == 0.0
    assert all(not hasattr(decision, "fault_active") for decision in frame.decisions)


def test_real_mode_without_truth_suppresses_absolute_metrics():
    engine = ExperimentEngine(seed=3, truth_available=False)

    frame = engine.step()

    assert frame.truth is None
    assert frame.metrics is None


def test_export_contains_measurement_times_filter_decisions_and_summary(tmp_path):
    engine = ExperimentEngine(seed=4)
    run_steps(engine, 100)

    result = export_run(engine, tmp_path)

    with result.summary_json.open(encoding="utf-8") as stream:
        summary = json.load(stream)
    with result.samples_csv.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert summary["seed"] == 4
    assert "limitations" in summary
    assert rows
    assert {
        "sim_time",
        "truth_x",
        "estimate_x",
        "cov_xx",
        "gnss_measured_at",
        "gnss_received_at",
        "decisions",
    }.issubset(rows[0])


def test_render_history_is_sampled_without_losing_latest_state():
    engine = ExperimentEngine(seed=5)

    run_steps(engine, 500)
    frame = engine.frame()

    assert len(frame.history) <= 110
    assert frame.history[-1].sim_time == pytest.approx(frame.sim_time)


def test_gnss_only_dispatcher_point_disappears_when_gnss_is_lost():
    engine = ExperimentEngine(seed=21, navigation_mode=NavigationMode.GNSS_ONLY)
    engine.sensors.faults.gnss.enabled = False

    run_steps(engine, 150)

    assert engine.frame().dispatcher.pose is None
    assert engine.frame().dispatcher.status == "Нет координат GNSS"


def test_gnss_only_dispatcher_receives_spoofed_point_without_gating():
    engine = ExperimentEngine(seed=22, navigation_mode=NavigationMode.GNSS_ONLY)
    engine.sensors.faults.gnss.jump_x = 25.0
    engine.sensors.faults.gnss.jump_y = -18.0

    run_steps(engine, 30)
    frame = engine.frame()

    assert frame.dispatcher.pose is not None
    assert frame.truth is not None
    assert abs(frame.dispatcher.pose.x - frame.truth.x) > 15.0
    assert frame.dispatcher.source == "GNSS без проверки"


def test_fused_dispatcher_continues_during_gnss_loss():
    engine = ExperimentEngine(seed=23, navigation_mode=NavigationMode.FUSION)
    engine.sensors.faults.gnss.enabled = False
    engine.sensors.faults.camera.enabled = False

    run_steps(engine, 150)
    frame = engine.frame()

    assert frame.dispatcher.pose is not None
    assert frame.dispatcher.fresh is True
    assert frame.dispatcher.source == "Устойчивое слияние"


def test_changing_navigation_mode_resets_time_and_replays_seed():
    engine = ExperimentEngine(seed=24, navigation_mode=NavigationMode.GNSS_ONLY)
    first = run_steps(engine, 30)

    reset_frame = engine.set_navigation_mode(NavigationMode.FUSION)
    engine.set_navigation_mode(NavigationMode.GNSS_ONLY)
    replay = run_steps(engine, 30)

    assert reset_frame.sim_time == 0.0
    assert first == replay


def test_changing_mode_preserves_moved_beacons_and_applies_source_defaults():
    engine = ExperimentEngine(seed=25)
    engine.sensors.faults.gnss.enabled = False
    engine.sensors.faults.camera.enabled = False
    engine.world.move_beacon("B1", 17.0, 19.0)

    engine.set_navigation_mode(NavigationMode.GNSS_ONLY)

    assert engine.sensors.faults.gnss.enabled is True
    assert engine.sensors.faults.camera.enabled is False
    assert engine.sensors.faults.motion.odom_enabled is False
    assert engine.sensors.faults.motion.imu_enabled is False
    assert engine.sensors.faults.uwb.enabled is False
    assert engine.world.beacon("B1").x == 17.0
    assert engine.world.beacon("B1").y == 19.0

    engine.set_navigation_mode(NavigationMode.FUSION)

    assert engine.sensors.faults.gnss.enabled is True
    assert engine.sensors.faults.camera.enabled is True
    assert engine.sensors.faults.motion.odom_enabled is True
    assert engine.sensors.faults.motion.imu_enabled is True
    assert engine.sensors.faults.uwb.enabled is True


def test_mode_comparison_clears_all_legacy_automatic_effects():
    engine = ExperimentEngine(seed=26)
    engine.sensors.faults.gnss.automatic_jump_x = 25.0

    engine.set_navigation_mode(NavigationMode.GNSS_ONLY)

    assert engine.frame().sim_time == 0.0
    assert engine.sensors.faults.gnss.automatic_jump_x == 0.0
    assert engine.sensors.faults.gnss.enabled is True


def test_frame_publishes_reliability_for_all_real_sources():
    engine = ExperimentEngine(seed=35, navigation_mode=NavigationMode.FUSION)

    frame = engine.step()

    assert {item.source for item in frame.reliability} == {"local_motion", "gnss", "camera", "uwb"}
    assert all(0.0 <= item.score <= 1.0 for item in frame.reliability)
    assert all(not hasattr(item, "truth") for item in frame.reliability)


def test_manual_gnss_spoofing_lowers_gnss_trust_below_independent_sources():
    engine = ExperimentEngine(seed=36, navigation_mode=NavigationMode.FUSION)
    engine.sensors.faults.gnss.jump_x = 25.0
    engine.sensors.faults.gnss.jump_y = -18.0

    run_steps(engine, 180)
    trust = {item.source: item for item in engine.frame().reliability}

    assert trust["gnss"].score < trust["uwb"].score
    assert trust["gnss"].covariance_multiplier > trust["uwb"].covariance_multiplier
    assert "расхожд" in trust["gnss"].reason.lower() or "отклон" in trust["gnss"].reason.lower()


def test_gnss_trust_recovers_only_after_spoofing_is_removed_and_series_agrees():
    engine = ExperimentEngine(seed=42)
    engine.sensors.faults.gnss.jump_x = 25.0
    engine.sensors.faults.gnss.jump_y = -18.0
    run_steps(engine, 300)
    rejected = next(item for item in engine.frame().reliability if item.source == "gnss")
    assert rejected.state.value == "Отклонён"

    engine.sensors.faults.gnss.jump_x = 0.0
    engine.sensors.faults.gnss.jump_y = 0.0
    run_steps(engine, 600)
    recovered = next(item for item in engine.frame().reliability if item.source == "gnss")

    assert recovered.score > 0.6
    assert recovered.state.value in {"Восстановление", "Норма"}


def test_city_map_exposes_mapped_gnss_risk_without_truth_flag_to_filter():
    config_dir = __import__("pathlib").Path(__file__).parents[1] / "config"
    engine = ExperimentEngine(seed=37, world_path=config_dir / "world_city.json")

    frame = engine.step()
    gnss = next(item for item in frame.reliability if item.source == "gnss")

    assert frame.urban_gnss_risk is True
    assert gnss.covariance_multiplier >= 2.0
    assert "застрой" in gnss.reason.lower()


def test_vehicle_control_remains_automatic_for_the_whole_run():
    engine = ExperimentEngine(seed=38)
    start = engine.frame().truth

    run_steps(engine, 200)
    end = engine.frame().truth

    assert engine._control.auto is True
    assert (end.x, end.y) != pytest.approx((start.x, start.y))


def test_uncertainty_grows_when_all_motion_and_external_sources_are_disabled():
    engine = ExperimentEngine(seed=41)
    faults = engine.sensors.faults
    faults.motion.odom_enabled = False
    faults.motion.imu_enabled = False
    faults.gnss.enabled = False
    faults.camera.enabled = False
    faults.uwb.enabled = False
    before = engine.frame().estimate.covariance.trace()

    run_steps(engine, 100)

    assert engine.frame().estimate.covariance.trace() > before


def test_dispatcher_warns_when_all_external_references_are_lost():
    engine = ExperimentEngine(seed=43)
    engine.sensors.faults.gnss.enabled = False
    engine.sensors.faults.camera.enabled = False
    engine.sensors.faults.uwb.enabled = False

    run_steps(engine, 120)

    assert "внешней привязки" in engine.frame().dispatcher.status.lower()


def test_gnss_only_dispatcher_respects_measurement_delivery_time():
    engine = ExperimentEngine(seed=27, navigation_mode=NavigationMode.GNSS_ONLY)
    engine.sensors.faults.gnss.latency = 5.0
    engine.reset(faults=engine.sensors.faults)

    run_steps(engine, 200)
    assert engine.frame().dispatcher.pose is None

    run_steps(engine, 60)
    assert engine.frame().dispatcher.pose is not None
    assert engine.frame().dispatcher.fresh is False or engine.frame().dispatcher.age >= 0.0


def test_gnss_only_export_uses_dispatcher_error_not_hidden_filter_error(tmp_path):
    engine = ExperimentEngine(seed=28, navigation_mode=NavigationMode.GNSS_ONLY)
    engine.sensors.faults.gnss.jump_x = 25.0
    engine.sensors.faults.gnss.jump_y = -18.0
    run_steps(engine, 30)

    result = export_run(engine, tmp_path)
    summary = json.loads(result.summary_json.read_text(encoding="utf-8"))
    rows = list(csv.DictReader(result.samples_csv.open(encoding="utf-8", newline="")))

    assert summary["navigation_mode"] == "gnss_only"
    assert summary["metrics"]["current_error"] > 15.0
    assert float(rows[-1]["position_error"]) > 15.0
    assert float(rows[-1]["filter_position_error"]) < float(rows[-1]["position_error"])


def test_switching_world_resets_time_and_keeps_navigation_mode():
    config_dir = __import__("pathlib").Path(__file__).parents[1] / "config"
    engine = ExperimentEngine(seed=29, navigation_mode=NavigationMode.GNSS_ONLY)
    run_steps(engine, 20)

    frame = engine.set_world(config_dir / "world_forest.json")

    assert frame.sim_time == 0.0
    assert engine.navigation_mode is NavigationMode.GNSS_ONLY
    assert engine.world.name == "Лесная дорога"
    assert engine.world.kind == "forest"
