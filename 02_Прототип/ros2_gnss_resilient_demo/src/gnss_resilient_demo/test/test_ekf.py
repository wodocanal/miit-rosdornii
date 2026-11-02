import math

import numpy as np
import pytest

from gnss_resilient_demo.ekf import NavigationEKF
from gnss_resilient_demo.measurements import MotionMeasurement, PoseMeasurement


def motion(time, speed=2.0, yaw_rate=0.0):
    return MotionMeasurement("local_motion", time, time, speed, yaw_rate, 0.01, 0.001)


def pose(x, y, time=1.0, yaw=None, variance=0.25, received=None, source="gnss"):
    covariance = (
        (variance, 0.0, 0.0, variance)
        if yaw is None
        else (variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, 0.04)
    )
    return PoseMeasurement(source, time, time if received is None else received, x, y, yaw, covariance)


def test_prediction_wraps_yaw_and_grows_covariance():
    ekf = NavigationEKF(initial_yaw=math.pi - 0.05)
    before = np.trace(ekf.snapshot().covariance)

    state = ekf.predict(motion(1.0, yaw_rate=0.2), dt=1.0)

    assert -math.pi <= state.yaw < math.pi
    assert state.yaw == pytest.approx(-math.pi + 0.15)
    assert np.trace(state.covariance) > before


def test_consistent_pose_update_is_accepted_and_reduces_uncertainty():
    ekf = NavigationEKF()
    ekf.predict(motion(1.0), dt=1.0)
    before = ekf.snapshot()

    decision = ekf.update_pose(pose(2.2, 0.1), now=1.0)
    after = ekf.snapshot()

    assert decision.accepted is True
    assert decision.reason == "Измерение принято"
    assert after.x > before.x
    assert np.trace(after.covariance) < np.trace(before.covariance)


def test_large_innovation_is_rejected_without_changing_state():
    ekf = NavigationEKF()
    ekf.predict(motion(1.0), dt=1.0)
    before = ekf.snapshot()

    decision = ekf.update_pose(pose(100.0, 100.0), now=1.0)

    assert decision.accepted is False
    assert decision.reason == "Отклонено: большое расхождение"
    assert ekf.snapshot().x == before.x
    assert ekf.snapshot().y == before.y


def test_stale_pose_is_rejected_before_innovation_check():
    ekf = NavigationEKF(max_measurement_age=1.0)

    decision = ekf.update_pose(pose(0.0, 0.0, time=0.0, received=3.0), now=3.0)

    assert decision.accepted is False
    assert decision.reason == "Отклонено: устаревшее измерение"


def test_three_consistent_observations_recover_after_long_external_gap():
    ekf = NavigationEKF(recovery_after=5.0, recovery_required=3)
    decisions = [
        ekf.update_pose(pose(10.0, 4.0, time=10.0), now=10.0),
        ekf.update_pose(pose(10.2, 4.1, time=10.5), now=10.5),
        ekf.update_pose(pose(10.4, 4.2, time=11.0), now=11.0),
    ]

    assert [item.accepted for item in decisions] == [False, False, True]
    assert decisions[-1].reason == "Восстановление по серии наблюдений"
    assert ekf.snapshot().x > 5.0


def test_adaptive_covariance_reduces_measurement_correction():
    normal = NavigationEKF()
    cautious = NavigationEKF()
    normal.predict(motion(1.0), dt=1.0)
    cautious.predict(motion(1.0), dt=1.0)

    normal_decision = normal.update_pose(pose(3.0, 0.0, variance=1.0), now=1.0)
    cautious_decision = cautious.update_pose(
        pose(3.0, 0.0, variance=1.0),
        now=1.0,
        covariance_multiplier=9.0,
    )

    assert normal_decision.accepted is True
    assert cautious_decision.accepted is True
    assert cautious_decision.correction_m < normal_decision.correction_m
    assert cautious_decision.covariance_multiplier == 9.0


def test_adaptive_gate_can_reject_a_measurement_that_default_gate_accepts():
    normal = NavigationEKF()
    cautious = NavigationEKF()

    accepted = normal.update_pose(pose(2.0, 0.0, variance=1.0), now=1.0)
    rejected = cautious.update_pose(
        pose(2.0, 0.0, variance=1.0),
        now=1.0,
        gate_multiplier=0.2,
    )

    assert accepted.accepted is True
    assert rejected.accepted is False
    assert rejected.gate_threshold < accepted.gate_threshold


def test_process_noise_multiplier_grows_prediction_uncertainty():
    normal = NavigationEKF()
    cautious = NavigationEKF()

    normal_state = normal.predict(motion(1.0), dt=1.0)
    cautious_state = cautious.predict(motion(1.0), dt=1.0, process_noise_multiplier=4.0)

    assert np.trace(cautious_state.covariance) > np.trace(normal_state.covariance)


def test_missing_motion_measurements_coast_and_grow_uncertainty():
    ekf = NavigationEKF()
    ekf.predict(motion(1.0, speed=3.0), dt=1.0)
    before = ekf.snapshot()

    after = ekf.predict_without_motion(dt=1.0, process_noise_multiplier=3.0)

    assert after.x > before.x
    assert np.trace(after.covariance) > np.trace(before.covariance)
