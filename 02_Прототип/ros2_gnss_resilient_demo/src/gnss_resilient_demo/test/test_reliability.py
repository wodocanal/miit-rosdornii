from gnss_resilient_demo.ekf import UpdateDecision
from gnss_resilient_demo.measurements import PoseMeasurement
from gnss_resilient_demo.reliability import ReliabilityManager, ReliabilityState


def pose(source, x, y, time=1.0, variance=0.25):
    return PoseMeasurement(
        source,
        time,
        time,
        x,
        y,
        None,
        (variance, 0.0, 0.0, variance),
    )


def decision(source, accepted, innovation, reason="Измерение принято"):
    return UpdateDecision(
        source=source,
        measured_at=1.0,
        accepted=accepted,
        reason=reason,
        mahalanobis=1.0 if accepted else 25.0,
        correction_m=0.2 if accepted else 0.0,
        innovation_m=innovation,
    )


def test_urban_context_makes_gnss_cautious_without_disabling_it():
    manager = ReliabilityManager()

    policies = manager.assess_batch(
        [pose("gnss", 0.4, -0.2)],
        predicted_xy=(0.0, 0.0),
        now=1.0,
        urban_gnss_risk=True,
    )
    snapshot = manager.snapshot("gnss", now=1.0)

    assert policies["gnss"].hard_reject is False
    assert policies["gnss"].covariance_multiplier >= 2.0
    assert policies["gnss"].gate_multiplier < 1.0
    assert snapshot.score >= 0.5
    assert "застрой" in snapshot.reason.lower()


def test_source_disagreeing_with_two_independent_references_loses_trust():
    manager = ReliabilityManager()

    policies = manager.assess_batch(
        [
            pose("gnss", 18.0, -12.0),
            pose("camera", 0.2, 0.1),
            pose("uwb", -0.1, 0.2),
        ],
        predicted_xy=(0.0, 0.0),
        now=1.0,
        urban_gnss_risk=True,
    )
    gnss = manager.snapshot("gnss", now=1.0)
    camera = manager.snapshot("camera", now=1.0)

    assert gnss.score < camera.score
    assert gnss.state in {ReliabilityState.SUSPECT, ReliabilityState.DEGRADED, ReliabilityState.REJECTED}
    assert policies["gnss"].covariance_multiplier > policies["camera"].covariance_multiplier
    assert "расхожд" in gnss.reason.lower()


def test_external_consensus_increases_local_motion_process_noise():
    manager = ReliabilityManager()

    manager.assess_batch(
        [pose("gnss", 8.0, 0.1), pose("camera", 8.2, -0.1), pose("uwb", 7.9, 0.0)],
        predicted_xy=(0.0, 0.0),
        now=1.0,
    )

    motion = manager.snapshot("local_motion", now=1.0)
    assert motion.score < 0.75
    assert manager.process_noise_multiplier > 1.5
    assert "локаль" in motion.reason.lower()


def test_rejected_source_requires_three_consistent_samples_before_reentry():
    manager = ReliabilityManager(recovery_required=3)
    bad = [pose("gnss", 30.0, 30.0), pose("camera", 0.0, 0.0), pose("uwb", 0.1, 0.0)]
    for index in range(3):
        policies = manager.assess_batch(bad, (0.0, 0.0), now=1.0 + index, urban_gnss_risk=True)
        manager.observe_decision(decision("gnss", False, 40.0, "Отклонено: большое расхождение"), now=1.0 + index)

    assert manager.snapshot("gnss", now=3.0).state is ReliabilityState.REJECTED
    good = [pose("gnss", 0.2, 0.0), pose("camera", 0.0, 0.0), pose("uwb", 0.1, 0.0)]

    first = manager.assess_batch(good, (0.0, 0.0), now=4.0)["gnss"]
    second = manager.assess_batch(good, (0.0, 0.0), now=5.0)["gnss"]
    third = manager.assess_batch(good, (0.0, 0.0), now=6.0)["gnss"]

    assert first.hard_reject is True
    assert second.hard_reject is True
    assert third.hard_reject is False
    assert manager.snapshot("gnss", now=6.0).state is ReliabilityState.RECOVERING


def test_accepted_measurements_recover_trust_gradually_and_record_correction():
    manager = ReliabilityManager()
    manager.observe_decision(decision("camera", False, 9.0, "Отклонено"), now=1.0)
    degraded = manager.snapshot("camera", now=1.0).score

    manager.observe_decision(decision("camera", True, 0.3), now=2.0)
    snapshot = manager.snapshot("camera", now=2.0)

    assert degraded < snapshot.score < 1.0
    assert snapshot.last_correction_m == 0.2


def test_source_without_fresh_data_gradually_loses_trust():
    manager = ReliabilityManager()
    initial = manager.snapshot("camera", now=0.0).score

    manager.mark_availability("camera", False, now=0.0)
    manager.mark_availability("camera", False, now=1.2)
    manager.mark_availability("camera", False, now=1.8)
    missing = manager.snapshot("camera", now=1.8)

    assert missing.score < initial
    assert missing.score >= 0.52
    assert missing.available is False
    assert missing.reason == "Нет свежих данных"


def test_noisy_camera_observation_is_judged_using_its_reported_uncertainty():
    manager = ReliabilityManager()
    camera = PoseMeasurement(
        source="camera",
        measured_at=2.0,
        received_at=2.0,
        x=5.0,
        y=0.0,
        yaw=0.0,
        covariance=(4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.1),
    )
    gnss = pose("gnss", 0.1, 0.0)

    policy = manager.assess_batch([camera, gnss], (0.0, 0.0), now=2.0)["camera"]

    assert policy.hard_reject is False
    assert manager.snapshot("camera", now=2.0).score >= 0.9
