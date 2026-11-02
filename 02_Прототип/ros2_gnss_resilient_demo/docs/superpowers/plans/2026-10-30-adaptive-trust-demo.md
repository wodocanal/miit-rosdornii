# Adaptive Trust Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add observable adaptive source trust to the simulator and simplify the UI into a readable manual demonstration.

**Architecture:** A new truth-free `ReliabilityManager` computes per-source state and EKF policy from measurement health, mapped urban risk, innovations, and cross-source consistency. `ExperimentEngine` applies those policies before EKF updates and publishes immutable trust snapshots to the dispatcher UI. Qt controls are reduced to manual fault toggles and large diagnostic rows.

**Tech Stack:** Python 3.12, NumPy, PyQt5, pytest, ROS 2 Jazzy packaging.

**Spec:** `docs/superpowers/specs/2026-10-30-adaptive-trust-demo-design.md`

## Global Constraints

- Pure simulator only; no rosbag or hardware integration.
- Reliability code receives no truth pose and no injected-fault flags.
- Vehicle control is always automatic; automatic fault scenarios are absent from the UI.
- Base UI text is at least 28 px and primary controls are at least 58 px high.
- GNSS-only mode shows no local localization controls or diagnostics.

## Review Focus

- A large GNSS jump in an urban-risk area is rapidly downgraded while consistent camera/UWB data remain usable.
- A rejected source cannot immediately regain full trust from one good sample.
- Loss of camera or UWB produces «Нет данных» rather than reusing stale observations.
- Disabling odometry and IMU does not freeze covariance or fabricate local motion.
- Large fonts remain usable at the intended two-window geometry without hiding primary fault toggles.

---

### Task 1: Truth-free reliability manager and adaptive EKF policy

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/reliability.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/ekf.py`
- Create: `src/gnss_resilient_demo/test/test_reliability.py`
- Modify: `src/gnss_resilient_demo/test/test_ekf.py`

**Interfaces:**
- Produces: `ReliabilityManager.assess_batch(...)`, `ReliabilityManager.observe_decision(...)`, `ReliabilitySnapshot`, `UpdatePolicy`.
- Produces: `NavigationEKF.predict(..., process_noise_multiplier=...)` and `update_pose(..., covariance_multiplier=..., gate_multiplier=...)`.

- [ ] Write failing tests for urban GNSS caution, multi-source disagreement, motion disagreement, hysteretic recovery and EKF covariance/gate policies.
- [ ] Run targeted tests and verify expected failures.
- [ ] Implement the manager and minimal EKF policy parameters.
- [ ] Run targeted and full tests.

### Task 2: Engine integration and manual-only demonstration

**Files:**
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/engine.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/world.py`
- Modify: `src/gnss_resilient_demo/test/test_engine.py`

**Interfaces:**
- Consumes: Task 1 reliability APIs.
- Produces: `Frame.reliability` and mapped context derived from the filter estimate.

- [ ] Write failing engine tests for adaptive GNSS downgrade, published trust, truth separation and automatic vehicle control.
- [ ] Run targeted tests and verify expected failures.
- [ ] Integrate batch assessment, adaptive prediction/update and source recovery.
- [ ] Run deterministic manual-fault experiments and full tests.

### Task 3: Large compact operator UI

**Files:**
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/style.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/gui.py`
- Modify: `src/gnss_resilient_demo/test/test_gui.py`

**Interfaces:**
- Consumes: `Frame.reliability`.
- Produces: manual-only fault panel and `NavigationWindow.trust_rows`.

- [ ] Write failing UI tests for removed auto/scenario controls, compact manual toggles, 28 px typography and trust rows.
- [ ] Run targeted tests and verify expected failures.
- [ ] Implement the simplified controls and trust panel.
- [ ] Render both windows offscreen at target sizes and run all tests.

### Task 4: Acceptance and documentation

**Files:**
- Modify: `README.md`
- Modify: `DEMO_SCRIPT_RU.md`

- [ ] Update the 3–5 minute manual demonstration script and reliability terminology.
- [ ] Run the full pytest suite, headless comparisons for GNSS loss/spoofing/recovery, and `colcon build`.
- [ ] Launch the two-window simulator and visually inspect both modes and all four maps.
