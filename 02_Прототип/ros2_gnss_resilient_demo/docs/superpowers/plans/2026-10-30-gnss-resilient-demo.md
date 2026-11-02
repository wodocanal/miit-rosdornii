# GNSS-Resilient Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reproducible ROS 2 Jazzy demonstration with a deterministic 2D simulator, transparent navigation filter, two Qt windows, diagnostics and export.

**Architecture:** A pure-Python core keeps world truth, generated measurements, filter state and evaluation strictly separated. A PyQt5 presentation layer renders two synchronized windows; a thin ROS package and adapters expose the same input contracts to later Frob hardware integration.

**Tech Stack:** Python 3.12, NumPy, PyQt5, pytest, ROS 2 Jazzy/rclpy, colcon/ament_python, JSON/CSV.

**Spec:** `docs/superpowers/specs/2026-10-30-gnss-resilient-demo-design.md`

## Global Constraints

- Do not expose truth or fault flags to `NavigationCore`.
- Use simulation time and independent deterministic RNG streams from seed `516`.
- Run without lidar, Nav2, heavy simulators, GPU or network services.
- UI labels and operator-facing documentation are Russian.
- Existing Frob fused odometry is one movement source, never duplicated with its raw inputs.

## Review Focus

- Delayed observations must be rejected as stale rather than treated as current.
- Disabling one source must not change another source's deterministic noise sequence.
- Smooth spoofing may pass gating; the UI must describe this limitation rather than claiming detection.
- Reset after pause/acceleration must not create a large time step.
- Real mode without truth must suppress absolute error metrics.

---

### Task 1: Deterministic world and motion model

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/models.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/world.py`
- Create: `src/gnss_resilient_demo/config/world.json`
- Test: `src/gnss_resilient_demo/test/test_world.py`

**Interfaces:**
- Produces: `WorldModel.reset()`, `WorldModel.step(dt, control)`, `WorldSnapshot`, collision and line-of-sight queries.

- [ ] Write tests for deterministic route following, collision blocking and segment occlusion; run and observe missing-module failure.
- [ ] Implement immutable public snapshots and configurable geometry without navigation dependencies.
- [ ] Run `pytest -q src/gnss_resilient_demo/test/test_world.py` and expect all tests to pass.

### Task 2: Independent sensor models

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/measurements.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/sensors.py`
- Test: `src/gnss_resilient_demo/test/test_sensors.py`

**Interfaces:**
- Consumes: `WorldSnapshot` and world visibility queries.
- Produces: timestamped `MotionMeasurement`, `PoseMeasurement`, `MarkerObservation`, `SensorStatus` with no truth fields.

- [ ] Write tests for independent RNG, GNSS tunnel/manual precedence, camera FOV/occlusion/back-face/range and no stale re-publication; observe failure.
- [ ] Implement noise, latency queues, dropout and fault controls with separate RNG streams.
- [ ] Run the sensor test file and expect all tests to pass.

### Task 3: Navigation EKF and recovery

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/ekf.py`
- Test: `src/gnss_resilient_demo/test/test_ekf.py`

**Interfaces:**
- Consumes: measurement dataclasses only.
- Produces: `FilterSnapshot`, `UpdateDecision`, source diagnostics; `predict()` and `update_pose()`.

- [ ] Write tests for angle wrapping, covariance growth, accepted update, innovation rejection, stale rejection and consistent-series recovery; observe failure.
- [ ] Implement five-state EKF with Joseph covariance update, chi-square gates and recovery buffer independent of truth.
- [ ] Run EKF tests and expect all tests to pass.

### Task 4: Experiment engine, scenario, metrics and export

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/engine.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/scenario.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/recording.py`
- Test: `src/gnss_resilient_demo/test/test_engine.py`

**Interfaces:**
- Consumes: world, sensors and EKF.
- Produces: `Frame`, rolling metrics, source table rows, event log, CSV/JSON report.

- [ ] Write tests for deterministic replay, fixed-step pause/speed/reset, scripted fault separation, no-truth metrics and exported schema; observe failure.
- [ ] Implement the main 180-second scenario and evaluator/recorder outside `NavigationCore`.
- [ ] Run engine tests and expect all tests to pass.

### Task 5: Two-window Qt operator UI

**Files:**
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/gui.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/widgets.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/style.py`
- Test: `src/gnss_resilient_demo/test/test_gui.py`

**Interfaces:**
- Consumes: `ExperimentEngine` commands and `Frame` snapshots.
- Produces: `RangeWindow`, `NavigationWindow`, `run_gui()`.

- [ ] Write offscreen tests that both windows construct, controls mutate only fault configuration, layer toggles work and no-truth mode hides metrics; observe failure.
- [ ] Implement custom `QPainter` maps/plots, Russian controls, table, event log and side-by-side placement.
- [ ] Run GUI tests with `QT_QPA_PLATFORM=offscreen` and expect all tests to pass.

### Task 6: ROS package and Frob adapter boundary

**Files:**
- Create: `src/gnss_resilient_demo/package.xml`
- Create: `src/gnss_resilient_demo/setup.py`
- Create: `src/gnss_resilient_demo/setup.cfg`
- Create: `src/gnss_resilient_demo/resource/gnss_resilient_demo`
- Create: `src/gnss_resilient_demo/launch/demo.launch.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/ros_node.py`
- Create: `src/gnss_resilient_demo/gnss_resilient_demo/adapters.py`
- Test: `src/gnss_resilient_demo/test/test_adapters.py`

**Interfaces:**
- Consumes: standard ROS `Odometry`, `PoseWithCovarianceStamped` at runtime.
- Produces: internal measurement conversion and `/resilient_nav/odometry`; optional `map→odom` ownership is documented, not enabled by default.

- [ ] Write pure conversion tests for covariance, timestamp and yaw; observe failure.
- [ ] Implement converters and thin rclpy node without importing ROS in the core modules.
- [ ] Run adapter tests, then `colcon build --packages-select gnss_resilient_demo --symlink-install` and expect exit 0.

### Task 7: One-command launch, documentation and acceptance run

**Files:**
- Create: `run_demo.sh`
- Create: `README.md`
- Create: `DEMO_SCRIPT_RU.md`
- Create: `ARCHITECTURE.md`

**Interfaces:**
- Consumes: built or source package.
- Produces: one-command GUI, headless acceptance run and 3–5 minute demo script.

- [ ] Add a headless CLI acceptance test that runs the scenario, exports files and verifies finite metrics.
- [ ] Document exact setup/build/run commands, realized scope, simplifications, limits and Frob inspection commands.
- [ ] Run the full pytest suite, colcon build and a headless scenario; inspect generated CSV/JSON.
