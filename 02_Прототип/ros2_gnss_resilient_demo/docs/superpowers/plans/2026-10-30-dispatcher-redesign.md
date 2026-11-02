# Dispatcher Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the engineering demo into a clear road/dispatcher comparison with GNSS-only and fused modes plus working movable UWB beacons.

**Architecture:** `ExperimentEngine` exposes a mode-specific dispatcher output while preserving the truth boundary. UWB lives in the sensor layer as ranges and a derived pose. Both windows render a shared scalable road map, with the range window owning beacon dragging and the dispatcher window owning received-position presentation.

**Tech Stack:** Python 3.12, NumPy, PyQt5/QPainter, pytest. Существующий ROS 2 package остаётся совместимым, но не расширяется.

**Spec:** `docs/superpowers/specs/2026-10-30-dispatcher-redesign.md`

## Global Constraints

- Dispatcher GNSS-only output is raw GNSS or absent; it must never silently fall back to truth or EKF.
- Fused navigation still has no access to truth.
- UWB position must be derived from ranges to movable known beacons.
- All map geometry scales from widget dimensions.
- Russian labels, simple controls and explicit limitation copy remain mandatory.
- No rosbag, new ROS topics, hardware adapters or physical-device integration in this redesign.

## Review Focus

- Mode switching must clear mixed history and replay the same seed.
- GNSS-only spoofing must visibly move the dispatcher point rather than be gated.
- Fused mode must continue during GNSS loss and expose growing covariance.
- Moving one beacon must change its ranges without corrupting route truth.
- Disabling odometry, IMU, GNSS, camera or UWB must stop that method's fresh contribution.

### Task 1: Mode-specific dispatcher output and stable road following

**Files:** modify `engine.py`, `world.py`; test `test_engine.py`, `test_world.py`.

- [ ] Write failing tests for GNSS-only disappearance/spoof shift, fused continuity, reset-on-mode-change and road-corridor following.
- [ ] Add `NavigationMode`, `DispatcherState` and stable route controller behavior.
- [ ] Run focused and full core tests.

### Task 2: Movable UWB beacons and trilateration

**Files:** modify `models.py`, `world.py`, `world.json`, `measurements.py`, `sensors.py`, `engine.py`; create `test_uwb.py`.

- [ ] Write failing tests for ranges, independent RNG, insufficient geometry, trilateration accuracy, NLOS bias and beacon movement.
- [ ] Implement beacon model, UWB faults, range observations and least-squares position.
- [ ] Feed valid UWB poses through the existing gated update path and diagnostics.

### Task 3: Scalable colored maps and beacon dragging

**Files:** modify `widgets.py`; test `test_gui.py`.

- [ ] Write offscreen interaction tests for scaling, red car semantics and beacon hit/move behavior.
- [ ] Render green terrain, grey road, markings, red car, tunnel, camera landmarks and UWB links.
- [ ] Implement mouse drag with world-coordinate conversion and bounds clamping.

### Task 4: Simple two-mode operator and dispatcher UI

**Files:** modify `gui.py`, `style.py`; test `test_gui.py`.

- [ ] Write failing tests for mode selector, reset, method toggles and dispatcher freshness states.
- [ ] Replace the navigation debugger hierarchy with a road-first dispatcher map and concise source cards while retaining optional diagnostics.
- [ ] Verify both modes visually at multiple window sizes.

### Task 5: Documentation and acceptance comparison

**Files:** modify `README.md`, `DEMO_SCRIPT_RU.md`, `ARCHITECTURE.md`; update preview PNGs.

- [ ] Add headless comparison export for both modes using the same seed.
- [ ] Run full simulator tests, both long scenarios and visual renders; only smoke-check that the existing package still builds.
- [ ] Document implemented behavior, limits and demo sequence.
