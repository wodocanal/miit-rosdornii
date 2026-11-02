# Large UI and Map Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a readable two-mode simulator UI with strict source visibility, UWB coverage display, and four detailed selectable maps.

**Architecture:** Extend JSON worlds with presentation-only decorations and metadata while keeping truth/navigation separation intact. Mode changes control sensor defaults and UI visibility; canvases render infrastructure only when its source is enabled. World switching performs a deterministic engine reset.

**Tech Stack:** Python 3.12, PyQt5 QPainter, NumPy, pytest, ROS 2 Jazzy packaging.

**Spec:** `docs/superpowers/specs/2026-10-30-large-ui-map-library-design.md`

## Global Constraints

- Pure simulator only; no rosbag or hardware integration.
- Minimum body text is 16 px and primary controls are at least 44 px high.
- GNSS-only mode must not expose or draw local positioning infrastructure.
- All localization sources default to enabled in fusion mode.

## Review Focus

- Switching modes after manually disabling sources restores the required defaults.
- Switching maps preserves the selected navigation mode and deterministic seed.
- Disabled camera/UWB infrastructure is absent rather than merely faded.
- UWB coverage circles scale correctly with window resizing.
- Every map supports a long collision-free autoroute.

---

### Task 1: Mode-specific source policy

**Files:**
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/engine.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/gui.py`
- Test: `src/gnss_resilient_demo/test/test_engine.py`
- Test: `src/gnss_resilient_demo/test/test_gui.py`

**Interfaces:**
- Produces: `ExperimentEngine.set_navigation_mode()` applies mode defaults; `RangeWindow.apply_mode_visibility()` hides irrelevant panels.

- [ ] Write failing tests for mode defaults and panel visibility.
- [ ] Run targeted tests and confirm failure.
- [ ] Implement source policy and simplified GNSS fault controls.
- [ ] Run full tests and commit.

### Task 2: Map catalog and deterministic switching

**Files:**
- Create: `src/gnss_resilient_demo/config/world_city.json`
- Create: `src/gnss_resilient_demo/config/world_fields.json`
- Create: `src/gnss_resilient_demo/config/world_forest.json`
- Create: `src/gnss_resilient_demo/config/world_tunnels.json`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/world.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/engine.py`
- Test: `src/gnss_resilient_demo/test/test_world.py`

**Interfaces:**
- Produces: world metadata/decorations and `ExperimentEngine.set_world(path)`.

- [ ] Write failing catalog, switch, and long-route tests.
- [ ] Run targeted tests and confirm failure.
- [ ] Add four presets and deterministic switching.
- [ ] Run full tests and commit.

### Task 3: Large scalable visual system and coverage layers

**Files:**
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/style.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/widgets.py`
- Modify: `src/gnss_resilient_demo/gnss_resilient_demo/gui.py`
- Test: `src/gnss_resilient_demo/test/test_gui.py`

**Interfaces:**
- Produces: large typography, map selector, coverage toggle, conditional infrastructure rendering.

- [ ] Write failing UI tests for font size, visibility and coverage toggle.
- [ ] Run targeted tests and confirm failure.
- [ ] Implement scalable rendering and controls.
- [ ] Render screenshots at two sizes, run full tests, build and commit.

### Task 4: Documentation and acceptance

**Files:**
- Modify: `README.md`
- Modify: `DEMO_SCRIPT_RU.md`

- [ ] Document the mode contract, maps and coverage control.
- [ ] Run all tests, ROS build, four long autoroutes and a 130-second scenario.
- [ ] Launch the final simulator and commit documentation.
