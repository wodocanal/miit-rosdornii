from dataclasses import FrozenInstanceError

import pytest

from gnss_resilient_demo.models import Control, Vec2
from gnss_resilient_demo.world import WorldModel


def make_world(obstacles=None):
    return WorldModel.from_dict(
        {
            "bounds": [-5.0, -5.0, 30.0, 20.0],
            "route": [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0]],
            "initial_pose": [0.0, 0.0, 0.0],
            "vehicle": {"radius": 0.3, "auto_speed": 5.0, "max_yaw_rate": 1.4},
            "obstacles": obstacles or [],
            "tunnels": [],
            "gnss_zones": [],
            "markers": [],
        }
    )


def test_autoroute_is_deterministic_and_snapshot_is_immutable():
    first = make_world()
    second = make_world()

    for _ in range(100):
        a = first.step(0.02, Control(auto=True))
        b = second.step(0.02, Control(auto=True))

    assert (a.x, a.y, a.yaw, a.speed) == pytest.approx(
        (b.x, b.y, b.yaw, b.speed), abs=1e-12
    )
    assert a.x > 8.0
    with pytest.raises(FrozenInstanceError):
        a.x = 99.0


def test_manual_motion_cannot_cross_a_wall():
    world = make_world([{"id": "wall", "rect": [1.0, -2.0, 1.0, 4.0]}])

    snapshot = world.step(1.0, Control(speed=3.0, yaw_rate=0.0, auto=False))

    assert snapshot.x == pytest.approx(0.0)
    assert snapshot.y == pytest.approx(0.0)
    assert snapshot.collided is True


def test_line_of_sight_is_blocked_only_when_segment_crosses_obstacle():
    world = make_world([{"id": "wall", "rect": [1.0, -1.0, 1.0, 2.0]}])

    assert world.has_line_of_sight(Vec2(0.0, 0.0), Vec2(3.0, 0.0)) is False
    assert world.has_line_of_sight(Vec2(0.0, 2.0), Vec2(3.0, 2.0)) is True


def test_long_autodrive_stays_inside_road_corridor():
    config = __import__("json").loads(
        (__import__("pathlib").Path(__file__).parents[1] / "config" / "world.json").read_text()
    )
    world = WorldModel.from_dict(config)
    maximum_offset = 0.0

    for _ in range(6000):
        state = world.step(0.02, Control(auto=True))
        maximum_offset = max(maximum_offset, world.distance_to_route(Vec2(state.x, state.y)))

    assert state.collided is False
    assert maximum_offset <= 4.5


def test_virtual_beacon_can_be_moved_within_world_bounds():
    config = {
        "bounds": [0, 0, 20, 20],
        "route": [[1, 1], [19, 1]],
        "initial_pose": [1, 1, 0],
        "beacons": [{"id": "B1", "x": 3, "y": 4}],
    }
    world = WorldModel.from_dict(config)

    moved = world.move_beacon("B1", 25, -3)

    assert moved.id == "B1"
    assert (moved.x, moved.y) == (20.0, 0.0)
    assert world.beacon("B1") == moved


def test_map_catalog_contains_four_detailed_worlds_with_safe_autoroutes():
    config_dir = __import__("pathlib").Path(__file__).parents[1] / "config"
    paths = sorted(config_dir.glob("world_*.json"))

    assert {path.stem for path in paths} >= {
        "world_city",
        "world_fields",
        "world_forest",
        "world_tunnels",
    }
    for path in paths:
        world = WorldModel.from_json(path)
        assert world.name
        assert world.kind in {"city", "fields", "forest", "tunnels"}
        assert len(world.decorations) >= 4
        maximum_offset = 0.0
        collided = False
        for _ in range(4500):
            state = world.step(0.02, Control(auto=True))
            maximum_offset = max(maximum_offset, world.distance_to_route(Vec2(state.x, state.y)))
            collided = collided or state.collided
        assert not collided, path.name
        assert maximum_offset <= 4.5, path.name
