"""Configurable planar world and kinematic rover truth model."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .models import Beacon, Control, Marker, Rect, Vec2, WorldSnapshot


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class WorldModel:
    """Owns simulation truth. Navigation code must never receive this object."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.name = str(config.get("name", "Полигон"))
        self.kind = str(config.get("kind", "fields"))
        self.decorations = tuple(config.get("decorations", []))
        self.bounds = tuple(float(v) for v in config["bounds"])
        self.route = tuple(Vec2(float(p[0]), float(p[1])) for p in config["route"])
        self.obstacles = tuple(self._rect(item) for item in config.get("obstacles", []))
        self.tunnels = tuple(self._rect(item) for item in config.get("tunnels", []))
        self.gnss_zones = tuple(self._rect(item) for item in config.get("gnss_zones", []))
        self.markers = tuple(
            Marker(
                int(item["id"]),
                float(item["x"]),
                float(item["y"]),
                float(item["facing"]),
                float(item.get("size", 0.6)),
            )
            for item in config.get("markers", [])
        )
        self.beacons = tuple(
            Beacon(str(item["id"]), float(item["x"]), float(item["y"]), float(item.get("coverage", 140.0)))
            for item in config.get("beacons", [])
        )
        vehicle = config.get("vehicle", {})
        self.radius = float(vehicle.get("radius", 0.3))
        self.auto_speed = float(vehicle.get("auto_speed", 5.0))
        self.max_yaw_rate = float(vehicle.get("max_yaw_rate", 1.4))
        self.reset()

    @classmethod
    def from_dict(cls, config: dict[str, Any]) -> "WorldModel":
        return cls(config)

    @classmethod
    def from_json(cls, path: str | Path) -> "WorldModel":
        with Path(path).open(encoding="utf-8") as stream:
            return cls(json.load(stream))

    @staticmethod
    def _rect(item: dict[str, Any]) -> Rect:
        x, y, width, height = (float(v) for v in item["rect"])
        return Rect(str(item["id"]), x, y, width, height)

    def reset(self) -> WorldSnapshot:
        x, y, yaw = (float(v) for v in self.config.get("initial_pose", [0, 0, 0]))
        self._x, self._y, self._yaw = x, y, yaw
        self._speed = self._yaw_rate = self._time = 0.0
        self._route_index = 1 if len(self.route) > 1 else 0
        return self.snapshot(False)

    def snapshot(self, collided: bool = False) -> WorldSnapshot:
        point = Vec2(self._x, self._y)
        return WorldSnapshot(
            sim_time=self._time,
            x=self._x,
            y=self._y,
            yaw=self._yaw,
            speed=self._speed,
            yaw_rate=self._yaw_rate,
            collided=collided,
            in_tunnel=any(rect.contains(point) for rect in self.tunnels),
            in_gnss_zone=any(rect.contains(point) for rect in self.gnss_zones),
        )

    def step(self, dt: float, control: Control) -> WorldSnapshot:
        if dt <= 0.0:
            return self.snapshot(False)
        speed, yaw_rate = self._resolve_control(control)
        candidate_yaw = wrap_angle(self._yaw + yaw_rate * dt)
        candidate = Vec2(
            self._x + speed * math.cos(candidate_yaw) * dt,
            self._y + speed * math.sin(candidate_yaw) * dt,
        )
        collided = self._collides(Vec2(self._x, self._y), candidate)
        if not collided:
            self._x, self._y, self._yaw = candidate.x, candidate.y, candidate_yaw
            self._speed, self._yaw_rate = speed, yaw_rate
            self._advance_route()
        else:
            self._speed = self._yaw_rate = 0.0
        self._time += dt
        return self.snapshot(collided)

    def _resolve_control(self, control: Control) -> tuple[float, float]:
        if not control.auto or not self.route:
            return float(control.speed), max(-self.max_yaw_rate, min(self.max_yaw_rate, control.yaw_rate))
        target = self.route[self._route_index]
        desired = math.atan2(target.y - self._y, target.x - self._x)
        yaw_error = wrap_angle(desired - self._yaw)
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_error * 3.5))
        # A road vehicle must slow for a corner.  Keeping full speed while the
        # heading controller saturates creates wide, visibly unstable arcs.
        corner_factor = max(0.30, 1.0 - abs(yaw_error) / 1.15)
        return self.auto_speed * corner_factor, yaw_rate

    def _advance_route(self) -> None:
        if not self.route:
            return
        target = self.route[self._route_index]
        if math.hypot(target.x - self._x, target.y - self._y) < max(0.7, self.auto_speed * 0.1):
            self._route_index = (self._route_index + 1) % len(self.route)

    def _collides(self, start: Vec2, point: Vec2) -> bool:
        min_x, min_y, max_x, max_y = self.bounds
        if not (
            min_x + self.radius <= point.x <= max_x - self.radius
            and min_y + self.radius <= point.y <= max_y - self.radius
        ):
            return True
        return any(
            rect.contains(point, self.radius)
            or self._segment_intersects_rect(
                start,
                point,
                Rect(
                    rect.id,
                    rect.x - self.radius,
                    rect.y - self.radius,
                    rect.width + 2.0 * self.radius,
                    rect.height + 2.0 * self.radius,
                ),
            )
            for rect in self.obstacles
        )

    def has_line_of_sight(self, start: Vec2, end: Vec2) -> bool:
        return not any(self._segment_intersects_rect(start, end, rect) for rect in self.obstacles)

    def gnss_risk_at(self, point: Vec2) -> bool:
        """Mapped environment risk available to navigation without simulation truth.

        A production adapter can replace this query with road-map metadata.  The
        simulator calls it with the current estimate, never with the truth pose.
        """
        return self.kind == "city" or any(rect.contains(point) for rect in self.gnss_zones)

    def beacon(self, beacon_id: str) -> Beacon | None:
        return next((item for item in self.beacons if item.id == str(beacon_id)), None)

    def move_beacon(self, beacon_id: str, x: float, y: float) -> Beacon:
        """Move a virtual test beacon; coordinates are clamped to the map."""
        beacon_id = str(beacon_id)
        if self.beacon(beacon_id) is None:
            raise KeyError(f"unknown beacon: {beacon_id}")
        min_x, min_y, max_x, max_y = self.bounds
        previous = self.beacon(beacon_id)
        moved = Beacon(
            beacon_id,
            max(min_x, min(max_x, float(x))),
            max(min_y, min(max_y, float(y))),
            previous.coverage,
        )
        self.beacons = tuple(moved if item.id == beacon_id else item for item in self.beacons)
        return moved

    def distance_to_route(self, point: Vec2) -> float:
        """Shortest distance to the configured road centreline."""
        if not self.route:
            return math.inf
        if len(self.route) == 1:
            return math.hypot(point.x - self.route[0].x, point.y - self.route[0].y)
        return min(
            self._distance_to_segment(point, start, end)
            for start, end in zip(self.route, self.route[1:])
        )

    @staticmethod
    def _distance_to_segment(point: Vec2, start: Vec2, end: Vec2) -> float:
        dx, dy = end.x - start.x, end.y - start.y
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            return math.hypot(point.x - start.x, point.y - start.y)
        fraction = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
        fraction = max(0.0, min(1.0, fraction))
        nearest_x = start.x + fraction * dx
        nearest_y = start.y + fraction * dy
        return math.hypot(point.x - nearest_x, point.y - nearest_y)

    @staticmethod
    def _segment_intersects_rect(start: Vec2, end: Vec2, rect: Rect) -> bool:
        """Liang-Barsky segment/axis-aligned rectangle intersection."""
        dx, dy = end.x - start.x, end.y - start.y
        p = (-dx, dx, -dy, dy)
        q = (
            start.x - rect.x,
            rect.x + rect.width - start.x,
            start.y - rect.y,
            rect.y + rect.height - start.y,
        )
        lower, upper = 0.0, 1.0
        for pi, qi in zip(p, q):
            if abs(pi) < 1e-12:
                if qi < 0.0:
                    return False
                continue
            ratio = qi / pi
            if pi < 0.0:
                lower = max(lower, ratio)
            else:
                upper = min(upper, ratio)
            if lower > upper:
                return False
        return True
