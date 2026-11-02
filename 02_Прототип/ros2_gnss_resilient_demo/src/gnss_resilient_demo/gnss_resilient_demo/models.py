"""Shared immutable geometry and world-state value objects."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float


@dataclass(frozen=True)
class Rect:
    id: str
    x: float
    y: float
    width: float
    height: float

    def contains(self, point: Vec2, margin: float = 0.0) -> bool:
        return (
            self.x - margin <= point.x <= self.x + self.width + margin
            and self.y - margin <= point.y <= self.y + self.height + margin
        )


@dataclass(frozen=True)
class Marker:
    id: int
    x: float
    y: float
    facing: float
    size: float = 0.6


@dataclass(frozen=True)
class Beacon:
    id: str
    x: float
    y: float
    coverage: float = 140.0


@dataclass(frozen=True)
class Control:
    speed: float = 0.0
    yaw_rate: float = 0.0
    auto: bool = False


@dataclass(frozen=True)
class WorldSnapshot:
    sim_time: float
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    collided: bool
    in_tunnel: bool
    in_gnss_zone: bool
