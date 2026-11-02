"""Custom lightweight QPainter maps and plots."""

from __future__ import annotations

import math

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget

from .engine import Frame, NavigationMode
from .style import COLORS


class WorldCanvas(QWidget):
    CAMERA_HIGHLIGHT_SECONDS = 0.75
    UWB_LINK_HIGHLIGHT_SECONDS = 0.45

    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self.frame = engine.frame()
        self._dragged_beacon = None
        self._world_identity = id(engine.world)
        self._marker_seen_at: dict[int, float] = {}
        self._beacon_seen_at: dict[str, float] = {}
        self.show_uwb_coverage = False
        self.setMinimumSize(480, 400)
        self.setMouseTracking(True)
        self._remember_sensor_activity(self.frame)

    def set_frame(self, frame: Frame) -> None:
        if id(self.engine.world) != self._world_identity or frame.sim_time < self.frame.sim_time:
            self._world_identity = id(self.engine.world)
            self._marker_seen_at.clear()
            self._beacon_seen_at.clear()
        self.frame = frame
        self._remember_sensor_activity(frame)
        self.update()

    def _remember_sensor_activity(self, frame: Frame) -> None:
        camera = self.engine.sensors.faults.camera
        uwb = self.engine.sensors.faults.uwb
        if not (camera.enabled and camera.automatic_enabled) or camera.hide_all or camera.automatic_hide_all:
            self._marker_seen_at.clear()
        else:
            for observation in frame.packet.camera:
                self._marker_seen_at[observation.marker_id] = observation.measured_at
        if not (uwb.enabled and uwb.automatic_enabled):
            self._beacon_seen_at.clear()
        else:
            for measurement in frame.packet.uwb_ranges:
                self._beacon_seen_at[measurement.beacon_id] = measurement.measured_at

    def recognized_marker_ids(self) -> set[int]:
        now = self.frame.sim_time
        return {
            marker_id for marker_id, measured_at in self._marker_seen_at.items()
            if now - measured_at <= self.CAMERA_HIGHLIGHT_SECONDS
        }

    def active_beacon_ids(self) -> set[str]:
        now = self.frame.sim_time
        return {
            beacon_id for beacon_id, measured_at in self._beacon_seen_at.items()
            if now - measured_at <= self.UWB_LINK_HIGHLIGHT_SECONDS
        }

    def set_uwb_coverage(self, enabled: bool) -> None:
        self.show_uwb_coverage = bool(enabled)
        self.update()

    def infrastructure_visibility(self) -> dict[str, bool]:
        fusion = self.engine.navigation_mode is NavigationMode.FUSION
        camera = self.engine.sensors.faults.camera
        return {
            "markers": fusion and camera.enabled and camera.automatic_enabled and not camera.hide_all and not camera.automatic_hide_all,
            "beacons": fusion and self.engine.sensors.faults.uwb.enabled and self.engine.sensors.faults.uwb.automatic_enabled,
        }

    def _point(self, x: float, y: float) -> QPointF:
        scale, left, bottom = self._transform()
        min_x, min_y, max_x, max_y = self.engine.world.bounds
        return QPointF(left + (x - min_x) * scale, bottom - (y - min_y) * scale)

    def _transform(self):
        min_x, min_y, max_x, max_y = self.engine.world.bounds
        margin = 22.0
        scale = min(
            max(1.0, self.width() - 2 * margin) / (max_x - min_x),
            max(1.0, self.height() - 2 * margin) / (max_y - min_y),
        )
        map_width = (max_x - min_x) * scale
        map_height = (max_y - min_y) * scale
        left = (self.width() - map_width) / 2.0
        bottom = (self.height() + map_height) / 2.0
        return scale, left, bottom

    def _world_point(self, point: QPointF) -> tuple[float, float]:
        scale, left, bottom = self._transform()
        min_x, min_y, _, _ = self.engine.world.bounds
        return min_x + (point.x() - left) / scale, min_y + (bottom - point.y()) / scale

    def _rect(self, rect) -> QRectF:
        top_left = self._point(rect.x, rect.y + rect.height)
        bottom_right = self._point(rect.x + rect.width, rect.y)
        return QRectF(top_left, bottom_right)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        world = self.engine.world
        backgrounds = {"city": "#9da6a8", "fields": "#91bd67", "forest": "#567f4d", "tunnels": "#9b927d"}
        painter.fillRect(self.rect(), QColor(backgrounds.get(world.kind, "#8fbd69")))
        self._draw_decorations(painter)
        visibility = self.infrastructure_visibility()
        if visibility["beacons"] and self.show_uwb_coverage:
            self._draw_uwb_coverage(painter)
        if world.route:
            points = QPolygonF([self._point(item.x, item.y) for item in world.route])
            road_width = max(22.0, min(54.0, self._transform()[0] * 8.0))
            painter.setPen(QPen(QColor("#d6d9da"), road_width + 5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawPolyline(points)
            painter.setPen(QPen(QColor("#5e6367"), road_width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawPolyline(points)
            painter.setPen(QPen(QColor("#f4f1cf"), 2, Qt.DashLine, Qt.RoundCap))
            painter.drawPolyline(points)
        for zone in world.gnss_zones:
            painter.fillRect(self._rect(zone), QColor(255, 98, 109, 42))
            painter.setPen(QPen(QColor(COLORS["red"]), 1, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(self._rect(zone))
            painter.drawText(self._rect(zone), Qt.AlignTop | Qt.AlignHCenter, "ЗОНА ПОМЕХ GNSS")
        for obstacle in world.obstacles:
            painter.fillRect(self._rect(obstacle), QColor("#c7ae8a"))
            painter.setPen(QPen(QColor("#765e49"), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(self._rect(obstacle))
        for tunnel in world.tunnels:
            painter.fillRect(self._rect(tunnel), QColor(84, 207, 229, 55))
            painter.setPen(QPen(QColor(COLORS["cyan"]), 2, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(self._rect(tunnel))
            painter.drawText(self._rect(tunnel), Qt.AlignTop | Qt.AlignHCenter, "ТОННЕЛЬ")
        visible = self.recognized_marker_ids()
        rover = self.frame.truth
        if visibility["markers"]:
            for marker in world.markers:
                point = self._point(marker.x, marker.y)
                active = marker.id in visible
                color = QColor(COLORS["amber"] if active else COLORS["cyan"])
                painter.setPen(QPen(color, 4 if active else 2))
                painter.setBrush(QBrush(color if active else QColor("#17353a")))
                size = 18 if active else 14
                painter.drawRect(QRectF(point.x() - size / 2, point.y() - size / 2, size, size))
                painter.drawText(point + QPointF(10, -9), str(marker.id))
                if active and rover is not None:
                    painter.setPen(QPen(QColor(COLORS["amber"]), 3, Qt.DotLine))
                    painter.drawLine(self._point(rover.x, rover.y), point)
        active_beacons = self.active_beacon_ids()
        if visibility["beacons"]:
            for beacon in world.beacons:
                point = self._point(beacon.x, beacon.y)
                active = beacon.id in active_beacons
                if active and rover is not None:
                    painter.setPen(QPen(QColor(132, 88, 214, 130), 2, Qt.DashLine))
                    painter.drawLine(self._point(rover.x, rover.y), point)
                painter.setPen(QPen(QColor("#ffffff"), 3))
                painter.setBrush(QBrush(QColor("#7045c5" if active else "#8f819d")))
                painter.drawEllipse(point, 10, 10)
                painter.setPen(QColor("#29133d"))
                painter.drawText(point + QPointF(13, -9), beacon.id)
        if rover is not None and visibility["markers"]:
            self._draw_fov(painter, rover)
        trail = [point.truth for point in self.frame.history if point.truth is not None]
        if len(trail) > 1:
            painter.setPen(QPen(QColor(255, 255, 255, 150), 2, Qt.DotLine))
            painter.drawPolyline(QPolygonF([self._point(item.x, item.y) for item in trail[-1600:]]))
        if rover is not None:
            self._draw_rover(painter, rover.x, rover.y, rover.yaw, QColor("#df2f38"))
        painter.setPen(QColor("#18241b"))
        footer = "Маяки можно перетаскивать мышью" if visibility["beacons"] else ""
        painter.drawText(14, self.height() - 8, footer)

    def _decoration_rect(self, values) -> QRectF:
        x, y, width, height = (float(value) for value in values)
        return QRectF(self._point(x, y + height), self._point(x + width, y))

    def _draw_decorations(self, painter):
        painter.setPen(Qt.NoPen)
        for item in self.engine.world.decorations:
            kind = item.get("type", "field")
            if "rect" in item:
                rect = self._decoration_rect(item["rect"])
            if kind == "field":
                painter.setBrush(QColor(item.get("color", "#cfbd65"))); painter.drawRoundedRect(rect, 12, 12)
                if item.get("rows"):
                    painter.setPen(QPen(QColor(110, 91, 48, 90), 2, Qt.DashLine))
                    for offset in range(8, int(rect.height()), 11):
                        painter.drawLine(
                            QPointF(rect.left(), rect.top() + offset),
                            QPointF(rect.right(), rect.top() + offset),
                        )
                    painter.setPen(Qt.NoPen)
            elif kind in {"water", "pond", "lake", "river"}:
                painter.setBrush(QColor("#62a9c5")); painter.drawRoundedRect(rect, 18, 18)
                painter.setPen(QPen(QColor(220, 245, 250, 130), 2))
                for offset in range(8, int(rect.height()), 14):
                    painter.drawLine(
                        QPointF(rect.left() + 5, rect.top() + offset),
                        QPointF(rect.right() - 5, rect.top() + offset),
                    )
                painter.setPen(Qt.NoPen)
            elif kind in {"forest", "orchard"}:
                painter.setBrush(QColor("#3f6e3c")); painter.drawRoundedRect(rect, 18, 18)
                density = int(item.get("density", 18))
                cols = max(3, int(math.sqrt(density * max(rect.width(), 1) / max(rect.height(), 1))))
                rows = max(2, math.ceil(density / cols))
                for row in range(rows):
                    for column in range(cols):
                        if row * cols + column >= density:
                            break
                        x = rect.left() + (column + 0.6 + 0.25 * (row % 2)) * rect.width() / cols
                        y = rect.top() + (row + 0.6) * rect.height() / rows
                        painter.setBrush(QColor("#244f31" if (row + column) % 2 else "#2f6038"))
                        painter.drawEllipse(QPointF(x, y), 4.5, 4.5)
            elif kind in {"park", "parking", "rock", "bridge"}:
                colors = {"park":"#72a85c", "parking":"#778083", "rock":"#756f64", "bridge":"#9b7650"}
                painter.setBrush(QColor(colors[kind])); painter.drawRoundedRect(rect, 8, 8)
            elif kind == "crosswalk":
                center = self._point(float(item["x"]), float(item["y"]))
                painter.setBrush(QColor("#f2efe0"))
                for offset in range(-12, 13, 6):
                    painter.drawRect(QRectF(center.x() + offset, center.y() - 9, 3, 18))
            elif kind == "label":
                painter.setPen(QPen(QColor(25, 35, 28, 180), 1))
                painter.drawText(self._point(float(item["x"]), float(item["y"])), str(item["text"]))
                painter.setPen(Qt.NoPen)

    def _draw_uwb_coverage(self, painter):
        for beacon in self.engine.world.beacons:
            center = self._point(beacon.x, beacon.y)
            radius = abs(self._point(beacon.x + beacon.coverage, beacon.y).x() - center.x())
            painter.setPen(QPen(QColor(112, 69, 197, 150), 2, Qt.DashLine))
            painter.setBrush(QBrush(QColor(112, 69, 197, 22)))
            painter.drawEllipse(center, radius, radius)

    def _draw_grid(self, painter):
        painter.setPen(QPen(QColor(255, 255, 255, 38), 1, Qt.DotLine))
        for x in range(-10, 111, 10):
            painter.drawLine(self._point(x, -10), self._point(x, 70))
        for y in range(-10, 71, 10):
            painter.drawLine(self._point(-10, y), self._point(110, y))

    def _draw_fov(self, painter, rover):
        max_range = self.engine.sensors.faults.camera.max_range
        half = self.engine.sensors.faults.camera.fov / 2.0
        origin = self._point(rover.x, rover.y)
        polygon = [origin]
        for index in range(19):
            angle = rover.yaw - half + index * (2 * half / 18)
            polygon.append(self._point(rover.x + max_range * math.cos(angle), rover.y + max_range * math.sin(angle)))
        painter.setPen(QPen(QColor(84, 207, 229, 90), 1))
        painter.setBrush(QBrush(QColor(84, 207, 229, 28)))
        painter.drawPolygon(QPolygonF(polygon))

    def _draw_rover(self, painter, x, y, yaw, color):
        center = self._point(x, y)
        painter.save()
        painter.translate(center)
        painter.rotate(-math.degrees(yaw))
        painter.setPen(QPen(QColor("#77131a"), 2))
        painter.setBrush(QBrush(color))
        painter.drawRoundedRect(QRectF(-13, -8, 26, 16), 5, 5)
        painter.setBrush(QColor("#b9d8e5"))
        painter.drawRect(QRectF(0, -6, 7, 12))
        painter.setBrush(QColor("#22272a"))
        for wheel in (QRectF(-8, -10, 8, 3), QRectF(-8, 7, 8, 3)):
            painter.drawRoundedRect(wheel, 1, 1)
        painter.restore()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.infrastructure_visibility()["beacons"]:
            nearest = None
            for beacon in self.engine.world.beacons:
                distance = math.hypot(event.pos().x() - self._point(beacon.x, beacon.y).x(), event.pos().y() - self._point(beacon.x, beacon.y).y())
                if distance <= 15.0 and (nearest is None or distance < nearest[0]):
                    nearest = (distance, beacon.id)
            self._dragged_beacon = nearest[1] if nearest else None
            if self._dragged_beacon:
                self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._dragged_beacon:
            x, y = self._world_point(event.pos())
            self.engine.world.move_beacon(self._dragged_beacon, x, y)
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragged_beacon = None
            self.unsetCursor()


class NavigationCanvas(QWidget):
    _decoration_rect = WorldCanvas._decoration_rect
    _draw_decorations = WorldCanvas._draw_decorations

    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self.frame = engine.frame()
        self._world_identity = id(engine.world)
        self._marker_seen_at: dict[int, float] = {}
        self.layers = {"truth": False, "gnss": False, "odometry": False, "camera": False, "estimate": True}
        self.setMinimumSize(480, 360)
        self._remember_marker_activity(self.frame)

    def set_frame(self, frame: Frame) -> None:
        if id(self.engine.world) != self._world_identity or frame.sim_time < self.frame.sim_time:
            self._world_identity = id(self.engine.world)
            self._marker_seen_at.clear()
        self.frame = frame
        self._remember_marker_activity(frame)
        self.update()

    def _remember_marker_activity(self, frame: Frame) -> None:
        camera = self.engine.sensors.faults.camera
        if not (camera.enabled and camera.automatic_enabled) or camera.hide_all or camera.automatic_hide_all:
            self._marker_seen_at.clear()
            return
        for observation in frame.packet.camera:
            self._marker_seen_at[observation.marker_id] = observation.measured_at

    def recognized_marker_ids(self) -> set[int]:
        return {
            marker_id for marker_id, measured_at in self._marker_seen_at.items()
            if self.frame.sim_time - measured_at <= WorldCanvas.CAMERA_HIGHLIGHT_SECONDS
        }

    def set_layer(self, name: str, enabled: bool) -> None:
        self.layers[name] = enabled
        self.update()

    def _point(self, x, y):
        min_x, min_y, max_x, max_y = self.engine.world.bounds
        margin = 22.0
        scale = min((self.width() - 2 * margin) / (max_x - min_x), (self.height() - 2 * margin) / (max_y - min_y))
        map_width = (max_x - min_x) * scale
        map_height = (max_y - min_y) * scale
        left = (self.width() - map_width) / 2.0
        bottom = (self.height() + map_height) / 2.0
        return QPointF(
            left + (x - min_x) * scale,
            bottom - (y - min_y) * scale,
        )

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        backgrounds = {"city": "#a8afb0", "fields": "#9bc578", "forest": "#62895a", "tunnels": "#a49b87"}
        painter.fillRect(self.rect(), QColor(backgrounds.get(self.engine.world.kind, "#9bc578")))
        self._draw_decorations(painter)
        route = self.engine.world.route
        if route:
            points = QPolygonF([self._point(item.x, item.y) for item in route])
            painter.setPen(QPen(QColor("#d8dcdd"), 42, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)); painter.drawPolyline(points)
            painter.setPen(QPen(QColor("#62676b"), 36, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)); painter.drawPolyline(points)
            painter.setPen(QPen(QColor("#f5f1ce"), 2, Qt.DashLine)); painter.drawPolyline(points)
        for obstacle in self.engine.world.obstacles:
            top_left = self._point(obstacle.x, obstacle.y + obstacle.height)
            bottom_right = self._point(obstacle.x + obstacle.width, obstacle.y)
            painter.fillRect(QRectF(top_left, bottom_right), QColor("#c9b08c"))
            painter.setPen(QPen(QColor("#765e49"), 2)); painter.setBrush(Qt.NoBrush)
            painter.drawRect(QRectF(top_left, bottom_right))
        for tunnel in self.engine.world.tunnels:
            top_left = self._point(tunnel.x, tunnel.y + tunnel.height)
            bottom_right = self._point(tunnel.x + tunnel.width, tunnel.y)
            painter.fillRect(QRectF(top_left, bottom_right), QColor(60, 65, 70, 120))
        for zone in self.engine.world.gnss_zones:
            top_left = self._point(zone.x, zone.y + zone.height)
            bottom_right = self._point(zone.x + zone.width, zone.y)
            painter.setPen(QPen(QColor(COLORS["red"]), 2, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.fillRect(QRectF(top_left, bottom_right), QColor(255, 98, 109, 26))
            painter.drawRect(QRectF(top_left, bottom_right))
            painter.drawText(QRectF(top_left, bottom_right), Qt.AlignTop | Qt.AlignHCenter, "ПОМЕХИ GNSS")
        if self.engine.navigation_mode is NavigationMode.FUSION:
            if self.engine.sensors.faults.camera.enabled:
                recognized = self.recognized_marker_ids()
                for marker in self.engine.world.markers:
                    point = self._point(marker.x, marker.y)
                    active = marker.id in recognized
                    color = QColor(COLORS["amber"] if active else COLORS["cyan"])
                    painter.setPen(QPen(color, 3 if active else 2))
                    painter.setBrush(color if active else QColor("#17353a"))
                    size = 14 if active else 10
                    painter.drawRect(QRectF(point.x() - size / 2, point.y() - size / 2, size, size))
            if self.engine.sensors.faults.uwb.enabled:
                for beacon in self.engine.world.beacons:
                    point = self._point(beacon.x, beacon.y)
                    painter.setPen(QPen(QColor("#ffffff"), 2)); painter.setBrush(QColor(COLORS["purple"]))
                    painter.drawEllipse(point, 7, 7)
        history = self.frame.history
        self._line(painter, [p.truth for p in history if p.truth], "truth", COLORS["truth"], Qt.DotLine, 2)
        self._line(painter, [p.odometry for p in history], "odometry", COLORS["orange"], Qt.DashLine, 2)
        if self.layers["gnss"]:
            painter.setPen(QPen(QColor(COLORS["red"]), 4))
            for point in history:
                if point.gnss:
                    painter.drawPoint(self._point(*point.gnss))
        if self.layers["camera"]:
            painter.setPen(QPen(QColor(COLORS["cyan"]), 6, Qt.SolidLine, Qt.RoundCap))
            for point in history:
                if point.camera:
                    painter.drawPoint(self._point(*point.camera))
        dispatcher_points = [p.dispatcher for p in history if p.dispatcher is not None]
        if self.layers["estimate"] and len(dispatcher_points) > 1:
            color = COLORS["red"] if self.frame.dispatcher.mode.value == "gnss_only" else COLORS["green"]
            painter.setPen(QPen(QColor(color), 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawPolyline(QPolygonF([self._point(p.x, p.y) for p in dispatcher_points[-2500:]]))
        if self.frame.dispatcher.pose is not None:
            self._draw_covariance(painter)
            pose = self.frame.dispatcher.pose
            center = self._point(pose.x, pose.y)
            color = QColor(COLORS["red"] if self.frame.dispatcher.mode.value == "gnss_only" else COLORS["green"])
            painter.setPen(QPen(QColor("#ffffff"), 3)); painter.setBrush(QBrush(color)); painter.drawEllipse(center, 9, 9)
            painter.setPen(QPen(color, 3)); painter.drawLine(center, self._point(pose.x + 4 * math.cos(pose.yaw), pose.y + 4 * math.sin(pose.yaw)))
        else:
            painter.setPen(QColor("#7b242c"))
            painter.setFont(painter.font())
            painter.drawText(self.rect(), Qt.AlignCenter, "НЕТ КООРДИНАТ\nТРАНСПОРТ НЕ ОТОБРАЖАЕТСЯ")

    def _line(self, painter, points, layer, color, style, width):
        if not self.layers[layer] or len(points) < 2:
            return
        painter.setPen(QPen(QColor(color), width, style))
        painter.drawPolyline(QPolygonF([self._point(point.x, point.y) for point in points[-4000:]]))

    def _draw_covariance(self, painter):
        if self.frame.dispatcher.covariance is None or self.frame.dispatcher.pose is None:
            return
        xx, xy, yy = self.frame.dispatcher.covariance
        covariance = np.asarray(((xx, xy), (xy, yy)))
        values, vectors = np.linalg.eigh(covariance)
        values = np.maximum(values, 0.0)
        angle = math.degrees(math.atan2(vectors[1, 1], vectors[0, 1]))
        min_x, min_y, max_x, max_y = self.engine.world.bounds
        scale_x = (self.width() - 56) / (max_x - min_x)
        scale_y = (self.height() - 56) / (max_y - min_y)
        axes = 2.4477 * np.sqrt(values)
        center = self._point(self.frame.dispatcher.pose.x, self.frame.dispatcher.pose.y)
        painter.save()
        painter.translate(center)
        painter.rotate(-angle)
        painter.setPen(QPen(QColor(81, 216, 138, 190), 2, Qt.DashLine))
        painter.setBrush(QBrush(QColor(81, 216, 138, 24)))
        painter.drawEllipse(QRectF(-axes[1] * scale_x, -axes[0] * scale_y, 2 * axes[1] * scale_x, 2 * axes[0] * scale_y))
        painter.restore()


class ErrorPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.frame = None
        self.setMinimumHeight(160)

    def set_frame(self, frame: Frame):
        self.frame = frame
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111920"))
        painter.setPen(QColor(COLORS["muted"]))
        painter.drawText(10, 32, "ФАКТИЧЕСКАЯ ОШИБКА, м")
        if not self.frame:
            return
        values = [(p.sim_time, p.dispatcher_error) for p in self.frame.history if p.dispatcher_error is not None]
        if len(values) < 2:
            return
        values = values[-1500:]
        t0, t1 = values[0][0], values[-1][0]
        maximum = max(1.0, max(item[1] for item in values) * 1.1)
        points = []
        for time, value in values:
            x = 36 + (time - t0) / max(t1 - t0, 1e-6) * (self.width() - 46)
            y = self.height() - 14 - value / maximum * (self.height() - 68)
            points.append(QPointF(x, y))
        painter.setPen(QPen(QColor(COLORS["red"]), 2))
        painter.drawPolyline(QPolygonF(points))
        painter.setPen(QColor(COLORS["muted"]))
        painter.drawText(5, 62, f"{maximum:.1f}")
