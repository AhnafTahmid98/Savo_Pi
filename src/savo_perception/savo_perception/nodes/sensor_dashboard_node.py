#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Perception Sensor Dashboard (ROS2, curses)

Shows a live table for:
  - /depth/min_front_m
  - /savo_perception/range/front_ultrasonic_m
  - /savo_perception/range/left_m
  - /savo_perception/range/right_m
  - /safety/stop
  - /safety/slowdown_factor

- Dependency-free (uses built-in curses)
- Clean Ctrl+C handling (restores terminal)
"""

from __future__ import annotations

import math
import time
import curses
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Bool


def _is_valid_number(x: Optional[float]) -> bool:
    return x is not None and not (isinstance(x, float) and math.isnan(x))


def _fmt_m(x: Optional[float]) -> str:
    if not _is_valid_number(x):
        return "—"
    return f"{x:.3f} m"


def _fmt_bool(x: Optional[bool]) -> str:
    if x is None:
        return "—"
    return "TRUE" if x else "FALSE"


def _age_s(now: float, t: Optional[float]) -> str:
    if t is None:
        return "—"
    a = now - t
    if a < 0:
        a = 0.0
    return f"{a:.2f}s"


@dataclass
class TopicValue:
    value_f: Optional[float] = None
    value_b: Optional[bool] = None
    last_rx: Optional[float] = None


class SensorDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_dashboard")

        # ---------- Params ----------
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("stale_timeout_s", 0.30)

        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.refresh_hz = max(2.0, min(self.refresh_hz, 30.0))
        self.stale_timeout_s = max(0.05, min(self.stale_timeout_s, 5.0))

        # ---------- Storage ----------
        self.depth = TopicValue()
        self.ultra = TopicValue()
        self.left = TopicValue()
        self.right = TopicValue()
        self.stop = TopicValue()
        self.slow = TopicValue()

        # ---------- QoS ----------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        qos_ctrl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------- Subs ----------
        self.create_subscription(Float32, "/depth/min_front_m", self._on_depth, qos_sensor)
        self.create_subscription(Float32, "/savo_perception/range/front_ultrasonic_m", self._on_ultra, qos_sensor)
        self.create_subscription(Float32, "/savo_perception/range/left_m", self._on_left, qos_sensor)
        self.create_subscription(Float32, "/savo_perception/range/right_m", self._on_right, qos_sensor)

        self.create_subscription(Bool, "/safety/stop", self._on_stop, qos_ctrl)
        self.create_subscription(Float32, "/safety/slowdown_factor", self._on_slow, qos_ctrl)

        self.get_logger().info(
            f"SensorDashboard online | refresh={self.refresh_hz:.1f}Hz stale_timeout={self.stale_timeout_s:.2f}s"
        )

    # ---------- Callbacks ----------
    def _stamp(self) -> float:
        # monotonic time for ages
        return time.perf_counter()

    def _on_depth(self, msg: Float32) -> None:
        self.depth.value_f = float(msg.data)
        self.depth.last_rx = self._stamp()

    def _on_ultra(self, msg: Float32) -> None:
        self.ultra.value_f = float(msg.data)
        self.ultra.last_rx = self._stamp()

    def _on_left(self, msg: Float32) -> None:
        self.left.value_f = float(msg.data)
        self.left.last_rx = self._stamp()

    def _on_right(self, msg: Float32) -> None:
        self.right.value_f = float(msg.data)
        self.right.last_rx = self._stamp()

    def _on_stop(self, msg: Bool) -> None:
        self.stop.value_b = bool(msg.data)
        self.stop.last_rx = self._stamp()

    def _on_slow(self, msg: Float32) -> None:
        self.slow.value_f = float(msg.data)
        self.slow.last_rx = self._stamp()

    # ---------- Helpers ----------
    def _status(self, tv: TopicValue, now: float) -> str:
        if tv.last_rx is None:
            return "NO_DATA"
        if (now - tv.last_rx) > self.stale_timeout_s:
            return "STALE"
        return "OK"


def _draw(stdscr, node: SensorDashboardNode) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)

    # colors (optional)
    if curses.has_colors():
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK
        curses.init_pair(2, curses.COLOR_YELLOW, -1)  # STALE
        curses.init_pair(3, curses.COLOR_RED, -1)     # NO_DATA / STOP

    def color_for(status: str, is_stop: bool = False):
        if not curses.has_colors():
            return 0
        if is_stop:
            return curses.color_pair(3)
        if status == "OK":
            return curses.color_pair(1)
        if status == "STALE":
            return curses.color_pair(2)
        return curses.color_pair(3)

    period = 1.0 / node.refresh_hz

    while rclpy.ok():
        # let ROS callbacks run
        rclpy.spin_once(node, timeout_sec=0.0)

        now = time.perf_counter()

        rows = [
            ("Depth front min", "/depth/min_front_m", _fmt_m(node.depth.value_f), _age_s(now, node.depth.last_rx), node._status(node.depth, now)),
            ("Ultrasonic front", "/savo_perception/range/front_ultrasonic_m", _fmt_m(node.ultra.value_f), _age_s(now, node.ultra.last_rx), node._status(node.ultra, now)),
            ("ToF left", "/savo_perception/range/left_m", _fmt_m(node.left.value_f), _age_s(now, node.left.last_rx), node._status(node.left, now)),
            ("ToF right", "/savo_perception/range/right_m", _fmt_m(node.right.value_f), _age_s(now, node.right.last_rx), node._status(node.right, now)),
            ("STOP", "/safety/stop", _fmt_bool(node.stop.value_b), _age_s(now, node.stop.last_rx), node._status(node.stop, now)),
            ("Slowdown", "/safety/slowdown_factor", ("—" if not _is_valid_number(node.slow.value_f) else f"{node.slow.value_f:.2f}"), _age_s(now, node.slow.last_rx), node._status(node.slow, now)),
        ]

        # overall summary
        stop_active = (node.stop.value_b is True) and (node._status(node.stop, now) == "OK")

        stdscr.erase()
        h, w = stdscr.getmaxyx()

        title = "Robot Savo — Perception Dashboard  (Ctrl+C to exit)"
        stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)

        summary = f"stale_timeout={node.stale_timeout_s:.2f}s | refresh={node.refresh_hz:.1f}Hz"
        stdscr.addstr(1, 2, summary)

        if stop_active:
            stdscr.addstr(1, w - 18, "!! STOP ACTIVE !!", color_for("OK", is_stop=True) | curses.A_BOLD)

        # table header
        y = 3
        header = f"{'Signal':<18} {'Topic':<44} {'Value':<12} {'Age':<7} {'Status':<7}"
        stdscr.addstr(y, 2, header, curses.A_UNDERLINE | curses.A_BOLD)
        y += 1

        # table rows
        for name, topic, val, age, status in rows:
            line = f"{name:<18} {topic:<44} {val:<12} {age:<7} {status:<7}"
            attr = color_for(status, is_stop=(name == "STOP" and val == "TRUE"))
            stdscr.addstr(y, 2, line, attr)
            y += 1
            if y >= h - 2:
                break

        stdscr.addstr(h - 1, 2, "Tip: if Depth shows NO_DATA/STALE, confirm realsense2_camera is running and topic QoS matches.")
        stdscr.refresh()

        # allow Ctrl+C to work and not block
        time.sleep(period)


def main() -> None:
    rclpy.init()
    node = SensorDashboardNode()
    try:
        curses.wrapper(_draw, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
