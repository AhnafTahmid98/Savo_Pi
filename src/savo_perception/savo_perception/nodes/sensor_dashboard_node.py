#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — Perception Sensor Dashboard (ROS2 Jazzy)
-----------------------------------------------------
Terminal UI dashboard showing:
- Depth front min (/depth/min_front_m)
- Front ultrasonic (/savo_perception/range/front_ultrasonic_m)
- Left/Right ToF (/savo_perception/range/left_m, /savo_perception/range/right_m)
- Safety outputs (/safety/stop, /safety/slowdown_factor)
- cmd_vel_safe (/cmd_vel_safe) if running cmd_vel_safety_gate

Key fix:
- Subscribe with QoS compatible with sensor publishers (BEST_EFFORT).
  Uses qos_profile_sensor_data for sensor-ish streams to avoid RELIABILITY mismatch.

Run:
  ros2 run savo_perception sensor_dashboard
"""

from __future__ import annotations

import math
import time
import curses
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


@dataclass
class Val:
    value: Optional[float] = None
    t_last: float = 0.0


def _fmt_m(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return f"{v:0.3f} m"


def _fmt_f(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return f"{v:0.2f}"


def _fmt_bool(v: Optional[bool]) -> str:
    if v is None:
        return "—"
    return "TRUE" if v else "FALSE"


class SensorDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_dashboard")

        # Params
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("refresh_hz", 10.0)

        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)

        # Topic params
        self.declare_parameter("depth_topic", "/depth/min_front_m")
        self.declare_parameter("ultra_topic", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("left_topic", "/savo_perception/range/left_m")
        self.declare_parameter("right_topic", "/savo_perception/range/right_m")
        self.declare_parameter("stop_topic", "/safety/stop")
        self.declare_parameter("slow_topic", "/safety/slowdown_factor")
        self.declare_parameter("cmd_safe_topic", "/cmd_vel_safe")

        self.t_depth = str(self.get_parameter("depth_topic").value)
        self.t_ultra = str(self.get_parameter("ultra_topic").value)
        self.t_left = str(self.get_parameter("left_topic").value)
        self.t_right = str(self.get_parameter("right_topic").value)
        self.t_stop = str(self.get_parameter("stop_topic").value)
        self.t_slow = str(self.get_parameter("slow_topic").value)
        self.t_cmd_safe = str(self.get_parameter("cmd_safe_topic").value)

        # Storage
        self.depth = Val()
        self.ultra = Val()
        self.left = Val()
        self.right = Val()
        self.stop = Val()
        self.slow = Val()
        self.cmd = {"vx": Val(), "vy": Val(), "wz": Val()}

        # ---------------- QoS Profiles ----------------
        # Sensor streams (ToF/ultrasonic/cmd_vel_safe, and depth typically) are best with sensor profile:
        # - BEST_EFFORT, VOLATILE, small queue
        sensor_qos = qos_profile_sensor_data

        # Safety outputs often are RELIABLE; subscribing as RELIABLE is safe (it can also receive BEST_EFFORT? no).
        # We'll keep stop/slow RELIABLE to match typical publishers.
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Depth: your depth_front_min_node publishes RELIABLE, so RELIABLE subscription is fine.
        # But using sensor_qos also works if publisher is BEST_EFFORT (not the case here).
        # We'll use RELIABLE for depth to match your current setup.
        depth_qos = reliable_qos

        # ---------------- Subscriptions ----------------
        # Depth (RELIABLE publisher from depth_front_min_node)
        self.create_subscription(Float32, self.t_depth, self._on_depth, depth_qos)

        # Ultrasonic + ToF publishers are sensor style (BEST_EFFORT) -> MUST use sensor_qos.
        self.create_subscription(Float32, self.t_ultra, self._on_ultra, sensor_qos)
        self.create_subscription(Float32, self.t_left, self._on_left, sensor_qos)
        self.create_subscription(Float32, self.t_right, self._on_right, sensor_qos)

        # Safety outputs (typically RELIABLE)
        self.create_subscription(Bool, self.t_stop, self._on_stop, reliable_qos)
        self.create_subscription(Float32, self.t_slow, self._on_slow, reliable_qos)

        # cmd_vel_safe often published at high rate, sensor-ish -> use sensor_qos to avoid mismatch
        self.create_subscription(Twist, self.t_cmd_safe, self._on_cmd, sensor_qos)

        self.get_logger().info(
            f"Dashboard running. stale_timeout_s={self.stale_timeout_s:0.2f}, refresh_hz={self.refresh_hz:0.1f} "
            f"| QoS: depth=RELIABLE, tof/ultra/cmd=BEST_EFFORT(sensor), stop/slow=RELIABLE"
        )

    def _now(self) -> float:
        return time.monotonic()

    def _is_stale(self, t_last: float) -> bool:
        if t_last <= 0.0:
            return True
        return (self._now() - t_last) > self.stale_timeout_s

    def _on_depth(self, msg: Float32) -> None:
        self.depth.value = float(msg.data)
        self.depth.t_last = self._now()

    def _on_ultra(self, msg: Float32) -> None:
        self.ultra.value = float(msg.data)
        self.ultra.t_last = self._now()

    def _on_left(self, msg: Float32) -> None:
        self.left.value = float(msg.data)
        self.left.t_last = self._now()

    def _on_right(self, msg: Float32) -> None:
        self.right.value = float(msg.data)
        self.right.t_last = self._now()

    def _on_stop(self, msg: Bool) -> None:
        self.stop.value = bool(msg.data)
        self.stop.t_last = self._now()

    def _on_slow(self, msg: Float32) -> None:
        self.slow.value = float(msg.data)
        self.slow.t_last = self._now()

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd["vx"].value = float(msg.linear.x)
        self.cmd["vy"].value = float(msg.linear.y)
        self.cmd["wz"].value = float(msg.angular.z)
        t = self._now()
        self.cmd["vx"].t_last = t
        self.cmd["vy"].t_last = t
        self.cmd["wz"].t_last = t

    def build_rows(self) -> Dict[str, Tuple[str, str, str, str]]:
        def row(label: str, val_s: str, stale: bool, topic: str) -> Tuple[str, str, str, str]:
            return (label, val_s, "STALE" if stale else "OK", topic)

        rows: Dict[str, Tuple[str, str, str, str]] = {}
        rows["depth"] = row(
            "Depth front min",
            _fmt_m(self.depth.value),
            self._is_stale(self.depth.t_last) or (self.depth.value is not None and math.isnan(self.depth.value)),
            self.t_depth,
        )
        rows["ultra"] = row(
            "Ultrasonic front",
            _fmt_m(self.ultra.value),
            self._is_stale(self.ultra.t_last) or (self.ultra.value is not None and math.isnan(self.ultra.value)),
            self.t_ultra,
        )
        rows["left"] = row(
            "ToF left",
            _fmt_m(self.left.value),
            self._is_stale(self.left.t_last) or (self.left.value is not None and math.isnan(self.left.value)),
            self.t_left,
        )
        rows["right"] = row(
            "ToF right",
            _fmt_m(self.right.value),
            self._is_stale(self.right.t_last) or (self.right.value is not None and math.isnan(self.right.value)),
            self.t_right,
        )
        rows["stop"] = row(
            "SAFETY STOP",
            _fmt_bool(self.stop.value if isinstance(self.stop.value, bool) else None),
            self._is_stale(self.stop.t_last),
            self.t_stop,
        )
        rows["slow"] = row(
            "Slowdown factor",
            _fmt_f(self.slow.value),
            self._is_stale(self.slow.t_last) or (self.slow.value is not None and math.isnan(self.slow.value)),
            self.t_slow,
        )
        rows["cmd"] = row(
            "cmd_vel_safe (vx,vy,wz)",
            f"{_fmt_f(self.cmd['vx'].value)}, {_fmt_f(self.cmd['vy'].value)}, {_fmt_f(self.cmd['wz'].value)}",
            self._is_stale(self.cmd["vx"].t_last),
            self.t_cmd_safe,
        )
        return rows


def _curses_main(stdscr, node: SensorDashboardNode) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    header = ["Signal", "Value", "State", "Topic"]
    colw = [18, 22, 8, 44]

    def draw_line(y: int, text: str, attr=0) -> None:
        try:
            stdscr.addstr(y, 0, text[: (curses.COLS - 1)], attr)
        except Exception:
            pass

    dt = 1.0 / max(1.0, node.refresh_hz)
    last_draw = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        now = time.monotonic()
        if (now - last_draw) < dt:
            time.sleep(0.005)
            continue
        last_draw = now

        stdscr.erase()
        draw_line(0, "Robot Savo — Perception Dashboard  (q to quit)", curses.A_BOLD)
        draw_line(1, f"stale_timeout_s={node.stale_timeout_s:0.2f}   refresh_hz={node.refresh_hz:0.1f}")
        draw_line(2, "-" * (sum(colw) + 6))

        y = 3
        draw_line(y, f"{header[0]:<{colw[0]}} | {header[1]:<{colw[1]}} | {header[2]:<{colw[2]}} | {header[3]}")
        y += 1
        draw_line(y, "-" * (sum(colw) + 6))
        y += 1

        rows = node.build_rows()
        order = ["depth", "ultra", "left", "right", "stop", "slow", "cmd"]

        for k in order:
            label, val_s, state, topic = rows[k]
            attr = curses.A_BOLD if ("STOP" in label) else 0
            if state == "STALE":
                attr |= curses.A_DIM
            draw_line(
                y,
                f"{label:<{colw[0]}} | {val_s:<{colw[1]}} | {state:<{colw[2]}} | {topic}",
                attr,
            )
            y += 1

        draw_line(y + 1, "Tip: If a signal is STALE, check ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY and QoS compatibility.")
        stdscr.refresh()

        ch = stdscr.getch()
        if ch in (ord("q"), ord("Q")):
            break


def main() -> None:
    rclpy.init()
    node = SensorDashboardNode()
    try:
        curses.wrapper(_curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
