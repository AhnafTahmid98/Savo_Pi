#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / control_dashboard_node.py
=====================================================

Terminal dashboard for Robot Savo control-layer monitoring.

Purpose
-------
This node displays a live terminal dashboard for the control command chain:

    /cmd_vel_manual
    /cmd_vel_auto
    /cmd_vel_nav
    /cmd_vel_recovery
        -> twist_mux_node
        -> /cmd_vel_mux
        -> cmd_vel_shaper_node
        -> /cmd_vel
        -> safety gate
        -> /cmd_vel_safe
        -> savo_base

It also watches:

    /savo_control/mode_cmd
    /savo_control/mode_state
    /savo_control/status
    /savo_control/auto_test_status
    /savo_control/recovery_test_status
    /savo_control/recovery_monitor_status
    /savo_control/stuck_state
    /safety/stop
    /safety/slowdown_factor
    /odometry/filtered
    /depth/min_front_m

Architecture rules
------------------
- This node does NOT publish velocity commands.
- This node does NOT publish /cmd_vel_safe.
- This node does NOT control hardware.
- This node is display/diagnostics only.

Usage
-----
Run in a real terminal:

    ros2 run savo_control control_dashboard_node.py

Quit:

    Ctrl+C

Notes
-----
This node uses curses. It is best run directly in a terminal, not inside a
launch file output window.
"""

from __future__ import annotations

import curses
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String


@dataclass
class TwistSample:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp: Optional[float] = None

    @property
    def nonzero(self) -> bool:
        return abs(self.vx) > 1.0e-4 or abs(self.vy) > 1.0e-4 or abs(self.wz) > 1.0e-4


@dataclass
class BoolSample:
    value: Optional[bool] = None
    stamp: Optional[float] = None


@dataclass
class ScalarSample:
    value: Optional[float] = None
    stamp: Optional[float] = None


@dataclass
class StringSample:
    value: str = ""
    stamp: Optional[float] = None


@dataclass
class OdomSample:
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp: Optional[float] = None


class ControlDashboardNode(Node):
    """Live terminal dashboard for Robot Savo control-layer status."""

    def __init__(self) -> None:
        super().__init__("control_dashboard_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("refresh_hz", 5.0)
        self.declare_parameter("stale_timeout_s", 0.75)

        self.declare_parameter("cmd_vel_manual_topic", "/cmd_vel_manual")
        self.declare_parameter("cmd_vel_auto_topic", "/cmd_vel_auto")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_recovery_topic", "/cmd_vel_recovery")
        self.declare_parameter("cmd_vel_mux_topic", "/cmd_vel_mux")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")

        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("mode_state_topic", "/savo_control/mode_state")

        self.declare_parameter("control_status_topic", "/savo_control/status")
        self.declare_parameter("auto_test_status_topic", "/savo_control/auto_test_status")
        self.declare_parameter("recovery_test_status_topic", "/savo_control/recovery_test_status")
        self.declare_parameter(
            "recovery_monitor_status_topic",
            "/savo_control/recovery_monitor_status",
        )

        self.declare_parameter("stuck_state_topic", "/savo_control/stuck_state")
        self.declare_parameter("recovery_request_topic", "/savo_control/recovery_request")
        self.declare_parameter("recovery_active_topic", "/savo_control/recovery_active")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_factor_topic", "/safety/slowdown_factor")

        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("depth_front_topic", "/depth/min_front_m")

        self.declare_parameter("watch_odom", True)
        self.declare_parameter("watch_depth", True)
        self.declare_parameter("watch_slowdown_factor", True)

        self._refresh_hz = float(self.get_parameter("refresh_hz").value)
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)

        self._cmd_vel_manual_topic = str(self.get_parameter("cmd_vel_manual_topic").value)
        self._cmd_vel_auto_topic = str(self.get_parameter("cmd_vel_auto_topic").value)
        self._cmd_vel_nav_topic = str(self.get_parameter("cmd_vel_nav_topic").value)
        self._cmd_vel_recovery_topic = str(self.get_parameter("cmd_vel_recovery_topic").value)
        self._cmd_vel_mux_topic = str(self.get_parameter("cmd_vel_mux_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)

        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)

        self._control_status_topic = str(self.get_parameter("control_status_topic").value)
        self._auto_test_status_topic = str(self.get_parameter("auto_test_status_topic").value)
        self._recovery_test_status_topic = str(
            self.get_parameter("recovery_test_status_topic").value
        )
        self._recovery_monitor_status_topic = str(
            self.get_parameter("recovery_monitor_status_topic").value
        )

        self._stuck_state_topic = str(self.get_parameter("stuck_state_topic").value)
        self._recovery_request_topic = str(self.get_parameter("recovery_request_topic").value)
        self._recovery_active_topic = str(self.get_parameter("recovery_active_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(self.get_parameter("slowdown_factor_topic").value)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._depth_front_topic = str(self.get_parameter("depth_front_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._watch_depth = bool(self.get_parameter("watch_depth").value)
        self._watch_slowdown_factor = bool(
            self.get_parameter("watch_slowdown_factor").value
        )

        self._validate_params()

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._lock = threading.Lock()

        self._cmd_manual = TwistSample()
        self._cmd_auto = TwistSample()
        self._cmd_nav = TwistSample()
        self._cmd_recovery = TwistSample()
        self._cmd_mux = TwistSample()
        self._cmd = TwistSample()
        self._cmd_safe = TwistSample()

        self._mode_cmd = StringSample()
        self._mode_state = StringSample()

        self._control_status = StringSample()
        self._auto_test_status = StringSample()
        self._recovery_test_status = StringSample()
        self._recovery_monitor_status = StringSample()

        self._stuck_state = BoolSample()
        self._recovery_request = BoolSample()
        self._recovery_active = BoolSample()
        self._safety_stop = BoolSample()
        self._slowdown_factor = ScalarSample()
        self._odom = OdomSample()
        self._depth_front = ScalarSample()

        self._running = True

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(
            Twist, self._cmd_vel_manual_topic, self._make_twist_cb("manual"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_auto_topic, self._make_twist_cb("auto"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_nav_topic, self._make_twist_cb("nav"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_recovery_topic, self._make_twist_cb("recovery"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_mux_topic, self._make_twist_cb("mux"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_topic, self._make_twist_cb("cmd"), 10
        )
        self.create_subscription(
            Twist, self._cmd_vel_safe_topic, self._make_twist_cb("safe"), 10
        )

        self.create_subscription(String, self._mode_cmd_topic, self._on_mode_cmd, 10)
        self.create_subscription(String, self._mode_state_topic, self._on_mode_state, 10)

        self.create_subscription(
            String, self._control_status_topic, self._on_control_status, 10
        )
        self.create_subscription(
            String, self._auto_test_status_topic, self._on_auto_test_status, 10
        )
        self.create_subscription(
            String, self._recovery_test_status_topic, self._on_recovery_test_status, 10
        )
        self.create_subscription(
            String,
            self._recovery_monitor_status_topic,
            self._on_recovery_monitor_status,
            10,
        )

        self.create_subscription(Bool, self._stuck_state_topic, self._on_stuck_state, 10)
        self.create_subscription(Bool, self._recovery_request_topic, self._on_recovery_request, 10)
        self.create_subscription(Bool, self._recovery_active_topic, self._on_recovery_active, 10)
        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        if self._watch_slowdown_factor:
            self.create_subscription(
                Float32,
                self._slowdown_factor_topic,
                self._on_slowdown_float32,
                10,
            )
            self.create_subscription(
                Float64,
                self._slowdown_factor_topic,
                self._on_slowdown_float64,
                10,
            )

        if self._watch_odom:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        if self._watch_depth:
            self.create_subscription(
                Float32,
                self._depth_front_topic,
                self._on_depth_float32,
                10,
            )
            self.create_subscription(
                Float64,
                self._depth_front_topic,
                self._on_depth_float64,
                10,
            )

        self.get_logger().info(
            "ControlDashboardNode started | "
            f"refresh={self._refresh_hz:.1f} Hz | stale={self._stale_timeout_s:.2f}s"
        )

    # ----------------------------------------------------------------------
    # Validation
    # ----------------------------------------------------------------------
    def _validate_params(self) -> None:
        if self._refresh_hz <= 0.0:
            raise ValueError("refresh_hz must be > 0")
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0")

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------
    def _make_twist_cb(self, name: str):
        def _cb(msg: Twist) -> None:
            sample = TwistSample(
                vx=self._safe_float(msg.linear.x),
                vy=self._safe_float(msg.linear.y),
                wz=self._safe_float(msg.angular.z),
                stamp=time.monotonic(),
            )
            with self._lock:
                if name == "manual":
                    self._cmd_manual = sample
                elif name == "auto":
                    self._cmd_auto = sample
                elif name == "nav":
                    self._cmd_nav = sample
                elif name == "recovery":
                    self._cmd_recovery = sample
                elif name == "mux":
                    self._cmd_mux = sample
                elif name == "cmd":
                    self._cmd = sample
                elif name == "safe":
                    self._cmd_safe = sample

        return _cb

    def _on_mode_cmd(self, msg: String) -> None:
        with self._lock:
            self._mode_cmd = StringSample(str(msg.data), time.monotonic())

    def _on_mode_state(self, msg: String) -> None:
        with self._lock:
            self._mode_state = StringSample(str(msg.data), time.monotonic())

    def _on_control_status(self, msg: String) -> None:
        with self._lock:
            self._control_status = StringSample(str(msg.data), time.monotonic())

    def _on_auto_test_status(self, msg: String) -> None:
        with self._lock:
            self._auto_test_status = StringSample(str(msg.data), time.monotonic())

    def _on_recovery_test_status(self, msg: String) -> None:
        with self._lock:
            self._recovery_test_status = StringSample(str(msg.data), time.monotonic())

    def _on_recovery_monitor_status(self, msg: String) -> None:
        with self._lock:
            self._recovery_monitor_status = StringSample(str(msg.data), time.monotonic())

    def _on_stuck_state(self, msg: Bool) -> None:
        with self._lock:
            self._stuck_state = BoolSample(bool(msg.data), time.monotonic())

    def _on_recovery_request(self, msg: Bool) -> None:
        with self._lock:
            self._recovery_request = BoolSample(bool(msg.data), time.monotonic())

    def _on_recovery_active(self, msg: Bool) -> None:
        with self._lock:
            self._recovery_active = BoolSample(bool(msg.data), time.monotonic())

    def _on_safety_stop(self, msg: Bool) -> None:
        with self._lock:
            self._safety_stop = BoolSample(bool(msg.data), time.monotonic())

    def _on_slowdown_float32(self, msg: Float32) -> None:
        with self._lock:
            self._slowdown_factor = ScalarSample(
                self._safe_float(msg.data),
                time.monotonic(),
            )

    def _on_slowdown_float64(self, msg: Float64) -> None:
        with self._lock:
            self._slowdown_factor = ScalarSample(
                self._safe_float(msg.data),
                time.monotonic(),
            )

    def _on_depth_float32(self, msg: Float32) -> None:
        with self._lock:
            self._depth_front = ScalarSample(self._safe_float(msg.data), time.monotonic())

    def _on_depth_float64(self, msg: Float64) -> None:
        with self._lock:
            self._depth_front = ScalarSample(self._safe_float(msg.data), time.monotonic())

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        vx = self._safe_float(msg.twist.twist.linear.x)
        vy = self._safe_float(msg.twist.twist.linear.y)
        wz = self._safe_float(msg.twist.twist.angular.z)

        with self._lock:
            self._odom = OdomSample(
                linear_speed=math.sqrt(vx * vx + vy * vy),
                angular_speed=abs(wz),
                x=self._safe_float(msg.pose.pose.position.x),
                y=self._safe_float(msg.pose.pose.position.y),
                yaw=0.0 if yaw is None else yaw,
                stamp=time.monotonic(),
            )

    # ----------------------------------------------------------------------
    # Dashboard
    # ----------------------------------------------------------------------
    def run_dashboard(self) -> None:
        curses.wrapper(self._curses_main)

    def _curses_main(self, screen) -> None:
        curses.curs_set(0)
        screen.nodelay(True)
        screen.timeout(0)

        period = 1.0 / self._refresh_hz

        while rclpy.ok() and self._running:
            start = time.monotonic()

            key = screen.getch()
            if key in (ord("q"), ord("Q")):
                self._running = False
                break

            self._draw(screen)

            elapsed = time.monotonic() - start
            time.sleep(max(0.0, period - elapsed))

    def _draw(self, screen) -> None:
        screen.erase()
        height, width = screen.getmaxyx()

        with self._lock:
            lines = self._build_lines()

        for row, line in enumerate(lines):
            if row >= height - 1:
                break
            safe_line = line[: max(0, width - 1)]
            try:
                screen.addstr(row, 0, safe_line)
            except curses.error:
                pass

        try:
            screen.refresh()
        except curses.error:
            pass

    def _build_lines(self) -> list[str]:
        now_text = time.strftime("%Y-%m-%d %H:%M:%S")

        level = self._compute_level()

        lines = [
            "Robot Savo — savo_control dashboard",
            "=" * 78,
            f"Time: {now_text}    Level: {level}    Quit: q or Ctrl+C",
            "",
            "[Modes]",
            f"  mode_cmd   : {self._string_text(self._mode_cmd)}",
            f"  mode_state : {self._string_text(self._mode_state)}",
            "",
            "[Command chain]",
            f"  manual   /cmd_vel_manual   : {self._twist_text(self._cmd_manual)}",
            f"  auto     /cmd_vel_auto     : {self._twist_text(self._cmd_auto)}",
            f"  nav      /cmd_vel_nav      : {self._twist_text(self._cmd_nav)}",
            f"  recovery /cmd_vel_recovery : {self._twist_text(self._cmd_recovery)}",
            f"  mux      /cmd_vel_mux      : {self._twist_text(self._cmd_mux)}",
            f"  shaped   /cmd_vel          : {self._twist_text(self._cmd)}",
            f"  safe     /cmd_vel_safe     : {self._twist_text(self._cmd_safe)}",
            "",
            "[Safety / perception]",
            f"  safety_stop       : {self._bool_text(self._safety_stop)}",
            f"  slowdown_factor   : {self._scalar_text(self._slowdown_factor)}",
            f"  front_depth_min_m : {self._scalar_text(self._depth_front)}",
            "",
            "[Localization]",
            f"  odometry_filtered : {self._odom_text(self._odom)}",
            "",
            "[Recovery / stuck]",
            f"  stuck_state       : {self._bool_text(self._stuck_state)}",
            f"  recovery_request  : {self._bool_text(self._recovery_request)}",
            f"  recovery_active   : {self._bool_text(self._recovery_active)}",
            f"  recovery_status   : {self._string_text(self._recovery_monitor_status)}",
            f"  recovery_test     : {self._string_text(self._recovery_test_status)}",
            "",
            "[Control status]",
            f"  control_status    : {self._string_text(self._control_status)}",
            f"  auto_test_status  : {self._string_text(self._auto_test_status)}",
            "",
            "[Expected production chain]",
            "  keyboard/nav/auto/recovery -> /cmd_vel_mux -> /cmd_vel -> /cmd_vel_safe -> savo_base",
            "",
            "[Warnings]",
        ]

        warnings = self._warnings()
        if warnings:
            lines.extend([f"  - {w}" for w in warnings])
        else:
            lines.append("  none")

        return lines

    # ----------------------------------------------------------------------
    # Status logic
    # ----------------------------------------------------------------------
    def _compute_level(self) -> str:
        if self._fresh(self._safety_stop.stamp) and self._safety_stop.value is True:
            return "SAFETY_STOP"

        warnings = self._warnings()
        if warnings:
            return "WARN"

        return "OK"

    def _warnings(self) -> list[str]:
        warnings: list[str] = []

        source_active = any(
            [
                self._fresh(self._cmd_manual.stamp) and self._cmd_manual.nonzero,
                self._fresh(self._cmd_auto.stamp) and self._cmd_auto.nonzero,
                self._fresh(self._cmd_nav.stamp) and self._cmd_nav.nonzero,
                self._fresh(self._cmd_recovery.stamp) and self._cmd_recovery.nonzero,
            ]
        )

        if source_active and not self._fresh(self._cmd_mux.stamp):
            warnings.append("source command active but /cmd_vel_mux is stale")

        if self._fresh(self._cmd_mux.stamp) and self._cmd_mux.nonzero and not self._fresh(
            self._cmd.stamp
        ):
            warnings.append("/cmd_vel_mux active but /cmd_vel is stale")

        if self._fresh(self._cmd.stamp) and self._cmd.nonzero and not self._fresh(
            self._cmd_safe.stamp
        ):
            warnings.append("/cmd_vel active but /cmd_vel_safe is stale")

        if self._watch_odom and not self._fresh(self._odom.stamp):
            warnings.append("/odometry/filtered is stale")

        if self._watch_depth and not self._fresh(self._depth_front.stamp):
            warnings.append("/depth/min_front_m is stale")

        return warnings

    # ----------------------------------------------------------------------
    # Formatting
    # ----------------------------------------------------------------------
    def _fresh(self, stamp: Optional[float]) -> bool:
        if stamp is None:
            return False
        return (time.monotonic() - stamp) <= self._stale_timeout_s

    def _age_text(self, stamp: Optional[float]) -> str:
        if stamp is None:
            return "never"
        return f"{time.monotonic() - stamp:.2f}s"

    def _twist_text(self, sample: TwistSample) -> str:
        if not self._fresh(sample.stamp):
            return f"STALE age={self._age_text(sample.stamp)}"
        return (
            f"vx={sample.vx:+.2f} vy={sample.vy:+.2f} wz={sample.wz:+.2f} "
            f"age={self._age_text(sample.stamp)}"
        )

    def _bool_text(self, sample: BoolSample) -> str:
        if not self._fresh(sample.stamp):
            return f"STALE age={self._age_text(sample.stamp)}"
        return f"{sample.value} age={self._age_text(sample.stamp)}"

    def _scalar_text(self, sample: ScalarSample) -> str:
        if not self._fresh(sample.stamp) or sample.value is None:
            return f"STALE age={self._age_text(sample.stamp)}"
        return f"{sample.value:.3f} age={self._age_text(sample.stamp)}"

    def _string_text(self, sample: StringSample) -> str:
        if not self._fresh(sample.stamp):
            return f"STALE age={self._age_text(sample.stamp)}"
        text = sample.value if sample.value else ""
        if len(text) > 100:
            text = text[:97] + "..."
        return f"{text} age={self._age_text(sample.stamp)}"

    def _odom_text(self, sample: OdomSample) -> str:
        if not self._fresh(sample.stamp):
            return f"STALE age={self._age_text(sample.stamp)}"
        return (
            f"x={sample.x:+.2f} y={sample.y:+.2f} yaw={sample.yaw:+.2f} "
            f"lin={sample.linear_speed:.3f} ang={sample.angular_speed:.3f} "
            f"age={self._age_text(sample.stamp)}"
        )

    # ----------------------------------------------------------------------
    # Math helpers
    # ----------------------------------------------------------------------
    @staticmethod
    def _safe_float(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return value

    @staticmethod
    def _yaw_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> Optional[float]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-9 or not math.isfinite(norm):
            return None

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlDashboardNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_dashboard()

    except KeyboardInterrupt:
        pass

    finally:
        node._running = False
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()