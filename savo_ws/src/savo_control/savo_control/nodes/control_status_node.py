#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / control_status_node.py
==================================================

Control-layer status monitor for Robot Savo.

Purpose
-------
This node monitors the health of the control command chain:

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

    /safety/stop
    /safety/slowdown_factor
    /savo_control/mode_cmd
    /savo_control/mode_state
    /odometry/filtered

Architecture rules
------------------
- This node does NOT publish velocity commands.
- This node does NOT publish /cmd_vel_safe.
- This node does NOT control hardware.
- This node is for diagnostics/status only.

Output
------
Publishes a human-readable compact status string to:

    /savo_control/status

This is useful for:
- terminal monitoring
- future dashboard nodes
- launch diagnostics
- real robot test proof
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String


@dataclass
class TwistSample:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp: Optional[float] = None

    @property
    def is_nonzero(self) -> bool:
        return abs(self.vx) > 1.0e-4 or abs(self.vy) > 1.0e-4 or abs(self.wz) > 1.0e-4


@dataclass
class ScalarSample:
    value: Optional[float] = None
    stamp: Optional[float] = None


@dataclass
class BoolSample:
    value: Optional[bool] = None
    stamp: Optional[float] = None


@dataclass
class StringSample:
    value: str = ""
    stamp: Optional[float] = None


@dataclass
class OdomSample:
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    stamp: Optional[float] = None


class ControlStatusNode(Node):
    """Status monitor for Robot Savo control chain."""

    def __init__(self) -> None:
        super().__init__("control_status_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.75)
        self.declare_parameter("log_hz", 0.5)

        self.declare_parameter("status_topic", "/savo_control/status")

        self.declare_parameter("cmd_vel_manual_topic", "/cmd_vel_manual")
        self.declare_parameter("cmd_vel_auto_topic", "/cmd_vel_auto")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_recovery_topic", "/cmd_vel_recovery")
        self.declare_parameter("cmd_vel_mux_topic", "/cmd_vel_mux")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")

        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_factor_topic", "/safety/slowdown_factor")

        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("mode_state_topic", "/savo_control/mode_state")

        self.declare_parameter("odom_topic", "/odometry/filtered")

        self.declare_parameter("watch_odom", True)
        self.declare_parameter("watch_slowdown_factor", True)
        self.declare_parameter("publish_console_log", True)

        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self._log_hz = float(self.get_parameter("log_hz").value)

        self._status_topic = str(self.get_parameter("status_topic").value)

        self._cmd_vel_manual_topic = str(self.get_parameter("cmd_vel_manual_topic").value)
        self._cmd_vel_auto_topic = str(self.get_parameter("cmd_vel_auto_topic").value)
        self._cmd_vel_nav_topic = str(self.get_parameter("cmd_vel_nav_topic").value)
        self._cmd_vel_recovery_topic = str(self.get_parameter("cmd_vel_recovery_topic").value)
        self._cmd_vel_mux_topic = str(self.get_parameter("cmd_vel_mux_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)

        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(self.get_parameter("slowdown_factor_topic").value)

        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)

        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._watch_slowdown_factor = bool(
            self.get_parameter("watch_slowdown_factor").value
        )
        self._publish_console_log = bool(self.get_parameter("publish_console_log").value)

        self._validate_params()

        # ------------------------------------------------------------------
        # State storage
        # ------------------------------------------------------------------
        self._twists: Dict[str, TwistSample] = {
            "manual": TwistSample(),
            "auto": TwistSample(),
            "nav": TwistSample(),
            "recovery": TwistSample(),
            "mux": TwistSample(),
            "cmd": TwistSample(),
            "safe": TwistSample(),
        }

        self._safety_stop = BoolSample()
        self._slowdown = ScalarSample()
        self._mode_cmd = StringSample()
        self._mode_state = StringSample()
        self._odom = OdomSample()

        self._last_console_log_time = 0.0

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

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

        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        # Support either Float32 or Float64 for slowdown factor.
        if self._watch_slowdown_factor:
            self.create_subscription(
                Float32, self._slowdown_factor_topic, self._on_slowdown_float32, 10
            )
            self.create_subscription(
                Float64, self._slowdown_factor_topic, self._on_slowdown_float64, 10
            )

        self.create_subscription(String, self._mode_cmd_topic, self._on_mode_cmd, 10)
        self.create_subscription(String, self._mode_state_topic, self._on_mode_state, 10)

        if self._watch_odom:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            "ControlStatusNode started | "
            f"status={self._status_topic} | stale_timeout={self._stale_timeout_s:.2f}s"
        )

    # ----------------------------------------------------------------------
    # Parameter validation
    # ----------------------------------------------------------------------
    def _validate_params(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0")
        if self._log_hz < 0.0:
            raise ValueError("log_hz must be >= 0")

    # ----------------------------------------------------------------------
    # Subscriptions
    # ----------------------------------------------------------------------
    def _make_twist_cb(self, key: str):
        def _cb(msg: Twist) -> None:
            self._twists[key] = TwistSample(
                vx=self._safe_float(msg.linear.x),
                vy=self._safe_float(msg.linear.y),
                wz=self._safe_float(msg.angular.z),
                stamp=time.monotonic(),
            )

        return _cb

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = BoolSample(value=bool(msg.data), stamp=time.monotonic())

    def _on_slowdown_float32(self, msg: Float32) -> None:
        self._slowdown = ScalarSample(value=self._safe_float(msg.data), stamp=time.monotonic())

    def _on_slowdown_float64(self, msg: Float64) -> None:
        self._slowdown = ScalarSample(value=self._safe_float(msg.data), stamp=time.monotonic())

    def _on_mode_cmd(self, msg: String) -> None:
        self._mode_cmd = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_mode_state(self, msg: String) -> None:
        self._mode_state = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_odom(self, msg: Odometry) -> None:
        vx = self._safe_float(msg.twist.twist.linear.x)
        vy = self._safe_float(msg.twist.twist.linear.y)
        wz = self._safe_float(msg.twist.twist.angular.z)

        linear_speed = math.sqrt(vx * vx + vy * vy)
        angular_speed = abs(wz)

        self._odom = OdomSample(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            stamp=time.monotonic(),
        )

    # ----------------------------------------------------------------------
    # Main timer
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        status = self._build_status_string()

        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

        if self._publish_console_log:
            self._log_status_throttled(status)

    # ----------------------------------------------------------------------
    # Status building
    # ----------------------------------------------------------------------
    def _build_status_string(self) -> str:
        active_sources = []
        for name in ("manual", "auto", "nav", "recovery"):
            sample = self._twists[name]
            if self._is_fresh(sample.stamp) and sample.is_nonzero:
                active_sources.append(name.upper())

        mux = self._twists["mux"]
        cmd = self._twists["cmd"]
        safe = self._twists["safe"]

        mode_cmd = self._mode_cmd.value if self._is_fresh(self._mode_cmd.stamp) else "STALE"
        mode_state = (
            self._mode_state.value if self._is_fresh(self._mode_state.stamp) else "STALE"
        )

        safety = (
            str(self._safety_stop.value)
            if self._is_fresh(self._safety_stop.stamp)
            else "STALE"
        )

        slowdown = (
            f"{self._slowdown.value:.2f}"
            if self._watch_slowdown_factor
            and self._slowdown.value is not None
            and self._is_fresh(self._slowdown.stamp)
            else "STALE"
        )

        odom_text = "disabled"
        if self._watch_odom:
            if self._is_fresh(self._odom.stamp):
                odom_text = (
                    f"lin={self._odom.linear_speed:.3f},ang={self._odom.angular_speed:.3f}"
                )
            else:
                odom_text = "STALE"

        active_text = ",".join(active_sources) if active_sources else "none"

        status_level = self._compute_status_level()

        return (
            f"level={status_level}; "
            f"mode_cmd={mode_cmd}; "
            f"mode_state={mode_state}; "
            f"active_sources={active_text}; "
            f"mux={self._twist_text(mux)}; "
            f"cmd={self._twist_text(cmd)}; "
            f"safe={self._twist_text(safe)}; "
            f"safety_stop={safety}; "
            f"slowdown={slowdown}; "
            f"odom={odom_text}"
        )

    def _compute_status_level(self) -> str:
        # Safety stop active is not an error; it is a safety state.
        if self._is_fresh(self._safety_stop.stamp) and self._safety_stop.value is True:
            return "SAFETY_STOP"

        # If /cmd_vel is active but /cmd_vel_safe is stale, safety/base chain may not be alive.
        cmd = self._twists["cmd"]
        safe = self._twists["safe"]

        if self._is_fresh(cmd.stamp) and cmd.is_nonzero and not self._is_fresh(safe.stamp):
            return "WARN_SAFE_STALE"

        # If mux is active but shaped cmd is stale, shaper may not be alive.
        mux = self._twists["mux"]
        if self._is_fresh(mux.stamp) and mux.is_nonzero and not self._is_fresh(cmd.stamp):
            return "WARN_SHAPER_STALE"

        # If source is active but mux stale, mux may not be alive.
        source_active = any(
            self._is_fresh(self._twists[name].stamp) and self._twists[name].is_nonzero
            for name in ("manual", "auto", "nav", "recovery")
        )
        if source_active and not self._is_fresh(mux.stamp):
            return "WARN_MUX_STALE"

        if self._watch_odom and not self._is_fresh(self._odom.stamp):
            return "WARN_ODOM_STALE"

        return "OK"

    def _twist_text(self, sample: TwistSample) -> str:
        if not self._is_fresh(sample.stamp):
            return "STALE"
        return f"vx={sample.vx:.2f},vy={sample.vy:.2f},wz={sample.wz:.2f}"

    # ----------------------------------------------------------------------
    # Utility
    # ----------------------------------------------------------------------
    def _is_fresh(self, stamp: Optional[float]) -> bool:
        if stamp is None:
            return False
        return (time.monotonic() - stamp) <= self._stale_timeout_s

    @staticmethod
    def _safe_float(value: float) -> float:
        if not math.isfinite(float(value)):
            return 0.0
        return float(value)

    def _log_status_throttled(self, status: str) -> None:
        if self._log_hz <= 0.0:
            return

        now = time.monotonic()
        period = 1.0 / self._log_hz
        if (now - self._last_console_log_time) < period:
            return

        self._last_console_log_time = now
        self.get_logger().info(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlStatusNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting control status node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()