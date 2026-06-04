#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Monitors recovery topics and publishes a compact status string to /savo_control/recovery_monitor_status."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String


@dataclass
class Stamp:
    stamp: Optional[float] = None

    def age(self) -> Optional[float]:
        if self.stamp is None:
            return None
        return time.monotonic() - self.stamp


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
    stamp: Optional[float] = None


class RecoveryStatusNode(Node):
    """Recovery diagnostics/status monitor."""

    def __init__(self) -> None:
        super().__init__("recovery_status_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.75)
        self.declare_parameter("log_hz", 0.5)

        self.declare_parameter("output_status_topic", "/savo_control/recovery_monitor_status")

        self.declare_parameter("cmd_vel_recovery_topic", "/cmd_vel_recovery")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")

        self.declare_parameter("recovery_status_topic", "/savo_control/recovery_status")
        self.declare_parameter("backup_escape_status_topic", "/savo_control/backup_escape_status")
        self.declare_parameter("stuck_state_topic", "/savo_control/stuck_state")
        self.declare_parameter("recovery_request_topic", "/savo_control/recovery_request")
        self.declare_parameter("recovery_active_topic", "/savo_control/recovery_active")

        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("mode_state_topic", "/savo_control/mode_state")

        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_factor_topic", "/safety/slowdown_factor")

        self.declare_parameter("odom_topic", "/odometry/filtered")

        self.declare_parameter("watch_odom", True)
        self.declare_parameter("watch_slowdown_factor", True)
        self.declare_parameter("publish_console_log", True)

        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self._log_hz = float(self.get_parameter("log_hz").value)

        self._output_status_topic = str(self.get_parameter("output_status_topic").value)

        self._cmd_vel_recovery_topic = str(self.get_parameter("cmd_vel_recovery_topic").value)
        self._cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)

        self._recovery_status_topic = str(self.get_parameter("recovery_status_topic").value)
        self._backup_escape_status_topic = str(
            self.get_parameter("backup_escape_status_topic").value
        )
        self._stuck_state_topic = str(self.get_parameter("stuck_state_topic").value)
        self._recovery_request_topic = str(self.get_parameter("recovery_request_topic").value)
        self._recovery_active_topic = str(self.get_parameter("recovery_active_topic").value)

        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)

        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(self.get_parameter("slowdown_factor_topic").value)

        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._watch_slowdown_factor = bool(self.get_parameter("watch_slowdown_factor").value)
        self._publish_console_log = bool(self.get_parameter("publish_console_log").value)

        self._validate_params()

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._cmd_vel_recovery = TwistSample()
        self._cmd_vel_safe = TwistSample()

        self._recovery_status = StringSample()
        self._backup_escape_status = StringSample()

        self._stuck_state = BoolSample()
        self._recovery_request = BoolSample()
        self._recovery_active = BoolSample()

        self._mode_cmd = StringSample()
        self._mode_state = StringSample()

        self._safety_stop = BoolSample()
        self._slowdown_factor = ScalarSample()
        self._odom = OdomSample()

        self._last_console_log_time = 0.0

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._status_pub = self.create_publisher(String, self._output_status_topic, 10)

        self.create_subscription(
            Twist,
            self._cmd_vel_recovery_topic,
            self._on_cmd_vel_recovery,
            10,
        )

        self.create_subscription(
            Twist,
            self._cmd_vel_safe_topic,
            self._on_cmd_vel_safe,
            10,
        )

        self.create_subscription(
            String,
            self._recovery_status_topic,
            self._on_recovery_status,
            10,
        )

        self.create_subscription(
            String,
            self._backup_escape_status_topic,
            self._on_backup_escape_status,
            10,
        )

        self.create_subscription(
            Bool,
            self._stuck_state_topic,
            self._on_stuck_state,
            10,
        )

        self.create_subscription(
            Bool,
            self._recovery_request_topic,
            self._on_recovery_request,
            10,
        )

        self.create_subscription(
            Bool,
            self._recovery_active_topic,
            self._on_recovery_active,
            10,
        )

        self.create_subscription(String, self._mode_cmd_topic, self._on_mode_cmd, 10)
        self.create_subscription(String, self._mode_state_topic, self._on_mode_state, 10)

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

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            "RecoveryStatusNode started | "
            f"status_out={self._output_status_topic} | "
            f"stale_timeout={self._stale_timeout_s:.2f}s"
        )

    # ----------------------------------------------------------------------
    # Validation
    # ----------------------------------------------------------------------
    def _validate_params(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")

        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0")

        if self._log_hz < 0.0:
            raise ValueError("log_hz must be >= 0")

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------
    def _on_cmd_vel_recovery(self, msg: Twist) -> None:
        self._cmd_vel_recovery = TwistSample(
            vx=self._safe_float(msg.linear.x),
            vy=self._safe_float(msg.linear.y),
            wz=self._safe_float(msg.angular.z),
            stamp=time.monotonic(),
        )

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        self._cmd_vel_safe = TwistSample(
            vx=self._safe_float(msg.linear.x),
            vy=self._safe_float(msg.linear.y),
            wz=self._safe_float(msg.angular.z),
            stamp=time.monotonic(),
        )

    def _on_recovery_status(self, msg: String) -> None:
        self._recovery_status = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_backup_escape_status(self, msg: String) -> None:
        self._backup_escape_status = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_stuck_state(self, msg: Bool) -> None:
        self._stuck_state = BoolSample(value=bool(msg.data), stamp=time.monotonic())

    def _on_recovery_request(self, msg: Bool) -> None:
        self._recovery_request = BoolSample(value=bool(msg.data), stamp=time.monotonic())

    def _on_recovery_active(self, msg: Bool) -> None:
        self._recovery_active = BoolSample(value=bool(msg.data), stamp=time.monotonic())

    def _on_mode_cmd(self, msg: String) -> None:
        self._mode_cmd = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_mode_state(self, msg: String) -> None:
        self._mode_state = StringSample(value=str(msg.data), stamp=time.monotonic())

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = BoolSample(value=bool(msg.data), stamp=time.monotonic())

    def _on_slowdown_float32(self, msg: Float32) -> None:
        self._slowdown_factor = ScalarSample(
            value=self._safe_float(msg.data),
            stamp=time.monotonic(),
        )

    def _on_slowdown_float64(self, msg: Float64) -> None:
        self._slowdown_factor = ScalarSample(
            value=self._safe_float(msg.data),
            stamp=time.monotonic(),
        )

    def _on_odom(self, msg: Odometry) -> None:
        vx = self._safe_float(msg.twist.twist.linear.x)
        vy = self._safe_float(msg.twist.twist.linear.y)
        wz = self._safe_float(msg.twist.twist.angular.z)

        self._odom = OdomSample(
            linear_speed=math.sqrt(vx * vx + vy * vy),
            angular_speed=abs(wz),
            stamp=time.monotonic(),
        )

    # ----------------------------------------------------------------------
    # Main loop
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        status = self._build_status_string()

        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

        if self._publish_console_log:
            self._log_status_throttled(status)

    # ----------------------------------------------------------------------
    # Status logic
    # ----------------------------------------------------------------------
    def _build_status_string(self) -> str:
        level = self._compute_level()

        mode_cmd = self._fresh_string(self._mode_cmd, default="STALE")
        mode_state = self._fresh_string(self._mode_state, default="STALE")

        safety_stop = self._fresh_bool_text(self._safety_stop)
        stuck = self._fresh_bool_text(self._stuck_state)
        recovery_request = self._fresh_bool_text(self._recovery_request)
        recovery_active = self._fresh_bool_text(self._recovery_active)

        slowdown = "STALE"
        if self._watch_slowdown_factor:
            slowdown = self._fresh_scalar_text(self._slowdown_factor)

        recovery_cmd = self._twist_text(self._cmd_vel_recovery)
        safe_cmd = self._twist_text(self._cmd_vel_safe)

        recovery_status = self._fresh_string(self._recovery_status, default="STALE")
        backup_status = self._fresh_string(self._backup_escape_status, default="STALE")

        odom = "disabled"
        if self._watch_odom:
            if self._is_fresh(self._odom.stamp):
                odom = (
                    f"lin={self._odom.linear_speed:.3f},"
                    f"ang={self._odom.angular_speed:.3f}"
                )
            else:
                odom = "STALE"

        return (
            f"level={level}; "
            f"mode_cmd={mode_cmd}; "
            f"mode_state={mode_state}; "
            f"safety_stop={safety_stop}; "
            f"slowdown={slowdown}; "
            f"stuck={stuck}; "
            f"recovery_request={recovery_request}; "
            f"recovery_active={recovery_active}; "
            f"cmd_recovery={recovery_cmd}; "
            f"cmd_safe={safe_cmd}; "
            f"recovery_status={recovery_status}; "
            f"backup_status={backup_status}; "
            f"odom={odom}"
        )

    def _compute_level(self) -> str:
        if self._is_fresh(self._safety_stop.stamp) and self._safety_stop.value is True:
            return "BLOCKED_SAFETY_STOP"

        if self._is_fresh(self._stuck_state.stamp) and self._stuck_state.value is True:
            if self._is_fresh(self._recovery_request.stamp) and self._recovery_request.value is True:
                return "STUCK_RECOVERY_REQUESTED"
            return "STUCK_DETECTED"

        if self._is_fresh(self._recovery_active.stamp) and self._recovery_active.value is True:
            return "RECOVERY_ACTIVE"

        if self._is_fresh(self._cmd_vel_recovery.stamp) and self._cmd_vel_recovery.nonzero:
            if not self._is_fresh(self._cmd_vel_safe.stamp):
                return "WARN_SAFE_STALE_DURING_RECOVERY"

            if not self._cmd_vel_safe.nonzero:
                return "RECOVERY_BLOCKED_OR_ZERO_SAFE_CMD"

            return "RECOVERY_ACTIVE"

        if self._watch_odom and not self._is_fresh(self._odom.stamp):
            return "WARN_ODOM_STALE"

        return "OK"

    # ----------------------------------------------------------------------
    # Formatting helpers
    # ----------------------------------------------------------------------
    def _twist_text(self, sample: TwistSample) -> str:
        if not self._is_fresh(sample.stamp):
            return "STALE"
        return f"vx={sample.vx:.2f},vy={sample.vy:.2f},wz={sample.wz:.2f}"

    def _fresh_bool_text(self, sample: BoolSample) -> str:
        if not self._is_fresh(sample.stamp):
            return "STALE"
        return str(sample.value)

    def _fresh_scalar_text(self, sample: ScalarSample) -> str:
        if not self._is_fresh(sample.stamp) or sample.value is None:
            return "STALE"
        return f"{sample.value:.2f}"

    def _fresh_string(self, sample: StringSample, default: str = "STALE") -> str:
        if not self._is_fresh(sample.stamp):
            return default
        return sample.value if sample.value else ""

    # ----------------------------------------------------------------------
    # Generic helpers
    # ----------------------------------------------------------------------
    def _is_fresh(self, stamp: Optional[float]) -> bool:
        if stamp is None:
            return False
        return (time.monotonic() - stamp) <= self._stale_timeout_s

    @staticmethod
    def _safe_float(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return value

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
    node = RecoveryStatusNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting recovery status node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()