#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / straight_line_pid_test_node.py
==========================================================

Straight-line PID test node for Robot Savo.

Purpose
-------
This node drives Robot Savo forward or backward for a target distance while
using fused odometry feedback from:

    /odometry/filtered

It publishes motion commands to:

    /cmd_vel_auto

Correct command chain:

    straight_line_pid_test_node
        -> /cmd_vel_auto
        -> twist_mux_node
        -> /cmd_vel_mux
        -> cmd_vel_shaper_node
        -> /cmd_vel
        -> savo_perception/cmd_vel_safety_gate
        -> /cmd_vel_safe
        -> savo_base/base_driver_node
        -> motors

Architecture rules
------------------
- This node does NOT publish directly to /cmd_vel_safe.
- This node does NOT control hardware.
- This node does NOT read raw encoders.
- This node does NOT read raw IMU.
- This node does NOT read raw VO.
- It only uses /odometry/filtered from savo_localization.

Future /odometry/filtered inputs:
    - 4 wheel encoder odometry on savo-core
    - IMU on savo-core
    - VO / visual odometry from savo-edge

First real test:
    - wheels lifted if possible
    - very short distance, e.g. 0.20 m
    - very low speed
    - safety gate running
    - hand near power switch / E-stop
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class StraightLineState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    ODOM_STALE = "ODOM_STALE"
    ODOM_INVALID = "ODOM_INVALID"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: Optional[float] = None
    previous_time: Optional[float] = None
    derivative_filtered: float = 0.0


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp: Optional[float] = None


class StraightLinePidTestNode(Node):
    """Straight-line distance test controller using fused odometry."""

    def __init__(self) -> None:
        super().__init__("straight_line_pid_test_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("enabled", True)
        self.declare_parameter("auto_start", False)

        # Topics
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_auto")
        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("status_topic", "/savo_control/straight_line_status")

        # Mode behavior
        self.declare_parameter("request_auto_mode_on_start", True)
        self.declare_parameter("required_mode", "AUTO")

        # Target
        self.declare_parameter("target_distance_m", 0.30)
        self.declare_parameter("direction", "forward")  # forward or backward
        self.declare_parameter("tolerance_m", 0.03)
        self.declare_parameter("hold_time_s", 0.30)

        # PID
        self.declare_parameter("kp", 0.55)
        self.declare_parameter("ki", 0.00)
        self.declare_parameter("kd", 0.04)
        self.declare_parameter("integral_limit", 0.12)
        self.declare_parameter("integral_active_error_m", 0.15)
        self.declare_parameter("derivative_filter_alpha", 0.35)

        # Output limits
        self.declare_parameter("max_forward_vx", 0.14)
        self.declare_parameter("max_backward_vx", 0.10)
        self.declare_parameter("min_vx_when_active", 0.04)
        self.declare_parameter("disable_min_vx_below_error_m", 0.06)

        # Heading hold while driving straight
        self.declare_parameter("hold_heading", True)
        self.declare_parameter("heading_kp", 0.60)
        self.declare_parameter("max_heading_correction_wz", 0.25)
        self.declare_parameter("heading_tolerance_rad", 0.04)

        # Timing
        self.declare_parameter("loop_hz", 30.0)
        self.declare_parameter("odom_timeout_s", 0.50)
        self.declare_parameter("timeout_s", 8.0)
        self.declare_parameter("shutdown_zero_count", 5)

        # Safety
        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("zero_on_odom_stale", True)
        self.declare_parameter("allow_direct_cmd_vel_safe_publish", False)

        # Diagnostics
        self.declare_parameter("publish_status", True)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("log_throttle_s", 2.0)

        self._load_parameters()
        self._validate_parameters()

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self._mode_pub = self.create_publisher(String, self._mode_cmd_topic, 10)

        self._status_pub = None
        if self._publish_status:
            self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        self._timer = self.create_timer(1.0 / self._loop_hz, self._on_timer)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._state = StraightLineState.IDLE if self._enabled else StraightLineState.DISABLED

        self._latest_pose = Pose2D()
        self._start_pose: Optional[Pose2D] = None
        self._target_heading: Optional[float] = None

        self._pid = PIDState()
        self._safety_stop = False

        self._started_time: Optional[float] = None
        self._goal_enter_time: Optional[float] = None

        self._last_status_time = 0.0
        self._last_log_time = 0.0

        if self._auto_start and self._enabled:
            self._start_test()

        self.get_logger().info(
            "StraightLinePidTestNode started | "
            f"odom={self._odom_topic} | output={self._cmd_vel_out_topic} | "
            f"target={self._target_distance_m:.2f} m | direction={self._direction} | "
            f"auto_start={self._auto_start}"
        )

    # ----------------------------------------------------------------------
    # Parameter handling
    # ----------------------------------------------------------------------
    def _load_parameters(self) -> None:
        self._enabled = bool(self.get_parameter("enabled").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)

        self._request_auto_mode_on_start = bool(
            self.get_parameter("request_auto_mode_on_start").value
        )
        self._required_mode = str(self.get_parameter("required_mode").value)

        self._target_distance_m = abs(float(self.get_parameter("target_distance_m").value))
        self._direction = str(self.get_parameter("direction").value).lower().strip()
        self._tolerance_m = float(self.get_parameter("tolerance_m").value)
        self._hold_time_s = float(self.get_parameter("hold_time_s").value)

        self._kp = float(self.get_parameter("kp").value)
        self._ki = float(self.get_parameter("ki").value)
        self._kd = float(self.get_parameter("kd").value)
        self._integral_limit = float(self.get_parameter("integral_limit").value)
        self._integral_active_error_m = float(
            self.get_parameter("integral_active_error_m").value
        )
        self._derivative_filter_alpha = float(
            self.get_parameter("derivative_filter_alpha").value
        )

        self._max_forward_vx = abs(float(self.get_parameter("max_forward_vx").value))
        self._max_backward_vx = abs(float(self.get_parameter("max_backward_vx").value))
        self._min_vx_when_active = abs(float(self.get_parameter("min_vx_when_active").value))
        self._disable_min_vx_below_error_m = abs(
            float(self.get_parameter("disable_min_vx_below_error_m").value)
        )

        self._hold_heading = bool(self.get_parameter("hold_heading").value)
        self._heading_kp = float(self.get_parameter("heading_kp").value)
        self._max_heading_correction_wz = abs(
            float(self.get_parameter("max_heading_correction_wz").value)
        )
        self._heading_tolerance_rad = abs(float(self.get_parameter("heading_tolerance_rad").value))

        self._loop_hz = float(self.get_parameter("loop_hz").value)
        self._odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)
        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)

        self._respect_safety_stop = bool(self.get_parameter("respect_safety_stop").value)
        self._zero_on_odom_stale = bool(self.get_parameter("zero_on_odom_stale").value)
        self._allow_direct_cmd_vel_safe_publish = bool(
            self.get_parameter("allow_direct_cmd_vel_safe_publish").value
        )

        self._publish_status = bool(self.get_parameter("publish_status").value)
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._log_throttle_s = float(self.get_parameter("log_throttle_s").value)

    def _validate_parameters(self) -> None:
        if self._loop_hz <= 0.0:
            raise ValueError("loop_hz must be > 0")

        if self._target_distance_m <= 0.0:
            raise ValueError("target_distance_m must be > 0")

        if self._direction not in ("forward", "backward"):
            raise ValueError("direction must be 'forward' or 'backward'")

        if self._tolerance_m <= 0.0:
            raise ValueError("tolerance_m must be > 0")

        if self._odom_timeout_s <= 0.0:
            raise ValueError("odom_timeout_s must be > 0")

        if self._timeout_s <= 0.0:
            raise ValueError("timeout_s must be > 0")

        if not (0.0 <= self._derivative_filter_alpha <= 1.0):
            raise ValueError("derivative_filter_alpha must be in [0, 1]")

        if self._allow_direct_cmd_vel_safe_publish:
            raise ValueError(
                "allow_direct_cmd_vel_safe_publish must remain false in savo_control"
            )

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        if yaw is None:
            self._state = StraightLineState.ODOM_INVALID
            self._throttled_warn("Invalid odometry quaternion")
            return

        x = self._safe_float(msg.pose.pose.position.x)
        y = self._safe_float(msg.pose.pose.position.y)

        if not math.isfinite(x) or not math.isfinite(y):
            self._state = StraightLineState.ODOM_INVALID
            self._throttled_warn("Invalid odometry position")
            return

        self._latest_pose = Pose2D(
            x=x,
            y=y,
            yaw=yaw,
            stamp=time.monotonic(),
        )

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    # ----------------------------------------------------------------------
    # Main loop
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        if not self._enabled:
            self._state = StraightLineState.DISABLED
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._state == StraightLineState.IDLE and not self._auto_start:
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._state == StraightLineState.IDLE and self._auto_start:
            self._start_test()

        if self._respect_safety_stop and self._safety_stop:
            self._state = StraightLineState.SAFETY_STOP
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if not self._odom_is_fresh():
            self._state = StraightLineState.ODOM_STALE
            if self._zero_on_odom_stale:
                self._publish_zero()
            self._publish_status_if_needed()
            self._throttled_warn("Odometry stale or not received")
            return

        if self._start_pose is None:
            self._start_test()

        if self._started_time is not None:
            elapsed = time.monotonic() - self._started_time
            if elapsed > self._timeout_s:
                self._state = StraightLineState.TIMEOUT
                self._publish_zero()
                self._publish_status_if_needed()
                self._throttled_warn("Straight-line test timeout")
                return

        travelled = self._travelled_distance()
        remaining = self._target_distance_m - travelled

        if remaining <= self._tolerance_m:
            if self._goal_enter_time is None:
                self._goal_enter_time = time.monotonic()

            if (time.monotonic() - self._goal_enter_time) >= self._hold_time_s:
                self._state = StraightLineState.GOAL_REACHED
                self._publish_zero()
                self._publish_status_if_needed()
                return
        else:
            self._goal_enter_time = None

        self._state = StraightLineState.RUNNING

        vx = self._compute_distance_pid_vx(remaining)
        wz = self._compute_heading_correction()

        if self._direction == "backward":
            vx = -abs(vx)
        else:
            vx = abs(vx)

        self._publish_cmd(vx=vx, wz=wz)
        self._publish_status_if_needed()

    # ----------------------------------------------------------------------
    # Test lifecycle
    # ----------------------------------------------------------------------
    def _start_test(self) -> None:
        if not self._odom_is_fresh():
            self._state = StraightLineState.ODOM_STALE
            self._publish_zero()
            return

        self._start_pose = Pose2D(
            x=self._latest_pose.x,
            y=self._latest_pose.y,
            yaw=self._latest_pose.yaw,
            stamp=time.monotonic(),
        )
        self._target_heading = self._latest_pose.yaw
        self._pid = PIDState()
        self._started_time = time.monotonic()
        self._goal_enter_time = None
        self._state = StraightLineState.RUNNING

        if self._request_auto_mode_on_start:
            msg = String()
            msg.data = self._required_mode
            self._mode_pub.publish(msg)

        self.get_logger().info(
            f"Straight-line test started | target={self._target_distance_m:.2f} m | "
            f"direction={self._direction} | heading={self._target_heading:.3f} rad"
        )

    # ----------------------------------------------------------------------
    # Control math
    # ----------------------------------------------------------------------
    def _travelled_distance(self) -> float:
        if self._start_pose is None:
            return 0.0

        dx = self._latest_pose.x - self._start_pose.x
        dy = self._latest_pose.y - self._start_pose.y
        return math.sqrt(dx * dx + dy * dy)

    def _compute_distance_pid_vx(self, remaining_m: float) -> float:
        now = time.monotonic()

        if self._pid.previous_time is None:
            dt = 1.0 / self._loop_hz
        else:
            dt = max(1.0e-4, now - self._pid.previous_time)

        error = max(0.0, remaining_m)

        if error <= self._integral_active_error_m:
            self._pid.integral += error * dt
            self._pid.integral = self._clamp(
                self._pid.integral,
                -self._integral_limit,
                self._integral_limit,
            )
        else:
            self._pid.integral = 0.0

        if self._pid.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._pid.previous_error) / dt

        alpha = self._derivative_filter_alpha
        self._pid.derivative_filtered = (
            alpha * derivative + (1.0 - alpha) * self._pid.derivative_filtered
        )

        raw = (
            self._kp * error
            + self._ki * self._pid.integral
            + self._kd * self._pid.derivative_filtered
        )

        self._pid.previous_error = error
        self._pid.previous_time = now

        if self._direction == "backward":
            limit = self._max_backward_vx
        else:
            limit = self._max_forward_vx

        vx = self._clamp(raw, 0.0, limit)

        if (
            vx > 0.0
            and error > self._disable_min_vx_below_error_m
            and vx < self._min_vx_when_active
        ):
            vx = self._min_vx_when_active

        return self._safe_float(vx)

    def _compute_heading_correction(self) -> float:
        if not self._hold_heading:
            return 0.0

        if self._target_heading is None:
            return 0.0

        error = self._normalize_angle(self._target_heading - self._latest_pose.yaw)

        if abs(error) < self._heading_tolerance_rad:
            return 0.0

        wz = self._heading_kp * error
        return self._clamp(wz, -self._max_heading_correction_wz, self._max_heading_correction_wz)

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def _publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = self._safe_float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._safe_float(wz)
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _publish_status_if_needed(self) -> None:
        if not self._publish_status or self._status_pub is None:
            return

        now = time.monotonic()
        period = 1.0 / max(0.1, self._status_hz)
        if (now - self._last_status_time) < period:
            return

        self._last_status_time = now

        travelled = self._travelled_distance() if self._start_pose is not None else 0.0
        remaining = max(0.0, self._target_distance_m - travelled)

        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"target_m={self._target_distance_m:.3f}; "
            f"travelled_m={travelled:.3f}; "
            f"remaining_m={remaining:.3f}; "
            f"direction={self._direction}; "
            f"safety_stop={self._safety_stop}; "
            f"odom_fresh={self._odom_is_fresh()}"
        )
        self._status_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Validity helpers
    # ----------------------------------------------------------------------
    def _odom_is_fresh(self) -> bool:
        if self._latest_pose.stamp is None:
            return False
        return (time.monotonic() - self._latest_pose.stamp) <= self._odom_timeout_s

    # ----------------------------------------------------------------------
    # Math helpers
    # ----------------------------------------------------------------------
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

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _safe_float(value: float) -> float:
        if not math.isfinite(float(value)):
            return 0.0
        return float(value)

    def _throttled_warn(self, text: str) -> None:
        now = time.monotonic()
        if (now - self._last_log_time) >= self._log_throttle_s:
            self._last_log_time = now
            self.get_logger().warning(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightLinePidTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Sending zero command.")

    finally:
        try:
            for _ in range(max(1, node._shutdown_zero_count)):
                node._publish_zero()
                time.sleep(0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()