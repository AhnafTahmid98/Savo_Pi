#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Front-distance approach using /depth/min_front_m from savo-edge. Publishes to /cmd_vel_auto.

First test: wheels lifted or open floor, low speed, safety gate running.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String


class ApproachState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    GOAL_REACHED = "GOAL_REACHED"
    STOPPED_TOO_CLOSE = "STOPPED_TOO_CLOSE"
    SENSOR_STALE = "SENSOR_STALE"
    SENSOR_INVALID = "SENSOR_INVALID"
    SAFETY_STOP = "SAFETY_STOP"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: Optional[float] = None
    previous_time: Optional[float] = None
    derivative_filtered: float = 0.0


class DistancePidTestNode(Node):
    """Conservative distance approach controller."""

    def __init__(self) -> None:
        super().__init__("distance_pid_test_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("enabled", True)

        # Topics
        self.declare_parameter("distance_topic", "/depth/min_front_m")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_auto")
        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("status_topic", "/savo_control/distance_approach_status")

        # Behavior
        self.declare_parameter("auto_start", False)
        self.declare_parameter("request_auto_mode_on_start", True)
        self.declare_parameter("required_mode", "AUTO")
        self.declare_parameter("stop_on_goal", True)
        self.declare_parameter("stop_if_too_close", True)

        # Target
        self.declare_parameter("target_distance_m", 0.60)
        self.declare_parameter("tolerance_m", 0.04)
        self.declare_parameter("hold_time_s", 0.40)
        self.declare_parameter("hard_min_distance_m", 0.35)

        # Sensor validity
        self.declare_parameter("min_valid_distance_m", 0.05)
        self.declare_parameter("max_valid_distance_m", 3.00)
        self.declare_parameter("distance_timeout_s", 0.40)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_alpha", 0.45)

        # PID
        self.declare_parameter("kp", 0.45)
        self.declare_parameter("ki", 0.00)
        self.declare_parameter("kd", 0.03)
        self.declare_parameter("integral_limit", 0.15)
        self.declare_parameter("integral_active_error_m", 0.20)
        self.declare_parameter("derivative_filter_alpha", 0.35)

        # Output limits
        self.declare_parameter("max_forward_vx", 0.12)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("max_reverse_vx", 0.06)
        self.declare_parameter("min_vx_when_active", 0.04)
        self.declare_parameter("disable_min_vx_below_error_m", 0.08)

        # Timing
        self.declare_parameter("loop_hz", 30.0)
        self.declare_parameter("timeout_s", 10.0)
        self.declare_parameter("stop_hold_s", 0.50)
        self.declare_parameter("shutdown_zero_count", 5)

        # Safety
        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("zero_on_invalid_distance", True)
        self.declare_parameter("zero_on_distance_timeout", True)

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

        self._distance_sub = self.create_subscription(
            Float32,
            self._distance_topic,
            self._on_distance_float32,
            10,
        )

        # Also support Float64 by using a second subscription on the same topic.
        # ROS 2 will only match the publisher type that exists.
        self._distance_sub64 = self.create_subscription(
            Float64,
            self._distance_topic,
            self._on_distance_float64,
            10,
        )

        self._safety_sub = self.create_subscription(
            Bool,
            self._safety_stop_topic,
            self._on_safety_stop,
            10,
        )

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._pid = PIDState()
        self._state = ApproachState.IDLE

        self._latest_distance_m: Optional[float] = None
        self._filtered_distance_m: Optional[float] = None
        self._last_distance_time: Optional[float] = None

        self._safety_stop = False
        self._started_time: Optional[float] = None
        self._goal_enter_time: Optional[float] = None
        self._last_status_time = 0.0
        self._last_log_time = 0.0

        if self._auto_start and self._enabled:
            self._start_approach()
        elif not self._enabled:
            self._state = ApproachState.DISABLED

        self._timer = self.create_timer(1.0 / self._loop_hz, self._on_timer)

        self.get_logger().info(
            "DistancePidTestNode started | "
            f"distance={self._distance_topic} | output={self._cmd_vel_out_topic} | "
            f"target={self._target_distance_m:.2f} m | auto_start={self._auto_start}"
        )

    # ----------------------------------------------------------------------
    # Parameter loading
    # ----------------------------------------------------------------------
    def _load_parameters(self) -> None:
        self._enabled = self.get_parameter("enabled").value

        self._distance_topic = str(self.get_parameter("distance_topic").value)
        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)

        self._auto_start = self.get_parameter("auto_start").value
        self._request_auto_mode_on_start = self.get_parameter(
            "request_auto_mode_on_start"
        ).value
        self._required_mode = str(self.get_parameter("required_mode").value)
        self._stop_on_goal = self.get_parameter("stop_on_goal").value
        self._stop_if_too_close = self.get_parameter("stop_if_too_close").value

        self._target_distance_m = float(self.get_parameter("target_distance_m").value)
        self._tolerance_m = float(self.get_parameter("tolerance_m").value)
        self._hold_time_s = float(self.get_parameter("hold_time_s").value)
        self._hard_min_distance_m = float(
            self.get_parameter("hard_min_distance_m").value
        )

        self._min_valid_distance_m = float(
            self.get_parameter("min_valid_distance_m").value
        )
        self._max_valid_distance_m = float(
            self.get_parameter("max_valid_distance_m").value
        )
        self._distance_timeout_s = float(self.get_parameter("distance_timeout_s").value)
        self._smoothing_enabled = self.get_parameter("smoothing_enabled").value
        self._smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)

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

        self._max_forward_vx = float(self.get_parameter("max_forward_vx").value)
        self._allow_reverse = self.get_parameter("allow_reverse").value
        self._max_reverse_vx = float(self.get_parameter("max_reverse_vx").value)
        self._min_vx_when_active = float(self.get_parameter("min_vx_when_active").value)
        self._disable_min_vx_below_error_m = float(
            self.get_parameter("disable_min_vx_below_error_m").value
        )

        self._loop_hz = float(self.get_parameter("loop_hz").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)
        self._stop_hold_s = float(self.get_parameter("stop_hold_s").value)
        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)

        self._respect_safety_stop = self.get_parameter("respect_safety_stop").value
        self._zero_on_invalid_distance = self.get_parameter(
            "zero_on_invalid_distance"
        ).value
        self._zero_on_distance_timeout = self.get_parameter(
            "zero_on_distance_timeout"
        ).value

        self._publish_status = self.get_parameter("publish_status").value
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._log_throttle_s = float(self.get_parameter("log_throttle_s").value)

    def _validate_parameters(self) -> None:
        if self._loop_hz <= 0:
            raise ValueError("loop_hz must be > 0")

        if self._target_distance_m <= 0:
            raise ValueError("target_distance_m must be > 0")

        if self._tolerance_m <= 0:
            raise ValueError("tolerance_m must be > 0")

        if self._min_valid_distance_m <= 0:
            raise ValueError("min_valid_distance_m must be > 0")

        if self._max_valid_distance_m <= self._min_valid_distance_m:
            raise ValueError("max_valid_distance_m must be greater than min_valid_distance_m")

        if not (0.0 < self._smoothing_alpha <= 1.0):
            raise ValueError("smoothing_alpha must be in (0, 1]")

        if not (0.0 <= self._derivative_filter_alpha <= 1.0):
            raise ValueError("derivative_filter_alpha must be in [0, 1]")

        self._max_forward_vx = abs(self._max_forward_vx)
        self._max_reverse_vx = abs(self._max_reverse_vx)
        self._min_vx_when_active = abs(self._min_vx_when_active)

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------
    def _on_distance_float32(self, msg: Float32) -> None:
        self._update_distance(float(msg.data))

    def _on_distance_float64(self, msg: Float64) -> None:
        self._update_distance(float(msg.data))

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _update_distance(self, distance_m: float) -> None:
        now = time.monotonic()

        if not self._is_valid_distance(distance_m):
            self._latest_distance_m = None
            self._last_distance_time = now
            self._state = ApproachState.SENSOR_INVALID
            self._throttled_warn(f"Invalid distance reading: {distance_m}")
            return

        self._latest_distance_m = distance_m
        self._last_distance_time = now

        if self._filtered_distance_m is None or not self._smoothing_enabled:
            self._filtered_distance_m = distance_m
        else:
            alpha = self._smoothing_alpha
            self._filtered_distance_m = (
                alpha * distance_m + (1.0 - alpha) * self._filtered_distance_m
            )

    # ----------------------------------------------------------------------
    # Main loop
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        if not self._enabled:
            self._state = ApproachState.DISABLED
            self._publish_zero()
            self._publish_status_if_needed()
            return

        # If not auto-starting, node remains idle but alive.
        # It can still be changed later by adding service/action control.
        if self._state == ApproachState.IDLE and not self._auto_start:
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._state == ApproachState.IDLE and self._auto_start:
            self._start_approach()

        if self._respect_safety_stop and self._safety_stop:
            self._state = ApproachState.SAFETY_STOP
            self._publish_zero()
            self._publish_status_if_needed()
            return

        distance = self._get_valid_current_distance()
        if distance is None:
            if self._zero_on_distance_timeout:
                self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._started_time is not None:
            elapsed = time.monotonic() - self._started_time
            if elapsed > self._timeout_s:
                self._state = ApproachState.TIMEOUT
                self._publish_zero()
                self._publish_status_if_needed()
                self._throttled_warn("Distance approach timed out")
                return

        if self._stop_if_too_close and distance < self._hard_min_distance_m:
            self._state = ApproachState.STOPPED_TOO_CLOSE
            self._publish_zero()
            self._publish_status_if_needed()
            self._throttled_warn(
                f"Too close: distance={distance:.3f} m < hard_min={self._hard_min_distance_m:.3f} m"
            )
            return

        error = distance - self._target_distance_m

        if abs(error) <= self._tolerance_m:
            if self._goal_enter_time is None:
                self._goal_enter_time = time.monotonic()

            if (time.monotonic() - self._goal_enter_time) >= self._hold_time_s:
                self._state = ApproachState.GOAL_REACHED
                self._publish_zero()
                self._publish_status_if_needed()
                return
        else:
            self._goal_enter_time = None

        self._state = ApproachState.RUNNING
        vx = self._compute_pid_vx(error)
        self._publish_cmd(vx=vx)
        self._publish_status_if_needed()

    # ----------------------------------------------------------------------
    # Behavior
    # ----------------------------------------------------------------------
    def _start_approach(self) -> None:
        self._pid = PIDState()
        self._started_time = time.monotonic()
        self._goal_enter_time = None
        self._state = ApproachState.RUNNING

        if self._request_auto_mode_on_start:
            msg = String()
            msg.data = self._required_mode
            self._mode_pub.publish(msg)

        self.get_logger().info(
            f"Distance approach started | target={self._target_distance_m:.2f} m | "
            f"mode={self._required_mode}"
        )

    def _compute_pid_vx(self, error_m: float) -> float:
        now = time.monotonic()

        if self._pid.previous_time is None:
            dt = 1.0 / self._loop_hz
        else:
            dt = max(1.0e-4, now - self._pid.previous_time)

        # Integral only near target.
        if abs(error_m) <= self._integral_active_error_m:
            self._pid.integral += error_m * dt
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
            derivative = (error_m - self._pid.previous_error) / dt

        alpha = self._derivative_filter_alpha
        self._pid.derivative_filtered = (
            alpha * derivative + (1.0 - alpha) * self._pid.derivative_filtered
        )

        raw = (
            self._kp * error_m
            + self._ki * self._pid.integral
            + self._kd * self._pid.derivative_filtered
        )

        self._pid.previous_error = error_m
        self._pid.previous_time = now

        # If too far away, raw is positive and robot moves forward.
        if raw >= 0.0:
            vx = min(raw, self._max_forward_vx)
        else:
            if self._allow_reverse:
                vx = max(raw, -self._max_reverse_vx)
            else:
                vx = 0.0

        # Minimum active command, but not very close to target.
        if (
            vx > 0.0
            and abs(error_m) > self._disable_min_vx_below_error_m
            and vx < self._min_vx_when_active
        ):
            vx = self._min_vx_when_active

        return self._safe_float(vx)

    # ----------------------------------------------------------------------
    # Validation helpers
    # ----------------------------------------------------------------------
    def _get_valid_current_distance(self) -> Optional[float]:
        now = time.monotonic()

        if self._last_distance_time is None:
            self._state = ApproachState.SENSOR_STALE
            self._throttled_warn("No distance data received yet")
            return None

        if (now - self._last_distance_time) > self._distance_timeout_s:
            self._state = ApproachState.SENSOR_STALE
            self._throttled_warn("Distance data stale")
            return None

        if self._filtered_distance_m is None:
            self._state = ApproachState.SENSOR_INVALID
            return None

        if not self._is_valid_distance(self._filtered_distance_m):
            self._state = ApproachState.SENSOR_INVALID
            self._throttled_warn(f"Filtered distance invalid: {self._filtered_distance_m}")
            return None

        return self._filtered_distance_m

    def _is_valid_distance(self, distance_m: float) -> bool:
        return (
            math.isfinite(distance_m)
            and self._min_valid_distance_m <= distance_m <= self._max_valid_distance_m
        )

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def _publish_cmd(self, vx: float) -> None:
        msg = Twist()
        msg.linear.x = self._safe_float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd(0.0)

    def _publish_status_if_needed(self) -> None:
        if not self._publish_status or self._status_pub is None:
            return

        now = time.monotonic()
        period = 1.0 / max(0.1, self._status_hz)
        if (now - self._last_status_time) < period:
            return

        self._last_status_time = now

        distance = self._filtered_distance_m
        distance_text = "nan" if distance is None else f"{distance:.3f}"

        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"distance_m={distance_text}; "
            f"target_m={self._target_distance_m:.3f}; "
            f"safety_stop={self._safety_stop}"
        )
        self._status_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Utility
    # ----------------------------------------------------------------------
    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _safe_float(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return float(value)

    def _throttled_warn(self, text: str) -> None:
        now = time.monotonic()
        if (now - self._last_log_time) >= self._log_throttle_s:
            self._last_log_time = now
            self.get_logger().warning(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistancePidTestNode()

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