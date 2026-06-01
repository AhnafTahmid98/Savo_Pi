#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control / nodes / distance_pid_test_node.py (ROS2 Jazzy)
============================================================================

Purpose
-------
Real-robot-safe ROS2 test node for distance PID approach control using the
Python controller wrappers (`pid_py.py` + `distance_pid_py.py`).

This node is intended for:
- controlled tuning sessions
- diagnostics / bringup
- validating distance control behavior before/alongside C++ runtime controllers

Safety-first defaults
---------------------
- Publishes to /cmd_vel_test (NOT /cmd_vel) by default
- Zero command on sensor stale / invalid distance / disabled / safety stop
- Conservative default max velocity and deadband
- Optional target updates from topic, otherwise uses parameter target_distance_m

Expected sensor input
---------------------
By default subscribes to a std_msgs/Float32 distance topic (meters), e.g.:
- /depth/min_front_m
or another range estimate topic you choose.

Notes
-----
- This node is ROS2-only wiring + safety guards around the Python DistancePid wrapper.
- Higher-level mode arbitration / muxing should remain in C++ for production.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist

from savo_control.controllers import (
    PidConfig,
    DistancePid,
    DistanceControllerConfig,
)


class DistancePidTestNode(Node):
    def __init__(self) -> None:
        super().__init__("distance_pid_test_node")

        # ---------------------------------------------------------------------
        # Parameters (topics / runtime)
        # ---------------------------------------------------------------------
        self.declare_parameter("update_rate_hz", 20.0)

        self.declare_parameter("distance_topic", "/depth/min_front_m")
        self.declare_parameter("target_distance_topic", "")         # optional std_msgs/Float32
        self.declare_parameter("enable_topic", "/savo_control/distance_pid_enable")  # optional Bool
        self.declare_parameter("safety_stop_topic", "/safety/stop")                  # optional Bool

        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_test")  # safe default
        self.declare_parameter("status_topic", "/savo_control/distance_pid/status")
        self.declare_parameter("debug_topic", "/savo_control/distance_pid/debug")

        self.declare_parameter("publish_debug", True)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("publish_zero_when_disabled", True)

        # Sensor validity / stale handling
        self.declare_parameter("distance_min_valid_m", 0.02)
        self.declare_parameter("distance_max_valid_m", 5.00)
        self.declare_parameter("distance_stale_timeout_s", 0.30)

        # Enable state
        self.declare_parameter("start_enabled", False)

        # Target
        self.declare_parameter("target_distance_m", 0.80)
        self.declare_parameter("use_target_topic", False)

        # Frame/control options
        self.declare_parameter("command_axis", "x")  # currently only "x" used
        self.declare_parameter("invert_output_sign", False)

        # Stop behavior
        self.declare_parameter("zero_cmd_on_safety_stop", True)
        self.declare_parameter("zero_cmd_on_stale", True)

        # ---------------------------------------------------------------------
        # Parameters (distance controller config)
        # ---------------------------------------------------------------------
        # PID gains
        self.declare_parameter("pid.kp", 0.8)
        self.declare_parameter("pid.ki", 0.0)
        self.declare_parameter("pid.kd", 0.02)

        # PID clamps / filters
        self.declare_parameter("pid.output_min", -0.12)
        self.declare_parameter("pid.output_max", 0.12)
        self.declare_parameter("pid.integral_clamp", 0.50)
        self.declare_parameter("pid.d_filter_alpha", 0.10)
        self.declare_parameter("pid.min_dt_sec", 1e-4)
        self.declare_parameter("pid.max_dt_sec", 0.50)
        self.declare_parameter("pid.freeze_integral_on_invalid_dt", True)
        self.declare_parameter("pid.derivative_on_measurement", False)  # reserved compatibility

        # Distance controller wrapper settings
        self.declare_parameter("distance_tolerance_m", 0.03)
        self.declare_parameter("output_deadband_m_s", 0.005)
        self.declare_parameter("min_effective_vx_m_s", 0.02)
        self.declare_parameter("max_vx_m_s", 0.10)
        self.declare_parameter("zero_output_within_tolerance", True)
        self.declare_parameter("reset_pid_on_target_change", True)
        self.declare_parameter("target_change_reset_threshold_m", 0.01)
        self.declare_parameter("ctrl_min_dt_sec", 1e-4)
        self.declare_parameter("ctrl_max_dt_sec", 0.50)
        self.declare_parameter("direction_mode", "bidirectional")

        # Logging
        self.declare_parameter("log_status_throttle_sec", 2.0)

        # ---------------------------------------------------------------------
        # Internal state
        # ---------------------------------------------------------------------
        self._enabled: bool = False
        self._safety_stop_active: bool = False

        self._distance_m: Optional[float] = None
        self._distance_stamp_sec: Optional[float] = None

        self._target_distance_m: float = 0.80
        self._last_loop_time_sec: Optional[float] = None

        self._last_status_log_sec: float = 0.0
        self._last_cmd_vx: float = 0.0

        # Controller
        self._controller = DistancePid()
        self._rebuild_controller_from_params()

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self._distance_topic = self.get_parameter("distance_topic").get_parameter_value().string_value
        self._target_distance_topic = self.get_parameter("target_distance_topic").get_parameter_value().string_value
        self._enable_topic = self.get_parameter("enable_topic").get_parameter_value().string_value
        self._safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self._cmd_vel_out_topic = self.get_parameter("cmd_vel_out_topic").get_parameter_value().string_value
        self._status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self._debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value

        # Subscribers
        self.sub_distance = self.create_subscription(
            Float32, self._distance_topic, self._on_distance, 10
        )

        self.sub_target = None
        if self.get_parameter("use_target_topic").get_parameter_value().bool_value:
            if self._target_distance_topic:
                self.sub_target = self.create_subscription(
                    Float32, self._target_distance_topic, self._on_target_distance, 10
                )
            else:
                self.get_logger().warn(
                    "use_target_topic=True but target_distance_topic is empty; using parameter target_distance_m"
                )

        self.sub_enable = None
        if self._enable_topic:
            self.sub_enable = self.create_subscription(
                Bool, self._enable_topic, self._on_enable, 10
            )

        self.sub_safety_stop = None
        if self._safety_stop_topic:
            self.sub_safety_stop = self.create_subscription(
                Bool, self._safety_stop_topic, self._on_safety_stop, 10
            )

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self.pub_status = None
        if self.get_parameter("publish_status").get_parameter_value().bool_value:
            self.pub_status = self.create_publisher(String, self._status_topic, 10)

        self.pub_debug = None
        if self.get_parameter("publish_debug").get_parameter_value().bool_value:
            self.pub_debug = self.create_publisher(String, self._debug_topic, 10)

        # Startup enable state and target
        self._enabled = self.get_parameter("start_enabled").get_parameter_value().bool_value
        self._target_distance_m = self.get_parameter("target_distance_m").get_parameter_value().double_value
        self._controller.set_target_distance(self._target_distance_m)

        # Timer
        rate_hz = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        if (not math.isfinite(rate_hz)) or rate_hz <= 0.0:
            rate_hz = 20.0
        self._update_rate_hz = rate_hz
        self._timer = self.create_timer(1.0 / self._update_rate_hz, self._on_timer)

        self.get_logger().info(
            f"distance_pid_test_node started | out={self._cmd_vel_out_topic} | "
            f"distance_topic={self._distance_topic} | target={self._target_distance_m:.3f} m | "
            f"enabled={self._enabled} | safety_topic={self._safety_stop_topic or '<disabled>'}"
        )

    # =========================================================================
    # Parameter handling
    # =========================================================================
    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        # Accept all, then rebuild selected runtime config safely.
        # (Topic changes usually require restart; controller tuning can update live.)
        rebuild_controller = False
        update_target = False

        for p in params:
            if p.name in {
                "pid.kp", "pid.ki", "pid.kd",
                "pid.output_min", "pid.output_max", "pid.integral_clamp",
                "pid.d_filter_alpha", "pid.min_dt_sec", "pid.max_dt_sec",
                "pid.freeze_integral_on_invalid_dt", "pid.derivative_on_measurement",
                "distance_tolerance_m", "output_deadband_m_s", "min_effective_vx_m_s",
                "max_vx_m_s", "zero_output_within_tolerance",
                "reset_pid_on_target_change", "target_change_reset_threshold_m",
                "ctrl_min_dt_sec", "ctrl_max_dt_sec", "direction_mode",
            }:
                rebuild_controller = True

            if p.name == "target_distance_m":
                update_target = True

        if rebuild_controller:
            try:
                self._rebuild_controller_from_params()
            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Failed to rebuild controller: {e}"
                )

        if update_target:
            try:
                target = self.get_parameter("target_distance_m").get_parameter_value().double_value
                if math.isfinite(target):
                    self._target_distance_m = float(target)
                    self._controller.set_target_distance(self._target_distance_m)
            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Failed to set target_distance_m: {e}"
                )

        return SetParametersResult(successful=True)

    def _rebuild_controller_from_params(self) -> None:
        pid_cfg = PidConfig(
            kp=self.get_parameter("pid.kp").get_parameter_value().double_value,
            ki=self.get_parameter("pid.ki").get_parameter_value().double_value,
            kd=self.get_parameter("pid.kd").get_parameter_value().double_value,
            output_min=self.get_parameter("pid.output_min").get_parameter_value().double_value,
            output_max=self.get_parameter("pid.output_max").get_parameter_value().double_value,
            integral_clamp=self.get_parameter("pid.integral_clamp").get_parameter_value().double_value,
            d_filter_alpha=self.get_parameter("pid.d_filter_alpha").get_parameter_value().double_value,
            min_dt_sec=self.get_parameter("pid.min_dt_sec").get_parameter_value().double_value,
            max_dt_sec=self.get_parameter("pid.max_dt_sec").get_parameter_value().double_value,
            freeze_integral_on_invalid_dt=self.get_parameter("pid.freeze_integral_on_invalid_dt").get_parameter_value().bool_value,
            derivative_on_measurement=self.get_parameter("pid.derivative_on_measurement").get_parameter_value().bool_value,
        )

        ctrl_cfg = DistanceControllerConfig(
            pid=pid_cfg,
            distance_tolerance_m=self.get_parameter("distance_tolerance_m").get_parameter_value().double_value,
            output_deadband_m_s=self.get_parameter("output_deadband_m_s").get_parameter_value().double_value,
            min_effective_vx_m_s=self.get_parameter("min_effective_vx_m_s").get_parameter_value().double_value,
            max_vx_m_s=self.get_parameter("max_vx_m_s").get_parameter_value().double_value,
            zero_output_within_tolerance=self.get_parameter("zero_output_within_tolerance").get_parameter_value().bool_value,
            reset_pid_on_target_change=self.get_parameter("reset_pid_on_target_change").get_parameter_value().bool_value,
            target_change_reset_threshold_m=self.get_parameter("target_change_reset_threshold_m").get_parameter_value().double_value,
            min_dt_sec=self.get_parameter("ctrl_min_dt_sec").get_parameter_value().double_value,
            max_dt_sec=self.get_parameter("ctrl_max_dt_sec").get_parameter_value().double_value,
            direction_mode=self.get_parameter("direction_mode").get_parameter_value().string_value,
        )

        self._controller.set_config(ctrl_cfg)

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_distance(self, msg: Float32) -> None:
        if msg is None:
            return
        d = float(msg.data)
        self._distance_m = d
        self._distance_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _on_target_distance(self, msg: Float32) -> None:
        if msg is None:
            return
        d = float(msg.data)
        if not math.isfinite(d):
            return
        self._target_distance_m = d
        self._controller.set_target_distance(d)

    def _on_enable(self, msg: Bool) -> None:
        if msg is None:
            return
        new_state = bool(msg.data)
        if new_state != self._enabled:
            self._enabled = new_state
            self._controller.reset()  # safe reset on enable edge
            self.get_logger().info(f"Distance PID {'ENABLED' if self._enabled else 'DISABLED'}")

    def _on_safety_stop(self, msg: Bool) -> None:
        if msg is None:
            return
        was_active = self._safety_stop_active
        self._safety_stop_active = bool(msg.data)
        if self._safety_stop_active and not was_active:
            self._controller.reset()  # prevent integral/derivative carryover after stop

    # =========================================================================
    # Main control loop
    # =========================================================================
    def _on_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_loop_time_sec is None:
            self._last_loop_time_sec = now_sec
            self._publish_zero(reason="startup")
            self._publish_status_debug(now_sec, reason="startup")
            return

        dt_sec = now_sec - self._last_loop_time_sec
        self._last_loop_time_sec = now_sec

        # Read params used frequently (cheap enough here; keeps behavior live)
        dist_min = self.get_parameter("distance_min_valid_m").get_parameter_value().double_value
        dist_max = self.get_parameter("distance_max_valid_m").get_parameter_value().double_value
        stale_timeout = self.get_parameter("distance_stale_timeout_s").get_parameter_value().double_value
        zero_on_stale = self.get_parameter("zero_cmd_on_stale").get_parameter_value().bool_value
        zero_on_safety = self.get_parameter("zero_cmd_on_safety_stop").get_parameter_value().bool_value
        pub_zero_when_disabled = self.get_parameter("publish_zero_when_disabled").get_parameter_value().bool_value
        invert_sign = self.get_parameter("invert_output_sign").get_parameter_value().bool_value

        # Target from parameter if not using topic
        use_target_topic = self.get_parameter("use_target_topic").get_parameter_value().bool_value
        if not use_target_topic:
            p_target = self.get_parameter("target_distance_m").get_parameter_value().double_value
            if math.isfinite(p_target) and p_target != self._target_distance_m:
                self._target_distance_m = p_target
                self._controller.set_target_distance(p_target)

        # Enable gate
        if not self._enabled:
            self._controller.reset()
            if pub_zero_when_disabled:
                self._publish_zero(reason="disabled")
            self._publish_status_debug(now_sec, reason="disabled")
            return

        # Safety stop gate
        if self._safety_stop_active and zero_on_safety:
            self._controller.reset()
            self._publish_zero(reason="safety_stop")
            self._publish_status_debug(now_sec, reason="safety_stop")
            return

        # Sensor availability / staleness
        if self._distance_m is None or self._distance_stamp_sec is None:
            self._controller.reset()
            self._publish_zero(reason="no_distance")
            self._publish_status_debug(now_sec, reason="no_distance")
            return

        age_sec = now_sec - self._distance_stamp_sec
        if (not math.isfinite(age_sec)) or age_sec < 0.0 or age_sec > stale_timeout:
            self._controller.reset()
            if zero_on_stale:
                self._publish_zero(reason="distance_stale")
            self._publish_status_debug(now_sec, reason="distance_stale")
            return

        # Distance range validity
        current_distance = float(self._distance_m)
        if (not math.isfinite(current_distance)) or current_distance < dist_min or current_distance > dist_max:
            self._controller.reset()
            self._publish_zero(reason="distance_invalid")
            self._publish_status_debug(now_sec, reason="distance_invalid")
            return

        # Controller update
        result = self._controller.update(current_distance, dt_sec)

        vx_cmd = 0.0
        reason = "ok"
        if result.valid:
            vx_cmd = result.vx_cmd_m_s
            if invert_sign:
                vx_cmd = -vx_cmd
        else:
            self._controller.reset()
            vx_cmd = 0.0
            reason = "controller_invalid"

        self._publish_cmd(vx_cmd)
        self._publish_status_debug(now_sec, result=result, reason=reason, sensor_age_sec=age_sec)

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_cmd(self, vx_cmd: float) -> None:
        cmd = Twist()
        axis = self.get_parameter("command_axis").get_parameter_value().string_value.lower().strip()

        # For this node we only support linear.x intentionally (real-robot safety simplicity)
        if axis != "x":
            axis = "x"

        cmd.linear.x = float(vx_cmd)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)
        self._last_cmd_vx = float(vx_cmd)

    def _publish_zero(self, reason: str = "zero") -> None:
        self._publish_cmd(0.0)

    def _publish_status_debug(
        self,
        now_sec: float,
        *,
        result=None,
        reason: str = "",
        sensor_age_sec: Optional[float] = None,
    ) -> None:
        # Throttled INFO log
        throttle_s = self.get_parameter("log_status_throttle_sec").get_parameter_value().double_value
        if math.isfinite(throttle_s) and throttle_s > 0.0:
            if (now_sec - self._last_status_log_sec) >= throttle_s:
                self._last_status_log_sec = now_sec
                dist_str = f"{self._distance_m:.3f}" if self._distance_m is not None and math.isfinite(self._distance_m) else "nan"
                age_str = f"{sensor_age_sec:.3f}" if sensor_age_sec is not None and math.isfinite(sensor_age_sec) else "nan"
                self.get_logger().info(
                    f"state enabled={self._enabled} safety={self._safety_stop_active} "
                    f"dist={dist_str}m target={self._target_distance_m:.3f}m "
                    f"cmd_vx={self._last_cmd_vx:+.3f}m/s age={age_str}s reason={reason}"
                )

        if self.pub_status is not None:
            msg = String()
            msg.data = (
                "{"
                f"\"node\":\"distance_pid_test\","
                f"\"enabled\":{str(self._enabled).lower()},"
                f"\"safety_stop\":{str(self._safety_stop_active).lower()},"
                f"\"target_distance_m\":{self._fmt_num(self._target_distance_m)},"
                f"\"current_distance_m\":{self._fmt_num(self._distance_m)},"
                f"\"cmd_vx_m_s\":{self._fmt_num(self._last_cmd_vx)},"
                f"\"sensor_age_sec\":{self._fmt_num(sensor_age_sec)},"
                f"\"reason\":\"{reason}\""
                "}"
            )
            self.pub_status.publish(msg)

        if self.pub_debug is not None:
            msg = String()
            if result is None:
                msg.data = (
                    f"enabled={int(self._enabled)} safety_stop={int(self._safety_stop_active)} "
                    f"dist={self._fmt_num(self._distance_m)} target={self._fmt_num(self._target_distance_m)} "
                    f"cmd_vx={self._fmt_num(self._last_cmd_vx)} reason={reason}"
                )
            else:
                msg.data = (
                    f"enabled={int(self._enabled)} safety_stop={int(self._safety_stop_active)} "
                    f"dist={self._fmt_num(getattr(result, 'current_distance_m', None))} "
                    f"target={self._fmt_num(getattr(result, 'target_distance_m', None))} "
                    f"err={self._fmt_num(getattr(result, 'error_distance_m', None))} "
                    f"vx={self._fmt_num(getattr(result, 'vx_cmd_m_s', None))} "
                    f"tol={int(bool(getattr(result, 'within_tolerance', False)))} "
                    f"valid={int(bool(getattr(result, 'valid', False)))} "
                    f"dir_lim={int(bool(getattr(result, 'direction_limited', False)))} "
                    f"reason={reason}"
                )
            self.pub_debug.publish(msg)

    @staticmethod
    def _fmt_num(v) -> str:
        try:
            if v is None:
                return "null"
            x = float(v)
            if not math.isfinite(x):
                return "null"
            return f"{x:.6f}"
        except Exception:
            return "null"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistancePidTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safe stop on exit
        try:
            node._publish_zero(reason="shutdown")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()