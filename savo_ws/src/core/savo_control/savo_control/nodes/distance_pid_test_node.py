#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback distance PID test node.

This node is for diagnostics and fallback testing. The C++ distance approach
node remains the production runtime target.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String

from savo_control.controllers.distance_pid_py import (
    DistanceControllerConfig,
    DistancePid,
)
from savo_control.controllers.pid_py import PidConfig
from savo_control.models import DistanceApproachConfig, TwistCommand
from savo_control.ros import (
    CMD_VEL_AUTO,
    DEPTH_MIN_FRONT,
    DISTANCE_TEST_STATE,
    DISTANCE_TEST_TARGET,
    SAFETY_STOP,
    bool_msg_value,
    float_msg_value,
    make_string_msg,
)
from savo_control.utils import finite_or_zero, validate_rate, validate_timeout


DEFAULT_ENABLE_TOPIC = "/savo_control/distance_approach_enable"


class DistancePidTestNode(Node):
    """Run a Python fallback distance PID controller from a distance topic."""

    def __init__(self) -> None:
        super().__init__("distance_pid_test_node")

        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("stale_timeout_s", 0.40)
        self.declare_parameter("status_hz", 2.0)

        self.declare_parameter("distance_topic", DEPTH_MIN_FRONT)
        self.declare_parameter("cmd_vel_out_topic", CMD_VEL_AUTO)
        self.declare_parameter("state_topic", DISTANCE_TEST_STATE)
        self.declare_parameter("target_topic", DISTANCE_TEST_TARGET)
        self.declare_parameter("enable_topic", DEFAULT_ENABLE_TOPIC)
        self.declare_parameter("safety_stop_topic", SAFETY_STOP)

        self.declare_parameter("auto_start", False)
        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("publish_zero_when_disabled", True)

        self.declare_parameter("target_distance_m", 0.60)
        self.declare_parameter("tolerance_m", 0.04)
        self.declare_parameter("hard_min_distance_m", 0.35)
        self.declare_parameter("min_valid_distance_m", 0.05)
        self.declare_parameter("max_valid_distance_m", 3.00)
        self.declare_parameter("distance_timeout_s", 0.40)

        self.declare_parameter("kp", 0.45)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.03)

        self.declare_parameter("max_forward_vx", 0.10)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("max_reverse_vx", 0.05)
        self.declare_parameter("min_vx_when_active", 0.04)
        self.declare_parameter("disable_min_vx_below_error_m", 0.08)

        self.declare_parameter("output_deadband_m_s", 0.0)

        self._publish_hz = validate_rate(
            self.get_parameter("publish_hz").value,
            name="publish_hz",
        )
        self._stale_timeout_s = validate_timeout(
            self.get_parameter("stale_timeout_s").value,
            name="stale_timeout_s",
        )
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0.0")

        self._status_hz = float(self.get_parameter("status_hz").value)
        if self._status_hz < 0.0:
            raise ValueError("status_hz must be >= 0.0")

        self._distance_topic = str(self.get_parameter("distance_topic").value)
        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._target_topic = str(self.get_parameter("target_topic").value)
        self._enable_topic = str(self.get_parameter("enable_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self._enabled = bool(self.get_parameter("auto_start").value)
        self._respect_safety_stop = bool(
            self.get_parameter("respect_safety_stop").value
        )
        self._publish_zero_when_disabled = bool(
            self.get_parameter("publish_zero_when_disabled").value
        )

        approach_cfg = self._load_distance_config()
        controller_cfg = DistanceControllerConfig.from_distance_approach_config(
            approach_cfg,
        )
        controller_cfg.output_deadband_m_s = float(
            self.get_parameter("output_deadband_m_s").value
        )

        self._controller = DistancePid(controller_cfg)
        self._controller.set_target_distance(approach_cfg.target_distance_m)

        self._target_distance_m = approach_cfg.target_distance_m
        self._latest_distance_m: float | None = None
        self._latest_distance_stamp_s: float | None = None
        self._last_update_s = time.monotonic()
        self._last_status_s = 0.0
        self._safety_stop = False
        self._safety_stamp_s: float | None = None

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)

        self.create_subscription(
            Float32,
            self._distance_topic,
            self._on_distance,
            10,
        )
        self.create_subscription(
            Float64,
            self._distance_topic,
            self._on_distance,
            10,
        )
        self.create_subscription(
            Float32,
            self._target_topic,
            self._on_target,
            10,
        )
        self.create_subscription(
            Float64,
            self._target_topic,
            self._on_target,
            10,
        )
        self.create_subscription(
            Bool,
            self._enable_topic,
            self._on_enable,
            10,
        )
        self.create_subscription(
            Bool,
            self._safety_stop_topic,
            self._on_safety_stop,
            10,
        )

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            "DistancePidTestNode started | "
            f"distance={self._distance_topic} | "
            f"cmd_out={self._cmd_vel_out_topic} | "
            f"target={self._target_distance_m:.2f}m | "
            f"enabled={self._enabled}"
        )

    def _load_distance_config(self) -> DistanceApproachConfig:
        return DistanceApproachConfig(
            target_distance_m=float(self.get_parameter("target_distance_m").value),
            tolerance_m=float(self.get_parameter("tolerance_m").value),
            hard_min_distance_m=float(
                self.get_parameter("hard_min_distance_m").value
            ),
            min_valid_distance_m=float(
                self.get_parameter("min_valid_distance_m").value
            ),
            max_valid_distance_m=float(
                self.get_parameter("max_valid_distance_m").value
            ),
            distance_timeout_s=float(self.get_parameter("distance_timeout_s").value),
            kp=float(self.get_parameter("kp").value),
            ki=float(self.get_parameter("ki").value),
            kd=float(self.get_parameter("kd").value),
            max_forward_vx=float(self.get_parameter("max_forward_vx").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
            max_reverse_vx=float(self.get_parameter("max_reverse_vx").value),
            min_vx_when_active=float(
                self.get_parameter("min_vx_when_active").value
            ),
            disable_min_vx_below_error_m=float(
                self.get_parameter("disable_min_vx_below_error_m").value
            ),
        ).sanitized()

    def _on_distance(self, msg: Float32 | Float64) -> None:
        self._latest_distance_m = finite_or_zero(float_msg_value(msg))
        self._latest_distance_stamp_s = time.monotonic()

    def _on_target(self, msg: Float32 | Float64) -> None:
        target = finite_or_zero(float_msg_value(msg))
        self._target_distance_m = target
        self._controller.set_target_distance(target)
        self.get_logger().info(f"Distance target updated: {target:.3f} m")

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = bool_msg_value(msg)
        self.get_logger().info(f"Distance PID enable: {self._enabled}")

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool_msg_value(msg)
        self._safety_stamp_s = time.monotonic()

    def _on_timer(self) -> None:
        now_s = time.monotonic()
        dt_s = now_s - self._last_update_s
        self._last_update_s = now_s

        cmd = TwistCommand.zero(source="distance_pid_test")
        state = "IDLE"
        reason = "disabled"

        distance_fresh = self._distance_fresh(now_s=now_s)
        safety_active = self._safety_active(now_s=now_s)

        if self._enabled and safety_active:
            state = "BLOCKED"
            reason = "safety_stop"

        elif self._enabled and not distance_fresh:
            state = "STALE"
            reason = "distance_stale"

        elif self._enabled and self._latest_distance_m is not None:
            result = self._controller.update(self._latest_distance_m, dt_s)

            if result.valid:
                cmd = TwistCommand(
                    vx=result.vx_cmd_m_s,
                    source="distance_pid_test",
                    stamp_sec=now_s,
                ).sanitized()

                if result.within_tolerance:
                    state = "GOAL"
                    reason = "within_tolerance"
                elif result.direction_limited:
                    state = "LIMITED"
                    reason = "direction_limited"
                else:
                    state = "RUNNING"
                    reason = "tracking"
            else:
                state = "INVALID"
                reason = "controller_invalid"

        if self._enabled or self._publish_zero_when_disabled:
            self._publish_twist(cmd)

        self._publish_status_throttled(
            now_s=now_s,
            state=state,
            reason=reason,
            cmd=cmd,
            distance_fresh=distance_fresh,
            safety_active=safety_active,
        )

    def _distance_fresh(self, *, now_s: float) -> bool:
        if self._latest_distance_stamp_s is None:
            return False

        return (now_s - self._latest_distance_stamp_s) <= self._stale_timeout_s

    def _safety_active(self, *, now_s: float) -> bool:
        if not self._respect_safety_stop:
            return False

        if self._safety_stamp_s is None:
            return False

        if (now_s - self._safety_stamp_s) > self._stale_timeout_s:
            return False

        return self._safety_stop

    def _publish_twist(self, cmd: TwistCommand) -> None:
        msg = Twist()
        safe = cmd.sanitized()

        msg.linear.x = safe.vx
        msg.linear.y = safe.vy
        msg.angular.z = safe.wz

        self._cmd_pub.publish(msg)

    def _publish_status_throttled(
        self,
        *,
        now_s: float,
        state: str,
        reason: str,
        cmd: TwistCommand,
        distance_fresh: bool,
        safety_active: bool,
    ) -> None:
        if self._status_hz <= 0.0:
            return

        period_s = 1.0 / self._status_hz

        if (now_s - self._last_status_s) < period_s:
            return

        self._last_status_s = now_s

        distance = (
            "none"
            if self._latest_distance_m is None
            else f"{self._latest_distance_m:.3f}"
        )

        status = (
            f"state={state}; "
            f"enabled={str(self._enabled).lower()}; "
            f"reason={reason}; "
            f"distance_m={distance}; "
            f"target_m={self._target_distance_m:.3f}; "
            f"distance_fresh={str(distance_fresh).lower()}; "
            f"safety_active={str(safety_active).lower()}; "
            f"vx={cmd.sanitized().vx:.3f}"
        )

        self._state_pub.publish(make_string_msg(status, msg_type=String))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistancePidTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting distance PID test node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
