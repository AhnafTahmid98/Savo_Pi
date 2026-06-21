#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Run predefined recovery diagnostic tests."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from savo_control.adapters import twist_to_ros_msg
from savo_control.nodes.recovery_test_helpers import (
    RecoveryTestLimits,
    RecoveryTestState,
    default_recovery_tests,
    profile_from_mapping,
    start_active_test,
    status_text,
    step_active_test,
    stop_command,
)
from savo_control.ros import (
    CMD_VEL_RECOVERY,
    CONTROL_MODE_CMD,
    RECOVERY_REQUEST,
    SAFETY_STOP,
    bool_msg_value,
    make_string_msg,
    string_msg_value,
)
from savo_control.utils import validate_rate, validate_timeout


RECOVERY_TEST_CMD_TOPIC = "/savo_control/recovery_test_cmd"
RECOVERY_TEST_ENABLE_TOPIC = "/savo_control/recovery_test_enable"
RECOVERY_TEST_STATUS_TOPIC = "/savo_control/recovery_test_status"


class RecoveryTestManagerNode(Node):
    """Runs safe recovery diagnostic test profiles."""

    def __init__(self) -> None:
        super().__init__("recovery_test_manager_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("auto_start", False)

        self.declare_parameter("output_topic", CMD_VEL_RECOVERY)
        self.declare_parameter("mode_cmd_topic", CONTROL_MODE_CMD)
        self.declare_parameter("recovery_request_topic", RECOVERY_REQUEST)
        self.declare_parameter("test_cmd_topic", RECOVERY_TEST_CMD_TOPIC)
        self.declare_parameter("enable_topic", RECOVERY_TEST_ENABLE_TOPIC)
        self.declare_parameter("status_topic", RECOVERY_TEST_STATUS_TOPIC)
        self.declare_parameter("safety_stop_topic", SAFETY_STOP)

        self.declare_parameter("request_recovery_mode_on_start", True)
        self.declare_parameter("required_mode", "RECOVERY")

        self.declare_parameter("send_stop_before_test", True)
        self.declare_parameter("send_stop_after_test", True)
        self.declare_parameter("stop_hold_s", 0.5)

        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("max_test_duration_s", 8.0)
        self.declare_parameter("default_test_name", "backup_short")

        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("log_throttle_s", 2.0)
        self.declare_parameter("shutdown_zero_count", 5)

        self.declare_parameter("limits.max_backup_vx", 0.10)
        self.declare_parameter("limits.max_turn_wz", 0.35)
        self.declare_parameter("limits.max_duration_s", 8.0)

        self._declare_default_test_parameters()
        self._load_parameters()

        self._cmd_pub = self.create_publisher(Twist, self._output_topic, 10)
        self._mode_pub = self.create_publisher(String, self._mode_cmd_topic, 10)
        self._request_pub = self.create_publisher(Bool, self._recovery_request_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(String, self._test_cmd_topic, self._on_test_cmd, 10)
        self.create_subscription(Bool, self._enable_topic, self._on_enable, 10)
        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        self._state = RecoveryTestState.IDLE if self._enabled else RecoveryTestState.DISABLED
        self._active_test = None
        self._safety_stop = False
        self._stop_until_s = 0.0
        self._pending_test_name = ""
        self._last_status_s = 0.0
        self._last_log_s = 0.0

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        if self._auto_start and self._enabled:
            self._request_start_test(self._default_test_name)

        self.get_logger().info(
            "RecoveryTestManagerNode started | "
            f"output={self._output_topic} | cmd={self._test_cmd_topic} | "
            f"default_test={self._default_test_name} | auto_start={self._auto_start}"
        )

    def _declare_default_test_parameters(self) -> None:
        defaults = {
            "request_only": {"type": "request_only"},
            "backup_short": {
                "type": "backup",
                "duration_s": 1.0,
                "vx": -0.06,
            },
            "backup_long": {
                "type": "backup",
                "duration_s": 2.0,
                "vx": -0.06,
            },
            "rotate_left": {
                "type": "rotate",
                "duration_s": 1.2,
                "wz": 0.25,
            },
            "rotate_right": {
                "type": "rotate",
                "duration_s": 1.2,
                "wz": -0.25,
            },
            "backup_then_left": {
                "type": "backup_then_rotate",
                "backup_s": 1.0,
                "rotate_s": 1.0,
                "vx": -0.06,
                "wz": 0.25,
            },
            "backup_then_right": {
                "type": "backup_then_rotate",
                "backup_s": 1.0,
                "rotate_s": 1.0,
                "vx": -0.06,
                "wz": -0.25,
            },
        }

        for name, values in defaults.items():
            for key, value in values.items():
                self.declare_parameter(f"tests.{name}.{key}", value)

    def _load_parameters(self) -> None:
        self._enabled = bool(self.get_parameter("enabled").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)

        self._output_topic = str(self.get_parameter("output_topic").value)
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._recovery_request_topic = str(
            self.get_parameter("recovery_request_topic").value
        )
        self._test_cmd_topic = str(self.get_parameter("test_cmd_topic").value)
        self._enable_topic = str(self.get_parameter("enable_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self._request_recovery_mode_on_start = bool(
            self.get_parameter("request_recovery_mode_on_start").value
        )
        self._required_mode = str(self.get_parameter("required_mode").value)

        self._send_stop_before_test = bool(self.get_parameter("send_stop_before_test").value)
        self._send_stop_after_test = bool(self.get_parameter("send_stop_after_test").value)
        self._stop_hold_s = validate_timeout(
            self.get_parameter("stop_hold_s").value,
            name="stop_hold_s",
        )

        self._publish_hz = validate_rate(
            self.get_parameter("publish_hz").value,
            name="publish_hz",
        )
        self._max_test_duration_s = validate_timeout(
            self.get_parameter("max_test_duration_s").value,
            name="max_test_duration_s",
        )
        self._default_test_name = str(self.get_parameter("default_test_name").value)

        self._respect_safety_stop = bool(self.get_parameter("respect_safety_stop").value)
        self._publish_status = bool(self.get_parameter("publish_status").value)
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._log_throttle_s = float(self.get_parameter("log_throttle_s").value)
        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)

        if self._status_hz < 0.0:
            raise ValueError("status_hz must be >= 0.0")
        if self._log_throttle_s < 0.0:
            raise ValueError("log_throttle_s must be >= 0.0")
        if self._shutdown_zero_count < 1:
            self._shutdown_zero_count = 1

        self._limits = RecoveryTestLimits(
            max_backup_vx=float(self.get_parameter("limits.max_backup_vx").value),
            max_turn_wz=float(self.get_parameter("limits.max_turn_wz").value),
            max_duration_s=float(self.get_parameter("limits.max_duration_s").value),
        ).sanitized()

        self._known_tests = self._load_known_tests()

    def _load_known_tests(self) -> dict:
        known = default_recovery_tests(self._limits)

        for name in list(known):
            data = {
                "type": self.get_parameter(f"tests.{name}.type").value,
                "duration_s": self._param_or_none(f"tests.{name}.duration_s"),
                "backup_s": self._param_or_none(f"tests.{name}.backup_s"),
                "rotate_s": self._param_or_none(f"tests.{name}.rotate_s"),
                "vx": self._param_or_none(f"tests.{name}.vx"),
                "wz": self._param_or_none(f"tests.{name}.wz"),
            }
            data = {key: value for key, value in data.items() if value is not None}
            known[name] = profile_from_mapping(name, data, limits=self._limits)

        return known

    def _param_or_none(self, name: str):
        if not self.has_parameter(name):
            return None
        return self.get_parameter(name).value

    def _on_test_cmd(self, msg: String) -> None:
        command = string_msg_value(msg).strip()

        if command.upper() in {"STOP", "CANCEL", "IDLE"}:
            self._stop_test(reason="command_stop")
            return

        self._request_start_test(command)

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = bool_msg_value(msg)

        if not self._enabled:
            self._stop_test(reason="disabled")
            self._state = RecoveryTestState.DISABLED
        elif self._state == RecoveryTestState.DISABLED:
            self._state = RecoveryTestState.IDLE

        self.get_logger().info(f"Recovery test enabled={self._enabled}")

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool_msg_value(msg)

    def _request_start_test(self, name: str) -> None:
        if not self._enabled:
            self._state = RecoveryTestState.DISABLED
            self._publish_status_now(reason="disabled", request_recovery=False)
            return

        test_name = str(name).strip()
        if test_name not in self._known_tests:
            self._state = RecoveryTestState.UNKNOWN_TEST
            self._publish_request(False)
            self._publish_zero()
            self._publish_status_now(
                reason=f"unknown_test:{test_name}",
                request_recovery=False,
            )
            self.get_logger().warning(f"Unknown recovery test: {test_name}")
            return

        if self._request_recovery_mode_on_start:
            self._mode_pub.publish(make_string_msg(self._required_mode, msg_type=String))

        self._pending_test_name = test_name

        if self._send_stop_before_test and self._stop_hold_s > 0.0:
            self._state = RecoveryTestState.STOPPING
            self._stop_until_s = time.monotonic() + self._stop_hold_s
            self._publish_request(False)
            self._publish_zero()
        else:
            self._start_pending_test()

    def _start_pending_test(self) -> None:
        if not self._pending_test_name:
            return

        profile = self._known_tests[self._pending_test_name]
        self._active_test = start_active_test(profile, now_s=time.monotonic())
        self._state = RecoveryTestState.RUNNING
        self._pending_test_name = ""
        self.get_logger().info(f"Starting recovery test: {profile.name}")

    def _stop_test(self, *, reason: str) -> None:
        self._active_test = None
        self._pending_test_name = ""
        self._state = RecoveryTestState.IDLE if self._enabled else RecoveryTestState.DISABLED
        self._publish_request(False)
        self._publish_zero()
        self._publish_status_now(reason=reason, request_recovery=False)

    def _on_timer(self) -> None:
        now_s = time.monotonic()

        if not self._enabled:
            self._state = RecoveryTestState.DISABLED
            self._publish_request(False)
            self._publish_zero()
            self._publish_status_throttled(
                now_s=now_s,
                reason="disabled",
                request_recovery=False,
            )
            return

        if self._respect_safety_stop and self._safety_stop:
            self._state = RecoveryTestState.SAFETY_STOP
            self._active_test = None
            self._pending_test_name = ""
            self._publish_request(False)
            self._publish_zero()
            self._publish_status_throttled(
                now_s=now_s,
                reason="safety_stop",
                request_recovery=False,
            )
            return

        if self._state == RecoveryTestState.STOPPING:
            self._publish_request(False)
            self._publish_zero()
            if now_s >= self._stop_until_s:
                self._start_pending_test()
            self._publish_status_throttled(
                now_s=now_s,
                reason="pre_stop",
                request_recovery=False,
            )
            return

        if self._active_test is None:
            self._state = RecoveryTestState.IDLE
            self._publish_request(False)
            self._publish_zero()
            self._publish_status_throttled(
                now_s=now_s,
                reason="idle",
                request_recovery=False,
            )
            return

        result = step_active_test(
            self._active_test,
            now_s=now_s,
            max_duration_s=self._limits.max_duration_s,
        )

        self._state = result.state
        self._active_test = result.active_test

        self._publish_request(result.request_recovery)
        self._cmd_pub.publish(twist_to_ros_msg(result.command, msg_type=Twist))

        if result.finished:
            self._publish_request(False)
            if self._send_stop_after_test:
                self._publish_zero()
            self._active_test = None
            if result.state == RecoveryTestState.TIMEOUT:
                self.get_logger().warning("Recovery test timed out")
            self._state = RecoveryTestState.IDLE

        self._publish_status_throttled(
            now_s=now_s,
            reason=result.reason,
            request_recovery=result.request_recovery,
        )

    def _publish_request(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self._request_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(
            twist_to_ros_msg(stop_command(stamp_sec=time.monotonic()), msg_type=Twist)
        )

    def _publish_status_now(self, *, reason: str, request_recovery: bool) -> None:
        if not self._publish_status:
            return

        self._status_pub.publish(
            make_string_msg(
                self._status_string(
                    reason=reason,
                    request_recovery=request_recovery,
                ),
                msg_type=String,
            )
        )

    def _publish_status_throttled(
        self,
        *,
        now_s: float,
        reason: str,
        request_recovery: bool,
    ) -> None:
        if not self._publish_status or self._status_hz <= 0.0:
            return

        period_s = 1.0 / self._status_hz
        if (now_s - self._last_status_s) < period_s:
            return

        self._last_status_s = now_s
        text = self._status_string(
            reason=reason,
            request_recovery=request_recovery,
        )
        self._status_pub.publish(make_string_msg(text, msg_type=String))

        if self._log_throttle_s > 0.0 and (now_s - self._last_log_s) >= self._log_throttle_s:
            self._last_log_s = now_s
            self.get_logger().info(text)

    def _status_string(self, *, reason: str, request_recovery: bool) -> str:
        active_name = ""
        command = stop_command()

        if self._active_test is not None:
            active_name = self._active_test.profile.name
            if self._active_test.profile.stages:
                command = self._active_test.profile.stages[
                    min(self._active_test.stage_index, len(self._active_test.profile.stages) - 1)
                ].command

        return status_text(
            state=self._state,
            active_name=active_name,
            reason=reason,
            enabled=self._enabled,
            safety_stop=self._safety_stop,
            request_recovery=request_recovery,
            command=command,
        )

    def publish_shutdown_zero(self) -> None:
        self._publish_request(False)
        for _ in range(max(1, self._shutdown_zero_count)):
            self._publish_zero()
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryTestManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping recovery test manager.")

    finally:
        try:
            node.publish_shutdown_zero()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
