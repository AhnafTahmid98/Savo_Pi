#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish a compact dashboard view for the Robot Savo control layer."""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from savo_control.models import TwistCommand
from savo_control.nodes.control_status_helpers import (
    BoolSample,
    CommandSample,
    OdomSample,
    ScalarSample,
    TextSample,
)
from savo_control.ros import (
    CMD_VEL_SAFE,
    CONTROL_MODE_STATE,
    CONTROL_STATUS,
    ODOM_FILTERED,
    RECOVERY_ACTIVE,
    RECOVERY_REQUEST,
    RECOVERY_STATUS,
    SAFETY_SLOWDOWN_FACTOR,
    SAFETY_STOP,
    STUCK_DETECTED,
    bool_msg_value,
    float_msg_value,
    make_string_msg,
    odom_twist_tuple,
    string_msg_value,
    twist_tuple,
)
from savo_control.utils import finite_or_zero, hypot2, validate_rate, validate_timeout


DASHBOARD_JSON_TOPIC = "/savo_control/dashboard"
DASHBOARD_TEXT_TOPIC = "/savo_control/dashboard_text"
RECOVERY_MONITOR_STATUS_TOPIC = "/savo_control/recovery_monitor_status"


class ControlDashboardNode(Node):
    """Collect control-layer status topics and publish dashboard summaries."""

    def __init__(self) -> None:
        super().__init__("control_dashboard_node")

        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("log_hz", 0.5)

        self.declare_parameter("dashboard_topic", DASHBOARD_JSON_TOPIC)
        self.declare_parameter("dashboard_text_topic", DASHBOARD_TEXT_TOPIC)

        self.declare_parameter("control_status_topic", CONTROL_STATUS)
        self.declare_parameter("recovery_status_topic", RECOVERY_STATUS)
        self.declare_parameter("recovery_monitor_status_topic", RECOVERY_MONITOR_STATUS_TOPIC)

        self.declare_parameter("mode_state_topic", CONTROL_MODE_STATE)
        self.declare_parameter("cmd_vel_safe_topic", CMD_VEL_SAFE)
        self.declare_parameter("safety_stop_topic", SAFETY_STOP)
        self.declare_parameter("slowdown_factor_topic", SAFETY_SLOWDOWN_FACTOR)
        self.declare_parameter("stuck_detected_topic", STUCK_DETECTED)
        self.declare_parameter("recovery_request_topic", RECOVERY_REQUEST)
        self.declare_parameter("recovery_active_topic", RECOVERY_ACTIVE)
        self.declare_parameter("odom_topic", ODOM_FILTERED)

        self.declare_parameter("watch_odom", True)
        self.declare_parameter("publish_console_log", True)

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

        self._log_hz = float(self.get_parameter("log_hz").value)
        if self._log_hz < 0.0:
            raise ValueError("log_hz must be >= 0.0")

        self._dashboard_topic = str(self.get_parameter("dashboard_topic").value)
        self._dashboard_text_topic = str(
            self.get_parameter("dashboard_text_topic").value
        )

        self._control_status_topic = str(
            self.get_parameter("control_status_topic").value
        )
        self._recovery_status_topic = str(
            self.get_parameter("recovery_status_topic").value
        )
        self._recovery_monitor_status_topic = str(
            self.get_parameter("recovery_monitor_status_topic").value
        )

        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)
        self._cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(
            self.get_parameter("slowdown_factor_topic").value
        )
        self._stuck_detected_topic = str(
            self.get_parameter("stuck_detected_topic").value
        )
        self._recovery_request_topic = str(
            self.get_parameter("recovery_request_topic").value
        )
        self._recovery_active_topic = str(
            self.get_parameter("recovery_active_topic").value
        )
        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._publish_console_log = bool(self.get_parameter("publish_console_log").value)

        self._control_status = TextSample()
        self._recovery_status = TextSample()
        self._recovery_monitor_status = TextSample()
        self._mode_state = TextSample()

        self._cmd_vel_safe = CommandSample(command=TwistCommand.zero(source="safe"))
        self._safety_stop = BoolSample()
        self._slowdown_factor = ScalarSample()
        self._stuck_detected = BoolSample()
        self._recovery_request = BoolSample()
        self._recovery_active = BoolSample()
        self._odom = OdomSample()

        self._last_console_log_time = 0.0

        self._dashboard_pub = self.create_publisher(String, self._dashboard_topic, 10)
        self._dashboard_text_pub = self.create_publisher(
            String,
            self._dashboard_text_topic,
            10,
        )

        self.create_subscription(
            String,
            self._control_status_topic,
            self._on_control_status,
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
            self._recovery_monitor_status_topic,
            self._on_recovery_monitor_status,
            10,
        )
        self.create_subscription(
            String,
            self._mode_state_topic,
            self._on_mode_state,
            10,
        )

        self.create_subscription(
            Twist,
            self._cmd_vel_safe_topic,
            self._on_cmd_vel_safe,
            10,
        )
        self.create_subscription(
            Bool,
            self._safety_stop_topic,
            self._on_safety_stop,
            10,
        )
        self.create_subscription(
            Float32,
            self._slowdown_factor_topic,
            self._on_slowdown_factor,
            10,
        )
        self.create_subscription(
            Bool,
            self._stuck_detected_topic,
            self._on_stuck_detected,
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

        if self._watch_odom:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            "ControlDashboardNode started | "
            f"dashboard={self._dashboard_topic} | "
            f"text={self._dashboard_text_topic} | "
            f"stale_timeout={self._stale_timeout_s:.2f}s"
        )

    def _on_control_status(self, msg: String) -> None:
        self._control_status = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_recovery_status(self, msg: String) -> None:
        self._recovery_status = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_recovery_monitor_status(self, msg: String) -> None:
        self._recovery_monitor_status = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_mode_state(self, msg: String) -> None:
        self._mode_state = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        vx, vy, wz = twist_tuple(msg)
        now_s = time.monotonic()

        self._cmd_vel_safe = CommandSample(
            command=TwistCommand(
                vx=finite_or_zero(vx),
                vy=finite_or_zero(vy),
                wz=finite_or_zero(wz),
                source="safe",
                stamp_sec=now_s,
            ).sanitized(),
            stamp_s=now_s,
        )

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_slowdown_factor(self, msg: Float32) -> None:
        self._slowdown_factor = ScalarSample(
            value=finite_or_zero(float_msg_value(msg)),
            stamp_s=time.monotonic(),
        )

    def _on_stuck_detected(self, msg: Bool) -> None:
        self._stuck_detected = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_recovery_request(self, msg: Bool) -> None:
        self._recovery_request = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_recovery_active(self, msg: Bool) -> None:
        self._recovery_active = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_odom(self, msg: Odometry) -> None:
        vx, vy, wz = odom_twist_tuple(msg)

        self._odom = OdomSample(
            linear_speed=hypot2(vx, vy),
            angular_speed=abs(finite_or_zero(wz)),
            stamp_s=time.monotonic(),
        )

    def _on_timer(self) -> None:
        payload = self._build_dashboard_payload()
        text = self._build_dashboard_text(payload)

        self._dashboard_pub.publish(
            make_string_msg(json.dumps(payload, sort_keys=True), msg_type=String)
        )
        self._dashboard_text_pub.publish(make_string_msg(text, msg_type=String))

        if self._publish_console_log:
            self._log_status_throttled(text)

    def _build_dashboard_payload(self) -> dict:
        now_s = time.monotonic()
        level = self._compute_level(now_s=now_s)

        return {
            "level": level,
            "mode_state": self._fresh_text(self._mode_state, now_s=now_s),
            "safety_stop": self._fresh_bool(self._safety_stop, now_s=now_s),
            "slowdown_factor": self._fresh_scalar(
                self._slowdown_factor,
                now_s=now_s,
            ),
            "stuck_detected": self._fresh_bool(
                self._stuck_detected,
                now_s=now_s,
            ),
            "recovery_request": self._fresh_bool(
                self._recovery_request,
                now_s=now_s,
            ),
            "recovery_active": self._fresh_bool(
                self._recovery_active,
                now_s=now_s,
            ),
            "cmd_vel_safe": self._command_dict(self._cmd_vel_safe, now_s=now_s),
            "odom": self._odom_dict(now_s=now_s),
            "control_status": self._fresh_text(self._control_status, now_s=now_s),
            "recovery_status": self._fresh_text(self._recovery_status, now_s=now_s),
            "recovery_monitor_status": self._fresh_text(
                self._recovery_monitor_status,
                now_s=now_s,
            ),
        }

    def _build_dashboard_text(self, payload: dict) -> str:
        cmd = payload["cmd_vel_safe"]
        odom = payload["odom"]

        return (
            f"level={payload['level']}; "
            f"mode={payload['mode_state']}; "
            f"safety_stop={payload['safety_stop']}; "
            f"slowdown={payload['slowdown_factor']}; "
            f"stuck={payload['stuck_detected']}; "
            f"recovery_active={payload['recovery_active']}; "
            f"safe_cmd=vx={cmd['vx']},vy={cmd['vy']},wz={cmd['wz']},fresh={cmd['fresh']}; "
            f"odom=lin={odom['linear_speed']},ang={odom['angular_speed']},fresh={odom['fresh']}"
        )

    def _compute_level(self, *, now_s: float) -> str:
        if (
            self._safety_stop.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and self._safety_stop.value is True
        ):
            return "SAFETY_STOP"

        if (
            self._stuck_detected.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and self._stuck_detected.value is True
        ):
            return "STUCK"

        if (
            self._recovery_active.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and self._recovery_active.value is True
        ):
            return "RECOVERY_ACTIVE"

        if self._watch_odom and not self._odom.fresh(
            now_s=now_s,
            timeout_s=self._stale_timeout_s,
        ):
            return "WARN_ODOM_STALE"

        if not self._cmd_vel_safe.fresh(
            now_s=now_s,
            timeout_s=self._stale_timeout_s,
        ):
            return "WARN_CMD_SAFE_STALE"

        return "OK"

    def _fresh_text(self, sample: TextSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return sample.value if sample.value else ""

    def _fresh_bool(self, sample: BoolSample, *, now_s: float) -> bool | str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return bool(sample.value)

    def _fresh_scalar(self, sample: ScalarSample, *, now_s: float) -> float | str:
        if (
            sample.value is None
            or not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        ):
            return "STALE"

        return round(float(sample.value), 3)

    def _command_dict(self, sample: CommandSample, *, now_s: float) -> dict:
        fresh = sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        cmd = sample.command.sanitized()

        return {
            "fresh": fresh,
            "moving": sample.moving() if fresh else False,
            "vx": round(cmd.vx, 3),
            "vy": round(cmd.vy, 3),
            "wz": round(cmd.wz, 3),
        }

    def _odom_dict(self, *, now_s: float) -> dict:
        if not self._watch_odom:
            return {
                "fresh": False,
                "enabled": False,
                "linear_speed": 0.0,
                "angular_speed": 0.0,
            }

        fresh = self._odom.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)

        return {
            "fresh": fresh,
            "enabled": True,
            "linear_speed": round(self._odom.linear_speed, 3),
            "angular_speed": round(self._odom.angular_speed, 3),
        }

    def _log_status_throttled(self, status: str) -> None:
        if self._log_hz <= 0.0:
            return

        now_s = time.monotonic()
        period_s = 1.0 / self._log_hz

        if (now_s - self._last_console_log_time) < period_s:
            return

        self._last_console_log_time = now_s
        self.get_logger().info(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlDashboardNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting control dashboard node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
