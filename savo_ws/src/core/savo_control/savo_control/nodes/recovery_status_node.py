#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Monitor recovery topics and publish compact recovery status."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String

from savo_control.models import TwistCommand
from savo_control.nodes.control_status_helpers import (
    BoolSample,
    CommandSample,
    OdomSample,
    ScalarSample,
    TextSample,
)
from savo_control.ros import (
    CMD_VEL_RECOVERY,
    CMD_VEL_SAFE,
    CONTROL_MODE_CMD,
    CONTROL_MODE_STATE,
    ODOM_FILTERED,
    RECOVERY_ACTIVE,
    RECOVERY_REQUEST,
    RECOVERY_STATUS,
    SAFETY_SLOWDOWN_FACTOR,
    SAFETY_STOP,
    bool_msg_value,
    float_msg_value,
    make_string_msg,
    odom_twist_tuple,
    string_msg_value,
    twist_tuple,
)
from savo_control.utils import finite_or_zero, hypot2, validate_rate, validate_timeout


RECOVERY_MONITOR_STATUS_TOPIC = "/savo_control/recovery_monitor_status"
BACKUP_ESCAPE_STATUS_TOPIC = "/savo_control/backup_escape_status"
STUCK_STATE_TOPIC = "/savo_control/stuck_state"


class RecoveryStatusNode(Node):
    """Recovery diagnostics/status monitor."""

    def __init__(self) -> None:
        super().__init__("recovery_status_node")

        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.75)
        self.declare_parameter("log_hz", 0.5)

        self.declare_parameter("output_status_topic", RECOVERY_MONITOR_STATUS_TOPIC)

        self.declare_parameter("cmd_vel_recovery_topic", CMD_VEL_RECOVERY)
        self.declare_parameter("cmd_vel_safe_topic", CMD_VEL_SAFE)

        self.declare_parameter("recovery_status_topic", RECOVERY_STATUS)
        self.declare_parameter("backup_escape_status_topic", BACKUP_ESCAPE_STATUS_TOPIC)
        self.declare_parameter("stuck_state_topic", STUCK_STATE_TOPIC)
        self.declare_parameter("recovery_request_topic", RECOVERY_REQUEST)
        self.declare_parameter("recovery_active_topic", RECOVERY_ACTIVE)

        self.declare_parameter("mode_cmd_topic", CONTROL_MODE_CMD)
        self.declare_parameter("mode_state_topic", CONTROL_MODE_STATE)

        self.declare_parameter("safety_stop_topic", SAFETY_STOP)
        self.declare_parameter("slowdown_factor_topic", SAFETY_SLOWDOWN_FACTOR)

        self.declare_parameter("odom_topic", ODOM_FILTERED)

        self.declare_parameter("watch_odom", True)
        self.declare_parameter("watch_slowdown_factor", True)
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

        self._output_status_topic = str(self.get_parameter("output_status_topic").value)

        self._cmd_vel_recovery_topic = str(
            self.get_parameter("cmd_vel_recovery_topic").value
        )
        self._cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)

        self._recovery_status_topic = str(
            self.get_parameter("recovery_status_topic").value
        )
        self._backup_escape_status_topic = str(
            self.get_parameter("backup_escape_status_topic").value
        )
        self._stuck_state_topic = str(self.get_parameter("stuck_state_topic").value)
        self._recovery_request_topic = str(
            self.get_parameter("recovery_request_topic").value
        )
        self._recovery_active_topic = str(
            self.get_parameter("recovery_active_topic").value
        )

        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)

        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(
            self.get_parameter("slowdown_factor_topic").value
        )

        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._watch_slowdown_factor = bool(
            self.get_parameter("watch_slowdown_factor").value
        )
        self._publish_console_log = bool(self.get_parameter("publish_console_log").value)

        self._cmd_vel_recovery = CommandSample(
            command=TwistCommand.zero(source="recovery")
        )
        self._cmd_vel_safe = CommandSample(command=TwistCommand.zero(source="safe"))

        self._recovery_status = TextSample()
        self._backup_escape_status = TextSample()

        self._stuck_state = BoolSample()
        self._recovery_request = BoolSample()
        self._recovery_active = BoolSample()

        self._mode_cmd = TextSample()
        self._mode_state = TextSample()

        self._safety_stop = BoolSample()
        self._slowdown_factor = ScalarSample()
        self._odom = OdomSample()

        self._last_console_log_time = 0.0

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

        self.create_subscription(Bool, self._stuck_state_topic, self._on_stuck_state, 10)
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
                self._on_slowdown,
                10,
            )
            self.create_subscription(
                Float64,
                self._slowdown_factor_topic,
                self._on_slowdown,
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

    def _on_cmd_vel_recovery(self, msg: Twist) -> None:
        vx, vy, wz = twist_tuple(msg)
        now_s = time.monotonic()

        self._cmd_vel_recovery = CommandSample(
            command=TwistCommand(
                vx=finite_or_zero(vx),
                vy=finite_or_zero(vy),
                wz=finite_or_zero(wz),
                source="recovery",
                stamp_sec=now_s,
            ).sanitized(),
            stamp_s=now_s,
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

    def _on_recovery_status(self, msg: String) -> None:
        self._recovery_status = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_backup_escape_status(self, msg: String) -> None:
        self._backup_escape_status = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_stuck_state(self, msg: Bool) -> None:
        self._stuck_state = BoolSample(
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

    def _on_mode_cmd(self, msg: String) -> None:
        self._mode_cmd = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_mode_state(self, msg: String) -> None:
        self._mode_state = TextSample(
            value=string_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_slowdown(self, msg: Float32 | Float64) -> None:
        self._slowdown_factor = ScalarSample(
            value=finite_or_zero(float_msg_value(msg)),
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
        status = self._build_status_string()
        self._status_pub.publish(make_string_msg(status, msg_type=String))

        if self._publish_console_log:
            self._log_status_throttled(status)

    def _build_status_string(self) -> str:
        now_s = time.monotonic()
        level = self._compute_level(now_s=now_s)

        mode_cmd = self._fresh_text(self._mode_cmd, now_s=now_s)
        mode_state = self._fresh_text(self._mode_state, now_s=now_s)

        safety_stop = self._fresh_bool_text(self._safety_stop, now_s=now_s)
        stuck = self._fresh_bool_text(self._stuck_state, now_s=now_s)
        recovery_request = self._fresh_bool_text(
            self._recovery_request,
            now_s=now_s,
        )
        recovery_active = self._fresh_bool_text(
            self._recovery_active,
            now_s=now_s,
        )

        slowdown = "disabled"
        if self._watch_slowdown_factor:
            slowdown = self._fresh_scalar_text(self._slowdown_factor, now_s=now_s)

        recovery_cmd = self._twist_text(self._cmd_vel_recovery, now_s=now_s)
        safe_cmd = self._twist_text(self._cmd_vel_safe, now_s=now_s)

        recovery_status = self._fresh_text(self._recovery_status, now_s=now_s)
        backup_status = self._fresh_text(self._backup_escape_status, now_s=now_s)

        odom = "disabled"
        if self._watch_odom:
            odom = self._odom_text(now_s=now_s)

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

    def _compute_level(self, *, now_s: float) -> str:
        if (
            self._safety_stop.fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            )
            and self._safety_stop.value is True
        ):
            return "BLOCKED_SAFETY_STOP"

        if (
            self._stuck_state.fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            )
            and self._stuck_state.value is True
        ):
            if (
                self._recovery_request.fresh(
                    now_s=now_s,
                    timeout_s=self._stale_timeout_s,
                )
                and self._recovery_request.value is True
            ):
                return "STUCK_RECOVERY_REQUESTED"

            return "STUCK_DETECTED"

        if (
            self._recovery_active.fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            )
            and self._recovery_active.value is True
        ):
            return "RECOVERY_ACTIVE"

        if (
            self._cmd_vel_recovery.fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            )
            and self._cmd_vel_recovery.moving()
        ):
            if not self._cmd_vel_safe.fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ):
                return "WARN_SAFE_STALE_DURING_RECOVERY"

            if not self._cmd_vel_safe.moving():
                return "RECOVERY_BLOCKED_OR_ZERO_SAFE_CMD"

            return "RECOVERY_ACTIVE"

        if self._watch_odom and not self._odom.fresh(
            now_s=now_s,
            timeout_s=self._stale_timeout_s,
        ):
            return "WARN_ODOM_STALE"

        return "OK"

    def _twist_text(self, sample: CommandSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        cmd = sample.command.sanitized()
        return f"vx={cmd.vx:.2f},vy={cmd.vy:.2f},wz={cmd.wz:.2f}"

    def _fresh_bool_text(self, sample: BoolSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return str(sample.value)

    def _fresh_scalar_text(self, sample: ScalarSample, *, now_s: float) -> str:
        if (
            sample.value is None
            or not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        ):
            return "STALE"

        return f"{sample.value:.2f}"

    def _fresh_text(self, sample: TextSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return sample.value if sample.value else ""

    def _odom_text(self, *, now_s: float) -> str:
        if not self._odom.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return f"lin={self._odom.linear_speed:.3f},ang={self._odom.angular_speed:.3f}"

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
