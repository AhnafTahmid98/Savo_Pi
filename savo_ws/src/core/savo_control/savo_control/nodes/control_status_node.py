#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Monitor the control command chain and publish compact control status."""

from __future__ import annotations

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
    CMD_VEL,
    CMD_VEL_AUTO,
    CMD_VEL_MANUAL,
    CMD_VEL_MUX,
    CMD_VEL_NAV,
    CMD_VEL_RECOVERY,
    CMD_VEL_SAFE,
    CONTROL_MODE_CMD,
    CONTROL_MODE_STATE,
    ODOM_FILTERED,
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


STATUS_TOPIC_DEFAULT = "/savo_control/status"
class ControlStatusNode(Node):
    """Status monitor for Robot Savo control command chain."""

    def __init__(self) -> None:
        super().__init__("control_status_node")

        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.75)
        self.declare_parameter("log_hz", 0.5)

        self.declare_parameter("status_topic", STATUS_TOPIC_DEFAULT)

        self.declare_parameter("cmd_vel_manual_topic", CMD_VEL_MANUAL)
        self.declare_parameter("cmd_vel_auto_topic", CMD_VEL_AUTO)
        self.declare_parameter("cmd_vel_nav_topic", CMD_VEL_NAV)
        self.declare_parameter("cmd_vel_recovery_topic", CMD_VEL_RECOVERY)
        self.declare_parameter("cmd_vel_mux_topic", CMD_VEL_MUX)
        self.declare_parameter("cmd_vel_topic", CMD_VEL)
        self.declare_parameter("cmd_vel_safe_topic", CMD_VEL_SAFE)

        self.declare_parameter("safety_stop_topic", SAFETY_STOP)
        self.declare_parameter("slowdown_factor_topic", SAFETY_SLOWDOWN_FACTOR)

        self.declare_parameter("mode_cmd_topic", CONTROL_MODE_CMD)
        self.declare_parameter("mode_state_topic", CONTROL_MODE_STATE)

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

        self._status_topic = str(self.get_parameter("status_topic").value)

        self._cmd_topics = {
            "manual": str(self.get_parameter("cmd_vel_manual_topic").value),
            "auto": str(self.get_parameter("cmd_vel_auto_topic").value),
            "nav": str(self.get_parameter("cmd_vel_nav_topic").value),
            "recovery": str(self.get_parameter("cmd_vel_recovery_topic").value),
            "mux": str(self.get_parameter("cmd_vel_mux_topic").value),
            "cmd": str(self.get_parameter("cmd_vel_topic").value),
            "safe": str(self.get_parameter("cmd_vel_safe_topic").value),
        }

        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._slowdown_factor_topic = str(
            self.get_parameter("slowdown_factor_topic").value
        )
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._mode_state_topic = str(self.get_parameter("mode_state_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)

        self._watch_odom = bool(self.get_parameter("watch_odom").value)
        self._watch_slowdown_factor = bool(
            self.get_parameter("watch_slowdown_factor").value
        )
        self._publish_console_log = bool(self.get_parameter("publish_console_log").value)

        self._commands: dict[str, CommandSample] = {
            key: CommandSample(command=TwistCommand.zero(source=key))
            for key in self._cmd_topics
        }
        self._safety_stop = BoolSample()
        self._slowdown = ScalarSample()
        self._mode_cmd = TextSample()
        self._mode_state = TextSample()
        self._odom = OdomSample()

        self._last_console_log_time = 0.0

        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        for key, topic in self._cmd_topics.items():
            self.create_subscription(Twist, topic, self._make_twist_cb(key), 10)

        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        if self._watch_slowdown_factor:
            self.create_subscription(
                Float32,
                self._slowdown_factor_topic,
                self._on_slowdown,
                10,
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

    def _make_twist_cb(self, key: str):
        def _cb(msg: Twist) -> None:
            vx, vy, wz = twist_tuple(msg)
            self._commands[key] = CommandSample(
                command=TwistCommand(
                    vx=finite_or_zero(vx),
                    vy=finite_or_zero(vy),
                    wz=finite_or_zero(wz),
                    source=key,
                    stamp_sec=time.monotonic(),
                ).sanitized(),
                stamp_s=time.monotonic(),
            )

        return _cb

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = BoolSample(
            value=bool_msg_value(msg),
            stamp_s=time.monotonic(),
        )

    def _on_slowdown(self, msg: Float32) -> None:
        self._slowdown = ScalarSample(
            value=finite_or_zero(float_msg_value(msg)),
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

        active_sources = [
            name.upper()
            for name in ("manual", "auto", "nav", "recovery")
            if self._commands[name].fresh(
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            )
            and self._commands[name].moving()
        ]

        mode_cmd = self._fresh_text(self._mode_cmd, now_s=now_s)
        mode_state = self._fresh_text(self._mode_state, now_s=now_s)
        safety = self._safety_text(now_s=now_s)
        slowdown = self._slowdown_text(now_s=now_s)
        odom = self._odom_text(now_s=now_s)

        active_text = ",".join(active_sources) if active_sources else "none"
        status_level = self._compute_status_level(now_s=now_s)

        return (
            f"level={status_level}; "
            f"mode_cmd={mode_cmd}; "
            f"mode_state={mode_state}; "
            f"active_sources={active_text}; "
            f"mux={self._twist_text(self._commands['mux'], now_s=now_s)}; "
            f"cmd={self._twist_text(self._commands['cmd'], now_s=now_s)}; "
            f"safe={self._twist_text(self._commands['safe'], now_s=now_s)}; "
            f"safety_stop={safety}; "
            f"slowdown={slowdown}; "
            f"odom={odom}"
        )

    def _compute_status_level(self, *, now_s: float) -> str:
        if (
            self._safety_stop.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and self._safety_stop.value is True
        ):
            return "SAFETY_STOP"

        cmd = self._commands["cmd"]
        safe = self._commands["safe"]
        mux = self._commands["mux"]

        if (
            cmd.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and cmd.moving()
            and not safe.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        ):
            return "WARN_SAFE_STALE"

        if (
            mux.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and mux.moving()
            and not cmd.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        ):
            return "WARN_SHAPER_STALE"

        source_active = any(
            self._commands[name].fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
            and self._commands[name].moving()
            for name in ("manual", "auto", "nav", "recovery")
        )
        if source_active and not mux.fresh(
            now_s=now_s,
            timeout_s=self._stale_timeout_s,
        ):
            return "WARN_MUX_STALE"

        if self._watch_odom and not self._odom.fresh(
            now_s=now_s,
            timeout_s=self._stale_timeout_s,
        ):
            return "WARN_ODOM_STALE"

        return "OK"

    def _fresh_text(self, sample: TextSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"
        return sample.value

    def _safety_text(self, *, now_s: float) -> str:
        if not self._safety_stop.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"
        return str(self._safety_stop.value)

    def _slowdown_text(self, *, now_s: float) -> str:
        if not self._watch_slowdown_factor:
            return "disabled"

        if (
            self._slowdown.value is None
            or not self._slowdown.fresh(now_s=now_s, timeout_s=self._stale_timeout_s)
        ):
            return "STALE"

        return f"{self._slowdown.value:.2f}"

    def _odom_text(self, *, now_s: float) -> str:
        if not self._watch_odom:
            return "disabled"

        if not self._odom.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        return f"lin={self._odom.linear_speed:.3f},ang={self._odom.angular_speed:.3f}"

    def _twist_text(self, sample: CommandSample, *, now_s: float) -> str:
        if not sample.fresh(now_s=now_s, timeout_s=self._stale_timeout_s):
            return "STALE"

        cmd = sample.command.sanitized()
        return f"vx={cmd.vx:.2f},vy={cmd.vy:.2f},wz={cmd.wz:.2f}"

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
