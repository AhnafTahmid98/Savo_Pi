#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Interactive keyboard teleop node for Robot Savo."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from savo_control.adapters import twist_to_ros_msg
from savo_control.models import TwistCommand
from savo_control.nodes.keyboard_teleop_helpers import (
    TELEOP_SOURCE,
    TeleopLimits,
    TeleopSpeeds,
    apply_teleop_key,
    stop_command,
)
from savo_control.ros import CMD_VEL_MANUAL
from savo_control.utils import validate_rate, validate_timeout


class KeyboardTeleopNode(Node):
    """Keyboard teleoperation node for Robot Savo."""

    def __init__(self) -> None:
        super().__init__("keyboard_teleop_node")

        self.declare_parameter("cmd_vel_topic", CMD_VEL_MANUAL)
        self.declare_parameter("publish_hz", 20.0)

        self.declare_parameter("default_linear_speed", 0.12)
        self.declare_parameter("default_angular_speed", 0.35)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("linear_speed_step", 0.02)
        self.declare_parameter("angular_speed_step", 0.05)

        self.declare_parameter("key_timeout_s", 0.50)
        self.declare_parameter("shutdown_zero_count", 5)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._publish_hz = validate_rate(
            self.get_parameter("publish_hz").value,
            name="publish_hz",
        )
        self._key_timeout_s = validate_timeout(
            self.get_parameter("key_timeout_s").value,
            name="key_timeout_s",
        )
        if self._key_timeout_s <= 0.0:
            raise ValueError("key_timeout_s must be > 0.0")

        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)
        if self._shutdown_zero_count < 1:
            self._shutdown_zero_count = 1

        self._limits = TeleopLimits(
            default_linear=float(self.get_parameter("default_linear_speed").value),
            default_angular=float(self.get_parameter("default_angular_speed").value),
            max_linear=float(self.get_parameter("max_linear_speed").value),
            max_angular=float(self.get_parameter("max_angular_speed").value),
            linear_step=float(self.get_parameter("linear_speed_step").value),
            angular_step=float(self.get_parameter("angular_speed_step").value),
        ).sanitized()
        self._speeds: TeleopSpeeds = self._limits.default_speeds()

        self._current_cmd: TwistCommand = stop_command(source=TELEOP_SOURCE)
        self._last_key_time = time.monotonic()
        self._last_print_time = 0.0

        self._terminal_settings: Optional[list] = None
        self._terminal_ready = sys.stdin.isatty()

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        if self._terminal_ready:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warning(
                "stdin is not a TTY. Keyboard input may not work. "
                "Run this node in an interactive terminal."
            )

        self.get_logger().info(
            "KeyboardTeleopNode started | "
            f"topic={self._cmd_vel_topic} | "
            f"linear={self._speeds.linear:.2f} | "
            f"angular={self._speeds.angular:.2f}"
        )
        self._print_help()

    def _read_key_nonblocking(self) -> Optional[str]:
        if not self._terminal_ready:
            return None

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None

        return sys.stdin.read(1)

    def _handle_key(self, key: str) -> None:
        now_s = time.monotonic()

        result = apply_teleop_key(
            key,
            speeds=self._speeds,
            limits=self._limits,
            stamp_sec=now_s,
            source=TELEOP_SOURCE,
        )

        if not result.handled:
            return

        self._speeds = result.speeds

        if result.command is not None:
            self._current_cmd = result.command

        if result.stop_requested:
            self.get_logger().info("STOP command sent")

        if result.speed_changed:
            self._log_speed("Speed updated")

        if result.help_requested:
            self._print_help()

        self._last_key_time = now_s

    def _on_timer(self) -> None:
        key = self._read_key_nonblocking()
        if key is not None:
            self._handle_key(key)

        now_s = time.monotonic()
        if (now_s - self._last_key_time) > self._key_timeout_s:
            self._current_cmd = stop_command(stamp_sec=now_s, source=TELEOP_SOURCE)

        self._publish_current_command()

    def _publish_current_command(self) -> None:
        self._cmd_vel_pub.publish(twist_to_ros_msg(self._current_cmd, msg_type=Twist))

    def publish_zero(self) -> None:
        self._current_cmd = stop_command(stamp_sec=time.monotonic(), source=TELEOP_SOURCE)
        self._publish_current_command()

    def _log_speed(self, prefix: str = "Speed updated") -> None:
        self.get_logger().info(
            f"{prefix} | linear={self._speeds.linear:.2f}, "
            f"angular={self._speeds.angular:.2f}"
        )

    def _print_help(self) -> None:
        now_s = time.monotonic()
        if now_s - self._last_print_time < 0.5:
            return
        self._last_print_time = now_s

        help_text = f"""
Robot Savo keyboard teleop
--------------------------
Movement:
  w : forward
  s : backward
  a : strafe left
  d : strafe right
  q : rotate left / CCW
  e : rotate right / CW

Stop:
  x or SPACE : stop

Speed:
  t / g : increase / decrease linear speed
  y / h : increase / decrease angular speed
  r     : reset speeds

Other:
  p     : print help
  Ctrl+C: quit safely

Publishing:
  topic: {self._cmd_vel_topic}

Current speed:
  linear : {self._speeds.linear:.2f}
  angular: {self._speeds.angular:.2f}
"""
        print(help_text, flush=True)

    def restore_terminal(self) -> None:
        if self._terminal_ready and self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping robot command.")

    finally:
        try:
            for _ in range(max(1, node._shutdown_zero_count)):
                node.publish_zero()
                time.sleep(0.05)

        finally:
            node.restore_terminal()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
