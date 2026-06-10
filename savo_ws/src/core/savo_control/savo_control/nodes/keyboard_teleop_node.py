#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Keyboard teleop publishing to /cmd_vel_manual. Requires an interactive terminal.

Controls:
  w/s   forward/backward    q/e  rotate CCW/CW
  a/d   strafe left/right   x/space  stop
  t/g   linear speed +/-    y/h  angular speed +/-
  r     reset speeds        p    print help    Ctrl+C  quit

First real robot test: wheels lifted, safety gate running.
"""

from __future__ import annotations

import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class TeleopCommand:
    """Simple container for keyboard motion command."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class KeyboardTeleopNode(Node):
    """Keyboard teleoperation node for Robot Savo."""

    def __init__(self) -> None:
        super().__init__("keyboard_teleop_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_manual")
        self.declare_parameter("publish_hz", 20.0)

        self.declare_parameter("default_linear_speed", 0.12)
        self.declare_parameter("default_angular_speed", 0.35)

        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.60)

        self.declare_parameter("linear_speed_step", 0.02)
        self.declare_parameter("angular_speed_step", 0.05)

        # publish zero if no key received (guards against terminal focus loss)
        self.declare_parameter("key_timeout_s", 0.50)

        # Publish zero repeatedly on shutdown.
        self.declare_parameter("shutdown_zero_count", 5)

        self._cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._publish_hz = float(
            self.get_parameter("publish_hz").get_parameter_value().double_value
        )

        self._default_linear_speed = float(
            self.get_parameter("default_linear_speed").get_parameter_value().double_value
        )
        self._default_angular_speed = float(
            self.get_parameter("default_angular_speed").get_parameter_value().double_value
        )

        self._max_linear_speed = float(
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        self._max_angular_speed = float(
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )

        self._linear_speed_step = float(
            self.get_parameter("linear_speed_step").get_parameter_value().double_value
        )
        self._angular_speed_step = float(
            self.get_parameter("angular_speed_step").get_parameter_value().double_value
        )

        self._key_timeout_s = float(
            self.get_parameter("key_timeout_s").get_parameter_value().double_value
        )
        self._shutdown_zero_count = int(
            self.get_parameter("shutdown_zero_count").get_parameter_value().integer_value
        )

        self._validate_params()

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        timer_period = 1.0 / self._publish_hz
        self._timer = self.create_timer(timer_period, self._on_timer)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._linear_speed = self._default_linear_speed
        self._angular_speed = self._default_angular_speed

        self._current_cmd = TeleopCommand()
        self._last_key_time = time.monotonic()
        self._last_print_time = 0.0

        self._terminal_settings: Optional[list] = None
        self._terminal_ready = sys.stdin.isatty()

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
            f"publishing to {self._cmd_vel_topic} | "
            f"linear={self._linear_speed:.2f}, angular={self._angular_speed:.2f}"
        )

        self._print_help()

    # ----------------------------------------------------------------------
    # Parameter validation
    # ----------------------------------------------------------------------
    def _validate_params(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")

        if self._default_linear_speed < 0.0:
            raise ValueError("default_linear_speed must be >= 0")

        if self._default_angular_speed < 0.0:
            raise ValueError("default_angular_speed must be >= 0")

        if self._max_linear_speed <= 0.0:
            raise ValueError("max_linear_speed must be > 0")

        if self._max_angular_speed <= 0.0:
            raise ValueError("max_angular_speed must be > 0")

        if self._key_timeout_s <= 0.0:
            raise ValueError("key_timeout_s must be > 0")

        self._default_linear_speed = min(
            self._default_linear_speed, self._max_linear_speed
        )
        self._default_angular_speed = min(
            self._default_angular_speed, self._max_angular_speed
        )

    # ----------------------------------------------------------------------
    # Keyboard handling
    # ----------------------------------------------------------------------
    def _read_key_nonblocking(self) -> Optional[str]:
        """Read one key from stdin without blocking."""
        if not self._terminal_ready:
            return None

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None

        key = sys.stdin.read(1)
        return key

    def _handle_key(self, key: str) -> None:
        """Convert a key press into a teleop command or speed change."""
        key = key.lower()

        # Movement keys
        if key == "w":
            self._set_command(vx=self._linear_speed, vy=0.0, wz=0.0)
        elif key == "s":
            self._set_command(vx=-self._linear_speed, vy=0.0, wz=0.0)
        elif key == "a":
            self._set_command(vx=0.0, vy=self._linear_speed, wz=0.0)
        elif key == "d":
            self._set_command(vx=0.0, vy=-self._linear_speed, wz=0.0)
        elif key == "q":
            self._set_command(vx=0.0, vy=0.0, wz=self._angular_speed)
        elif key == "e":
            self._set_command(vx=0.0, vy=0.0, wz=-self._angular_speed)

        # Stop keys
        elif key == "x" or key == " ":
            self._set_command(vx=0.0, vy=0.0, wz=0.0)
            self.get_logger().info("STOP command sent")

        # Speed adjustment
        elif key == "t":
            self._linear_speed = min(
                self._max_linear_speed, self._linear_speed + self._linear_speed_step
            )
            self._log_speed()
        elif key == "g":
            self._linear_speed = max(0.0, self._linear_speed - self._linear_speed_step)
            self._log_speed()
        elif key == "y":
            self._angular_speed = min(
                self._max_angular_speed, self._angular_speed + self._angular_speed_step
            )
            self._log_speed()
        elif key == "h":
            self._angular_speed = max(0.0, self._angular_speed - self._angular_speed_step)
            self._log_speed()

        # Reset speeds
        elif key == "r":
            self._linear_speed = self._default_linear_speed
            self._angular_speed = self._default_angular_speed
            self._log_speed(prefix="Speeds reset")

        # Help
        elif key == "p":
            self._print_help()

        # Ignore unknown keys
        else:
            return

        self._last_key_time = time.monotonic()

    def _set_command(self, vx: float, vy: float, wz: float) -> None:
        self._current_cmd = TeleopCommand(
            vx=self._clamp(vx, -self._max_linear_speed, self._max_linear_speed),
            vy=self._clamp(vy, -self._max_linear_speed, self._max_linear_speed),
            wz=self._clamp(wz, -self._max_angular_speed, self._max_angular_speed),
        )

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        key = self._read_key_nonblocking()
        if key is not None:
            self._handle_key(key)

        now = time.monotonic()
        if (now - self._last_key_time) > self._key_timeout_s:
            self._current_cmd = TeleopCommand()

        self._publish_current_command()

    def _publish_current_command(self) -> None:
        msg = Twist()
        msg.linear.x = self._safe_float(self._current_cmd.vx)
        msg.linear.y = self._safe_float(self._current_cmd.vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._safe_float(self._current_cmd.wz)

        self._cmd_vel_pub.publish(msg)

    def publish_zero(self) -> None:
        self._current_cmd = TeleopCommand()
        self._publish_current_command()

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _safe_float(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return float(value)

    def _log_speed(self, prefix: str = "Speed updated") -> None:
        self.get_logger().info(
            f"{prefix} | linear={self._linear_speed:.2f}, "
            f"angular={self._angular_speed:.2f}"
        )

    def _print_help(self) -> None:
        now = time.monotonic()
        if now - self._last_print_time < 0.5:
            return
        self._last_print_time = now

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
  linear : {self._linear_speed:.2f}
  angular: {self._angular_speed:.2f}
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