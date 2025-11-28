#!/usr/bin/env python3
"""
Robot Savo — UI debug node

This node is a small developer tool for testing the savo_ui display without
requiring the full speech / navigation / mapping stack.

It can:
- Periodically cycle UI modes:
    INTERACT -> NAVIGATE -> MAP -> INTERACT -> ...
- Publish a matching /savo_ui/status_text for each mode.
- Optionally publish a fake /savo_speech/mouth_level (0.0–1.0) ramp so you
  can see mouth animation even if savo_speech is not running.

WARNING:
- If you enable fake mouth_level publishing while the real speech stack is
  running, both may write to /savo_speech/mouth_level. Only use this node
  alone, or disable fake_mouth in launch parameters.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32


class UIDebugNode(Node):
    """Developer/debug node to drive the UI without the full robot stack."""

    def __init__(self) -> None:
        super().__init__("savo_ui_debug")

        self.get_logger().info("Initializing UIDebugNode (UI test driver)")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("robot_id", "robot_savo_pi")
        self.declare_parameter("cycle_interval_s", 10.0)
        self.declare_parameter("enable_fake_mouth", False)
        self.declare_parameter("mouth_wave_freq_hz", 1.2)

        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.cycle_interval_s: float = (
            self.get_parameter("cycle_interval_s")
            .get_parameter_value()
            .double_value
        )
        self.enable_fake_mouth: bool = (
            self.get_parameter("enable_fake_mouth")
            .get_parameter_value()
            .bool_value
        )
        self.mouth_wave_freq_hz: float = (
            self.get_parameter("mouth_wave_freq_hz")
            .get_parameter_value()
            .double_value
        )

        if self.enable_fake_mouth:
            self.get_logger().warn(
                "enable_fake_mouth is TRUE - this node will publish to "
                "/savo_speech/mouth_level. Make sure the real speech stack "
                "is NOT running at the same time."
            )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pub_mode = self.create_publisher(String, "/savo_ui/mode", 10)
        self.pub_status = self.create_publisher(String, "/savo_ui/status_text", 10)
        self.pub_mouth: Optional[rclpy.publisher.Publisher] = None

        if self.enable_fake_mouth:
            self.pub_mouth = self.create_publisher(
                Float32, "/savo_speech/mouth_level", 10
            )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._modes = ["INTERACT", "NAVIGATE", "MAP"]
        self._mode_index = 0

        # Timekeeping for mode cycle and fake mouth wave
        self._t = 0.0
        self._dt = 0.05  # ~20 Hz update

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        # One timer drives both mode cycling and optional mouth animation.
        self._timer = self.create_timer(self._dt, self._on_timer)

        # Publish initial state
        self._publish_mode_and_status(initial=True)

    # ======================================================================
    # Timer
    # ======================================================================

    def _on_timer(self) -> None:
        """Main debug loop: update time, cycle modes, animate fake mouth."""

        self._t += self._dt

        # 1) Mode cycling
        if self.cycle_interval_s > 0.0:
            # Compute mode index from time
            phase = self._t / self.cycle_interval_s
            new_index = int(phase) % len(self._modes)
            if new_index != self._mode_index:
                self._mode_index = new_index
                self._publish_mode_and_status(initial=False)

        # 2) Fake mouth animation (optional)
        if self.enable_fake_mouth and self.pub_mouth is not None:
            # Simple sine wave in [0.0, 1.0]
            level = 0.5 * (1.0 + math.sin(2.0 * math.pi * self.mouth_wave_freq_hz * self._t))
            msg = Float32()
            msg.data = float(level)
            self.pub_mouth.publish(msg)

    # ======================================================================
    # Helpers
    # ======================================================================

    def _publish_mode_and_status(self, *, initial: bool) -> None:
        """Publish /savo_ui/mode and /savo_ui/status_text for current index."""
        mode = self._modes[self._mode_index]

        if mode == "INTERACT":
            status = "Hello, I am Robot Savo!"
        elif mode == "NAVIGATE":
            status = "Guiding to A201 (demo)"
        elif mode == "MAP":
            status = "Mapping in progress (demo)"
        else:
            status = "Unknown mode (debug)"

        if initial:
            self.get_logger().info(
                f"Initial debug UI mode: {mode}, status='{status}'"
            )
        else:
            self.get_logger().info(
                f"Switching debug UI mode -> {mode}, status='{status}'"
            )

        msg_mode = String()
        msg_mode.data = mode
        self.pub_mode.publish(msg_mode)

        msg_status = String()
        msg_status.data = status
        self.pub_status.publish(msg_status)


# ==========================================================================#
# main()
# ==========================================================================#

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UIDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down UIDebugNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
