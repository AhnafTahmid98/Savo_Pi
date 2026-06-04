#!/usr/bin/env python3
"""
Robot Savo — UI debug driver node (status text blank)

This version still cycles modes and mouth, but publishes an EMPTY
/savo_ui/status_text so nothing appears even with older displays.
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32


class UIDebugNode(Node):
    """Debug node that drives UI topics for visual testing."""

    def __init__(self) -> None:
        super().__init__("savo_ui_debug")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("robot_id", "robot_savo_pi")
        self.declare_parameter("cycle_interval_s", 10.0)
        self.declare_parameter("enable_fake_mouth", False)
        self.declare_parameter("mouth_wave_freq_hz", 1.2)
        self.declare_parameter("publish_face_state", True)

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
        self.publish_face_state: bool = (
            self.get_parameter("publish_face_state")
            .get_parameter_value()
            .bool_value
        )

        if self.cycle_interval_s <= 0.0:
            self.get_logger().warn(
                "cycle_interval_s must be > 0.0, using 10.0 instead."
            )
            self.cycle_interval_s = 10.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pub_mode = self.create_publisher(String, "/savo_ui/mode", 10)
        self.pub_status = self.create_publisher(String, "/savo_ui/status_text", 10)

        self.pub_mouth_open = self.create_publisher(
            Bool, "/savo_speech/mouth_open", 10
        )
        self.pub_mouth_level = self.create_publisher(
            Float32, "/savo_speech/mouth_level", 10
        )
        self.pub_face_state = self.create_publisher(
            String, "/savo_ui/face_state", 10
        )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._time_s: float = 0.0
        self._last_mode: str = ""
        self._last_status: str = ""

        self._dt = 0.05
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            "UIDebugNode started (status text BLANK version).\n"
            f"  robot_id           = {self.robot_id}\n"
            f"  cycle_interval_s   = {self.cycle_interval_s:.2f}\n"
            f"  enable_fake_mouth  = {self.enable_fake_mouth}\n"
            f"  mouth_wave_freq_hz = {self.mouth_wave_freq_hz:.2f}\n"
            f"  publish_face_state = {self.publish_face_state}"
        )

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        self._time_s += self._dt

        # Mode cycle
        t_mod = self._time_s % self.cycle_interval_s
        segment = self.cycle_interval_s / 3.0

        if t_mod < segment:
            mode = "INTERACT"
        elif t_mod < 2.0 * segment:
            mode = "NAVIGATE"
        else:
            mode = "MAP"

        # BLANK status string (was "[DEBUG] Mode = ...")
        status = ""

        if mode != self._last_mode:
            self.get_logger().info(f"[DEBUG] Switching UI mode to: {mode}")
            self._publish_mode(mode)
            self._last_mode = mode

        if status != self._last_status:
            self._publish_status(status)
            self._last_status = status

        # Fake mouth
        if self.enable_fake_mouth:
            phase = 2.0 * math.pi * self.mouth_wave_freq_hz * self._time_s
            level = 0.5 * (1.0 + math.sin(phase))
            level_clamped = max(0.0, min(1.0, level))
            open_flag = level_clamped > 0.25
            self._publish_fake_mouth(level_clamped, open_flag)

        # face_state
        if self.publish_face_state:
            face_state = self._compute_face_state()
            self._publish_face_state(face_state)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _publish_fake_mouth(self, level: float, open_flag: bool) -> None:
        msg_open = Bool()
        msg_open.data = open_flag
        self.pub_mouth_open.publish(msg_open)

        msg_level = Float32()
        msg_level.data = float(level)
        self.pub_mouth_level.publish(msg_level)

    def _compute_face_state(self) -> str:
        frac = (self._time_s % self.cycle_interval_s) / self.cycle_interval_s
        if frac < 0.25:
            return "idle"
        elif frac < 0.50:
            return "listening"
        elif frac < 0.75:
            return "thinking"
        else:
            return "speaking"

    def _publish_face_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.pub_face_state.publish(msg)


def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = UIDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down UIDebugNode.")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main(sys.argv)
