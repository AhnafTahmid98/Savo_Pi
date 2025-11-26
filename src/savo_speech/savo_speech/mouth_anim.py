#!/usr/bin/env python3
"""
Robot Savo — Mouth Animation Node

This node converts raw "mouth activity" values from the TTS layer into a
smooth, UI-friendly mouth open/close level for the on-robot display.

Design:
- Input:  std_msgs/Float32 on /savo_speech/mouth_activity
          (0.0 = fully closed, 1.0 = fully open; can be rough / spiky)
- Output: std_msgs/Float32 on /savo_ui/mouth_level
          (smoothed value 0.0–1.0 for UI animation)

The idea:
- TTS node (Piper) will emit simple activity levels (per frame/segment).
- This node smooths those into a natural-looking mouth animation so the
  UI doesn't flicker with every tiny change.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MouthAnimNode(Node):
    """ROS 2 node that smooths mouth activity into a UI mouth level."""

    def __init__(self) -> None:
        super().__init__("mouth_anim")

        # ------------------------------------------------------------------
        # 1. Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter("input_topic", "/savo_speech/mouth_activity")
        self.declare_parameter("output_topic", "/savo_ui/mouth_level")
        self.declare_parameter("update_rate_hz", 30.0)

        # Smoothing / animation behavior
        self.declare_parameter("attack_gain", 0.5)   # how fast it opens
        self.declare_parameter("decay_gain", 0.15)   # how fast it closes

        self.declare_parameter("min_level", 0.0)     # optional lower cutoff
        self.declare_parameter("max_level", 1.0)     # clamp upper bound

        # ------------------------------------------------------------------
        # 2. Read parameters
        # ------------------------------------------------------------------
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        update_rate_hz = (
            self.get_parameter("update_rate_hz").get_parameter_value().double_value
        )

        self.attack_gain = (
            self.get_parameter("attack_gain").get_parameter_value().double_value
        )
        self.decay_gain = (
            self.get_parameter("decay_gain").get_parameter_value().double_value
        )
        self.min_level = (
            self.get_parameter("min_level").get_parameter_value().double_value
        )
        self.max_level = (
            self.get_parameter("max_level").get_parameter_value().double_value
        )

        if update_rate_hz <= 0.0:
            self.get_logger().warn(
                f"update_rate_hz={update_rate_hz:.3f} invalid; clamping to 30.0"
            )
            update_rate_hz = 30.0

        self.dt = 1.0 / update_rate_hz

        # ------------------------------------------------------------------
        # 3. State variables
        # ------------------------------------------------------------------
        self._target_activity: float = 0.0  # last raw value from input topic
        self._current_level: float = 0.0    # smoothed mouth level 0–1

        # ------------------------------------------------------------------
        # 4. ROS interfaces
        # ------------------------------------------------------------------
        self.sub = self.create_subscription(
            Float32,
            input_topic,
            self._activity_cb,
            10,
        )
        self.pub = self.create_publisher(Float32, output_topic, 10)

        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            "MouthAnimNode started with:\n"
            f"  input_topic     = {input_topic}\n"
            f"  output_topic    = {output_topic}\n"
            f"  update_rate_hz  = {update_rate_hz:.1f}\n"
            f"  attack_gain     = {self.attack_gain:.3f}\n"
            f"  decay_gain      = {self.decay_gain:.3f}\n"
            f"  min_level       = {self.min_level:.3f}\n"
            f"  max_level       = {self.max_level:.3f}"
        )

    # ----------------------------------------------------------------------
    # Subscriber callback: update target activity
    # ----------------------------------------------------------------------
    def _activity_cb(self, msg: Float32) -> None:
        # Clamp incoming value to [0, 1]
        raw = float(msg.data)
        if raw < 0.0:
            raw = 0.0
        elif raw > 1.0:
            raw = 1.0

        self._target_activity = raw

    # ----------------------------------------------------------------------
    # Timer callback: smooth current level towards target_activity
    # ----------------------------------------------------------------------
    def _timer_cb(self) -> None:
        target = self._target_activity
        current = self._current_level

        delta = target - current

        if delta > 0.0:
            # Mouth opening → attack
            gain = self.attack_gain
        else:
            # Mouth closing → decay (often slower)
            gain = self.decay_gain

        # Basic exponential smoothing step
        current += delta * gain

        # Clamp to [0, max_level]
        if current < 0.0:
            current = 0.0
        if current > self.max_level:
            current = self.max_level

        # Optional minimum visible level (for tiny noise wiggles)
        if current < self.min_level:
            current = 0.0

        self._current_level = current

        out_msg = Float32()
        out_msg.data = float(current)
        self.pub.publish(out_msg)

        # Debug logs can be noisy; keep at debug level
        self.get_logger().debug(
            f"MouthAnim update: target={target:.3f}, level={current:.3f}, gain={gain:.3f}"
        )


def main(args: Optional[list[str]] = None) -> None:
    """Entry point for ROS 2."""
    rclpy.init(args=args)
    node = MouthAnimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MouthAnimNode")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
