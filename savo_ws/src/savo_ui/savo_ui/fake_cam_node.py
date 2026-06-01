#!/usr/bin/env python3
"""
Robot Savo — Fake camera node for NAVIGATE UI testing

Publishes a synthetic animated RGB image on /camera/image_rect so that the
NAVIGATE mode of display_manager_node can be tested without a real camera.

- Topic: /camera/image_rect (sensor_msgs/msg/Image)
- Encoding: "rgb8"
- Content: smooth moving gradient that changes with time
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class FakeCamNode(Node):
    """Simple fake camera node producing an animated color gradient."""

    def __init__(self) -> None:
        super().__init__("fake_cam_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("width", 800)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "nav_cam")

        self.width: int = (
            self.get_parameter("width").get_parameter_value().integer_value
        )
        self.height: int = (
            self.get_parameter("height").get_parameter_value().integer_value
        )
        self.fps: float = (
            self.get_parameter("fps").get_parameter_value().double_value
        )
        if self.fps <= 0.0:
            self.get_logger().warn("fps <= 0, clamping to 15.0")
            self.fps = 15.0

        self.frame_id: str = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_image = self.create_publisher(Image, "/camera/image_rect", 10)

        # Precompute static coordinate grids for gradient (H x W)
        # Note: y varies along height (rows), x along width (cols)
        x = np.linspace(0.0, 1.0, self.width, dtype=np.float32)
        y = np.linspace(0.0, 1.0, self.height, dtype=np.float32)
        # yy, xx shapes are (H, W)
        self.xx, self.yy = np.meshgrid(x, y)

        self._t: float = 0.0  # simple time parameter

        # Timer for frame generation
        period = 1.0 / float(self.fps)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"FakeCamNode started: {self.width}x{self.height} @ {self.fps:.1f} FPS"
        )

    # ======================================================================
    # Timer callback
    # ======================================================================

    def _on_timer(self) -> None:
        """Generate and publish one synthetic frame."""
        # Small time step to animate gradient
        self._t += 0.03

        # ------------------------------------------------------------------
        # Build RGB channels as H x W arrays
        # ------------------------------------------------------------------
        # Red: horizontal gradient (left → right)
        r = (self.xx * 255.0).astype(np.uint8)  # H x W

        # Green: vertical gradient with slow wave in time
        g_base = self.yy
        g_wave = 0.25 * (1.0 + np.sin(self._t + 2.0 * math.pi * self.xx))
        g = np.clip((g_base * 0.6 + g_wave * 0.4) * 255.0, 0, 255).astype(np.uint8)

        # Blue: time-based wave, same H x W, using only y + time
        b_wave = 0.5 * (1.0 + np.sin(self._t * 1.5 + 2.0 * math.pi * self.yy))
        b = np.clip(b_wave * 255.0, 0, 255).astype(np.uint8)

        # Stack into a single (H, W, 3) array
        frame = np.dstack((r, g, b))  # shape: H x W x 3

        # ------------------------------------------------------------------
        # Create Image message
        # ------------------------------------------------------------------
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.width * 3  # 3 bytes per pixel (R,G,B)

        # Flatten to bytes
        msg.data = frame.tobytes()

        self.pub_image.publish(msg)


def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = FakeCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down FakeCamNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
