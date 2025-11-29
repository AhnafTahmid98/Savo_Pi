#!/usr/bin/env python3
"""
Robot Savo — Fake camera node for UI testing

Publishes a simple moving gradient image on /camera/image_rect so that
the NAVIGATE view can be tested without a real camera.
"""

from __future__ import annotations

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class FakeCamNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_cam_node")

        self.declare_parameter("width",  800)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps",    15.0)

        self.width: int = self.get_parameter("width").get_parameter_value().integer_value
        self.height: int = self.get_parameter("height").get_parameter_value().integer_value
        self.fps: float = self.get_parameter("fps").get_parameter_value().double_value

        self.pub = self.create_publisher(Image, "/camera/image_rect", 10)

        period = 1.0 / max(1.0, float(self.fps))
        self._t = 0.0
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"FakeCamNode started: {self.width}x{self.height} @ {self.fps:.1f} FPS"
        )

    def _on_timer(self) -> None:
        self._t += 0.03

        # Simple moving gradient pattern so you can see it's live
        y = np.linspace(0, 1, self.height, dtype=np.float32).reshape(-1, 1)
        x = np.linspace(0, 1, self.width, dtype=np.float32).reshape(1, -1)

        r = (x * 255.0).astype(np.uint8)
        g = (y * 255.0).astype(np.uint8)
        b_val = int((0.5 + 0.5 * math.sin(self._t)) * 255.0)
        b = np.full_like(r, b_val, dtype=np.uint8)

        frame = np.stack([r, g, b], axis=2)  # (H, W, 3) RGB

        msg = Image()
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
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
