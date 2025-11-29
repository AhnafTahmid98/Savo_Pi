#!/usr/bin/env python3
"""
Robot Savo — Real camera publisher for NAVIGATE UI

This node:

- Opens the Pi camera via V4L2 (/dev/videoX) using OpenCV.
- Grabs frames at a fixed FPS.
- Converts them to RGB (800x480 by default).
- Publishes sensor_msgs/Image on /camera/image_rect.

The display_manager_node already subscribes to /camera/image_rect and
nav_cam_view.py will show the live feed in NAVIGATE mode.

Requirements:
- python3-opencv installed in the ROS Python environment.
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


class RealCamNode(Node):
    """Simple V4L2 camera → ROS Image publisher for Robot Savo."""

    def __init__(self) -> None:
        super().__init__("savo_ui_real_cam")

        # ------------------------------------------------------------------
        # Parameters (you can override via ROS params)
        # ------------------------------------------------------------------
        self.declare_parameter("device_id", 0)          # /dev/video0
        self.declare_parameter("width", 800)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_topic", "/camera/image_rect")

        self.device_id: int = (
            self.get_parameter("device_id").get_parameter_value().integer_value
        )
        self.width: int = (
            self.get_parameter("width").get_parameter_value().integer_value
        )
        self.height: int = (
            self.get_parameter("height").get_parameter_value().integer_value
        )
        self.fps: float = (
            self.get_parameter("fps").get_parameter_value().double_value
        )
        self.frame_id: str = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.camera_topic: str = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )

        if self.fps <= 0.0:
            self.get_logger().warn(
                f"Invalid fps={self.fps}, clamping to 15.0"
            )
            self.fps = 15.0

        self.get_logger().info(
            f"RealCamNode starting: device=/dev/video{self.device_id}, "
            f"{self.width}x{self.height} @ {self.fps:.1f} FPS, "
            f"topic={self.camera_topic}"
        )

        # ------------------------------------------------------------------
        # Open camera via OpenCV
        # ------------------------------------------------------------------
        # CAP_V4L2 is usually the right backend on Linux/Pi
        self._cap: Optional[cv2.VideoCapture] = cv2.VideoCapture(
            self.device_id, cv2.CAP_V4L2
        )

        if not self._cap.isOpened():
            self.get_logger().error(
                f"Could not open camera /dev/video{self.device_id} "
                "via OpenCV. Check that the Pi camera is enabled and "
                "V4L2 compatibility is active."
            )
            self._cap = None
            return

        # Try to set resolution and fps (best-effort)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self._cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        # Publisher
        self._pub = self.create_publisher(Image, self.camera_topic, 10)

        # Timer for grabbing frames
        period = 1.0 / float(self.fps)
        self._timer = self.create_timer(period, self._on_timer)

        self._seq = 0
        self._drop_log_counter = 0

    # ----------------------------------------------------------------------
    # Timer callback: grab frame and publish
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._cap is None:
            # Camera failed at startup; nothing to do.
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            # Avoid spamming logs if the camera fails intermittently
            self._drop_log_counter += 1
            if self._drop_log_counter % int(self.fps * 2) == 0:
                self.get_logger().warn(
                    "Failed to read frame from camera (no data)."
                )
            return

        # frame is BGR (H, W, 3)
        h, w, _ = frame.shape

        # Resize if needed
        if (w != self.width) or (h != self.height):
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
            h, w, _ = frame.shape

        # Optional flips (uncomment if orientation is wrong physically)
        # frame = cv2.flip(frame, 0)  # vertical flip
        # frame = cv2.flip(frame, 1)  # horizontal flip

        # Convert BGR → RGB for the NAVIGATE UI
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Build sensor_msgs/Image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()  # type: ignore[assignment]
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = rgb.tobytes()

        self._pub.publish(msg)
        self._seq += 1

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------
    def destroy_node(self) -> bool:
        if self._cap is not None:
            try:
                self._cap.release()
                self.get_logger().info("Camera released.")
            except Exception as exc:
                self.get_logger().warn(f"Error releasing camera: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealCamNode()
    try:
        # If camera failed to open, we still spin (no timer),
        # so user can see errors in logs.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down RealCamNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
