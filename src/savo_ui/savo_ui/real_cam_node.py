#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Real camera → ROS Image node for NAVIGATE UI
---------------------------------------------------------

Publishes camera frames as sensor_msgs/Image on /camera/image_rect so that
savo_ui.display_manager_node + nav_cam_view.py can show a live video panel.

Design:
- Primary path: GStreamer + libcamerasrc + appsink (recommended on Pi 5).
- Fallback: plain /dev/videoN via V4L2 if use_gstreamer=False.

Supported output:
- Encoding: 'bgr8'
- Topic  : navigation.camera_topic param (default: /camera/image_rect)

Parameters (ROS):
- camera.width        (int, default: 800)
- camera.height       (int, default: 480)
- camera.fps          (int, default: 15)
- camera.device       (string, default: "/dev/video0")   # used only in V4L2 mode
- camera.use_gstreamer(bool, default: True)
- camera.gst_pipeline (string, default: "" → auto-built pipeline)
- camera.topic        (string, default: "/camera/image_rect")
"""

from __future__ import annotations

import sys
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as RosImage


class RealCamNode(Node):
    """ROS 2 node that captures Pi camera frames and publishes them as bgr8."""

    def __init__(self) -> None:
        super().__init__("savo_ui_real_cam")

        # --------------------------------------------------------------
        # Declare & read parameters
        # --------------------------------------------------------------
        self.declare_parameter("camera.width", 800)
        self.declare_parameter("camera.height", 480)
        self.declare_parameter("camera.fps", 15)
        self.declare_parameter("camera.device", "/dev/video0")
        self.declare_parameter("camera.use_gstreamer", True)
        self.declare_parameter("camera.gst_pipeline", "")
        self.declare_parameter("camera.topic", "/camera/image_rect")

        self.width: int = (
            self.get_parameter("camera.width").get_parameter_value().integer_value
        )
        self.height: int = (
            self.get_parameter("camera.height").get_parameter_value().integer_value
        )
        self.fps: int = (
            self.get_parameter("camera.fps").get_parameter_value().integer_value
        )
        self.device: str = (
            self.get_parameter("camera.device").get_parameter_value().string_value
        )
        self.use_gstreamer: bool = (
            self.get_parameter("camera.use_gstreamer")
            .get_parameter_value()
            .bool_value
        )
        self.gst_pipeline_param: str = (
            self.get_parameter("camera.gst_pipeline")
            .get_parameter_value()
            .string_value
        )
        self.topic_name: str = (
            self.get_parameter("camera.topic").get_parameter_value().string_value
        )

        # --------------------------------------------------------------
        # Publisher
        # --------------------------------------------------------------
        self.pub = self.create_publisher(RosImage, self.topic_name, 10)

        # --------------------------------------------------------------
        # Open the camera (GStreamer first, V4L2 fallback)
        # --------------------------------------------------------------
        self._cap: Optional[cv2.VideoCapture] = None
        self._open_camera()

        if self._cap is None or not self._cap.isOpened():
            self.get_logger().error(
                "RealCamNode could not open camera stream. "
                "Check GStreamer support / libcamera / device node."
            )
        else:
            self.get_logger().info(
                f"RealCamNode starting: "
                f"{'GStreamer pipeline' if self.use_gstreamer else self.device}, "
                f"{self.width}x{self.height} @ {self.fps}.0 FPS, topic={self.topic_name}"
            )

        # Timer for capturing frames
        period = 1.0 / float(max(self.fps, 1))
        self._timer = self.create_timer(period, self._on_timer)

        # For rate-limited warnings
        self._failed_reads: int = 0

    # ------------------------------------------------------------------
    # Camera open helpers
    # ------------------------------------------------------------------
    def _build_default_gst_pipeline(self) -> str:
        """
        Build a sane default GStreamer pipeline using libcamerasrc.

        Output format: BGR (for OpenCV) via appsink.

        This matches the style we used in camera_stream.py:
            libcamerasrc !
              video/x-raw,format=I420,width=WxH,framerate=FPS/1 !
              videoconvert !
              video/x-raw,format=BGR !
              appsink drop=true max-buffers=1
        """
        return (
            "libcamerasrc ! "
            f"video/x-raw,format=I420,width={self.width},height={self.height},"
            f"framerate={self.fps}/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1"
        )

    def _open_camera(self) -> None:
        """Open the camera using GStreamer or V4L2."""
        # Try GStreamer first if requested
        if self.use_gstreamer:
            pipeline = self.gst_pipeline_param or self._build_default_gst_pipeline()
            self.get_logger().info(
                f"[RealCamNode] Opening camera via GStreamer pipeline:\n  {pipeline}"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

            if cap.isOpened():
                self._cap = cap
                return
            else:
                self.get_logger().error(
                    "[RealCamNode] Failed to open GStreamer pipeline. "
                    "Falling back to V4L2 (/dev/video*)."
                )

        # Fallback: V4L2 /dev/videoN
        try:
            # If device is like "/dev/video0" → extract index 0
            dev_str = self.device
            idx = None
            if dev_str.startswith("/dev/video"):
                try:
                    idx = int(dev_str.replace("/dev/video", "").strip())
                except ValueError:
                    idx = 0
            else:
                # If param is "0" or "1" etc.
                try:
                    idx = int(dev_str)
                except ValueError:
                    idx = 0

            self.get_logger().info(
                f"[RealCamNode] Opening camera via V4L2: /dev/video{idx}"
            )
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)

            if not cap.isOpened():
                self.get_logger().error(
                    "[RealCamNode] V4L2 open failed as well. No camera available."
                )
                self._cap = None
                return

            # Try to configure size and FPS (best-effort)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
            cap.set(cv2.CAP_PROP_FPS, float(self.fps))

            self._cap = cap
        except Exception as exc:
            self.get_logger().error(
                f"[RealCamNode] Exception while opening camera: {exc}"
            )
            self._cap = None

    # ------------------------------------------------------------------
    # Timer callback: grab frame + publish
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._cap is None or not self._cap.isOpened():
            # Avoid spamming the log every tick
            if self._failed_reads % 60 == 0:
                self.get_logger().warn(
                    "[RealCamNode] Camera is not opened. Still waiting..."
                )
            self._failed_reads += 1
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            if self._failed_reads % 60 == 0:
                self.get_logger().warn(
                    "[RealCamNode] Failed to read frame from camera (no data)."
                )
            self._failed_reads += 1
            return

        self._failed_reads = 0  # reset since we got a good frame

        # Expect frame as BGR uint8
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)

        h, w = frame.shape[:2]
        if h == 0 or w == 0:
            return

        # Build ROS Image message (bgr8)
        msg = RosImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"  # can be changed later if needed

        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)


def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = RealCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down RealCamNode.")
    finally:
        if node._cap is not None:
            try:
                node._cap.release()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
