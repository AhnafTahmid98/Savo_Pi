#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Real camera → ROS Image node for NAVIGATE UI
---------------------------------------------------------

Publishes camera frames as sensor_msgs/Image on /camera/image_rect so that
savo_ui.display_manager_node + nav_cam_view.py can show a live video panel.

Design:
- Primary path: GStreamer + libcamerasrc + appsink (recommended on Pi 5).
- Fallback: plain /dev/videoN via V4L2 if use_gstreamer=False or pipeline fails.

Supported output:
- Encoding: 'bgr8'
- Topic  : camera.topic param (default: /camera/image_rect)

Parameters (ROS):
- camera.width          (int,   default: 800)
- camera.height         (int,   default: 480)
- camera.fps            (int,   default: 15)
- camera.device         (str,   default: "/dev/video0")     # used only in V4L2 mode
- camera.use_gstreamer  (bool,  default: True)
- camera.gst_pipeline   (str,   default: "" → auto-built pipeline)
- camera.topic          (str,   default: "/camera/image_rect")
- camera.frame_id       (str,   default: "camera_link")
- debug.log_fps         (bool,  default: False)  # simple FPS logging to ROS every few seconds
"""

from __future__ import annotations

import sys
import time
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
        self.declare_parameter("camera.frame_id", "camera_link")

        self.declare_parameter("debug.log_fps", False)

        self.width: int = int(self.get_parameter("camera.width").value)
        self.height: int = int(self.get_parameter("camera.height").value)
        self.fps: int = max(1, int(self.get_parameter("camera.fps").value))
        self.device: str = str(self.get_parameter("camera.device").value)
        self.use_gstreamer: bool = bool(
            self.get_parameter("camera.use_gstreamer").value
        )
        self.gst_pipeline_param: str = str(
            self.get_parameter("camera.gst_pipeline").value
        )
        self.topic_name: str = str(self.get_parameter("camera.topic").value)
        self.frame_id: str = str(self.get_parameter("camera.frame_id").value)

        self.log_fps: bool = bool(self.get_parameter("debug.log_fps").value)

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
                "RealCamNode starting: %s, %dx%d @ %d FPS, topic=%s, frame_id=%s"
                % (
                    "GStreamer pipeline" if self.use_gstreamer else self.device,
                    self.width,
                    self.height,
                    self.fps,
                    self.topic_name,
                    self.frame_id,
                )
            )

        # Timer for capturing frames
        period = 1.0 / float(self.fps)
        self._timer = self.create_timer(period, self._on_timer)

        # For rate-limited warnings + optional FPS logging
        self._failed_reads: int = 0
        self._frames_published: int = 0
        self._last_fps_log_time: float = time.time()

    # ------------------------------------------------------------------
    # Camera open helpers
    # ------------------------------------------------------------------
    def _build_default_gst_pipeline(self) -> str:
        """
        Build a sane default GStreamer pipeline using libcamerasrc.

        Output format: BGR (for OpenCV) via appsink.

        This matches the style we use elsewhere:
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
                "[RealCamNode] Opening camera via GStreamer pipeline:\n"
                f"  {pipeline}"
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
            dev_str = self.device
            idx: int

            if dev_str.startswith("/dev/video"):
                # If device is like "/dev/video0" → extract index 0
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
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[RealCamNode] Exception while opening camera: {exc}"
            )
            self._cap = None

    # ------------------------------------------------------------------
    # Timer callback: grab frame + publish
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._cap is None or not self._cap.isOpened():
            # Avoid spamming the log every tick (15–30 Hz)
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
        msg.header.frame_id = self.frame_id

        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)

        # Optional FPS logging
        if self.log_fps:
            self._frames_published += 1
            now = time.time()
            if now - self._last_fps_log_time >= 5.0:  # every ~5 seconds
                dt = now - self._last_fps_log_time
                fps_eff = self._frames_published / dt if dt > 0 else 0.0
                self.get_logger().info(
                    "[RealCamNode] Published %d frames in %.1f s (%.1f FPS)"
                    % (self._frames_published, dt, fps_eff)
                )
                self._frames_published = 0
                self._last_fps_log_time = now


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
