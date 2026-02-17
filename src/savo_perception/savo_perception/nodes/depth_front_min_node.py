#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Depth Front Min Node (ROS2, D435 reliable)

Subscribes:
  - /camera/camera/depth/image_rect_raw   (sensor_msgs/Image)

Publishes:
  - /depth/min_front_m   (std_msgs/Float32)

What it does:
  - Takes a front ROI in the depth image
  - Computes a robust "near obstacle distance" using a percentile (default p10)
  - Filters invalid values (min/max range)
  - Handles stale images (publishes NaN + warning)

Why this approach:
  - Works reliably with RealSense depth via ROS2 (16UC1 mm or 32FC1 meters)
  - No cv_bridge required (pure numpy)
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def _nan() -> float:
    return float("nan")


class DepthFrontMinNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_front_min_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("publish_topic", "/depth/min_front_m")

        # ROI as fractions of image size (0..1). Default: centered, lower-middle band.
        # Adjust later if needed for your camera mounting height/tilt.
        self.declare_parameter("roi_x", 0.35)   # left fraction
        self.declare_parameter("roi_y", 0.35)   # top fraction
        self.declare_parameter("roi_w", 0.30)   # width fraction
        self.declare_parameter("roi_h", 0.35)   # height fraction

        self.declare_parameter("percentile", 10.0)     # robust near distance (p10)
        self.declare_parameter("min_valid_m", 0.02)    # clamp invalid
        self.declare_parameter("max_valid_m", 3.00)

        self.declare_parameter("publish_hz", 30.0)     # publish loop; node uses latest frame
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("warn_every_s", 1.0)

        # For 16UC1 depth: RealSense typically publishes in millimeters.
        self.declare_parameter("depth_scale_m_per_unit", 0.001)

        depth_topic = str(self.get_parameter("depth_topic").value)
        publish_topic = str(self.get_parameter("publish_topic").value)

        self.roi_x = float(self.get_parameter("roi_x").value)
        self.roi_y = float(self.get_parameter("roi_y").value)
        self.roi_w = float(self.get_parameter("roi_w").value)
        self.roi_h = float(self.get_parameter("roi_h").value)

        self.percentile = float(self.get_parameter("percentile").value)
        self.min_valid_m = float(self.get_parameter("min_valid_m").value)
        self.max_valid_m = float(self.get_parameter("max_valid_m").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.warn_every_s = float(self.get_parameter("warn_every_s").value)

        self.depth_scale = float(self.get_parameter("depth_scale_m_per_unit").value)

        self.publish_hz = max(1.0, min(self.publish_hz, 60.0))
        self.percentile = max(0.0, min(self.percentile, 100.0))

        # ---------------- QoS ----------------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(Float32, publish_topic, qos_sensor)
        self.sub = self.create_subscription(Image, depth_topic, self._on_depth, qos_sensor)

        # Latest frame cache (store only what we need)
        self._latest_msg: Optional[Image] = None
        self._latest_rx_t: float = 0.0

        self._last_warn_t: float = 0.0

        self.get_logger().info(
            f"DepthFrontMinNode started. sub={depth_topic} pub={publish_topic} "
            f"ROI=({self.roi_x:.2f},{self.roi_y:.2f},{self.roi_w:.2f},{self.roi_h:.2f}) "
            f"p={self.percentile:.1f} valid=[{self.min_valid_m:.2f},{self.max_valid_m:.2f}]m "
            f"stale={self.stale_timeout_s:.2f}s hz={self.publish_hz:.1f}"
        )

        self._timer = self.create_timer(1.0 / self.publish_hz, self._tick)

    # ---------------- ROS callbacks ----------------

    def _on_depth(self, msg: Image) -> None:
        self._latest_msg = msg
        self._latest_rx_t = time.perf_counter()

    # ---------------- Core logic ----------------

    def _tick(self) -> None:
        now = time.perf_counter()

        if self._latest_msg is None or (now - self._latest_rx_t) > self.stale_timeout_s:
            # stale / no data
            self.pub.publish(Float32(data=_nan()))
            self._warn_rate_limited("Depth image stale/no data. Publishing NaN.")
            return

        msg = self._latest_msg

        try:
            d_m = self._compute_front_min_m(msg)
        except Exception as e:
            self.pub.publish(Float32(data=_nan()))
            self._warn_rate_limited(f"Depth compute error: {e}. Publishing NaN.")
            return

        if d_m is None:
            self.pub.publish(Float32(data=_nan()))
        else:
            self.pub.publish(Float32(data=float(d_m)))

    def _warn_rate_limited(self, text: str) -> None:
        now = time.perf_counter()
        if (now - self._last_warn_t) >= self.warn_every_s:
            self.get_logger().warn(text)
            self._last_warn_t = now

    def _roi_pixels(self, w: int, h: int) -> Tuple[int, int, int, int]:
        # clamp fractions
        rx = min(max(self.roi_x, 0.0), 0.95)
        ry = min(max(self.roi_y, 0.0), 0.95)
        rw = min(max(self.roi_w, 0.01), 1.0 - rx)
        rh = min(max(self.roi_h, 0.01), 1.0 - ry)

        x0 = int(round(rx * w))
        y0 = int(round(ry * h))
        x1 = int(round((rx + rw) * w))
        y1 = int(round((ry + rh) * h))

        # enforce bounds and non-empty ROI
        x0 = max(0, min(x0, w - 1))
        y0 = max(0, min(y0, h - 1))
        x1 = max(x0 + 1, min(x1, w))
        y1 = max(y0 + 1, min(y1, h))
        return x0, y0, x1, y1

    def _compute_front_min_m(self, msg: Image) -> Optional[float]:
        """
        Return robust near obstacle distance in meters (percentile in ROI),
        or None if no valid pixels.
        """
        if msg.height <= 0 or msg.width <= 0:
            return None

        enc = (msg.encoding or "").lower()
        w = int(msg.width)
        h = int(msg.height)

        # Convert ROS Image -> numpy array (no cv_bridge)
        if enc in ("16uc1", "mono16"):
            # uint16 depth, typically millimeters for RealSense
            frame = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            depth_m = frame.astype(np.float32) * float(self.depth_scale)
        elif enc in ("32fc1",):
            frame = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
            depth_m = frame
        else:
            # Try to infer based on step/size
            # Most RealSense setups are 16UC1; be strict to avoid wrong parses.
            raise ValueError(f"Unsupported depth encoding: '{msg.encoding}' (expected 16UC1 or 32FC1)")

        x0, y0, x1, y1 = self._roi_pixels(w, h)
        roi = depth_m[y0:y1, x0:x1].reshape(-1)

        if roi.size == 0:
            return None

        # Valid range filtering
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_valid_m) & (valid <= self.max_valid_m)]

        if valid.size < 20:
            # too few pixels -> treat as invalid (prevents noise spikes)
            return None

        # robust near estimate
        d = float(np.percentile(valid, self.percentile))
        return d


def main() -> None:
    rclpy.init()
    node = DepthFrontMinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
