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

Reliability improvements in this version:
  - Uses qos_profile_sensor_data by default for camera topic compatibility
  - Optional parameter to force RELIABLE subscription if needed
  - Robust numpy view using msg.step (handles row padding)
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
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

        # ROI as fractions of image size (0..1). Default tuned for your mount (~33 cm)
        self.declare_parameter("roi_x", 0.30)   # left fraction
        self.declare_parameter("roi_y", 0.45)   # top fraction
        self.declare_parameter("roi_w", 0.40)   # width fraction
        self.declare_parameter("roi_h", 0.40)   # height fraction

        self.declare_parameter("percentile", 10.0)
        self.declare_parameter("min_valid_m", 0.02)
        self.declare_parameter("max_valid_m", 3.00)

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("warn_every_s", 1.0)

        # For 16UC1 depth: RealSense typically publishes in millimeters.
        self.declare_parameter("depth_scale_m_per_unit", 0.001)

        # QoS control:
        #   sensor      -> qos_profile_sensor_data (recommended for camera topics)
        #   best_effort -> explicit BEST_EFFORT
        #   reliable    -> explicit RELIABLE (use if your DDS setup requires it)
        self.declare_parameter("depth_qos", "sensor")  # "sensor"|"best_effort"|"reliable"

        # Debug: log when frames are received (rate-limited)
        self.declare_parameter("log_rx", False)
        self.declare_parameter("log_rx_every_s", 2.0)

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

        self.depth_qos = str(self.get_parameter("depth_qos").value).strip().lower()
        self.log_rx = bool(self.get_parameter("log_rx").value)
        self.log_rx_every_s = float(self.get_parameter("log_rx_every_s").value)

        self.publish_hz = max(1.0, min(self.publish_hz, 60.0))
        self.percentile = max(0.0, min(self.percentile, 100.0))
        self.log_rx_every_s = max(0.2, self.log_rx_every_s)

        # ---------------- QoS selection ----------------
        if self.depth_qos == "sensor":
            sub_qos = qos_profile_sensor_data
        elif self.depth_qos == "reliable":
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        else:
            # best_effort (default fallback)
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

        # Publisher QoS: keep it simple/reliable for downstream safety nodes
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Float32, publish_topic, pub_qos)
        self.sub = self.create_subscription(Image, depth_topic, self._on_depth, sub_qos)

        # Latest frame cache
        self._latest_msg: Optional[Image] = None
        self._latest_rx_t: float = 0.0

        self._last_warn_t: float = 0.0
        self._last_rx_log_t: float = 0.0

        self.get_logger().info(
            f"DepthFrontMinNode started. sub={depth_topic} pub={publish_topic} "
            f"ROI=({self.roi_x:.2f},{self.roi_y:.2f},{self.roi_w:.2f},{self.roi_h:.2f}) "
            f"p={self.percentile:.1f} valid=[{self.min_valid_m:.2f},{self.max_valid_m:.2f}]m "
            f"stale={self.stale_timeout_s:.2f}s hz={self.publish_hz:.1f} "
            f"depth_qos={self.depth_qos}"
        )

        self._timer = self.create_timer(1.0 / self.publish_hz, self._tick)

    # ---------------- ROS callbacks ----------------

    def _on_depth(self, msg: Image) -> None:
        self._latest_msg = msg
        self._latest_rx_t = time.monotonic()

        if self.log_rx:
            now = time.monotonic()
            if (now - self._last_rx_log_t) >= self.log_rx_every_s:
                self.get_logger().info(
                    f"RX depth frame: enc={msg.encoding} size={msg.width}x{msg.height} step={msg.step}"
                )
                self._last_rx_log_t = now

    # ---------------- Core logic ----------------

    def _tick(self) -> None:
        now = time.monotonic()

        if self._latest_msg is None or (now - self._latest_rx_t) > self.stale_timeout_s:
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
        now = time.monotonic()
        if (now - self._last_warn_t) >= self.warn_every_s:
            self.get_logger().warn(text)
            self._last_warn_t = now

    def _roi_pixels(self, w: int, h: int) -> Tuple[int, int, int, int]:
        rx = min(max(self.roi_x, 0.0), 0.95)
        ry = min(max(self.roi_y, 0.0), 0.95)
        rw = min(max(self.roi_w, 0.01), 1.0 - rx)
        rh = min(max(self.roi_h, 0.01), 1.0 - ry)

        x0 = int(round(rx * w))
        y0 = int(round(ry * h))
        x1 = int(round((rx + rw) * w))
        y1 = int(round((ry + rh) * h))

        x0 = max(0, min(x0, w - 1))
        y0 = max(0, min(y0, h - 1))
        x1 = max(x0 + 1, min(x1, w))
        y1 = max(y0 + 1, min(y1, h))
        return x0, y0, x1, y1

    # -------- Robust ROS Image -> numpy (handles msg.step padding) --------

    def _as_depth_m(self, msg: Image) -> np.ndarray:
        """
        Returns a float32 (H,W) depth image in meters.
        Handles row padding using msg.step.
        """
        h = int(msg.height)
        w = int(msg.width)
        enc = (msg.encoding or "").lower()
        step = int(msg.step)

        if h <= 0 or w <= 0:
            raise ValueError("Invalid image dimensions")

        if enc in ("16uc1", "mono16"):
            bytes_per_px = 2
            row_bytes_needed = w * bytes_per_px
            if step < row_bytes_needed:
                raise ValueError(f"Invalid step {step} for 16UC1 width {w}")

            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if buf.size < step * h:
                raise ValueError("Image data smaller than step*height")

            rows = buf[: step * h].reshape((h, step))
            pix = rows[:, :row_bytes_needed].view(np.uint16).reshape((h, w))
            depth_m = pix.astype(np.float32) * float(self.depth_scale)
            return depth_m

        if enc in ("32fc1",):
            bytes_per_px = 4
            row_bytes_needed = w * bytes_per_px
            if step < row_bytes_needed:
                raise ValueError(f"Invalid step {step} for 32FC1 width {w}")

            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if buf.size < step * h:
                raise ValueError("Image data smaller than step*height")

            rows = buf[: step * h].reshape((h, step))
            pix = rows[:, :row_bytes_needed].view(np.float32).reshape((h, w))
            return pix

        raise ValueError(f"Unsupported depth encoding: '{msg.encoding}' (expected 16UC1 or 32FC1)")

    def _compute_front_min_m(self, msg: Image) -> Optional[float]:
        if msg.height <= 0 or msg.width <= 0:
            return None

        w = int(msg.width)
        h = int(msg.height)

        depth_m = self._as_depth_m(msg)

        x0, y0, x1, y1 = self._roi_pixels(w, h)
        roi = depth_m[y0:y1, x0:x1].reshape(-1)

        if roi.size == 0:
            return None

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_valid_m) & (valid <= self.max_valid_m)]

        if valid.size < 20:
            return None

        return float(np.percentile(valid, self.percentile))


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
