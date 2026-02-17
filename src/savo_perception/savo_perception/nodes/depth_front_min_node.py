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

Reliability improvements:
  - Selectable QoS: sensor/best_effort/reliable
  - In "reliable" mode, matches RealSense durability: TRANSIENT_LOCAL
  - Robust numpy view using msg.step (handles row padding)
  - Staleness uses ROS header stamp when valid; otherwise wall-time fallback
  - Startup grace period to suppress stale warnings during RealSense bringup
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

        # ROI fractions (0..1) — tuned for ~33 cm mounting height
        self.declare_parameter("roi_x", 0.30)
        self.declare_parameter("roi_y", 0.45)
        self.declare_parameter("roi_w", 0.40)
        self.declare_parameter("roi_h", 0.40)

        self.declare_parameter("percentile", 10.0)
        self.declare_parameter("min_valid_m", 0.02)
        self.declare_parameter("max_valid_m", 3.00)

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("stale_timeout_s", 1.0)   # recommended for USB jitter
        self.declare_parameter("warn_every_s", 1.0)

        # Startup grace: suppress stale warnings briefly after node start (RealSense bringup)
        self.declare_parameter("startup_grace_s", 1.5)

        # For 16UC1: RealSense typically publishes millimeters
        self.declare_parameter("depth_scale_m_per_unit", 0.001)

        # QoS control:
        #   sensor      -> qos_profile_sensor_data
        #   best_effort -> explicit BEST_EFFORT
        #   reliable    -> RELIABLE + TRANSIENT_LOCAL (matches your RealSense publisher)
        self.declare_parameter("depth_qos", "reliable")  # "sensor"|"best_effort"|"reliable"

        # Debug RX logging
        self.declare_parameter("log_rx", False)
        self.declare_parameter("log_rx_every_s", 2.0)

        # ---------------- Read params ----------------
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
        self.startup_grace_s = float(self.get_parameter("startup_grace_s").value)

        self.depth_scale = float(self.get_parameter("depth_scale_m_per_unit").value)

        self.depth_qos = str(self.get_parameter("depth_qos").value).strip().lower()
        self.log_rx = bool(self.get_parameter("log_rx").value)
        self.log_rx_every_s = float(self.get_parameter("log_rx_every_s").value)

        self.publish_hz = max(1.0, min(self.publish_hz, 60.0))
        self.percentile = max(0.0, min(self.percentile, 100.0))
        self.log_rx_every_s = max(0.2, self.log_rx_every_s)
        self.stale_timeout_s = max(0.05, self.stale_timeout_s)
        self.startup_grace_s = max(0.0, self.startup_grace_s)

        # ---------------- QoS selection ----------------
        if self.depth_qos == "sensor":
            sub_qos = qos_profile_sensor_data

        elif self.depth_qos == "reliable":
            # IMPORTANT: match the RealSense publisher:
            # Reliability: RELIABLE
            # Durability: TRANSIENT_LOCAL
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

        else:
            # best_effort
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

        # Publisher QoS: RELIABLE is best for safety consumers
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
        self._latest_rx_wall_t: float = 0.0      # wall-time monotonic fallback
        self._latest_stamp_s: float = 0.0        # ROS stamp seconds (preferred when valid)

        # Warning/log timers
        self._last_warn_t: float = 0.0
        self._last_rx_log_t: float = 0.0

        # Node start time (for startup grace)
        self._start_t: float = time.monotonic()

        self.get_logger().info(
            f"DepthFrontMinNode started. sub={depth_topic} pub={publish_topic} "
            f"ROI=({self.roi_x:.2f},{self.roi_y:.2f},{self.roi_w:.2f},{self.roi_h:.2f}) "
            f"p={self.percentile:.1f} valid=[{self.min_valid_m:.2f},{self.max_valid_m:.2f}]m "
            f"stale={self.stale_timeout_s:.2f}s hz={self.publish_hz:.1f} "
            f"depth_qos={self.depth_qos} startup_grace={self.startup_grace_s:.2f}s"
        )

        self._timer = self.create_timer(1.0 / self.publish_hz, self._tick)

    # ---------------- ROS callbacks ----------------

    def _on_depth(self, msg: Image) -> None:
        self._latest_msg = msg
        self._latest_rx_wall_t = time.monotonic()

        # ROS stamp (may be 0 in some setups)
        self._latest_stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if self.log_rx:
            now = time.monotonic()
            if (now - self._last_rx_log_t) >= self.log_rx_every_s:
                self.get_logger().info(
                    f"RX depth frame: enc={msg.encoding} size={msg.width}x{msg.height} step={msg.step} "
                    f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
                )
                self._last_rx_log_t = now

    # ---------------- Core logic ----------------

    def _tick(self) -> None:
        if self._latest_msg is None:
            self.pub.publish(Float32(data=_nan()))
            self._warn_stale_if_not_in_startup_grace()
            return

        # Prefer ROS stamp freshness when valid; otherwise wall-time freshness
        age_s = self._compute_age_s()
        if age_s is None or age_s > self.stale_timeout_s:
            self.pub.publish(Float32(data=_nan()))
            self._warn_stale_if_not_in_startup_grace()
            return

        try:
            d_m = self._compute_front_min_m(self._latest_msg)
        except Exception as e:
            self.pub.publish(Float32(data=_nan()))
            # Compute errors are real issues; warn even during startup (rate-limited)
            self._warn_rate_limited(f"Depth compute error: {e}. Publishing NaN.")
            return

        self.pub.publish(Float32(data=_nan() if d_m is None else float(d_m)))

    def _warn_stale_if_not_in_startup_grace(self) -> None:
        if (time.monotonic() - self._start_t) >= self.startup_grace_s:
            self._warn_rate_limited("Depth image stale/no data. Publishing NaN.")

    def _compute_age_s(self) -> Optional[float]:
        """
        Returns frame age in seconds.
        Uses ROS stamp if it looks valid AND ROS clock is usable; otherwise falls back to wall time.
        """
        # If stamp looks valid, try ROS clock
        if self._latest_stamp_s > 0.0:
            now_ros_s = float(self.get_clock().now().nanoseconds) * 1e-9
            # Guard against weird clock (e.g., jump backwards)
            if now_ros_s > 0.0:
                age = now_ros_s - self._latest_stamp_s
                if age >= 0.0:
                    return age

        # Fallback: wall time since callback
        if self._latest_rx_wall_t > 0.0:
            return time.monotonic() - self._latest_rx_wall_t

        return None

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
        Returns float32 (H,W) depth in meters. Handles row padding via msg.step.
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
            return pix.astype(np.float32) * float(self.depth_scale)

        if enc in ("32fc1",):
            bytes_per_px = 4
            row_bytes_needed = w * bytes_per_px
            if step < row_bytes_needed:
                raise ValueError(f"Invalid step {step} for 32FC1 width {w}")

            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if buf.size < step * h:
                raise ValueError("Image data smaller than step*height")

            rows = buf[: step * h].reshape((h, step))
            return rows[:, :row_bytes_needed].view(np.float32).reshape((h, w))

        raise ValueError(
            f"Unsupported depth encoding: '{msg.encoding}' (expected 16UC1 or 32FC1)"
        )

    def _compute_front_min_m(self, msg: Image) -> Optional[float]:
        w = int(msg.width)
        h = int(msg.height)

        depth_m = self._as_depth_m(msg)

        x0, y0, x1, y1 = self._roi_pixels(w, h)
        roi = depth_m[y0:y1, x0:x1].reshape(-1)

        if roi.size == 0:
            return None

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_valid_m) & (valid <= self.max_valid_m)]

        # avoid noise spikes when too few pixels are valid
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
