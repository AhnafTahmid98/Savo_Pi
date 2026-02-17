#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — VL53L1X Dual ToF ROS2 Node (savo_perception)

Publishes:
  - /savo_perception/range/left_m   (std_msgs/Float32)
  - /savo_perception/range/right_m  (std_msgs/Float32)

Source:
  - Dual VL53L1X sensors at 0x29
  - bus0 = LEFT
  - bus1 = RIGHT

Design goals (professional):
  - Robust against invalid readings (publishes NaN)
  - Stale detection (logs warnings, publishes NaN when stale)
  - Lightweight + stable publish loop
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

# Import your non-ROS dual reader copied into the ROS package:
#   tools/diag/sensors/api/vl53_dual_api.py  ->  savo_perception/sensors_api/vl53_api.py
from savo_perception.sensors_api.vl53_api import DualVL53  # type: ignore


def _as_float32(v: Optional[float]) -> float:
    """Float32 cannot be None; use NaN for invalid/unavailable."""
    return float(v) if v is not None else float("nan")


class VL53Node(Node):
    def __init__(self) -> None:
        super().__init__("vl53_node")

        # ---- Parameters (match project conventions) ----
        self.declare_parameter("rate_hz", 25.0)
        self.declare_parameter("median_n", 5)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("min_valid_m", 0.02)
        self.declare_parameter("max_valid_m", 3.00)
        self.declare_parameter("publish_raw", False)  # optional for debugging

        self.rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.median_n: int = int(self.get_parameter("median_n").value)
        self.stale_timeout_s: float = float(self.get_parameter("stale_timeout_s").value)
        self.min_valid_m: float = float(self.get_parameter("min_valid_m").value)
        self.max_valid_m: float = float(self.get_parameter("max_valid_m").value)
        self.publish_raw: bool = bool(self.get_parameter("publish_raw").value)

        self.rate_hz = max(1.0, min(self.rate_hz, 60.0))

        # ---- QoS: sensor data (best-effort, keep last) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_left = self.create_publisher(Float32, "/savo_perception/range/left_m", qos)
        self.pub_right = self.create_publisher(Float32, "/savo_perception/range/right_m", qos)

        # Optional debug topics (raw)
        self.pub_left_raw = None
        self.pub_right_raw = None
        if self.publish_raw:
            self.pub_left_raw = self.create_publisher(Float32, "/savo_perception/range/left_raw_m", qos)
            self.pub_right_raw = self.create_publisher(Float32, "/savo_perception/range/right_raw_m", qos)

        # ---- Start Dual ToF reader ----
        self.get_logger().info(
            f"Starting DualVL53: rate={self.rate_hz:.1f}Hz median={self.median_n} stale={self.stale_timeout_s:.2f}s "
            f"valid=[{self.min_valid_m:.2f},{self.max_valid_m:.2f}]m (bus0=LEFT, bus1=RIGHT @0x29)"
        )
        self.tof = DualVL53(
            rate_hz=self.rate_hz,
            median_n=self.median_n,
            stale_timeout_s=self.stale_timeout_s,
            start_immediately=True,
        )

        self._last_warn_t = 0.0
        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "tof") and self.tof is not None:
                self.tof.stop()
        except Exception:
            pass
        return super().destroy_node()

    def _clamp_valid(self, v: Optional[float]) -> Optional[float]:
        if v is None:
            return None
        if v < self.min_valid_m or v > self.max_valid_m:
            return None
        return v

    def _tick(self) -> None:
        """
        Read latest ToF values and publish filtered distance.
        Uses NaN for invalid or stale readings.
        """
        try:
            st = self.tof.get_latest()  # expected DualVL53State with .left/.right samples
        except Exception as e:
            # If something goes wrong, publish NaNs and warn (rate-limited)
            now = time.perf_counter()
            if now - self._last_warn_t > 1.0:
                self.get_logger().warn(f"DualVL53 read error: {e}")
                self._last_warn_t = now
            self.pub_left.publish(Float32(data=float("nan")))
            self.pub_right.publish(Float32(data=float("nan")))
            return

        # Filtered values (preferred for safety)
        left_f = self._clamp_valid(getattr(st.left, "filt_m", None))
        right_f = self._clamp_valid(getattr(st.right, "filt_m", None))

        # Raw (optional)
        left_r = self._clamp_valid(getattr(st.left, "raw_m", None))
        right_r = self._clamp_valid(getattr(st.right, "raw_m", None))

        # Stale handling: if stale, publish NaN and warn (rate-limited)
        now = time.perf_counter()
        left_stale = bool(getattr(st, "left_stale", False))
        right_stale = bool(getattr(st, "right_stale", False))

        if left_stale:
            left_f = None
        if right_stale:
            right_f = None

        if (left_stale or right_stale) and (now - self._last_warn_t > 1.0):
            self.get_logger().warn(
                f"ToF stale: left_stale={left_stale} right_stale={right_stale} "
                f"(stale_timeout_s={self.stale_timeout_s:.2f})"
            )
            self._last_warn_t = now

        # Publish filtered
        self.pub_left.publish(Float32(data=_as_float32(left_f)))
        self.pub_right.publish(Float32(data=_as_float32(right_f)))

        # Publish raw (debug)
        if self.publish_raw and self.pub_left_raw and self.pub_right_raw:
            self.pub_left_raw.publish(Float32(data=_as_float32(left_r)))
            self.pub_right_raw.publish(Float32(data=_as_float32(right_r)))


def main() -> None:
    rclpy.init()
    node = VL53Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
