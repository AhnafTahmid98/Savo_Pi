#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Ultrasonic ROS2 Node (savo_perception)

Publishes:
  - /savo_perception/range/front_ultrasonic_m   (std_msgs/Float32)

Hardware (locked project standard):
  - TRIG = GPIO27
  - ECHO = GPIO22

Design goals:
  - Stable publish loop (~15 Hz)
  - Invalid readings -> NaN
  - Stale detection -> rate-limited warning + NaN
  - GPIO logic lives in sensors_api (UltrasonicReader)
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

from savo_perception.sensors_api.ultrasonic_api import UltrasonicReader


def _as_float32(v: Optional[float]) -> float:
    return float(v) if v is not None else float("nan")


class UltrasonicNode(Node):
    def __init__(self) -> None:
        super().__init__("ultrasonic_node")

        # ---- Parameters ----
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("min_valid_m", 0.02)
        self.declare_parameter("max_valid_m", 3.00)
        self.declare_parameter("samples", 3)          # ultrasonic averaging
        self.declare_parameter("factory", "lgpio")    # "lgpio" | "pigpio"

        self.rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.stale_timeout_s: float = float(self.get_parameter("stale_timeout_s").value)
        self.min_valid_m: float = float(self.get_parameter("min_valid_m").value)
        self.max_valid_m: float = float(self.get_parameter("max_valid_m").value)
        self.samples: int = int(self.get_parameter("samples").value)
        self.factory: str = str(self.get_parameter("factory").value)

        self.rate_hz = max(2.0, min(self.rate_hz, 30.0))
        self.samples = max(1, min(self.samples, 10))

        # ---- QoS (sensor data) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(Float32, "/savo_perception/range/front_ultrasonic_m", qos)

        # ---- Persistent sensor object ----
        # (TRIG/ECHO are locked in the API defaults; we can be explicit anyway)
        self.reader = UltrasonicReader(trig_pin=27, echo_pin=22, factory=self.factory)

        self._last_ok_t = time.perf_counter()
        self._last_warn_t = 0.0

        self.get_logger().info(
            f"Ultrasonic node started: {self.rate_hz:.1f} Hz, samples={self.samples}, "
            f"stale_timeout={self.stale_timeout_s:.2f}s, "
            f"valid=[{self.min_valid_m:.2f},{self.max_valid_m:.2f}]m (TRIG=27, ECHO=22, factory={self.factory})"
        )

        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "reader") and self.reader is not None:
                self.reader.close()
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
        now = time.perf_counter()

        try:
            d_m = self.reader.read_m(samples=self.samples)  # meters or None
        except Exception as e:
            d_m = None
            if now - self._last_warn_t > 1.0:
                self.get_logger().warn(f"Ultrasonic read_m() error: {e}")
                self._last_warn_t = now

        d_m = self._clamp_valid(d_m)

        if d_m is not None:
            self._last_ok_t = now
        else:
            if (now - self._last_ok_t) > self.stale_timeout_s and (now - self._last_warn_t) > 1.0:
                self.get_logger().warn(
                    f"Ultrasonic stale/no-valid (> {self.stale_timeout_s:.2f}s). Publishing NaN."
                )
                self._last_warn_t = now

        self.pub.publish(Float32(data=_as_float32(d_m)))


def main() -> None:
    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
