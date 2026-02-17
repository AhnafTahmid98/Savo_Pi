#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Safety Stop Node (ROS2, savo_perception)

Purpose
-------
Fuse near-field safety sensors and publish:
  - /safety/stop            (std_msgs/Bool)
  - /safety/slowdown_factor (std_msgs/Float32)  in [min_global_scale..1.0]

Inputs (locked topic contracts)
------------------------------
  - /depth/min_front_m                          (std_msgs/Float32)  front depth min
  - /savo_perception/range/front_ultrasonic_m   (std_msgs/Float32)  front ultrasonic
  - /savo_perception/range/left_m               (std_msgs/Float32)  left ToF
  - /savo_perception/range/right_m              (std_msgs/Float32)  right ToF

Behavior (default matches your locked range_safety.yaml)
-------------------------------------------------------
Front:
  - stop_m = 0.28, slow_m = 0.60, hysteresis = 0.06
  - weights: depth=1.0, ultrasonic=0.5
Sides:
  - stop_m = 0.20, slow_m = 0.45, hysteresis = 0.05
Stale:
  - fail_safe_on_stale = True, stale_timeout_s = 0.30

Debounce:
  - stop_debounce_count = 2
  - clear_debounce_count = 4

Slowdown:
  - global slowdown_factor is EMA filtered
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32


def _is_finite(x: Optional[float]) -> bool:
    return x is not None and isinstance(x, (int, float)) and math.isfinite(float(x))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _nan_to_none(x: float) -> Optional[float]:
    return float(x) if math.isfinite(x) else None


@dataclass
class SensorValue:
    v_m: Optional[float] = None
    t_mono: float = 0.0


class SafetyStopNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_stop_node")

        # ---------------- Parameters (aligned with locked config) ----------------
        self.declare_parameter("fail_safe_on_stale", True)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("min_valid_m", 0.02)
        self.declare_parameter("max_valid_m", 3.00)

        # Front thresholds + fusion weights
        self.declare_parameter("front_stop_m", 0.28)
        self.declare_parameter("front_slow_m", 0.60)
        self.declare_parameter("front_hysteresis_m", 0.06)
        self.declare_parameter("front_weight_depth", 1.0)
        self.declare_parameter("front_weight_ultrasonic", 0.5)

        # Side thresholds
        self.declare_parameter("side_stop_m", 0.20)
        self.declare_parameter("side_slow_m", 0.45)
        self.declare_parameter("side_hysteresis_m", 0.05)
        self.declare_parameter("global_stop_on_side", False)

        # Debounce
        self.declare_parameter("stop_debounce_count", 2)
        self.declare_parameter("clear_debounce_count", 4)

        # Slowdown (global factor)
        self.declare_parameter("publish_global_factor", True)
        self.declare_parameter("min_global_scale", 0.20)
        self.declare_parameter("ema_alpha", 0.35)  # higher = more responsive
        self.declare_parameter("publish_hz", 30.0)

        # Topics (defaults match your README contract)
        self.declare_parameter("topic_depth_front", "/depth/min_front_m")
        self.declare_parameter("topic_ultrasonic_front", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("topic_left", "/savo_perception/range/left_m")
        self.declare_parameter("topic_right", "/savo_perception/range/right_m")
        self.declare_parameter("topic_stop", "/safety/stop")
        self.declare_parameter("topic_slowdown", "/safety/slowdown_factor")

        # ---------------- Load params ----------------
        self.fail_safe_on_stale = bool(self.get_parameter("fail_safe_on_stale").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.min_valid_m = float(self.get_parameter("min_valid_m").value)
        self.max_valid_m = float(self.get_parameter("max_valid_m").value)

        self.front_stop_m = float(self.get_parameter("front_stop_m").value)
        self.front_slow_m = float(self.get_parameter("front_slow_m").value)
        self.front_hys_m = float(self.get_parameter("front_hysteresis_m").value)
        self.w_depth = float(self.get_parameter("front_weight_depth").value)
        self.w_ultra = float(self.get_parameter("front_weight_ultrasonic").value)

        self.side_stop_m = float(self.get_parameter("side_stop_m").value)
        self.side_slow_m = float(self.get_parameter("side_slow_m").value)
        self.side_hys_m = float(self.get_parameter("side_hysteresis_m").value)
        self.global_stop_on_side = bool(self.get_parameter("global_stop_on_side").value)

        self.stop_db = int(self.get_parameter("stop_debounce_count").value)
        self.clear_db = int(self.get_parameter("clear_debounce_count").value)
        self.stop_db = max(1, min(self.stop_db, 20))
        self.clear_db = max(1, min(self.clear_db, 50))

        self.publish_global_factor = bool(self.get_parameter("publish_global_factor").value)
        self.min_global_scale = float(self.get_parameter("min_global_scale").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.publish_hz = max(5.0, min(self.publish_hz, 60.0))

        self.topic_depth = str(self.get_parameter("topic_depth_front").value)
        self.topic_ultra = str(self.get_parameter("topic_ultrasonic_front").value)
        self.topic_left = str(self.get_parameter("topic_left").value)
        self.topic_right = str(self.get_parameter("topic_right").value)
        self.topic_stop = str(self.get_parameter("topic_stop").value)
        self.topic_slow = str(self.get_parameter("topic_slowdown").value)

        # ---------------- QoS ----------------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------------- Publishers ----------------
        self.pub_stop = self.create_publisher(Bool, self.topic_stop, 10)
        self.pub_slow = self.create_publisher(Float32, self.topic_slow, 10)

        # ---------------- Subscribers ----------------
        self.depth = SensorValue()
        self.ultra = SensorValue()
        self.left = SensorValue()
        self.right = SensorValue()

        self.create_subscription(Float32, self.topic_depth, self._cb_depth, qos_sensor)
        self.create_subscription(Float32, self.topic_ultra, self._cb_ultra, qos_sensor)
        self.create_subscription(Float32, self.topic_left, self._cb_left, qos_sensor)
        self.create_subscription(Float32, self.topic_right, self._cb_right, qos_sensor)

        # ---------------- State ----------------
        self._stop_latched: bool = False
        self._stop_hits: int = 0
        self._clear_hits: int = 0

        self._slow_ema: float = 1.0
        self._last_warn_t: float = 0.0

        self.get_logger().info(
            "SafetyStopNode started. "
            f"front(stop={self.front_stop_m:.2f},slow={self.front_slow_m:.2f},hys={self.front_hys_m:.2f}) "
            f"side(stop={self.side_stop_m:.2f},slow={self.side_slow_m:.2f},hys={self.side_hys_m:.2f}) "
            f"stale_timeout={self.stale_timeout_s:.2f}s fail_safe={self.fail_safe_on_stale}"
        )

        self._timer = self.create_timer(1.0 / self.publish_hz, self._tick)

    # ---------------- Callbacks ----------------

    def _cb_depth(self, msg: Float32) -> None:
        self.depth.v_m = _nan_to_none(float(msg.data))
        self.depth.t_mono = time.perf_counter()

    def _cb_ultra(self, msg: Float32) -> None:
        self.ultra.v_m = _nan_to_none(float(msg.data))
        self.ultra.t_mono = time.perf_counter()

    def _cb_left(self, msg: Float32) -> None:
        self.left.v_m = _nan_to_none(float(msg.data))
        self.left.t_mono = time.perf_counter()

    def _cb_right(self, msg: Float32) -> None:
        self.right.v_m = _nan_to_none(float(msg.data))
        self.right.t_mono = time.perf_counter()

    # ---------------- Helpers ----------------

    def _valid(self, v: Optional[float]) -> Optional[float]:
        if not _is_finite(v):
            return None
        v = float(v)
        if v < self.min_valid_m or v > self.max_valid_m:
            return None
        return v

    def _is_stale(self, s: SensorValue, now: float) -> bool:
        if s.t_mono <= 0.0:
            return True
        return (now - s.t_mono) > self.stale_timeout_s

    def _warn_rl(self, text: str, every_s: float = 1.0) -> None:
        now = time.perf_counter()
        if now - self._last_warn_t >= every_s:
            self.get_logger().warn(text)
            self._last_warn_t = now

    def _front_fused(self, now: float) -> Tuple[Optional[float], bool]:
        """
        Returns (front_distance_m, front_stale)
        front_distance_m is a conservative fused estimate (smaller = closer).
        """
        d = self._valid(self.depth.v_m)
        u = self._valid(self.ultra.v_m)

        depth_stale = self._is_stale(self.depth, now) or (d is None)
        ultra_stale = self._is_stale(self.ultra, now) or (u is None)

        # If both missing -> stale
        if depth_stale and ultra_stale:
            return None, True

        # Weighted conservative fusion:
        # - If both available: use min(d, u) but allow ultrasonic to be down-weighted.
        # - Practically: take min(d, u / weight) where smaller dominates; weight<1 makes ultrasonic "less strong".
        candidates = []

        if d is not None:
            candidates.append(d)  # depth weight is 1.0

        if u is not None:
            # If w_ultra is 0.5, then u / 0.5 = 2u => ultrasonic becomes less likely to dominate.
            wu = max(0.01, float(self.w_ultra))
            candidates.append(u / wu)

        if not candidates:
            return None, True

        return min(candidates), False

    def _side_min(self, now: float) -> Tuple[Optional[float], bool]:
        """
        Returns (side_min_m, side_stale) for left/right ToF.
        """
        l = self._valid(self.left.v_m)
        r = self._valid(self.right.v_m)

        l_stale = self._is_stale(self.left, now) or (l is None)
        r_stale = self._is_stale(self.right, now) or (r is None)

        if l_stale and r_stale:
            return None, True

        vals = [v for v in (l, r) if v is not None]
        if not vals:
            return None, True
        return min(vals), False

    def _slow_factor_from_band(self, d: Optional[float], slow_m: float, stop_m: float) -> float:
        """
        Map distance -> factor in [min_global_scale..1].
        - >= slow_m -> 1
        - <= stop_m -> min_global_scale (but stop signal is separate)
        - linear in between
        """
        if d is None:
            return self.min_global_scale if self.fail_safe_on_stale else 1.0

        if d >= slow_m:
            return 1.0
        if d <= stop_m:
            return self.min_global_scale

        # linear interpolation: stop_m -> min_scale, slow_m -> 1
        t = (d - stop_m) / max(1e-6, (slow_m - stop_m))
        return self.min_global_scale + t * (1.0 - self.min_global_scale)

    # ---------------- Main loop ----------------

    def _tick(self) -> None:
        now = time.perf_counter()

        front_d, front_stale = self._front_fused(now)
        side_d, side_stale = self._side_min(now)

        # Fail-safe on stale (if enabled)
        if self.fail_safe_on_stale and (front_stale or side_stale):
            self._warn_rl(
                f"Safety stale: front_stale={front_stale} side_stale={side_stale} -> FAIL-SAFE STOP"
            )
            desired_stop = True
        else:
            # STOP decisions with hysteresis
            front_stop_th = self.front_stop_m
            front_clear_th = self.front_stop_m + self.front_hys_m

            side_stop_th = self.side_stop_m
            side_clear_th = self.side_stop_m + self.side_hys_m

            front_stop = (front_d is not None) and (front_d <= front_stop_th)
            front_clear = (front_d is not None) and (front_d >= front_clear_th)

            side_stop = (side_d is not None) and (side_d <= side_stop_th)
            side_clear = (side_d is not None) and (side_d >= side_clear_th)

            if self._stop_latched:
                # remain stopped until clear condition met (front clears; and side clears if global_stop_on_side)
                if self.global_stop_on_side:
                    desired_stop = not (front_clear and side_clear)
                else:
                    desired_stop = not front_clear
            else:
                # enter stop if front stop OR (side stop and global_stop_on_side)
                if self.global_stop_on_side:
                    desired_stop = bool(front_stop or side_stop)
                else:
                    desired_stop = bool(front_stop)

        # Debounce transitions
        if desired_stop:
            self._stop_hits += 1
            self._clear_hits = 0
        else:
            self._clear_hits += 1
            self._stop_hits = 0

        if not self._stop_latched and self._stop_hits >= self.stop_db:
            self._stop_latched = True
            self._warn_rl("SAFETY STOP asserted.", every_s=0.1)
        if self._stop_latched and self._clear_hits >= self.clear_db:
            self._stop_latched = False
            self._warn_rl("SAFETY STOP cleared.", every_s=0.1)

        # Publish STOP
        self.pub_stop.publish(Bool(data=bool(self._stop_latched)))

        # Slowdown factor (front + side)
        if self.publish_global_factor:
            f_front = self._slow_factor_from_band(front_d, self.front_slow_m, self.front_stop_m)
            f_side = self._slow_factor_from_band(side_d, self.side_slow_m, self.side_stop_m)
            raw_factor = min(f_front, f_side)

            a = _clamp(self.ema_alpha, 0.01, 0.99)
            self._slow_ema = (1.0 - a) * self._slow_ema + a * raw_factor
            self._slow_ema = _clamp(self._slow_ema, self.min_global_scale, 1.0)

            self.pub_slow.publish(Float32(data=float(self._slow_ema)))


def main() -> None:
    rclpy.init()
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
