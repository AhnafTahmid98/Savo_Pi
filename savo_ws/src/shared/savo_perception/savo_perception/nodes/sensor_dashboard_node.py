#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Simple perception sensor dashboard publisher."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, String

    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Bool = None
    Float32 = None
    String = None
    ROS_AVAILABLE = False

from savo_perception.constants import NODE_NAME_SENSOR_DASHBOARD
from savo_perception.ros.qos_profiles import (
    qos_depth_sensor,
    qos_range_sensor,
    qos_safety_bool,
    qos_slowdown_factor,
    qos_state_string,
)


class SensorDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME_SENSOR_DASHBOARD)

        self.declare_parameter("depth_front_topic", "/depth/min_front_m")
        self.declare_parameter("tof_left_topic", "/savo_perception/range/left_m")
        self.declare_parameter("tof_right_topic", "/savo_perception/range/right_m")
        self.declare_parameter("ultrasonic_front_topic", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_topic", "/safety/slowdown_factor")
        self.declare_parameter("range_health_topic", "/savo_perception/range_health")
        self.declare_parameter("safety_state_topic", "/savo_perception/safety_state")
        self.declare_parameter("dashboard_topic", "/savo_perception/dashboard")
        self.declare_parameter("dashboard_text_topic", "/savo_perception/dashboard_text")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.50)

        self.depth_front_topic = str(self.get_parameter("depth_front_topic").value)
        self.tof_left_topic = str(self.get_parameter("tof_left_topic").value)
        self.tof_right_topic = str(self.get_parameter("tof_right_topic").value)
        self.ultrasonic_front_topic = str(self.get_parameter("ultrasonic_front_topic").value)
        self.safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self.slowdown_topic = str(self.get_parameter("slowdown_topic").value)
        self.range_health_topic = str(self.get_parameter("range_health_topic").value)
        self.safety_state_topic = str(self.get_parameter("safety_state_topic").value)
        self.dashboard_topic = str(self.get_parameter("dashboard_topic").value)
        self.dashboard_text_topic = str(self.get_parameter("dashboard_text_topic").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)

        self.ranges: Dict[str, dict] = {
            "depth_front": self._empty_range("depth_front", optional=True),
            "tof_left": self._empty_range("tof_left"),
            "tof_right": self._empty_range("tof_right"),
            "ultrasonic_front": self._empty_range("ultrasonic_front"),
        }

        self.safety_stop = False
        self.safety_stop_stamp_s: Optional[float] = None
        self.slowdown_factor = 1.0
        self.slowdown_stamp_s: Optional[float] = None
        self.range_health = {}
        self.safety_state = {}

        self.create_subscription(
            Float32,
            self.depth_front_topic,
            lambda msg: self._on_range("depth_front", msg),
            qos_depth_sensor(),
        )
        self.create_subscription(
            Float32,
            self.tof_left_topic,
            lambda msg: self._on_range("tof_left", msg),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            self.tof_right_topic,
            lambda msg: self._on_range("tof_right", msg),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            self.ultrasonic_front_topic,
            lambda msg: self._on_range("ultrasonic_front", msg),
            qos_range_sensor(),
        )
        self.create_subscription(
            Bool,
            self.safety_stop_topic,
            self._on_safety_stop,
            qos_safety_bool(),
        )
        self.create_subscription(
            Float32,
            self.slowdown_topic,
            self._on_slowdown,
            qos_slowdown_factor(),
        )
        self.create_subscription(
            String,
            self.range_health_topic,
            self._on_range_health,
            qos_state_string(),
        )
        self.create_subscription(
            String,
            self.safety_state_topic,
            self._on_safety_state,
            qos_state_string(),
        )

        self.dashboard_pub = self.create_publisher(
            String,
            self.dashboard_topic,
            qos_state_string(),
        )
        self.dashboard_text_pub = self.create_publisher(
            String,
            self.dashboard_text_topic,
            qos_state_string(),
        )

        period_s = 1.0 / max(self.publish_hz, 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "Sensor dashboard node started: "
            f"publish={self.publish_hz:.2f}Hz, stale_timeout={self.stale_timeout_s:.2f}s"
        )

    def _on_range(self, name: str, msg) -> None:
        now_s = time.monotonic()
        value = float(getattr(msg, "data", math.nan))

        valid = math.isfinite(value) and value > 0.0

        self.ranges[name] = {
            "name": name,
            "distance_m": value if valid else None,
            "valid": valid,
            "stamp_mono_s": now_s,
            "age_s": 0.0,
            "stale": False,
        }

    def _on_safety_stop(self, msg) -> None:
        self.safety_stop = bool(getattr(msg, "data", False))
        self.safety_stop_stamp_s = time.monotonic()

    def _on_slowdown(self, msg) -> None:
        value = float(getattr(msg, "data", 1.0))
        if math.isfinite(value):
            self.slowdown_factor = max(0.0, min(1.0, value))
        self.slowdown_stamp_s = time.monotonic()

    def _on_range_health(self, msg) -> None:
        self.range_health = self._parse_json(getattr(msg, "data", ""))

    def _on_safety_state(self, msg) -> None:
        self.safety_state = self._parse_json(getattr(msg, "data", ""))

    def _on_timer(self) -> None:
        now_s = time.monotonic()

        ranges = {
            name: self._range_with_age(data, now_s)
            for name, data in self.ranges.items()
        }

        payload = {
            "stamp_mono_s": now_s,
            "ranges": ranges,
            "safety": {
                "stop": self.safety_stop,
                "slowdown_factor": self.slowdown_factor,
                "stop_age_s": self._age_or_none(self.safety_stop_stamp_s, now_s),
                "slowdown_age_s": self._age_or_none(self.slowdown_stamp_s, now_s),
            },
            "range_health": self.range_health,
            "safety_state": self.safety_state,
        }

        text = self._make_text(payload)

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.dashboard_pub.publish(msg)

        text_msg = String()
        text_msg.data = text
        self.dashboard_text_pub.publish(text_msg)

    def _range_with_age(self, data: dict, now_s: float) -> dict:
        out = dict(data)
        stamp = out.get("stamp_mono_s")

        if stamp is None:
            out["age_s"] = None
            out["stale"] = True
            return out

        age = max(0.0, now_s - float(stamp))
        out["age_s"] = age
        out["stale"] = age > self.stale_timeout_s
        return out

    def _make_text(self, payload: dict) -> str:
        ranges = payload["ranges"]
        safety = payload["safety"]

        def fmt(name: str) -> str:
            item = ranges[name]
            d = item.get("distance_m")
            stale = item.get("stale", True)

            if d is None:
                value = "---"
            else:
                value = f"{float(d):.3f}m"

            return f"{name}={value}{' STALE' if stale else ''}"

        return (
            f"{fmt('depth_front')} | "
            f"{fmt('tof_left')} | "
            f"{fmt('tof_right')} | "
            f"{fmt('ultrasonic_front')} | "
            f"stop={safety['stop']} | "
            f"slow={float(safety['slowdown_factor']):.2f}"
        )

    @staticmethod
    def _age_or_none(stamp_s: Optional[float], now_s: float) -> Optional[float]:
        if stamp_s is None:
            return None
        return max(0.0, now_s - float(stamp_s))

    @staticmethod
    def _parse_json(text: str) -> dict:
        try:
            data = json.loads(str(text))
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    @staticmethod
    def _empty_range(name: str, *, optional: bool = False) -> dict:
        return {
            "name": name,
            "distance_m": None,
            "valid": False,
            "stamp_mono_s": time.monotonic() if optional else None,
            "age_s": None,
            "stale": not optional,
        }


def main(args=None) -> int:
    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this node.")
        return 1

    rclpy.init(args=args)
    node = SensorDashboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())