#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback range health node."""

from __future__ import annotations

import json
import math
import time
from typing import Dict

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, String

    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Float32 = None
    String = None
    ROS_AVAILABLE = False

from savo_perception.constants import NODE_NAME_RANGE_HEALTH, STATUS_ERROR, STATUS_OK, STATUS_STALE
from savo_perception.models import RangeSample, SensorHealth
from savo_perception.ros.params import load_range_health_params
from savo_perception.ros.qos_profiles import (
    qos_depth_sensor,
    qos_range_sensor,
    qos_state_string,
)


class RangeHealthNodePy(Node):
    def __init__(self) -> None:
        super().__init__(f"{NODE_NAME_RANGE_HEALTH}_py")

        self.declare_parameter("depth_front_topic", "/depth/min_front_m")
        self.declare_parameter("tof_left_topic", "/savo_perception/range/left_m")
        self.declare_parameter("tof_right_topic", "/savo_perception/range/right_m")
        self.declare_parameter("ultrasonic_front_topic", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("range_health_topic", "/savo_perception/range_health")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("include_depth_in_overall_ok", False)

        values = {
            "depth_front_topic": self.get_parameter("depth_front_topic").value,
            "tof_left_topic": self.get_parameter("tof_left_topic").value,
            "tof_right_topic": self.get_parameter("tof_right_topic").value,
            "ultrasonic_front_topic": self.get_parameter("ultrasonic_front_topic").value,
            "range_health_topic": self.get_parameter("range_health_topic").value,
            "publish_hz": self.get_parameter("publish_hz").value,
            "stale_timeout_s": self.get_parameter("stale_timeout_s").value,
        }

        self.params = load_range_health_params(values)
        self.include_depth_in_overall_ok = bool(
            self.get_parameter("include_depth_in_overall_ok").value
        )

        self.samples: Dict[str, RangeSample] = {
            "depth_front": self._missing_sample("depth_front", required=False),
            "tof_left": self._missing_sample("tof_left", required=True),
            "tof_right": self._missing_sample("tof_right", required=True),
            "ultrasonic_front": self._missing_sample("ultrasonic_front", required=False),
        }

        self.create_subscription(
            Float32,
            self.params.depth_front_topic,
            lambda msg: self._on_range_msg("depth_front", msg, required=False),
            qos_depth_sensor(),
        )
        self.create_subscription(
            Float32,
            self.params.tof_left_topic,
            lambda msg: self._on_range_msg("tof_left", msg, required=True),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            self.params.tof_right_topic,
            lambda msg: self._on_range_msg("tof_right", msg, required=True),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            self.params.ultrasonic_front_topic,
            lambda msg: self._on_range_msg("ultrasonic_front", msg, required=False),
            qos_range_sensor(),
        )

        self.pub = self.create_publisher(
            String,
            self.params.range_health_topic,
            qos_state_string(),
        )

        period_s = 1.0 / max(float(self.params.publish_hz), 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "Range health fallback node started: "
            f"publish={self.params.publish_hz:.2f}Hz, "
            f"stale_timeout={self.params.stale_timeout_s:.2f}s, "
            f"include_depth={self.include_depth_in_overall_ok}"
        )

    def _on_range_msg(self, sensor_name: str, msg, *, required: bool) -> None:
        value = getattr(msg, "data", math.nan)
        self.samples[sensor_name] = self._sample_from_value(sensor_name, value, required=required)

    def _on_timer(self) -> None:
        now_s = time.monotonic()

        health = {
            name: SensorHealth.from_sample(
                sample,
                stale_timeout_s=self.params.stale_timeout_s,
                now_mono_s=now_s,
            )
            for name, sample in self.samples.items()
        }

        required_sensors = ["tof_left", "tof_right"]
        if self.include_depth_in_overall_ok:
            required_sensors.append("depth_front")

        required_health = [health[name] for name in required_sensors]
        ok = all(item.ok for item in required_health)

        stale_required = [item.sensor_name for item in required_health if item.stale]
        error_required = [
            item.sensor_name
            for item in required_health
            if not item.ok and not item.stale
        ]

        if ok:
            status = STATUS_OK
        elif stale_required:
            status = STATUS_STALE
        else:
            status = STATUS_ERROR

        payload = {
            "ok": ok,
            "status": status,
            "stamp_mono_s": now_s,
            "stale_timeout_s": self.params.stale_timeout_s,
            "required_sensors": required_sensors,
            "optional_sensors": [] if self.include_depth_in_overall_ok else ["depth_front"],
            "stale_sensors": stale_required,
            "error_sensors": error_required,
            "sensors": {name: item.to_dict() for name, item in health.items()},
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.pub.publish(msg)

    def _sample_from_value(self, sensor_name: str, value: float, *, required: bool) -> RangeSample:
        try:
            distance_m = float(value)
        except Exception:
            distance_m = math.nan

        if math.isnan(distance_m):
            return self._missing_sample(sensor_name, required=required, reason="nan_or_invalid")

        if distance_m <= 0.0:
            return self._missing_sample(sensor_name, required=required, reason="non_positive")

        return RangeSample.now(
            sensor_name=sensor_name,
            distance_m=distance_m,
            source="ros_topic",
        )

    @staticmethod
    def _missing_sample(
        sensor_name: str,
        *,
        required: bool,
        reason: str = "missing",
    ) -> RangeSample:
        stamp = 0.0 if required else time.monotonic()
        return RangeSample(
            sensor_name=sensor_name,
            distance_m=None,
            stamp_mono_s=stamp,
            valid=False,
            source="ros_topic",
            error=reason,
        )


def main(args=None) -> int:
    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this node.")
        return 1

    rclpy.init(args=args)
    node = RangeHealthNodePy()

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
