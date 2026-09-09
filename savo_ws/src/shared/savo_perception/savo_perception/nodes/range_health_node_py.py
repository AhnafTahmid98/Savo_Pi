#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback range health node."""

from __future__ import annotations

import json
import math
import time
from collections import deque
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
        self.declare_parameter(
            "ultrasonic_front_topic",
            "/savo_perception/range/front_ultrasonic_m",
        )
        self.declare_parameter("range_health_topic", "/savo_perception/range_health")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("stale_timeout_s", 0.30)
        self.declare_parameter("rate_window_s", 2.0)
        self.declare_parameter("rate_min_samples", 5)
        self.declare_parameter("tof_minimum_rate_hz", 5.0)
        self.declare_parameter("tof_good_rate_hz", 8.0)
        self.declare_parameter("tof_excellent_rate_hz", 9.0)
        self.declare_parameter("depth_minimum_rate_hz", 5.0)
        self.declare_parameter("depth_good_rate_hz", 10.0)
        self.declare_parameter("depth_excellent_rate_hz", 12.0)
        self.declare_parameter("include_depth_in_overall_ok", False)
        self.declare_parameter("use_ultrasonic", True)
        self.declare_parameter("required_sensors", ["tof_left", "tof_right"])
        self.declare_parameter(
            "optional_sensors",
            ["depth_front", "ultrasonic_front"],
        )

        values = {
            "depth_front_topic": self.get_parameter("depth_front_topic").value,
            "tof_left_topic": self.get_parameter("tof_left_topic").value,
            "tof_right_topic": self.get_parameter("tof_right_topic").value,
            "ultrasonic_front_topic": self.get_parameter("ultrasonic_front_topic").value,
            "range_health_topic": self.get_parameter("range_health_topic").value,
            "publish_hz": self.get_parameter("publish_hz").value,
            "stale_timeout_s": self.get_parameter("stale_timeout_s").value,
            "use_ultrasonic": self.get_parameter("use_ultrasonic").value,
        }

        self.params = load_range_health_params(values)
        self.include_depth_in_overall_ok = bool(
            self.get_parameter("include_depth_in_overall_ok").value
        )
        self.required_sensors = [
            str(name)
            for name in self.get_parameter("required_sensors").value
            if self.params.use_ultrasonic or str(name) != "ultrasonic_front"
        ]
        self.optional_sensors = [
            str(name) for name in self.get_parameter("optional_sensors").value
        ]
        self.rate_window_s = max(
            0.1, float(self.get_parameter("rate_window_s").value)
        )
        self.rate_min_samples = max(
            2, int(self.get_parameter("rate_min_samples").value)
        )
        self.tof_rate_thresholds = (
            float(self.get_parameter("tof_minimum_rate_hz").value),
            float(self.get_parameter("tof_good_rate_hz").value),
            float(self.get_parameter("tof_excellent_rate_hz").value),
        )
        self.depth_rate_thresholds = (
            float(self.get_parameter("depth_minimum_rate_hz").value),
            float(self.get_parameter("depth_good_rate_hz").value),
            float(self.get_parameter("depth_excellent_rate_hz").value),
        )
        self.receipt_times = {
            name: deque()
            for name in (
                "depth_front",
                "tof_left",
                "tof_right",
                "ultrasonic_front",
            )
        }

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
        if self.params.use_ultrasonic:
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
        self._record_receipt(sensor_name)
        value = getattr(msg, "data", math.nan)
        self.samples[sensor_name] = self._sample_from_value(sensor_name, value, required=required)

    def _record_receipt(self, sensor_name: str) -> None:
        now_s = time.monotonic()
        times = self.receipt_times[sensor_name]
        times.append(now_s)
        while len(times) > 2 and now_s - times[0] > self.rate_window_s:
            times.popleft()

    def _rate_valid(self, sensor_name: str) -> bool:
        return len(self.receipt_times[sensor_name]) >= self.rate_min_samples

    def _receive_rate_hz(self, sensor_name: str) -> float:
        times = self.receipt_times[sensor_name]
        if len(times) < 2:
            return 0.0
        interval_s = times[-1] - times[0]
        return (len(times) - 1) / interval_s if interval_s > 0.0 else 0.0

    def _rate_quality(self, sensor_name: str) -> str:
        if sensor_name == "ultrasonic_front":
            return "NOT_APPLICABLE"
        if not self._rate_valid(sensor_name):
            return "ESTABLISHING"
        thresholds = (
            self.depth_rate_thresholds
            if sensor_name == "depth_front"
            else self.tof_rate_thresholds
        )
        rate_hz = self._receive_rate_hz(sensor_name)
        if rate_hz < thresholds[0]:
            return "BELOW_MINIMUM"
        if rate_hz < thresholds[1]:
            return "MINIMUM"
        if rate_hz < thresholds[2]:
            return "GOOD"
        return "EXCELLENT"

    def _on_timer(self) -> None:
        now_s = time.monotonic()

        enabled_samples = {
            name: sample
            for name, sample in self.samples.items()
            if name != "ultrasonic_front" or self.params.use_ultrasonic
        }
        health = {
            name: SensorHealth.from_sample(
                sample,
                stale_timeout_s=self.params.stale_timeout_s,
                now_mono_s=now_s,
            )
            for name, sample in enabled_samples.items()
        }

        required_sensors = list(self.required_sensors)
        if self.include_depth_in_overall_ok:
            required_sensors.append("depth_front")
        optional_sensors = [
            name for name in self.optional_sensors if name not in required_sensors
        ]

        required_health = [health[name] for name in required_sensors]
        low_rate_required = [
            item.sensor_name
            for item in required_health
            if self._rate_quality(item.sensor_name) == "BELOW_MINIMUM"
        ]
        ok = all(item.ok for item in required_health) and not low_rate_required

        stale_required = [item.sensor_name for item in required_health if item.stale]
        error_required = [
            item.sensor_name
            for item in required_health
            if (not item.ok and not item.stale)
            or item.sensor_name in low_rate_required
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
            "optional_sensors": optional_sensors,
            "disabled_sensors": (
                [] if self.params.use_ultrasonic else ["ultrasonic_front"]
            ),
            "stale_sensors": stale_required,
            "error_sensors": error_required,
            "sensors": {
                name: {
                    **item.to_dict(),
                    "receive_rate_hz": self._receive_rate_hz(name),
                    "rate_valid": self._rate_valid(name),
                    "rate_quality": self._rate_quality(name),
                }
                for name, item in health.items()
            },
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.pub.publish(msg)

    def _sample_from_value(self, sensor_name: str, value: float, *, required: bool) -> RangeSample:
        try:
            distance_m = float(value)
        except Exception:
            distance_m = math.nan

        if not math.isfinite(distance_m):
            return self._missing_sample(sensor_name, required=required, reason="non_finite")

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
        print(
            "ERROR: rclpy/std_msgs are not available. "
            "Source ROS 2 Jazzy before running this node."
        )
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
