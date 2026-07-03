#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback perception safety stop node."""

from __future__ import annotations

import math
import time
from typing import Optional

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

from savo_perception.constants import NODE_NAME_SAFETY_STOP
from savo_perception.models import RangeSample, RangeSnapshot
from savo_perception.ros.params import load_safety_stop_params
from savo_perception.ros.qos_profiles import (
    qos_depth_sensor,
    qos_range_sensor,
    qos_safety_bool,
    qos_slowdown_factor,
    qos_state_string,
)
from savo_perception.safety import (
    RangeFusionConfig,
    SafetyPolicy,
    SafetyPolicyConfig,
    state_publish_payload,
)


class SafetyStopNodePy(Node):
    def __init__(self) -> None:
        super().__init__(f"{NODE_NAME_SAFETY_STOP}_py")

        self.declare_parameter("depth_front_topic", "/depth/min_front_m")
        self.declare_parameter("tof_left_topic", "/savo_perception/range/left_m")
        self.declare_parameter("tof_right_topic", "/savo_perception/range/right_m")
        self.declare_parameter("ultrasonic_front_topic", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_topic", "/safety/slowdown_factor")
        self.declare_parameter("safety_state_topic", "/savo_perception/safety_state")

        self.declare_parameter("loop_hz", 20.0)
        self.declare_parameter("stale_timeout_s", 0.30)

        self.declare_parameter("front_stop_m", 0.25)
        self.declare_parameter("front_slow_m", 0.80)
        self.declare_parameter("side_stop_m", 0.08)
        self.declare_parameter("side_slow_m", 0.25)

        self.declare_parameter("front_clear_hysteresis_m", 0.010)
        self.declare_parameter("side_clear_hysteresis_m", 0.010)

        self.declare_parameter("stop_debounce_count", 2)
        self.declare_parameter("clear_debounce_count", 4)

        self.declare_parameter("slowdown_min", 0.20)
        self.declare_parameter("slowdown_max", 1.0)
        self.declare_parameter("slowdown_ema_alpha", 0.35)

        self.declare_parameter("fail_safe_on_stale", True)
        self.declare_parameter(
            "required_sensors",
            ["tof_left", "tof_right", "ultrasonic_front"],
        )

        values = {
            "depth_front_topic": self.get_parameter("depth_front_topic").value,
            "tof_left_topic": self.get_parameter("tof_left_topic").value,
            "tof_right_topic": self.get_parameter("tof_right_topic").value,
            "ultrasonic_front_topic": self.get_parameter("ultrasonic_front_topic").value,
            "safety_stop_topic": self.get_parameter("safety_stop_topic").value,
            "slowdown_topic": self.get_parameter("slowdown_topic").value,
            "safety_state_topic": self.get_parameter("safety_state_topic").value,
            "loop_hz": self.get_parameter("loop_hz").value,
            "stale_timeout_s": self.get_parameter("stale_timeout_s").value,
            "front_stop_m": self.get_parameter("front_stop_m").value,
            "front_slow_m": self.get_parameter("front_slow_m").value,
            "side_stop_m": self.get_parameter("side_stop_m").value,
            "side_slow_m": self.get_parameter("side_slow_m").value,
            "front_clear_hysteresis_m": self.get_parameter("front_clear_hysteresis_m").value,
            "side_clear_hysteresis_m": self.get_parameter("side_clear_hysteresis_m").value,
            "stop_debounce_count": self.get_parameter("stop_debounce_count").value,
            "clear_debounce_count": self.get_parameter("clear_debounce_count").value,
            "slowdown_min": self.get_parameter("slowdown_min").value,
            "slowdown_max": self.get_parameter("slowdown_max").value,
            "slowdown_ema_alpha": self.get_parameter("slowdown_ema_alpha").value,
            "fail_safe_on_stale": self.get_parameter("fail_safe_on_stale").value,
        }

        self.params = load_safety_stop_params(values)
        required = tuple(str(x) for x in self.get_parameter("required_sensors").value)

        fusion_cfg = RangeFusionConfig(
            front_stop_m=self.params.front_stop_m,
            front_slow_m=self.params.front_slow_m,
            side_stop_m=self.params.side_stop_m,
            side_slow_m=self.params.side_slow_m,
            stale_timeout_s=self.params.stale_timeout_s,
            fail_safe_on_stale=self.params.fail_safe_on_stale,
            required_sensors=required,
        )

        policy_cfg = SafetyPolicyConfig(
            fusion=fusion_cfg,
            stop_debounce_count=self.params.stop_debounce_count,
            clear_debounce_count=self.params.clear_debounce_count,
            front_clear_hysteresis_m=self.params.front_clear_hysteresis_m,
            side_clear_hysteresis_m=self.params.side_clear_hysteresis_m,
            slowdown_ema_alpha=self.params.slowdown_ema_alpha,
        )

        self.policy = SafetyPolicy(policy_cfg)

        self.depth_front = self._missing_sample("depth_front", required=False)
        self.tof_left = self._missing_sample("tof_left", required=True)
        self.tof_right = self._missing_sample("tof_right", required=True)
        self.ultrasonic_front = self._missing_sample("ultrasonic_front", required=True)

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
            lambda msg: self._on_range_msg("ultrasonic_front", msg, required=True),
            qos_range_sensor(),
        )

        self.stop_pub = self.create_publisher(
            Bool,
            self.params.safety_stop_topic,
            qos_safety_bool(),
        )
        self.slowdown_pub = self.create_publisher(
            Float32,
            self.params.slowdown_topic,
            qos_slowdown_factor(),
        )
        self.state_pub = self.create_publisher(
            String,
            self.params.safety_state_topic,
            qos_state_string(),
        )

        period_s = 1.0 / max(float(self.params.loop_hz), 1.0)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "Safety stop fallback node started: "
            f"loop={self.params.loop_hz:.2f}Hz, "
            f"front_stop={self.params.front_stop_m:.2f}m, "
            f"side_stop={self.params.side_stop_m:.2f}m, "
            f"fail_safe_on_stale={self.params.fail_safe_on_stale}"
        )

    def _on_range_msg(self, sensor_name: str, msg, *, required: bool) -> None:
        value = getattr(msg, "data", math.nan)
        sample = self._sample_from_value(sensor_name, value, required=required)

        if sensor_name == "depth_front":
            self.depth_front = sample
        elif sensor_name == "tof_left":
            self.tof_left = sample
        elif sensor_name == "tof_right":
            self.tof_right = sample
        elif sensor_name == "ultrasonic_front":
            self.ultrasonic_front = sample

    def _on_timer(self) -> None:
        snapshot = RangeSnapshot(
            depth_front=self.depth_front,
            tof_left=self.tof_left,
            tof_right=self.tof_right,
            ultrasonic_front=self.ultrasonic_front,
        )

        update = self.policy.update(snapshot)
        decision = update.published_decision

        stop_msg = Bool()
        stop_msg.data = bool(decision.stop_required)
        self.stop_pub.publish(stop_msg)

        slowdown_msg = Float32()
        slowdown_msg.data = float(decision.slowdown_factor)
        self.slowdown_pub.publish(slowdown_msg)

        state_msg = String()
        state_msg.data = update.state.to_json()
        self.state_pub.publish(state_msg)

    def _sample_from_value(self, sensor_name: str, value: float, *, required: bool) -> RangeSample:
        try:
            distance_m = float(value)
        except Exception:
            distance_m = math.nan

        if not math.isfinite(distance_m):
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
    node = SafetyStopNodePy()

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
