#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback HC-SR04 ultrasonic range node."""

from __future__ import annotations

import math
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32

    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Float32 = None
    ROS_AVAILABLE = False

from savo_perception.constants import NODE_NAME_ULTRASONIC
from savo_perception.drivers import UltrasonicConfig, UltrasonicDriver
from savo_perception.ros.params import load_ultrasonic_params
from savo_perception.ros.qos_profiles import qos_range_sensor


class UltrasonicNodePy(Node):
    def __init__(self) -> None:
        super().__init__(f"{NODE_NAME_ULTRASONIC}_py")

        self.declare_parameter("trig_pin", 27)
        self.declare_parameter("echo_pin", 22)
        self.declare_parameter("max_distance_m", 3.0)
        self.declare_parameter("valid_min_m", 0.02)
        self.declare_parameter("valid_max_m", 3.0)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("queue_len", 3)
        self.declare_parameter("pin_factory", "lgpio")
        self.declare_parameter("output_topic", "/savo_perception/range/front_ultrasonic_m")
        self.declare_parameter("publish_nan_on_error", True)

        values = {
            "trig_pin": self.get_parameter("trig_pin").value,
            "echo_pin": self.get_parameter("echo_pin").value,
            "max_distance_m": self.get_parameter("max_distance_m").value,
            "valid_min_m": self.get_parameter("valid_min_m").value,
            "valid_max_m": self.get_parameter("valid_max_m").value,
            "rate_hz": self.get_parameter("rate_hz").value,
            "queue_len": self.get_parameter("queue_len").value,
            "pin_factory": self.get_parameter("pin_factory").value,
            "output_topic": self.get_parameter("output_topic").value,
        }

        self.params = load_ultrasonic_params(values)
        self.publish_nan_on_error = bool(self.get_parameter("publish_nan_on_error").value)

        driver_cfg = UltrasonicConfig(
            trig_pin=self.params.trig_pin,
            echo_pin=self.params.echo_pin,
            max_distance_m=self.params.max_distance_m,
            valid_min_m=self.params.valid_min_m,
            valid_max_m=self.params.valid_max_m,
            queue_len=self.params.queue_len,
            pin_factory=self.params.pin_factory,
        )

        self.driver = UltrasonicDriver(driver_cfg)
        self.driver_error: Optional[str] = None

        self.pub = self.create_publisher(
            Float32,
            self.params.output_topic,
            qos_range_sensor(),
        )

        self._start_driver()

        period_s = 1.0 / max(float(self.params.rate_hz), 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "Ultrasonic fallback node started: "
            f"TRIG={self.params.trig_pin}, ECHO={self.params.echo_pin}, "
            f"max={self.params.max_distance_m:.2f}m, "
            f"rate={self.params.rate_hz:.2f}Hz"
        )

    def destroy_node(self) -> bool:
        self._stop_driver()
        return super().destroy_node()

    def _start_driver(self) -> None:
        try:
            self.driver.start()
            self.driver_error = None
        except Exception as exc:
            self.driver_error = str(exc)
            self.get_logger().error(f"Ultrasonic driver start failed: {exc}")

    def _stop_driver(self) -> None:
        try:
            self.driver.close()
        except Exception:
            pass

    def _on_timer(self) -> None:
        if not self.driver.started:
            if self.publish_nan_on_error:
                self._publish_distance(None)
            return

        try:
            sample = self.driver.read_sample()
        except Exception as exc:
            self.driver_error = str(exc)
            self.get_logger().warn(f"Ultrasonic read failed: {exc}")
            if self.publish_nan_on_error:
                self._publish_distance(None)
            return

        self._publish_distance(sample.distance_m)

    def _publish_distance(self, distance_m: Optional[float]) -> None:
        msg = Float32()
        msg.data = float(distance_m) if distance_m is not None else math.nan
        self.pub.publish(msg)


def main(args=None) -> int:
    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this node.")
        return 1

    rclpy.init(args=args)
    node = UltrasonicNodePy()

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
