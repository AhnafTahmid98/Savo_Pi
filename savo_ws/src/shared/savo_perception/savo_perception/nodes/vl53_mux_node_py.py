#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback VL53L1X mux range node."""

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

from savo_perception.constants import NODE_NAME_VL53_MUX
from savo_perception.drivers import Vl53MuxConfig, Vl53MuxDriver
from savo_perception.ros.params import load_vl53_mux_params
from savo_perception.ros.qos_profiles import qos_range_sensor


class Vl53MuxNodePy(Node):
    def __init__(self) -> None:
        super().__init__(f"{NODE_NAME_VL53_MUX}_py")

        self.declare_parameter("bus", 1)
        self.declare_parameter("tca_addr", 0x70)
        self.declare_parameter("vl53_addr", 0x29)
        self.declare_parameter("right_channel", 3)
        self.declare_parameter("left_channel", 2)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("median_window", 5)
        self.declare_parameter("left_topic", "/savo_perception/range/left_m")
        self.declare_parameter("right_topic", "/savo_perception/range/right_m")
        self.declare_parameter("publish_nan_on_error", True)

        values = {
            "bus": self.get_parameter("bus").value,
            "tca_addr": self.get_parameter("tca_addr").value,
            "vl53_addr": self.get_parameter("vl53_addr").value,
            "right_channel": self.get_parameter("right_channel").value,
            "left_channel": self.get_parameter("left_channel").value,
            "rate_hz": self.get_parameter("rate_hz").value,
            "median_window": self.get_parameter("median_window").value,
            "left_topic": self.get_parameter("left_topic").value,
            "right_topic": self.get_parameter("right_topic").value,
        }

        self.params = load_vl53_mux_params(values)
        self.publish_nan_on_error = bool(self.get_parameter("publish_nan_on_error").value)

        driver_cfg = Vl53MuxConfig(
            bus=self.params.bus,
            tca_addr=self.params.tca_addr,
            vl53_addr=self.params.vl53_addr,
            right_channel=self.params.right_channel,
            left_channel=self.params.left_channel,
            median_window=self.params.median_window,
        )

        self.driver = Vl53MuxDriver(driver_cfg)
        self.driver_error: Optional[str] = None

        self.left_pub = self.create_publisher(Float32, self.params.left_topic, qos_range_sensor())
        self.right_pub = self.create_publisher(Float32, self.params.right_topic, qos_range_sensor())

        self._start_driver()

        period_s = 1.0 / max(float(self.params.rate_hz), 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            "VL53 mux fallback node started: "
            f"bus={self.params.bus}, tca=0x{self.params.tca_addr:02X}, "
            f"right_ch={self.params.right_channel}, left_ch={self.params.left_channel}, "
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
            self.get_logger().error(f"VL53 mux driver start failed: {exc}")

    def _stop_driver(self) -> None:
        try:
            self.driver.close()
        except Exception:
            pass

    def _on_timer(self) -> None:
        if not self.driver.started:
            if self.publish_nan_on_error:
                self._publish_nan_pair()
            return

        try:
            left, right = self.driver.read_samples()
        except Exception as exc:
            self.driver_error = str(exc)
            self.get_logger().warn(f"VL53 read failed: {exc}")
            if self.publish_nan_on_error:
                self._publish_nan_pair()
            return

        self._publish_distance(self.left_pub, left.distance_m)
        self._publish_distance(self.right_pub, right.distance_m)

    @staticmethod
    def _publish_distance(pub, distance_m: Optional[float]) -> None:
        msg = Float32()
        msg.data = float(distance_m) if distance_m is not None else math.nan
        pub.publish(msg)

    def _publish_nan_pair(self) -> None:
        self._publish_distance(self.left_pub, None)
        self._publish_distance(self.right_pub, None)


def main(args=None) -> int:
    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this node.")
        return 1

    rclpy.init(args=args)
    node = Vl53MuxNodePy()

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
