# Copyright 2026 Ahnaf Tahmid
import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from savo_realsense.constants import (
    DEFAULT_DEPTH_IMAGE_TOPIC,
    DEFAULT_FRONT_DEPTH_PERCENTILE,
    DEFAULT_MAX_VALID_DEPTH_M,
    DEFAULT_MIN_VALID_DEPTH_M,
)
from savo_realsense.ros.qos_profiles import sensor_data_qos, status_qos
from savo_realsense.utils.depth_image import front_roi_depth_m
from savo_realsense.utils.param_loader import get_float_param, get_str_param


class DepthFrontMinNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_front_min_node")

        self._bridge = CvBridge()

        self._input_topic = get_str_param(
            self,
            "input_topic",
            DEFAULT_DEPTH_IMAGE_TOPIC,
        )
        self._output_topic = get_str_param(
            self,
            "output_topic",
            "/depth/min_front_m",
        )

        self._depth_scale = get_float_param(self, "depth_scale", 0.001, min_value=0.000001)
        self._min_valid_m = get_float_param(
            self,
            "min_valid_m",
            DEFAULT_MIN_VALID_DEPTH_M,
            min_value=0.001,
        )
        self._max_valid_m = get_float_param(
            self,
            "max_valid_m",
            DEFAULT_MAX_VALID_DEPTH_M,
            min_value=self._min_valid_m,
        )
        self._percentile = get_float_param(
            self,
            "percentile",
            DEFAULT_FRONT_DEPTH_PERCENTILE,
            min_value=0.0,
        )

        self._x_min_ratio = get_float_param(self, "x_min_ratio", 0.35, min_value=0.0)
        self._x_max_ratio = get_float_param(self, "x_max_ratio", 0.65, min_value=0.0)
        self._y_min_ratio = get_float_param(self, "y_min_ratio", 0.35, min_value=0.0)
        self._y_max_ratio = get_float_param(self, "y_max_ratio", 0.75, min_value=0.0)

        self._publisher = self.create_publisher(
            Float32,
            self._output_topic,
            status_qos(),
        )

        self.create_subscription(
            Image,
            self._input_topic,
            self._on_depth_image,
            sensor_data_qos(),
        )

        self.get_logger().info(
            f"Depth front-min subscribed to {self._input_topic}, publishing {self._output_topic}"
        )

    def _on_depth_image(self, msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except (CvBridgeError, cv2.error) as exc:
            self.get_logger().warn(f"Failed to convert depth image: {exc}")
            self._publish_depth(math.nan)
            return

        try:
            distance_m = front_roi_depth_m(
                image=image,
                encoding=msg.encoding,
                x_min_ratio=self._x_min_ratio,
                x_max_ratio=self._x_max_ratio,
                y_min_ratio=self._y_min_ratio,
                y_max_ratio=self._y_max_ratio,
                percentile=self._percentile,
                min_valid_m=self._min_valid_m,
                max_valid_m=self._max_valid_m,
                depth_scale=self._depth_scale,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Invalid depth image input: {exc}")
            distance_m = math.nan

        self._publish_depth(distance_m)

    def _publish_depth(self, distance_m: float) -> None:
        self._publisher.publish(Float32(data=float(distance_m)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = DepthFrontMinNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
