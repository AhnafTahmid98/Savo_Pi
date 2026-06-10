"""ROS 2 node that republishes VO odometry for downstream consumers."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from savo_vo.contracts.frame_names import BASE_LINK_FRAME, ODOM_FRAME
from savo_vo.contracts.parameter_names import (
    BASE_FRAME_PARAM,
    ODOM_FRAME_PARAM,
    ODOM_RAW_TOPIC_PARAM,
    ODOM_TOPIC_PARAM,
)
from savo_vo.contracts.topic_names import VO_ODOM_RAW_TOPIC, VO_ODOM_TOPIC
from savo_vo.ros.qos import odometry_qos


class VORepublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("vo_republisher_node")

        self.declare_parameter(ODOM_RAW_TOPIC_PARAM, VO_ODOM_RAW_TOPIC)
        self.declare_parameter(ODOM_TOPIC_PARAM, VO_ODOM_TOPIC)
        self.declare_parameter(ODOM_FRAME_PARAM, ODOM_FRAME)
        self.declare_parameter(BASE_FRAME_PARAM, BASE_LINK_FRAME)

        self._input_topic = self.get_parameter(
            ODOM_RAW_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._output_topic = self.get_parameter(
            ODOM_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._odom_frame = self.get_parameter(
            ODOM_FRAME_PARAM
        ).get_parameter_value().string_value
        self._base_frame = self.get_parameter(
            BASE_FRAME_PARAM
        ).get_parameter_value().string_value

        self._publisher = self.create_publisher(
            Odometry,
            self._output_topic,
            odometry_qos(),
        )
        self.create_subscription(
            Odometry,
            self._input_topic,
            self._on_odom,
            odometry_qos(),
        )

        self.get_logger().info(
            f"VO republisher started: {self._input_topic} -> {self._output_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VORepublisherNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()