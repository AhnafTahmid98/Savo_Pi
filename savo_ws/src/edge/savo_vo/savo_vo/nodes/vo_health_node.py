"""ROS 2 node that monitors visual odometry health."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from savo_vo.contracts.parameter_names import (
    HEALTH_TOPIC_PARAM,
    ODOM_TOPIC_PARAM,
    STALE_TIMEOUT_S_PARAM,
    STATUS_TOPIC_PARAM,
)
from savo_vo.contracts.topic_names import (
    VO_HEALTH_TOPIC,
    VO_ODOM_TOPIC,
    VO_STATUS_TOPIC,
)
from savo_vo.core.timestamp_sync import age_s
from savo_vo.utils.ros_time import stamp_to_seconds
from savo_vo.ros.qos import odometry_qos, status_qos


class VOHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("vo_health_node")

        self.declare_parameter(ODOM_TOPIC_PARAM, VO_ODOM_TOPIC)
        self.declare_parameter(STATUS_TOPIC_PARAM, VO_STATUS_TOPIC)
        self.declare_parameter(HEALTH_TOPIC_PARAM, VO_HEALTH_TOPIC)
        self.declare_parameter(STALE_TIMEOUT_S_PARAM, 0.50)

        self._odom_topic = self.get_parameter(
            ODOM_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._status_topic = self.get_parameter(
            STATUS_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._health_topic = self.get_parameter(
            HEALTH_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._stale_timeout_s = self.get_parameter(
            STALE_TIMEOUT_S_PARAM
        ).get_parameter_value().double_value

        self._last_odom_stamp_s: float | None = None
        self._last_status: str = "waiting for visual odometry"

        self._health_pub = self.create_publisher(
            String,
            self._health_topic,
            status_qos(),
        )

        self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            odometry_qos(),
        )
        self.create_subscription(
            String,
            self._status_topic,
            self._on_status,
            status_qos(),
        )

        self.create_timer(0.20, self._publish_health)

        self.get_logger().info(
            f"VO health node started: odom={self._odom_topic}, "
            f"health={self._health_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_stamp_s = stamp_to_seconds(msg.header.stamp)

    def _on_status(self, msg: String) -> None:
        self._last_status = msg.data

    def _publish_health(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9

        if self._last_odom_stamp_s is None:
            health = "waiting"
        else:
            odom_age_s = age_s(
                timestamp_s=self._last_odom_stamp_s,
                now_s=now_s,
            )
            if odom_age_s > self._stale_timeout_s:
                health = f"stale: odom age {odom_age_s:.3f}s"
            else:
                health = f"ok: {self._last_status}"

        msg = String()
        msg.data = health
        self._health_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VOHealthNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()