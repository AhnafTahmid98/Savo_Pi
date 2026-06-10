"""ROS 2 node that publishes visual odometry diagnostics."""

from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_msgs.msg import String

from savo_vo.adapters.diagnostics_adapter import status_to_diagnostic_msg
from savo_vo.contracts.parameter_names import (
    DIAGNOSTICS_RATE_HZ_PARAM,
    HEALTH_TOPIC_PARAM,
    STATUS_TOPIC_PARAM,
)
from savo_vo.contracts.topic_names import (
    DIAGNOSTICS_TOPIC,
    VO_HEALTH_TOPIC,
    VO_STATUS_TOPIC,
)
from savo_vo.models.vo_status import VOStatus
from savo_vo.ros.qos import diagnostics_qos, status_qos


class VODiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("vo_diagnostics_node")

        self.declare_parameter(STATUS_TOPIC_PARAM, VO_STATUS_TOPIC)
        self.declare_parameter(HEALTH_TOPIC_PARAM, VO_HEALTH_TOPIC)
        self.declare_parameter("diagnostics_topic", DIAGNOSTICS_TOPIC)
        self.declare_parameter(DIAGNOSTICS_RATE_HZ_PARAM, 2.0)

        self._status_topic = self.get_parameter(
            STATUS_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._health_topic = self.get_parameter(
            HEALTH_TOPIC_PARAM
        ).get_parameter_value().string_value
        self._diagnostics_topic = self.get_parameter(
            "diagnostics_topic"
        ).get_parameter_value().string_value
        self._diagnostics_rate_hz = self.get_parameter(
            DIAGNOSTICS_RATE_HZ_PARAM
        ).get_parameter_value().double_value

        if self._diagnostics_rate_hz <= 0.0:
            raise ValueError("diagnostics_rate_hz must be positive")

        self._last_status = "waiting for visual odometry"
        self._last_health = "waiting"

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self._diagnostics_topic,
            diagnostics_qos(),
        )

        self.create_subscription(
            String,
            self._status_topic,
            self._on_status,
            status_qos(),
        )
        self.create_subscription(
            String,
            self._health_topic,
            self._on_health,
            status_qos(),
        )

        self.create_timer(1.0 / self._diagnostics_rate_hz, self._publish_diagnostics)

        self.get_logger().info(
            f"VO diagnostics node started: diagnostics={self._diagnostics_topic}"
        )

    def _on_status(self, msg: String) -> None:
        self._last_status = msg.data

    def _on_health(self, msg: String) -> None:
        self._last_health = msg.data

    def _publish_diagnostics(self) -> None:
        status = self._build_status_from_text()

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(
            status_to_diagnostic_msg(
                status=status,
                name="savo_vo",
                hardware_id="savo-edge/realsense",
            )
        )

        self._diagnostics_pub.publish(array)

    def _build_status_from_text(self) -> VOStatus:
        health_lower = self._last_health.lower()
        status_text = f"{self._last_health}; {self._last_status}"

        if health_lower.startswith("ok"):
            return VOStatus.ok(message=status_text)

        if health_lower.startswith("waiting"):
            return VOStatus.degraded(message=status_text)

        if health_lower.startswith("stale"):
            return VOStatus.stale(age_s=0.0, message=status_text)

        return VOStatus.error(status_text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VODiagnosticsNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()