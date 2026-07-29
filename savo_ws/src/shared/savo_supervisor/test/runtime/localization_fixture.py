#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class LocalizationFixture(Node):

    def __init__(self) -> None:
        super().__init__('savo_supervisor_localization_fixture')

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        heartbeat_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.health_publisher = self.create_publisher(
            String,
            '/savo_localization/health',
            status_qos,
        )

        self.summary_publisher = self.create_publisher(
            String,
            '/savo_localization/state_summary',
            status_qos,
        )

        self.heartbeat_publisher = self.create_publisher(
            String,
            '/savo_localization/heartbeat',
            heartbeat_qos,
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_localization,
        )

        self.get_logger().info(
            'Publishing healthy localization contracts at 10 Hz'
        )

    @staticmethod
    def encode(payload: dict) -> String:
        message = String()

        message.data = json.dumps(
            payload,
            separators=(',', ':'),
            allow_nan=False,
        )

        return message

    def publish_localization(self) -> None:
        stamp_s = (
            self.get_clock().now().nanoseconds /
            1_000_000_000.0
        )

        health = {
            'schema_version': 1,
            'node': 'localization_health_node',
            'state': 'OK',
            'ready': True,
            'degraded': False,
            'reason_code': 'localization_operational',
            'stamp_s': stamp_s,
        }

        summary = {
            'schema_version': 1,
            'state': 'OK',
            'ready': True,
            'degraded': False,
            'reason_code': 'localization_operational',
            'stamp_s': stamp_s,
        }

        heartbeat = {
            'schema_version': 1,
            'node': 'localization_health_node',
            'alive': True,
            'state': 'OK',
            'ready': True,
            'stamp_s': stamp_s,
        }

        self.health_publisher.publish(
            self.encode(health)
        )

        self.summary_publisher.publish(
            self.encode(summary)
        )

        self.heartbeat_publisher.publish(
            self.encode(heartbeat)
        )


def main() -> None:
    rclpy.init()

    node = LocalizationFixture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
