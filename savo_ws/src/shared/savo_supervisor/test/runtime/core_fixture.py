#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Publish deterministic healthy Phase 1 core contracts for runtime tests."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


_COMPONENTS = (
    'none',
    'base',
    'control',
    'perception',
    'lidar',
    'localization',
    'power',
    'safety',
)


def _arguments(argv: Sequence[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument('--drop-component', choices=_COMPONENTS, default='none')
    parser.add_argument('--safety-stop', action='store_true')
    parser.add_argument('--slowdown-factor', type=float, default=1.0)
    return parser.parse_known_args(argv)


class CoreFixture(Node):
    """Publish native package payload shapes without implementing package policy."""

    def __init__(
        self,
        *,
        drop_component: str,
        safety_stop: bool,
        slowdown_factor: float,
    ) -> None:
        super().__init__('savo_supervisor_core_fixture')
        if not 0.0 <= slowdown_factor <= 1.0:
            raise ValueError('slowdown_factor must be inside [0, 1]')

        self._drop_component = drop_component
        self._safety_stop = safety_stop
        self._slowdown_factor = slowdown_factor
        self._heartbeat_count = 0

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._base_state = self.create_publisher(
            String, '/savo_base/base_state', status_qos)

        self._control_mode = self.create_publisher(
            String, '/savo_control/control_status', status_qos)
        self._control_mux = self.create_publisher(
            String, '/savo_control/twist_mux/status', status_qos)
        self._control_shaper = self.create_publisher(
            String, '/savo_control/cmd_vel_shaper/status', status_qos)

        self._perception_health = self.create_publisher(
            String, '/savo_perception/range_health', status_qos)
        self._perception_state = self.create_publisher(
            String, '/savo_perception/safety_state', status_qos)
        self._perception_heartbeat = self.create_publisher(
            String, '/savo_perception/heartbeat', status_qos)

        self._lidar_state = self.create_publisher(
            String, '/savo_lidar/state', status_qos)
        self._lidar_heartbeat = self.create_publisher(
            String, '/savo_lidar/heartbeat', status_qos)

        self._localization_health = self.create_publisher(
            String, '/savo_localization/health', latched_qos)
        self._localization_summary = self.create_publisher(
            String, '/savo_localization/state_summary', latched_qos)
        self._localization_heartbeat = self.create_publisher(
            String, '/savo_localization/heartbeat', status_qos)

        self._power_health = self.create_publisher(
            String, '/savo_power/health', status_qos)
        self._power_status = self.create_publisher(
            String, '/savo_power/status', status_qos)

        self._safety_stop_publisher = self.create_publisher(
            Bool, '/safety/stop', status_qos)
        self._slowdown_publisher = self.create_publisher(
            Float32, '/safety/slowdown_factor', status_qos)

        self._timer = self.create_timer(0.1, self._publish)
        self.get_logger().info(
            'Phase 1 core fixture started | drop_component=%s safety_stop=%s slowdown=%.2f'
            % (drop_component, safety_stop, slowdown_factor)
        )

    @staticmethod
    def _string(payload: str | dict[str, object]) -> String:
        message = String()
        message.data = (
            json.dumps(payload, separators=(',', ':'), allow_nan=False)
            if isinstance(payload, dict)
            else payload
        )
        return message

    def _publish(self) -> None:
        self._heartbeat_count += 1
        stamp_s = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if self._drop_component != 'base':
            self._base_state.publish(self._string({
                'node': 'base_driver_node',
                'status_level': 'OK',
                'backend': {'connected': True},
                'diagnostics': {'last_board_error': ''},
            }))

        if self._drop_component != 'control':
            self._control_mode.publish(self._string(
                'mode=STOP; previous=STOP; reason=STARTUP; source=startup; '
                'safety_stop=false; external_stop=false; recovery_active=false; '
                'manual_override=false; request_stale=false; mux_mode=STOP'))
            self._control_mux.publish(self._string(
                'mode=STOP; source=STOP; reason=stop_mode; stale=false; '
                'safety_stop=false; recovery_active=false; last_mode_text=STOP; '
                'now_s=1.0; vx=0.0; vy=0.0; wz=0.0'))
            self._control_shaper.publish(self._string(
                'reason=input_timeout; stale=true; safety_stop=false; mode=STOP; '
                'slowdown=1.0; ignore_slowdown_in_recovery=true; '
                'target_vx=0.0; target_vy=0.0; target_wz=0.0; '
                'shaped_vx=0.0; shaped_vy=0.0; shaped_wz=0.0; '
                'timed_out=true; now_s=1.0'))

        if self._drop_component != 'perception':
            self._perception_health.publish(self._string({
                'overall_ok': True,
                'overall_status': 'OK',
                'stale_required_sensors': [],
                'error_required_sensors': [],
            }))
            self._perception_state.publish(self._string({
                'update_count': self._heartbeat_count,
                'stop_count': 0,
                'clear_count': self._heartbeat_count,
                'active_decision': {
                    'status': 'CLEAR',
                    'stop_required': False,
                    'slowdown_factor': 1.0,
                    'reason': 'fixture_clear',
                },
            }))
            self._perception_heartbeat.publish(self._string({
                'node': 'range_health_node',
                'count': self._heartbeat_count,
                'ok': True,
            }))

        if self._drop_component != 'lidar':
            self._lidar_state.publish(self._string({
                'component': 'lidar_driver_node',
                'status': 'OK',
                'message': 'LiDAR driver running',
                'hardware_ok': True,
                'scan_ok': True,
                'driver_running': True,
                'scan_count': self._heartbeat_count,
                'last_error': '',
            }))
            self._lidar_heartbeat.publish(self._string({
                'component': 'lidar_driver_node',
                'status': 'OK',
                'message': 'driver running',
                'driver_running': True,
                'scan_count': self._heartbeat_count,
                'last_error': '',
            }))

        if self._drop_component != 'localization':
            self._localization_health.publish(self._string({
                'schema_version': 1,
                'node': 'localization_health_node',
                'state': 'OK',
                'ready': True,
                'degraded': False,
                'reason_code': 'localization_operational',
                'stamp_s': stamp_s,
            }))
            self._localization_summary.publish(self._string({
                'schema_version': 1,
                'state': 'OK',
                'ready': True,
                'degraded': False,
                'reason_code': 'localization_operational',
                'stamp_s': stamp_s,
            }))
            self._localization_heartbeat.publish(self._string({
                'schema_version': 1,
                'node': 'localization_health_node',
                'alive': True,
                'state': 'OK',
                'ready': True,
                'stamp_s': stamp_s,
            }))

        if self._drop_component != 'power':
            self._power_status.publish(self._string(
                'overall=OK health=OK core=OK edge=OK base=OK'))
            self._power_health.publish(self._string(
                'level=OK state=OK reason=power_ok'))

        if self._drop_component != 'safety':
            stop = Bool()
            stop.data = self._safety_stop
            self._safety_stop_publisher.publish(stop)

            slowdown = Float32()
            slowdown.data = float(self._slowdown_factor)
            self._slowdown_publisher.publish(slowdown)


def main(argv: Sequence[str] | None = None) -> None:
    options, ros_args = _arguments(argv)
    rclpy.init(args=ros_args)
    node = CoreFixture(
        drop_component=options.drop_component,
        safety_stop=options.safety_stop,
        slowdown_factor=options.slowdown_factor,
    )
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
