#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Publish deterministic Phase 2 mission contracts and action endpoints."""

from __future__ import annotations

import json

from core_fixture import CoreFixture
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.action import (
    ConfirmAprilTag,
    ExecuteCoveragePath,
    RotateToHeading,
    RunAutonomousMapping,
)
from std_msgs.msg import Bool, String, UInt64


_CONTROL_TOPIC = '/savo_supervisor/test/control'


def _kv(key: str, value: str) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = value
    return item


class MissionFixture(Node):
    """Publish package-owned mission status while exposing fake action servers."""

    def __init__(self) -> None:
        super().__init__('savo_supervisor_phase2_fixture')
        self._drop_navigation = False
        self._drop_mapping = False
        self._drop_head = False
        self._drop_locations = False
        self._drop_bridge = False
        self._drop_realsense = False
        self._drop_speech = False
        self._drop_vo = False
        self._heartbeat = 0

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._mapping_status = self.create_publisher(
            String, '/savo_mapping/status', retained_qos)
        self._navigation_status = self.create_publisher(
            String, '/savo_nav/status', retained_qos)
        self._navigation_heartbeat = self.create_publisher(
            UInt64, '/savo_nav/heartbeat', status_qos)
        self._head_status = self.create_publisher(
            DiagnosticArray, '/savo_head/status', status_qos)
        self._locations_status = self.create_publisher(
            String, '/savo_locations/status', retained_qos)
        self._locations_heartbeat = self.create_publisher(
            UInt64, '/savo_locations/heartbeat', status_qos)
        self._bridge_state = self.create_publisher(
            String, '/savo_bridge/state', retained_qos)
        self._bridge_readiness = self.create_publisher(
            Bool, '/savo_bridge/readiness', retained_qos)
        self._bridge_heartbeat = self.create_publisher(
            UInt64, '/savo_bridge/heartbeat', status_qos)
        self._realsense_status = self.create_publisher(
            String, '/realsense/status', retained_qos)
        self._speech_readiness = self.create_publisher(
            String, '/savo_speech/readiness', retained_qos)
        self._speech_heartbeat = self.create_publisher(
            UInt64, '/savo_speech/heartbeat', status_qos)
        self._vo_health = self.create_publisher(
            String, '/vo/health', retained_qos)

        self._control = self.create_subscription(
            String, _CONTROL_TOPIC, self._on_control, status_qos)

        self._mapping_action = ActionServer(
            self,
            RunAutonomousMapping,
            '/savo_mapping/autonomous/run',
            self._execute_mapping,
        )
        self._rotate_action = ActionServer(
            self,
            RotateToHeading,
            '/savo_control/rotate_to_heading',
            self._execute_rotate,
        )
        self._coverage_action = ActionServer(
            self,
            ExecuteCoveragePath,
            '/savo_nav/coverage/execute_path',
            self._execute_coverage,
        )
        self._tag_action = ActionServer(
            self,
            ConfirmAprilTag,
            '/savo_head/apriltag/confirm',
            self._execute_tag,
        )

        self._timer = self.create_timer(0.1, self._publish)

    def _on_control(self, message: String) -> None:
        command = message.data.strip().lower()
        if command == 'drop_navigation':
            self._drop_navigation = True
        elif command == 'restore_navigation':
            self._drop_navigation = False
        elif command == 'drop_mapping':
            self._drop_mapping = True
        elif command == 'restore_mapping':
            self._drop_mapping = False
        elif command == 'drop_head':
            self._drop_head = True
        elif command == 'restore_head':
            self._drop_head = False
        elif command == 'drop_locations':
            self._drop_locations = True
        elif command == 'restore_locations':
            self._drop_locations = False
        elif command == 'drop_bridge':
            self._drop_bridge = True
        elif command == 'restore_bridge':
            self._drop_bridge = False
        elif command == 'drop_realsense':
            self._drop_realsense = True
        elif command == 'restore_realsense':
            self._drop_realsense = False
        elif command == 'drop_speech':
            self._drop_speech = True
        elif command == 'restore_speech':
            self._drop_speech = False
        elif command == 'drop_vo':
            self._drop_vo = True
        elif command == 'restore_vo':
            self._drop_vo = False

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
        self._heartbeat += 1
        if not self._drop_mapping:
            self._mapping_status.publish(self._string({
                'mode': 'online_async',
                'exploration_mode': 'frontier',
                'workflow_phase': 'active',
                'session_state': 'active',
                'healthy': True,
                'ready': True,
                'slam_active': True,
                'map_received': True,
                'scan_received': True,
                'tf_ok': True,
                'odom_ok': True,
                'quality_score': 1.0,
                'heartbeat_seq': self._heartbeat,
                'active_map_name': 'floor_2',
                'message': 'ready',
            }))

        if not self._drop_navigation:
            self._navigation_status.publish(self._string(
                'state=READY;goal_acceptance_allowed=true;'
                'reason=navigation_ready;failed_dependencies='))
            heartbeat = UInt64()
            heartbeat.data = self._heartbeat
            self._navigation_heartbeat.publish(heartbeat)

        if not self._drop_head:
            status = DiagnosticStatus()
            status.name = 'savo_head.head_status'
            status.hardware_id = 'savo_head_fixture'
            status.level = DiagnosticStatus.OK
            status.message = 'head operational'
            status.values = [
                _kv('pan_tilt_state', 'OK'),
                _kv('camera_stream_healthy', 'true'),
                _kv('camera_pose_ready', 'true'),
            ]
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [status]
            self._head_status.publish(array)

        if not self._drop_bridge:
            self._bridge_state.publish(self._string({
                'schema_name': 'savo_bridge_state',
                'schema_version': 2,
                'process_alive': True,
                'bridge_ready': True,
                'commands_enabled': True,
                'dds_active': True,
                'core_visible': True,
                'edge_visible': True,
                'stop_ready': True,
                'teleop_ready': True,
                'navigation_ready': True,
                'readiness_reason': 'bridge_ready',
            }))
            bridge_ready = Bool()
            bridge_ready.data = True
            self._bridge_readiness.publish(bridge_ready)
            edge_heartbeat = UInt64()
            edge_heartbeat.data = self._heartbeat
            self._bridge_heartbeat.publish(edge_heartbeat)

        if not self._drop_realsense:
            self._realsense_status.publish(self._string({
                'ok': True,
                'message': 'RealSense streams OK',
                'color_ok': True,
                'color_info_ok': True,
                'depth_ok': True,
                'depth_info_ok': True,
                'pointcloud_ok': True,
                'require_pointcloud': False,
            }))

        if not self._drop_speech:
            self._speech_readiness.publish(self._string('ready'))
            speech_heartbeat = UInt64()
            speech_heartbeat.data = self._heartbeat
            self._speech_heartbeat.publish(speech_heartbeat)

        if not self._drop_vo:
            self._vo_health.publish(self._string('ok: tracking'))

        if not self._drop_locations:
            self._locations_status.publish(self._string({
                'component': 'savo_locations',
                'mode': 'read_write',
                'state': 'ready',
                'ready': True,
                'read_ready': True,
                'write_ready': True,
                'storage_healthy': True,
                'mutation_in_progress': False,
                'schema_version': 1,
                'reason': 'locations_ready',
            }))
            heartbeat = UInt64()
            heartbeat.data = self._heartbeat
            self._locations_heartbeat.publish(heartbeat)

    async def _execute_mapping(self, goal_handle):
        goal_handle.abort()
        return RunAutonomousMapping.Result()

    async def _execute_rotate(self, goal_handle):
        goal_handle.abort()
        return RotateToHeading.Result()

    async def _execute_coverage(self, goal_handle):
        goal_handle.abort()
        return ExecuteCoveragePath.Result()

    async def _execute_tag(self, goal_handle):
        goal_handle.abort()
        return ConfirmAprilTag.Result()

    def destroy_node(self) -> bool:
        self._mapping_action.destroy()
        self._rotate_action.destroy()
        self._coverage_action.destroy()
        self._tag_action.destroy()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    core = CoreFixture(
        drop_component='none',
        safety_stop=False,
        slowdown_factor=1.0,
    )
    mission = MissionFixture()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(core)
    executor.add_node(mission)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mission.destroy_node()
        core.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
