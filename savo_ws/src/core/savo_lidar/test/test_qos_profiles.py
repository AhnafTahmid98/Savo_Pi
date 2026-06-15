# -*- coding: utf-8 -*-

from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from savo_lidar.ros import (
    command_qos,
    latched_state_qos,
    sensor_qos,
    state_qos,
)
from savo_lidar.utils import (
    health_qos,
    heartbeat_qos,
    scan_qos,
    service_command_qos,
    status_qos,
    watchdog_qos,
)


def test_sensor_qos_is_best_effort_volatile():
    qos = sensor_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_sensor_qos_clamps_depth_to_one():
    qos = sensor_qos(0)

    assert qos.depth == 1


def test_state_qos_is_reliable_volatile():
    qos = state_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_state_qos_clamps_depth_to_one():
    qos = state_qos(0)

    assert qos.depth == 1


def test_latched_state_qos_is_reliable_transient_local():
    qos = latched_state_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_latched_state_qos_clamps_depth_to_one():
    qos = latched_state_qos(0)

    assert qos.depth == 1


def test_command_qos_is_reliable_volatile():
    qos = command_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_command_qos_clamps_depth_to_one():
    qos = command_qos(0)

    assert qos.depth == 1


def test_utils_scan_qos_matches_sensor_qos_policy():
    qos = scan_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_utils_status_qos_is_reliable_volatile():
    qos = status_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_utils_health_qos_reuses_status_policy():
    qos = health_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_utils_watchdog_qos_reuses_status_policy():
    qos = watchdog_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_utils_heartbeat_qos_is_latched():
    qos = heartbeat_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_utils_service_command_qos_is_reliable_volatile():
    qos = service_command_qos()

    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_utils_qos_helpers_clamp_depth_to_one():
    assert scan_qos(0).depth == 1
    assert status_qos(0).depth == 1
    assert health_qos(0).depth == 1
    assert watchdog_qos(0).depth == 1
    assert heartbeat_qos(0).depth == 1
    assert service_command_qos(0).depth == 1
