# -*- coding: utf-8 -*-
"""ROS helpers for LiDAR nodes."""

from __future__ import annotations

try:
    from .adapters import (
        clamp_laser_ranges,
        copy_laser_scan,
        make_string_msg,
        scan_angle_deg,
        scan_index_for_angle_deg,
        valid_range_count,
        valid_range_ratio,
    )
except ModuleNotFoundError as exc:
    if exc.name not in ("sensor_msgs", "std_msgs"):
        raise

    def _missing_ros_messages(*args, **kwargs):
        raise ModuleNotFoundError("ROS message packages are required for LiDAR adapters")

    clamp_laser_ranges = _missing_ros_messages
    copy_laser_scan = _missing_ros_messages
    make_string_msg = _missing_ros_messages
    scan_angle_deg = _missing_ros_messages
    scan_index_for_angle_deg = _missing_ros_messages
    valid_range_count = _missing_ros_messages
    valid_range_ratio = _missing_ros_messages

try:
    from .params import (
        declare_if_missing,
        get_bool_param,
        get_float_param,
        get_int_param,
        get_positive_float_param,
        get_range_param,
        get_string_param,
        init_rclpy_if_needed,
    )
    from .qos_profiles import command_qos, latched_state_qos, sensor_qos, state_qos
except ModuleNotFoundError as exc:
    if exc.name != "rclpy":
        raise

    def _missing_rclpy(*args, **kwargs):
        raise ModuleNotFoundError("rclpy is required for LiDAR ROS helpers")

    declare_if_missing = _missing_rclpy
    get_bool_param = _missing_rclpy
    get_float_param = _missing_rclpy
    get_int_param = _missing_rclpy
    get_positive_float_param = _missing_rclpy
    get_range_param = _missing_rclpy
    get_string_param = _missing_rclpy
    init_rclpy_if_needed = _missing_rclpy
    command_qos = _missing_rclpy
    latched_state_qos = _missing_rclpy
    sensor_qos = _missing_rclpy
    state_qos = _missing_rclpy
from .topic_contract import (
    DEFAULT_TOPICS,
    LidarTopics,
    get_default_topics,
    normalize_topic_name,
    validate_lidar_topics,
)

__all__ = [
    "DEFAULT_TOPICS",
    "LidarTopics",
    "clamp_laser_ranges",
    "command_qos",
    "copy_laser_scan",
    "declare_if_missing",
    "get_bool_param",
    "get_default_topics",
    "get_float_param",
    "get_int_param",
    "get_positive_float_param",
    "get_range_param",
    "get_string_param",
    "init_rclpy_if_needed",
    "latched_state_qos",
    "make_string_msg",
    "normalize_topic_name",
    "scan_angle_deg",
    "scan_index_for_angle_deg",
    "sensor_qos",
    "state_qos",
    "valid_range_count",
    "valid_range_ratio",
    "validate_lidar_topics",
]
