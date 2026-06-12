"""ROS helpers and contracts for Robot Savo LiDAR nodes."""

from .adapters import (
    clamp_laser_ranges,
    copy_laser_scan,
    make_string_msg,
    scan_angle_deg,
    scan_index_for_angle_deg,
    valid_range_count,
    valid_range_ratio,
)
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