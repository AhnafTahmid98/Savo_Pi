"""Import smoke tests for pure savo_vo modules."""

import importlib


PURE_MODULES = [
    "savo_vo",
    "savo_vo.version",
    "savo_vo.contracts.topic_names",
    "savo_vo.contracts.frame_names",
    "savo_vo.contracts.parameter_names",
    "savo_vo.contracts",
    "savo_vo.models.vo_status",
    "savo_vo.models.tracking_report",
    "savo_vo.models.odometry_quality",
    "savo_vo.models.vo_sample",
    "savo_vo.models",
    "savo_vo.core.timestamp_sync",
    "savo_vo.core.tracking_quality",
    "savo_vo.core.motion_quality",
    "savo_vo.core.odom_checks",
    "savo_vo.core.covariance_builder",
    "savo_vo.core.vo_state_machine",
    "savo_vo.core",
    "savo_vo.utils.numeric",
    "savo_vo.utils.geometry",
    "savo_vo.utils.validation",
    "savo_vo.utils.ros_time",
    "savo_vo.utils",
    "savo_vo.adapters.camera_info_adapter",
    "savo_vo.adapters.realsense_topic_adapter",
    "savo_vo.adapters",
]


def test_pure_modules_import_without_ros_runtime() -> None:
    for module_name in PURE_MODULES:
        importlib.import_module(module_name)