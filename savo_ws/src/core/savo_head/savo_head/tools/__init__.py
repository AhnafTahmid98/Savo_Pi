# -*- coding: utf-8 -*-

"""Python diagnostic tools for Robot Savo active head."""

from __future__ import annotations

from savo_head.tools.apriltag_debug_cli import (
    build_parser as build_apriltag_debug_parser,
    make_observation,
    make_policy,
    make_registration,
    make_robot_pose,
    parse_aliases,
    run_debug,
)
from savo_head.tools.dump_effective_head_params import (
    CONFIG_FILES,
    build_payload,
    find_package_root,
    load_all_params,
    unknown_parameter_names,
)
from savo_head.tools.head_camera_view import (
    CameraStreamConfig,
    CameraStreamProcess,
    build_parser as build_head_camera_view_parser,
    make_scan_profile,
)
from savo_head.tools.head_manual_cli import (
    build_parser as build_head_manual_parser,
    command_from_key,
    make_driver,
    run_manual_cli,
)

__all__ = [
    # head_camera_view
    "CameraStreamConfig",
    "CameraStreamProcess",
    "build_head_camera_view_parser",
    "make_scan_profile",
    # head_manual_cli
    "build_head_manual_parser",
    "command_from_key",
    "make_driver",
    "run_manual_cli",
    # apriltag_debug_cli
    "build_apriltag_debug_parser",
    "parse_aliases",
    "make_registration",
    "make_observation",
    "make_robot_pose",
    "make_policy",
    "run_debug",
    # dump_effective_head_params
    "CONFIG_FILES",
    "find_package_root",
    "load_all_params",
    "unknown_parameter_names",
    "build_payload",
]
