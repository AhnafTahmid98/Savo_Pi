#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Import tests for Robot Savo mapping CLI scripts.

These tests make sure CLI modules can be imported safely without:
- starting ROS nodes
- calling rclpy.init()
- running CLI main logic
- requiring a live ROS graph
"""

from __future__ import annotations

import importlib
from types import ModuleType


# =============================================================================
# Script module list
# =============================================================================
SCRIPT_MODULES = (
    "scripts.mapping_readiness_cli",
    "scripts.mapping_smoke_test_cli",
    "scripts.dump_effective_params",
    "scripts.map_quality_cli",
    "scripts.map_metadata_cli",
    "scripts.load_map_check_cli",
    "scripts.save_map_cli",
    "scripts.manual_mapping_cli",
    "scripts.autonomous_mapping_cli",
    "scripts.pointcloud_echo_cli",
    "scripts.voxel_costmap_check_cli",
    "scripts.apriltag_check_cli",
    "scripts.semantic_landmark_cli",
    "scripts.location_bridge_check_cli",
    "scripts.clean_maps_cli",
    "scripts.operator_notes_cli",
)


# =============================================================================
# Import checks
# =============================================================================
def test_scripts_package_imports() -> None:
    module = importlib.import_module("scripts")

    assert isinstance(module, ModuleType)
    assert module.__all__ == []


def test_all_cli_scripts_import_without_side_effects() -> None:
    for module_name in SCRIPT_MODULES:
        module = importlib.import_module(module_name)

        assert isinstance(module, ModuleType), module_name
        assert hasattr(module, "main"), module_name
        assert callable(module.main), module_name


def test_all_cli_scripts_have_build_parser_when_expected() -> None:
    for module_name in SCRIPT_MODULES:
        module = importlib.import_module(module_name)

        assert hasattr(module, "build_parser"), module_name
        assert callable(module.build_parser), module_name


def test_cli_build_parsers_do_not_crash() -> None:
    for module_name in SCRIPT_MODULES:
        module = importlib.import_module(module_name)
        parser = module.build_parser()

        assert parser.prog
        assert parser.description


def test_ros_mode_script_import_does_not_initialize_rclpy() -> None:
    module = importlib.import_module("scripts.pointcloud_echo_cli")

    import rclpy

    assert isinstance(module, ModuleType)
    assert rclpy.ok() is False


def test_script_main_functions_accept_empty_argument_list() -> None:
    safe_modules = (
        "scripts.mapping_readiness_cli",
        "scripts.mapping_smoke_test_cli",
        "scripts.dump_effective_params",
        "scripts.map_quality_cli",
        "scripts.map_metadata_cli",
        "scripts.save_map_cli",
        "scripts.manual_mapping_cli",
        "scripts.autonomous_mapping_cli",
        "scripts.pointcloud_echo_cli",
        "scripts.voxel_costmap_check_cli",
        "scripts.apriltag_check_cli",
        "scripts.semantic_landmark_cli",
        "scripts.location_bridge_check_cli",
        "scripts.clean_maps_cli",
        "scripts.operator_notes_cli",
    )

    for module_name in safe_modules:
        module = importlib.import_module(module_name)
        result = module.main([])

        assert isinstance(result, int), module_name


def test_semantic_modules_import_without_side_effects() -> None:
    modules = [
        "savo_mapping.semantic",
        "savo_mapping.semantic.apriltag_mapper",
        "savo_mapping.semantic.apriltag_observation",
        "savo_mapping.semantic.human_label_session",
        "savo_mapping.semantic.location_bridge",
        "savo_mapping.semantic.location_candidate",
        "savo_mapping.semantic.location_confirmation",
        "savo_mapping.semantic.location_record",
        "savo_mapping.semantic.semantic_landmark_store",
        "savo_mapping.semantic.tag_database",
    ]

    for module_name in modules:
        module = importlib.import_module(module_name)
        assert module is not None


def test_mapping_node_modules_import_without_side_effects() -> None:
    modules = [
        "savo_mapping.nodes.mapping_supervisor_node",
        "savo_mapping.nodes.mapping_mode_manager_node",
        "savo_mapping.nodes.mapping_dashboard_node",
        "savo_mapping.nodes.frontier_explorer_node",
        "savo_mapping.nodes.location_bridge_node",
        "savo_mapping.nodes.location_confirmation_node",
        "savo_mapping.nodes.apriltag_mapper_node",
        "savo_mapping.nodes.pointcloud_monitor_node",
    ]

    for module_name in modules:
        module = importlib.import_module(module_name)
        assert module is not None
