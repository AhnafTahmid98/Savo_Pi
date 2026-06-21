#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package metadata and install-contract tests."""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET

from savo_control.version import VERSION


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_package_xml_identity_and_build_type():
    root = ET.fromstring(read_text("package.xml"))

    assert root.findtext("name") == "savo_control"
    assert root.findtext("version") == VERSION
    assert root.findtext("license") == "Proprietary"

    build_type = root.find("./export/build_type")
    assert build_type is not None
    assert build_type.text == "ament_cmake"


def test_package_xml_core_dependencies():
    root = ET.fromstring(read_text("package.xml"))
    deps = {element.text for element in root.findall("depend")}

    required = {
        "rclcpp",
        "rclpy",
        "std_msgs",
        "geometry_msgs",
        "nav_msgs",
        "builtin_interfaces",
        "savo_msgs",
        "tf2",
        "tf2_ros",
        "tf2_geometry_msgs",
    }

    assert required.issubset(deps)


def test_setup_py_matches_hybrid_package_contract():
    text = read_text("setup.py")

    assert 'package_name = "savo_control"' in text
    assert 'version="0.1.0"' in text
    assert '"console_scripts": []' in text

    # setup.py installs Python modules only; ROS executables are installed by CMake.
    assert "distance_pid_test_node.py" not in text
    assert "keyboard_teleop_node.py" not in text
    assert "control_smoke_test_cli.py" not in text


def test_python_support_package_layout_exists():
    required_files = [
        "savo_control/__init__.py",
        "savo_control/version.py",
        "savo_control/constants.py",

        "savo_control/models/__init__.py",
        "savo_control/models/control_mode.py",
        "savo_control/models/command_source.py",
        "savo_control/models/twist_command.py",
        "savo_control/models/distance_approach.py",
        "savo_control/models/control_status.py",
        "savo_control/models/recovery_state.py",
        "savo_control/models/stuck_state.py",

        "savo_control/ros/__init__.py",
        "savo_control/ros/topic_names.py",
        "savo_control/ros/param_utils.py",
        "savo_control/ros/qos_profiles.py",
        "savo_control/ros/message_utils.py",

        "savo_control/interfaces/__init__.py",
        "savo_control/interfaces/control_modes.py",

        "savo_control/adapters/__init__.py",
        "savo_control/adapters/twist_adapter.py",
        "savo_control/adapters/status_adapter.py",
        "savo_control/adapters/config_adapter.py",

        "savo_control/diagnostics/__init__.py",
        "savo_control/diagnostics/report_formatter.py",
        "savo_control/diagnostics/health.py",
        "savo_control/diagnostics/topic_probe.py",

        "savo_control/utils/__init__.py",
        "savo_control/utils/math_utils.py",
        "savo_control/utils/time_utils.py",
        "savo_control/utils/validation.py",
        "savo_control/utils/file_utils.py",

        "savo_control/nodes/control_status_helpers.py",
        "savo_control/nodes/keyboard_teleop_helpers.py",
        "savo_control/nodes/auto_test_helpers.py",
        "savo_control/nodes/recovery_test_helpers.py",
        "savo_control/nodes/straight_line_test_helpers.py",
    ]

    missing = [name for name in required_files if not (ROOT / name).is_file()]
    assert not missing, f"Missing Python support files: {missing}"


def test_cmake_registers_cpp_production_nodes():
    text = read_text("CMakeLists.txt")

    cpp_nodes = [
        "twist_mux_node",
        "cmd_vel_shaper_node",
        "control_mode_manager_node",
        "heading_pid_node",
        "rotate_to_heading_node",
        "distance_approach_node",
        "velocity_test_pattern_node",
        "recovery_manager_node",
        "backup_escape_node",
        "stuck_detector_node",
    ]

    for node in cpp_nodes:
        pattern = rf"savo_add_node\(\s*{re.escape(node)}\s+"
        assert re.search(pattern, text), f"Missing C++ node registration: {node}"

    assert "ament_python_install_package(${PROJECT_NAME})" in text


def test_cmake_installs_python_fallback_and_diagnostic_nodes():
    text = read_text("CMakeLists.txt")

    python_nodes = [
        "auto_test_manager_node.py",
        "control_dashboard_node.py",
        "control_status_node.py",
        "keyboard_teleop_node.py",
        "recovery_status_node.py",
        "recovery_test_manager_node.py",
        "straight_line_pid_test_node.py",
        "distance_pid_test_node.py",
    ]

    for node in python_nodes:
        assert node in text, f"Missing Python node install entry: {node}"


def test_cmake_installs_cli_scripts_as_ros_executables():
    text = read_text("CMakeLists.txt")

    cli_scripts = [
        "scripts/control_topic_check_cli.py",
        "scripts/mode_cmd_cli.py",
        "scripts/control_smoke_test_cli.py",
    ]

    for script in cli_scripts:
        assert script in text, f"Missing CLI script install entry: {script}"

    assert "DESTINATION lib/${PROJECT_NAME}" in text
    assert "DESTINATION share/${PROJECT_NAME}/scripts" in text


def test_cmake_registers_source_level_tests():
    text = read_text("CMakeLists.txt")

    tests = [
        "test_package_identity",
        "test_package_metadata",
        "test_launch_files",
        "test_config_files",
        "test_control_modes_interface",

        "test_control_mode_model",
        "test_command_source_model",
        "test_twist_command_model",
        "test_distance_approach_model",
        "test_distance_pid_controller",
        "test_heading_pid_controller",
        "test_pid_controller",
        "test_control_status_node_helpers",
        "test_recovery_status_node_compile",
        "test_control_dashboard_node_compile",
        "test_distance_pid_test_node_compile",
        "test_keyboard_teleop_helpers",
        "test_keyboard_teleop_node_compile",
        "test_auto_test_helpers",
        "test_auto_test_manager_node_compile",
        "test_recovery_test_helpers",
        "test_recovery_test_manager_node_compile",
        "test_straight_line_test_helpers",
        "test_straight_line_pid_test_node",
        "test_control_status_model",
        "test_recovery_state_model",
        "test_stuck_state_model",

        "test_topic_names",
        "test_param_utils",
        "test_qos_profiles",
        "test_message_utils",

        "test_math_utils",
        "test_time_utils",
        "test_validation_utils",
        "test_file_utils",

        "test_diagnostics_helpers",
        "test_health_helpers",
        "test_topic_probe",

        "test_twist_adapter",
        "test_status_adapter",
        "test_config_adapter",

        "test_scripts",
    ]

    for test_name in tests:
        assert f"add_pytest_if_exists({test_name} " in text, (
            f"Missing CMake test registration: {test_name}"
        )


def test_distance_approach_launch_supports_cpp_and_python():
    text = read_text("launch/distance_approach.launch.py")

    assert "approach_impl" in text
    assert "distance_approach_node" in text
    assert "distance_pid_test_node.py" in text
    assert "distance_approach.yaml" in text
    assert "control_mode_manager_node" in text
    assert "twist_mux_node" in text
    assert "cmd_vel_shaper_node" in text


def test_distance_approach_config_has_hybrid_sections():
    text = read_text("config/distance_approach.yaml")

    assert "distance_approach_node:" in text
    assert "distance_pid_test_node:" in text
    assert "enable_topic:" in text
    assert "target_topic:" in text


def test_core_installed_directories_are_declared():
    text = read_text("CMakeLists.txt")

    assert "install(DIRECTORY launch" in text
    assert "install(DIRECTORY config" in text
    assert "install(DIRECTORY include/" in text
