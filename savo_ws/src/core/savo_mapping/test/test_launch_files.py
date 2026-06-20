#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch-file tests for Robot Savo mapping.

These tests are intentionally mostly static.

Reason:
- launch files should be safe to inspect from the source tree
- tests should not require a live ROS graph
- tests should not start robot nodes
- tests should not require RViz
- tests should not require the package to be installed before parsing
"""

from __future__ import annotations

import ast
import re
from pathlib import Path


# =============================================================================
# Paths
# =============================================================================
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


# =============================================================================
# Expected launch files
# =============================================================================
EXPECTED_LAUNCH_FILES = (
    "manual_mapping.launch.py",
    "autonomous_mapping.launch.py",
    "mapping_save.launch.py",
    "mapping_rviz.launch.py",
)

ROBOT_RUNTIME_LAUNCH_FILES = (
    "manual_mapping.launch.py",
    "autonomous_mapping.launch.py",
)

PC_VISUALIZATION_LAUNCH_FILES = (
    "mapping_rviz.launch.py",
)


# =============================================================================
# Helpers
# =============================================================================
def launch_file_path(name: str) -> Path:
    return LAUNCH_DIR / name


def read_launch_file(name: str) -> str:
    return launch_file_path(name).read_text(encoding="utf-8")


def parse_launch_file(name: str) -> ast.Module:
    return ast.parse(read_launch_file(name), filename=str(launch_file_path(name)))


def function_names(tree: ast.Module) -> set[str]:
    return {
        node.name
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }


def assigned_names(tree: ast.Module) -> set[str]:
    names: set[str] = set()

    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue

        for target in node.targets:
            if isinstance(target, ast.Name):
                names.add(target.id)

    return names


def has_launch_argument(text: str, name: str) -> bool:
    patterns = (
        f'DeclareLaunchArgument("{name}"',
        f"DeclareLaunchArgument('{name}'",
        f'name="{name}"',
        f"name='{name}'",
    )

    return any(pattern in text for pattern in patterns)


def has_default_false_for_argument(text: str, name: str) -> bool:
    # Flexible static check. This accepts the common Robot Savo launch style:
    # DeclareLaunchArgument("use_rviz", default_value="false", ...)
    # or default_value='false'.
    argument_blocks = re.findall(
        rf"DeclareLaunchArgument\(\s*[\"']{re.escape(name)}[\"'].*?\)",
        text,
        flags=re.DOTALL,
    )

    for block in argument_blocks:
        if "default_value" not in block:
            continue

        if '"false"' in block or "'false'" in block:
            return True

    return False


def contains_any(text: str, values: tuple[str, ...]) -> bool:
    return any(value in text for value in values)


# =============================================================================
# Directory / file existence
# =============================================================================
def test_launch_directory_exists() -> None:
    assert LAUNCH_DIR.exists()
    assert LAUNCH_DIR.is_dir()


def test_expected_launch_files_exist() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        path = launch_file_path(launch_file)

        assert path.exists(), launch_file
        assert path.is_file(), launch_file


def test_launch_files_are_python_files() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        assert launch_file.endswith(".launch.py")


# =============================================================================
# Python structure
# =============================================================================
def test_launch_files_parse_as_python() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        tree = parse_launch_file(launch_file)

        assert isinstance(tree, ast.Module), launch_file


def test_launch_files_define_generate_launch_description() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        tree = parse_launch_file(launch_file)

        assert "generate_launch_description" in function_names(tree), launch_file


def test_launch_files_import_launch_description() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert "LaunchDescription" in text, launch_file


def test_launch_files_do_not_execute_main_logic_at_import_time() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert 'if __name__ == "__main__"' not in text, launch_file
        assert "rclpy.init" not in text, launch_file


# =============================================================================
# Runtime launch arguments
# =============================================================================
def test_manual_mapping_launch_arguments() -> None:
    text = read_launch_file("manual_mapping.launch.py")

    expected_args = (
        "profile",
        "driver_impl",
        "use_rviz",
        "map_name",
    )

    for arg in expected_args:
        assert has_launch_argument(text, arg), arg


def test_autonomous_mapping_launch_arguments() -> None:
    text = read_launch_file("autonomous_mapping.launch.py")

    expected_args = (
        "profile",
        "driver_impl",
        "use_rviz",
        "use_pointcloud",
        "map_name",
    )

    for arg in expected_args:
        assert has_launch_argument(text, arg), arg


def test_mapping_save_launch_arguments() -> None:
    text = read_launch_file("mapping_save.launch.py")

    expected_args = (
        "map_name",
        "maps_dir",
    )

    for arg in expected_args:
        assert has_launch_argument(text, arg), arg


def test_mapping_rviz_launch_arguments() -> None:
    text = read_launch_file("mapping_rviz.launch.py")

    expected_args = (
        "rviz_config",
    )

    for arg in expected_args:
        assert has_launch_argument(text, arg), arg


# =============================================================================
# Headless Pi rule
# =============================================================================
def test_robot_runtime_launch_files_default_to_no_rviz() -> None:
    for launch_file in ROBOT_RUNTIME_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert has_launch_argument(text, "use_rviz"), launch_file
        assert has_default_false_for_argument(text, "use_rviz"), launch_file


def test_robot_runtime_launch_files_do_not_force_rviz() -> None:
    for launch_file in ROBOT_RUNTIME_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        forbidden_patterns = (
            'package="rviz2"',
            "package='rviz2'",
            'executable="rviz2"',
            "executable='rviz2'",
        )

        # Robot runtime launches may contain optional RViz conditions later,
        # but they must not directly force RViz without the use_rviz switch.
        if contains_any(text, forbidden_patterns):
            assert "use_rviz" in text
            assert "IfCondition" in text or "UnlessCondition" in text


def test_mapping_rviz_launch_is_separate_visualization_launch() -> None:
    text = read_launch_file("mapping_rviz.launch.py")

    assert contains_any(
        text,
        (
            'package="rviz2"',
            "package='rviz2'",
            'executable="rviz2"',
            "executable='rviz2'",
        ),
    )


# =============================================================================
# Robot Savo launch ownership
# =============================================================================
def test_manual_mapping_launch_mentions_slam_toolbox_or_mapping_stack() -> None:
    text = read_launch_file("manual_mapping.launch.py")

    assert contains_any(
        text,
        (
            "slam_toolbox",
            "online_async_launch.py",
            "async_slam_toolbox_node",
            "sync_slam_toolbox_node",
        ),
    )


def test_autonomous_mapping_launch_mentions_frontier_or_nav2_stack() -> None:
    text = read_launch_file("autonomous_mapping.launch.py")

    assert contains_any(
        text,
        (
            "frontier",
            "NavigateToPose",
            "nav2",
            "bt_navigator",
            "planner_server",
        ),
    )


def test_mapping_save_launch_mentions_map_saver_or_slam_toolbox_save() -> None:
    text = read_launch_file("mapping_save.launch.py")

    assert contains_any(
        text,
        (
            "map_saver",
            "map_saver_cli",
            "slam_toolbox",
            "serialize_map",
            "save_map",
        ),
    )


# =============================================================================
# Config / share usage
# =============================================================================
def test_launch_files_use_package_share_or_launch_substitutions() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert contains_any(
            text,
            (
                "FindPackageShare",
                "get_package_share_directory",
                "PathJoinSubstitution",
                "LaunchConfiguration",
            ),
        ), launch_file


def test_launch_files_do_not_hardcode_user_home_paths() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        forbidden = (
            "/home/ahnaf",
            "/home/savo",
            "~/Savo_Pi",
            "/Users/",
        )

        for value in forbidden:
            assert value not in text, f"{launch_file} hardcodes {value}"


def test_launch_files_do_not_start_motors_directly() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        forbidden = (
            "base_driver_node",
            "base_driver_node_py",
            "pwm_sweep",
            "wheel_direction",
        )

        for value in forbidden:
            assert value not in text, f"{launch_file} should not directly start {value}"


# =============================================================================
# Static style checks
# =============================================================================
def test_launch_files_have_robot_savo_header_comment_or_docstring() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert "Robot Savo" in text, launch_file


def test_launch_files_use_generate_launch_description_return() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        text = read_launch_file(launch_file)

        assert "return LaunchDescription" in text, launch_file


def test_launch_files_are_reasonably_small() -> None:
    for launch_file in EXPECTED_LAUNCH_FILES:
        lines = read_launch_file(launch_file).splitlines()

        assert len(lines) < 350, f"{launch_file} is too large; split helper logic."