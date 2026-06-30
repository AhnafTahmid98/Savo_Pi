#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _build_nodes(context, *args, **kwargs):
    package_name = "savo_perception"

    driver_impl = LaunchConfiguration("driver_impl").perform(context).strip().lower()
    use_tof = _as_bool(LaunchConfiguration("use_tof").perform(context))
    use_ultrasonic = _as_bool(LaunchConfiguration("use_ultrasonic").perform(context))
    use_range_health = _as_bool(LaunchConfiguration("use_range_health").perform(context))
    use_dashboard = _as_bool(LaunchConfiguration("use_dashboard").perform(context))
    config_file = LaunchConfiguration("config_file")

    if driver_impl not in {"cpp", "py"}:
        raise RuntimeError("driver_impl must be either 'cpp' or 'py'")

    if driver_impl == "cpp":
        vl53_executable = "vl53_mux_node"
        vl53_node_name = "vl53_mux_node"

        ultrasonic_executable = "ultrasonic_node"
        ultrasonic_node_name = "ultrasonic_node"

        range_health_executable = "range_health_node"
        range_health_node_name = "range_health_node"
    else:
        vl53_executable = "vl53_mux_node_py"
        vl53_node_name = "vl53_mux_node_py"

        ultrasonic_executable = "ultrasonic_node_py"
        ultrasonic_node_name = "ultrasonic_node_py"

        range_health_executable = "range_health_node_py"
        range_health_node_name = "range_health_node_py"

    nodes = []

    if use_tof:
        nodes.append(
            Node(
                package=package_name,
                executable=vl53_executable,
                name=vl53_node_name,
                output="screen",
                parameters=[config_file],
            )
        )

    if use_ultrasonic:
        nodes.append(
            Node(
                package=package_name,
                executable=ultrasonic_executable,
                name=ultrasonic_node_name,
                output="screen",
                parameters=[config_file],
            )
        )

    if use_range_health:
        nodes.append(
            Node(
                package=package_name,
                executable=range_health_executable,
                name=range_health_node_name,
                output="screen",
                parameters=[config_file],
            )
        )

    if use_dashboard:
        nodes.append(
            Node(
                package=package_name,
                executable="sensor_dashboard_node",
                name="sensor_dashboard_node",
                output="screen",
                parameters=[config_file],
            )
        )

    return nodes


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_perception"),
            "config",
            "core",
            "perception_core.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_impl",
                default_value="cpp",
                description="Range sensor implementation: cpp or py. C++ is production default.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML parameter file for range sensor nodes.",
            ),
            DeclareLaunchArgument(
                "use_tof",
                default_value="true",
                description="Start VL53L1X ToF mux node.",
            ),
            DeclareLaunchArgument(
                "use_ultrasonic",
                default_value="true",
                description="Start front ultrasonic range node.",
            ),
            DeclareLaunchArgument(
                "use_range_health",
                default_value="true",
                description="Start range health monitor node.",
            ),
            DeclareLaunchArgument(
                "use_dashboard",
                default_value="false",
                description="Start Python sensor dashboard node.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )