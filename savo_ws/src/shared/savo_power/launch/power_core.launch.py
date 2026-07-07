from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cfg(*parts):
    return PathJoinSubstitution([
        FindPackageShare("savo_power"),
        "config",
        *parts,
    ])


def generate_launch_description():
    use_python_fallback = LaunchConfiguration("use_python_fallback")

    common_params = _cfg("power_common.yaml")
    topics_params = _cfg("topics.yaml")
    profile_params = _cfg("profiles", "core_real_robot_v1.yaml")

    core_ups_params = _cfg("core", "core_ups.yaml")
    kit_battery_params = _cfg("core", "kit_battery.yaml")
    aggregator_params = _cfg("core", "power_aggregator.yaml")
    health_params = _cfg("core", "power_health.yaml")
    dashboard_params = _cfg("core", "dashboard.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_python_fallback",
            default_value="false",
            description="Run Python fallback nodes instead of C++ production nodes.",
        ),

        Node(
            package="savo_power",
            executable="core_ups_node",
            name="core_ups_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                core_ups_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="base_battery_node",
            name="base_battery_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                kit_battery_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_aggregator_node",
            name="power_aggregator_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                aggregator_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_health_node",
            name="power_health_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                health_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_dashboard_node",
            name="power_dashboard_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                dashboard_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),

        Node(
            package="savo_power",
            executable="core_ups_node_py",
            name="core_ups_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                core_ups_params,
            ],
            condition=IfCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="base_battery_node_py",
            name="base_battery_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                kit_battery_params,
            ],
            condition=IfCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_aggregator_node_py",
            name="power_aggregator_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                aggregator_params,
            ],
            condition=IfCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_health_node_py",
            name="power_health_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                health_params,
            ],
            condition=IfCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="power_dashboard_node_py",
            name="power_dashboard_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                dashboard_params,
            ],
            condition=IfCondition(use_python_fallback),
        ),
    ])
