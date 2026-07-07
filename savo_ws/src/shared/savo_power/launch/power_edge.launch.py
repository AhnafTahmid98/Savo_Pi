from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cfg(*parts):
    return PathJoinSubstitution([
        FindPackageShare("savo_power"),
        "config",
        *parts,
    ])


def _enabled_with_cpp(enabled, use_python_fallback):
    return IfCondition(PythonExpression([
        "'", enabled, "' == 'true' and '", use_python_fallback, "' == 'false'",
    ]))


def _enabled_with_python(enabled, use_python_fallback):
    return IfCondition(PythonExpression([
        "'", enabled, "' == 'true' and '", use_python_fallback, "' == 'true'",
    ]))


def generate_launch_description():
    use_python_fallback = LaunchConfiguration("use_python_fallback")
    enable_power_health = LaunchConfiguration("enable_power_health")
    enable_power_dashboard = LaunchConfiguration("enable_power_dashboard")

    common_params = _cfg("power_common.yaml")
    topics_params = _cfg("topics.yaml")
    profile_params = _cfg("profiles", "edge_real_robot_v1.yaml")

    edge_ups_params = _cfg("edge", "edge_ups.yaml")
    health_params = _cfg("edge", "power_health.yaml")
    dashboard_params = _cfg("edge", "dashboard.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_python_fallback",
            default_value="false",
            description="Run Python fallback nodes instead of C++ production nodes.",
        ),
        DeclareLaunchArgument(
            "enable_power_health",
            default_value="false",
            description="Optionally run edge-side power health publisher.",
        ),
        DeclareLaunchArgument(
            "enable_power_dashboard",
            default_value="false",
            description="Optionally run edge-side power dashboard publisher.",
        ),

        Node(
            package="savo_power",
            executable="edge_ups_node",
            name="edge_ups_node",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                edge_ups_params,
            ],
            condition=UnlessCondition(use_python_fallback),
        ),
        Node(
            package="savo_power",
            executable="edge_ups_node_py",
            name="edge_ups_node_py",
            output="screen",
            parameters=[
                common_params,
                topics_params,
                profile_params,
                edge_ups_params,
            ],
            condition=IfCondition(use_python_fallback),
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
            condition=_enabled_with_cpp(enable_power_health, use_python_fallback),
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
            condition=_enabled_with_python(enable_power_health, use_python_fallback),
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
            condition=_enabled_with_cpp(enable_power_dashboard, use_python_fallback),
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
            condition=_enabled_with_python(enable_power_dashboard, use_python_fallback),
        ),
    ])
