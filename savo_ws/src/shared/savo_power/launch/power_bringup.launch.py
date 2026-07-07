from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _launch_file(name):
    return PathJoinSubstitution([
        FindPackageShare("savo_power"),
        "launch",
        name,
    ])


def _role_condition(role, target):
    return IfCondition(PythonExpression([
        "'", role, "' == '", target, "' or '", role, "' == 'both'",
    ]))


def generate_launch_description():
    role = LaunchConfiguration("role")
    use_python_fallback = LaunchConfiguration("use_python_fallback")
    enable_edge_power_health = LaunchConfiguration("enable_edge_power_health")
    enable_edge_power_dashboard = LaunchConfiguration("enable_edge_power_dashboard")

    return LaunchDescription([
        DeclareLaunchArgument(
            "role",
            default_value="core",
            description="Power bringup role: core, edge, or both.",
        ),
        DeclareLaunchArgument(
            "use_python_fallback",
            default_value="false",
            description="Run Python fallback nodes instead of C++ production nodes.",
        ),
        DeclareLaunchArgument(
            "enable_edge_power_health",
            default_value="false",
            description="Enable optional edge-side health publisher.",
        ),
        DeclareLaunchArgument(
            "enable_edge_power_dashboard",
            default_value="false",
            description="Enable optional edge-side dashboard publisher.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("power_core.launch.py")),
            launch_arguments={
                "use_python_fallback": use_python_fallback,
            }.items(),
            condition=_role_condition(role, "core"),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("power_edge.launch.py")),
            launch_arguments={
                "use_python_fallback": use_python_fallback,
                "enable_power_health": enable_edge_power_health,
                "enable_power_dashboard": enable_edge_power_dashboard,
            }.items(),
            condition=_role_condition(role, "edge"),
        ),
    ])
