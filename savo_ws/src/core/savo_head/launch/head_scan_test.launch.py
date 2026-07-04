from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_nodes(context):
    backend = LaunchConfiguration("backend").perform(context)
    use_python_fallback = _as_bool(LaunchConfiguration("use_python_fallback").perform(context))
    auto_start = _as_bool(LaunchConfiguration("auto_start").perform(context))
    center_on_start = _as_bool(LaunchConfiguration("center_on_start").perform(context))
    center_on_shutdown = _as_bool(LaunchConfiguration("center_on_shutdown").perform(context))

    executable_suffix = "_py" if use_python_fallback else ""

    return [
        Node(
            package="savo_head",
            executable=f"head_controller_node{executable_suffix}",
            name="head_controller_node",
            output="screen",
            parameters=[
                {
                    "backend": backend,
                    "center_on_start": center_on_start,
                    "center_on_shutdown": center_on_shutdown,
                }
            ],
        ),
        Node(
            package="savo_head",
            executable=f"head_scan_node{executable_suffix}",
            name="head_scan_node",
            output="screen",
            parameters=[
                {
                    "enabled": True,
                    "auto_start": auto_start,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="dryrun",
                description="Use dryrun for PC scan test, pca9685 for robot hardware scan test.",
            ),
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description="Use Python fallback scan/controller nodes instead of C++ defaults.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Start the scan pattern automatically.",
            ),
            DeclareLaunchArgument(
                "center_on_start",
                default_value="false",
                description="Center pan-tilt when controller starts.",
            ),
            DeclareLaunchArgument(
                "center_on_shutdown",
                default_value="false",
                description="Do not move servos on shutdown by default during scan tests.",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
