from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_diagnostics_node(context):
    implementation = LaunchConfiguration("implementation").perform(context).strip().lower()
    profile = LaunchConfiguration("profile").perform(context).strip()
    log_level = LaunchConfiguration("log_level").perform(context).strip()

    package_share = Path(get_package_share_directory("savo_vo"))
    profile_file = package_share / "config" / "profiles" / f"{profile}.yaml"

    executable = "vo_diagnostics_node"
    if implementation == "py":
        executable = "vo_diagnostics_node_py"
    elif implementation != "cpp":
        raise RuntimeError(
            f"Unsupported implementation '{implementation}'. Use 'cpp' or 'py'."
        )

    if not profile_file.exists():
        raise RuntimeError(f"savo_vo profile file not found: {profile_file}")

    return [
        Node(
            package="savo_vo",
            executable=executable,
            name="vo_diagnostics_node",
            output="screen",
            parameters=[str(profile_file)],
            arguments=["--ros-args", "--log-level", log_level],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "implementation",
            default_value="cpp",
            description="Node implementation: cpp or py.",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value="real_robot_v1",
            description="Config profile from config/profiles without .yaml.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level.",
        ),
        OpaqueFunction(function=_make_diagnostics_node),
    ])
