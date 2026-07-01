from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _select_executable(base_name: str, implementation: str) -> str:
    if implementation == "cpp":
        return base_name

    if implementation == "py":
        return f"{base_name}_py"

    raise RuntimeError(
        f"Unsupported implementation '{implementation}'. Use 'cpp' or 'py'."
    )


def _make_vo_nodes(context):
    implementation = LaunchConfiguration("implementation").perform(context).strip().lower()
    profile = LaunchConfiguration("profile").perform(context).strip()
    log_level = LaunchConfiguration("log_level").perform(context).strip()

    package_share = Path(get_package_share_directory("savo_vo"))
    profile_file = package_share / "config" / "profiles" / f"{profile}.yaml"

    if not profile_file.exists():
        raise RuntimeError(f"savo_vo profile file not found: {profile_file}")

    common_kwargs = {
        "package": "savo_vo",
        "output": "screen",
        "parameters": [str(profile_file)],
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    return [
        Node(
            executable=_select_executable("rgbd_odometry_node", implementation),
            name="rgbd_odometry_node",
            **common_kwargs,
        ),
        Node(
            executable=_select_executable("vo_republisher_node", implementation),
            name="vo_republisher_node",
            **common_kwargs,
        ),
        Node(
            executable=_select_executable("vo_health_node", implementation),
            name="vo_health_node",
            **common_kwargs,
        ),
        Node(
            executable=_select_executable("vo_diagnostics_node", implementation),
            name="vo_diagnostics_node",
            **common_kwargs,
        ),
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
        OpaqueFunction(function=_make_vo_nodes),
    ])
