"""Compatibility entry point for the read-only observer HTTP endpoint."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Delegate to the canonical dashboard observer."""
    launch_file = (
        Path(get_package_share_directory('savo_observer'))
        / 'launch'
        / 'dashboard_observer.launch.py'
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))]
    )
