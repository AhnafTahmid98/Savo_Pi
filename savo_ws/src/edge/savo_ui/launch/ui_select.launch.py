# Copyright 2026 Ahnaf Tahmid
"""Non-destructive selector for the classic and v2 Savo UI implementations."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    pkg_share = Path(get_package_share_directory('savo_ui'))
    ui_variant = LaunchConfiguration('ui_variant').perform(context).strip().lower()
    profile = LaunchConfiguration('profile').perform(context).strip().lower()
    page_mode = LaunchConfiguration('page_mode').perform(context).strip().lower()

    if ui_variant not in ('classic', 'v2'):
        raise RuntimeError("ui_variant must be 'classic' or 'v2'")

    if ui_variant == 'classic':
        launch_file = pkg_share / 'launch' / 'ui_bringup.launch.py'
        launch_arguments = {'profile': profile}
    else:
        launch_file = pkg_share / 'launch' / 'ui_v2_bringup.launch.py'
        launch_arguments = {
            'profile': profile,
            'page_mode': page_mode,
        }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ui_variant',
            default_value='classic',
            description='Savo UI implementation: classic or v2',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='pi',
            description='Runtime profile passed to the selected UI: pi or dryrun',
        ),
        DeclareLaunchArgument(
            'page_mode',
            default_value='auto',
            description='v2 page arbitration: auto, voice, or navigation',
        ),
        OpaqueFunction(function=launch_setup),
    ])
