from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory('savo_ui'))
    config_file = pkg_share / 'config' / 'ui.yaml'
    asset_root = pkg_share / 'assets'

    profile = LaunchConfiguration('profile').perform(context).strip().lower()

    if profile not in ['pi', 'dryrun']:
        raise RuntimeError(
            f"Unsupported savo_ui profile '{profile}'. Use profile:=pi or profile:=dryrun."
        )

    if profile == 'pi':
        enable_framebuffer = True
        enable_touch = False
        export_preview_frames = False
    else:
        enable_framebuffer = False
        enable_touch = False
        export_preview_frames = True

    return [
        Node(
            package='savo_ui',
            executable='ui_node',
            name='savo_ui_node',
            output='screen',
            parameters=[
                str(config_file),
                {
                    'asset_root': str(asset_root),
                    'enable_framebuffer': enable_framebuffer,
                    'enable_touch': enable_touch,
                    'export_preview_frames': export_preview_frames,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='pi',
            description='savo_ui runtime profile: pi or dryrun',
        ),
        OpaqueFunction(function=launch_setup),
    ])
