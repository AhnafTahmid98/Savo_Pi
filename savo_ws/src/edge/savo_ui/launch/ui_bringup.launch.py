from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('savo_ui'))
    config_file = pkg_share / 'config' / 'ui.yaml'
    asset_root = pkg_share / 'assets'

    return LaunchDescription([
        Node(
            package='savo_ui',
            executable='ui_node',
            name='savo_ui_node',
            output='screen',
            parameters=[
                str(config_file),
                {'asset_root': str(asset_root)},
            ],
        )
    ])
