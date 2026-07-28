# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Launch Robot Savo's baseline saved-map Nav2 stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the saved-map Nav2 launch description."""
    package_share = FindPackageShare('savo_nav')

    default_params = PathJoinSubstitution(
        [
            package_share,
            'config',
            'nav2',
            'saved_map.yaml',
        ]
    )

    default_readiness_params = PathJoinSubstitution(
        [
            package_share,
            'config',
            'readiness.yaml',
        ]
    )

    default_gateway_params = PathJoinSubstitution(
        [
            package_share,
            'config',
            'goal_gateway.yaml',
        ]
    )

    map_yaml = LaunchConfiguration('map')
    map_id = LaunchConfiguration('map_id')

    params_file = LaunchConfiguration('params_file')

    readiness_params = LaunchConfiguration(
        'readiness_params'
    )

    goal_gateway_params = LaunchConfiguration(
        'goal_gateway_params'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    start_readiness = LaunchConfiguration(
        'start_readiness'
    )

    start_goal_gateway = LaunchConfiguration(
        'start_goal_gateway'
    )

    log_level = LaunchConfiguration('log_level')

    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    nav_velocity_remappings = [
        ('cmd_vel', '/cmd_vel_nav'),
    ]

    localization_nodes = [
        'map_server',
        'amcl',
    ]

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    control_recovery_guard = Node(
        package='savo_nav',
        executable='control_recovery_guard_node',
        name='control_recovery_guard_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    package_share,
                    'config',
                    'control_recovery_guard.yaml',
                ]
            )
        ],
    )

    goal_admission_gate = Node(
        package='savo_nav',
        executable='goal_admission_gate_node',
        name='goal_admission_gate_node',
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map',
                default_value='',
                description=(
                    'Absolute path to the saved-map YAML file.'
                ),
            ),
            DeclareLaunchArgument(
                'map_id',
                default_value='saved_map',
                description=(
                    'Released map identity used by goal validation.'
                ),
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description=(
                    'Absolute path to Nav2 parameters.'
                ),
            ),
            DeclareLaunchArgument(
                'readiness_params',
                default_value=default_readiness_params,
                description=(
                    'Absolute path to readiness parameters.'
                ),
            ),
            DeclareLaunchArgument(
                'goal_gateway_params',
                default_value=default_gateway_params,
                description=(
                    'Absolute path to gateway parameters.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the ROS simulation clock.',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='false',
                description=(
                    'Activate lifecycle nodes automatically.'
                ),
            ),
            DeclareLaunchArgument(
                'start_readiness',
                default_value='true',
                description='Start the readiness monitor.',
            ),
            DeclareLaunchArgument(
                'start_goal_gateway',
                default_value='true',
                description='Start the Savo goal gateway.',
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='ROS logging level.',
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        'yaml_filename': map_yaml,
                        'use_sim_time': use_sim_time,
                    },
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=tf_remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=tf_remappings,
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=(
                    tf_remappings
                    + nav_velocity_remappings
                ),
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=tf_remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=(
                    tf_remappings
                    + nav_velocity_remappings
                ),
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=tf_remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=tf_remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': localization_nodes,
                    },
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': navigation_nodes,
                    },
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
            ),
            Node(
                package='savo_nav',
                executable='goal_gateway_node',
                name='goal_gateway_node',
                output='screen',
                emulate_tty=True,
                condition=IfCondition(start_goal_gateway),
                parameters=[
                    goal_gateway_params,
                    {'active_map_id': map_id},
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                remappings=[
                    (
                        '/savo_nav/navigation/navigate_to_pose',
                        '/savo_nav/_internal/navigation/navigate_to_pose',
                    ),
                    (
                        '/savo_nav/exploration/navigate_to_pose',
                        '/savo_nav/_internal/exploration/navigate_to_pose',
                    ),
                ],
            ),
            Node(
                package='savo_nav',
                executable='navigation_readiness_node',
                name='navigation_readiness_node',
                output='screen',
                emulate_tty=True,
                condition=IfCondition(start_readiness),
                parameters=[readiness_params],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
            ),
            control_recovery_guard,
            goal_admission_gate,
        ]
    )
