# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Launch Nav2 while SLAM Toolbox owns the live map and map-to-odom TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the guarded live-mapping navigation launch description."""
    package_share = FindPackageShare('savo_nav')

    default_params = PathJoinSubstitution(
        [package_share, 'config', 'nav2_live_mapping.yaml']
    )
    default_readiness_params = PathJoinSubstitution(
        [package_share, 'config', 'readiness.yaml']
    )
    default_gateway_params = PathJoinSubstitution(
        [package_share, 'config', 'goal_gateway.yaml']
    )
    default_gate_params = PathJoinSubstitution(
        [package_share, 'config', 'goal_admission_gate.yaml']
    )
    default_guard_params = PathJoinSubstitution(
        [package_share, 'config', 'control_recovery_guard.yaml']
    )
    default_navigation_behavior_tree = PathJoinSubstitution(
        [package_share, 'behavior_trees', 'navigate_to_pose.xml']
    )
    default_exploration_behavior_tree = PathJoinSubstitution(
        [package_share, 'behavior_trees', 'exploration_navigation.xml']
    )

    params_file = LaunchConfiguration('params_file')
    readiness_params = LaunchConfiguration('readiness_params')
    goal_gateway_params = LaunchConfiguration('goal_gateway_params')
    goal_admission_gate_params = LaunchConfiguration(
        'goal_admission_gate_params'
    )
    control_recovery_guard_params = LaunchConfiguration(
        'control_recovery_guard_params'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_readiness = LaunchConfiguration('start_readiness')
    start_goal_gateway = LaunchConfiguration('start_goal_gateway')
    log_level = LaunchConfiguration('log_level')

    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    nav_velocity_remappings = [('cmd_vel', '/cmd_vel_nav')]

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    common_arguments = [
        '--ros-args',
        '--log-level',
        log_level,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Absolute path to Nav2 parameters.',
            ),
            DeclareLaunchArgument(
                'readiness_params',
                default_value=default_readiness_params,
                description='Navigation readiness parameters.',
            ),
            DeclareLaunchArgument(
                'goal_gateway_params',
                default_value=default_gateway_params,
                description='Goal gateway parameters.',
            ),
            DeclareLaunchArgument(
                'goal_admission_gate_params',
                default_value=default_gate_params,
                description='Goal-admission gate parameters.',
            ),
            DeclareLaunchArgument(
                'control_recovery_guard_params',
                default_value=default_guard_params,
                description='Control and recovery guard parameters.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the ROS simulation clock.',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='true',
                description='Activate Nav2 lifecycle nodes automatically.',
            ),
            DeclareLaunchArgument(
                'start_readiness',
                default_value='true',
                description='Start the readiness monitor.',
            ),
            DeclareLaunchArgument(
                'start_goal_gateway',
                default_value='true',
                description='Start the internal guarded goal gateway.',
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='ROS logging level.',
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
                arguments=common_arguments,
                remappings=tf_remappings + nav_velocity_remappings,
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
                arguments=common_arguments,
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
                arguments=common_arguments,
                remappings=tf_remappings + nav_velocity_remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'default_nav_to_pose_bt_xml':
                            default_navigation_behavior_tree,
                    },
                ],
                arguments=common_arguments,
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
                arguments=common_arguments,
                remappings=tf_remappings,
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
                arguments=common_arguments,
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
                    {
                        'map_mode': 'live_mapping',
                        'active_map_id': '',
                        'navigation_behavior_tree':
                            default_navigation_behavior_tree,
                        'exploration_behavior_tree':
                            default_exploration_behavior_tree,
                    },
                ],
                arguments=common_arguments,
                remappings=[
                    (
                        '/savo_nav/navigation/navigate_to_pose',
                        '/savo_nav/_internal/navigation/navigate_to_pose',
                    ),
                    (
                        '/savo_nav/exploration/navigate_to_pose',
                        '/savo_nav/_internal/exploration/navigate_to_pose',
                    ),
                    (
                        '/savo_nav/coverage/execute_path',
                        '/savo_nav/_internal/coverage/execute_path',
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
                arguments=common_arguments,
            ),
            Node(
                package='savo_nav',
                executable='control_recovery_guard_node',
                name='control_recovery_guard_node',
                output='screen',
                parameters=[control_recovery_guard_params],
            ),
            Node(
                package='savo_nav',
                executable='goal_admission_gate_node',
                name='goal_admission_gate_node',
                output='screen',
                parameters=[goal_admission_gate_params],
            ),
        ]
    )
