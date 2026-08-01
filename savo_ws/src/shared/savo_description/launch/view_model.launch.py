from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    description_launch = PathJoinSubstitution(
        [
            FindPackageShare("savo_observer"),
            "launch",
            "description.launch.py",
        ]
    )

    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_description"),
            "rviz",
            "robot_model.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("savo_observer"),
                            "launch",
                            "rviz_observer.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "view": "robot_model",
                    "rviz_config": rviz_config,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
