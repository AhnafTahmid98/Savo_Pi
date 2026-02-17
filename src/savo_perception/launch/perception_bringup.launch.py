# File: Savo_Pi/src/savo_perception/launch/perception_bringup.launch.py
#
# Robot Savo — Perception Bringup (professional, one-command)
#
# What it starts (optional flags):
#   - RealSense driver (realsense2_camera)  [optional]
#   - depth_front_min_node                  [optional, depends on RealSense topics]
#   - vl53_node
#   - ultrasonic_node
#   - safety_stop_node
#   - cmd_vel_safety_gate (C++ node)
#
# Run:
#   source /opt/ros/jazzy/setup.bash
#   source ~/Savo_Pi/install/setup.bash
#   ros2 launch savo_perception perception_bringup.launch.py
#
# Common variants:
#   # run everything (recommended)
#   ros2 launch savo_perception perception_bringup.launch.py use_realsense:=true
#
#   # run without RealSense (ToF + ultrasonic + safety still work, depth node disabled)
#   ros2 launch savo_perception perception_bringup.launch.py use_realsense:=false
#
# Notes:
#   - Your depth node subscribes to: /camera/camera/depth/image_rect_raw
#   - Those topics only exist while realsense2_camera is running.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("savo_perception")

    # ---------------------------
    # Launch arguments
    # ---------------------------
    use_realsense = LaunchConfiguration("use_realsense")
    use_depth_node = LaunchConfiguration("use_depth_node")
    use_vl53 = LaunchConfiguration("use_vl53")
    use_ultrasonic = LaunchConfiguration("use_ultrasonic")
    use_safety_stop = LaunchConfiguration("use_safety_stop")
    use_cmd_gate = LaunchConfiguration("use_cmd_gate")

    # RealSense profile switches (keep safe defaults for Pi)
    rs_enable_depth = LaunchConfiguration("rs_enable_depth")
    rs_enable_color = LaunchConfiguration("rs_enable_color")
    rs_enable_infra1 = LaunchConfiguration("rs_enable_infra1")
    rs_enable_infra2 = LaunchConfiguration("rs_enable_infra2")
    rs_pointcloud = LaunchConfiguration("rs_pointcloud")
    rs_align_depth = LaunchConfiguration("rs_align_depth")

    # Config files (your locked baselines)
    depth_yaml = PathJoinSubstitution([pkg_share, "config", "depth_front.yaml"])
    safety_yaml = PathJoinSubstitution([pkg_share, "config", "range_safety.yaml"])

    # RealSense launch file path
    realsense_launch = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    # ---------------------------
    # RealSense driver (optional)
    # ---------------------------
    realsense_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    # Keep resource usage sane on Pi 5
                    "enable_depth": rs_enable_depth,
                    "enable_color": rs_enable_color,
                    "enable_infra1": rs_enable_infra1,
                    "enable_infra2": rs_enable_infra2,
                    "pointcloud.enable": rs_pointcloud,
                    "align_depth.enable": rs_align_depth,
                }.items(),
            )
        ],
        condition=IfCondition(use_realsense),
    )

    # ---------------------------
    # Nodes (all optional toggles)
    # ---------------------------
    depth_front_min_node = Node(
        package="savo_perception",
        executable="depth_front_min_node",
        name="depth_front_min_node",
        output="screen",
        parameters=[depth_yaml],
        condition=IfCondition(use_depth_node),
    )

    vl53_node = Node(
        package="savo_perception",
        executable="vl53_node",
        name="vl53_node",
        output="screen",
        # uses node defaults; override here if needed
        # parameters=[{"rate_hz": 25.0, "median_n": 5}],
        condition=IfCondition(use_vl53),
    )

    ultrasonic_node = Node(
        package="savo_perception",
        executable="ultrasonic_node",
        name="ultrasonic_node",
        output="screen",
        # parameters=[{"rate_hz": 15.0}],
        condition=IfCondition(use_ultrasonic),
    )

    safety_stop_node = Node(
        package="savo_perception",
        executable="safety_stop_node",
        name="safety_stop_node",
        output="screen",
        parameters=[safety_yaml],
        condition=IfCondition(use_safety_stop),
    )

    cmd_vel_safety_gate = Node(
        package="savo_perception",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate",
        output="screen",
        # You can override topics here if you later add a mux:
        # parameters=[{"cmd_in_topic": "/cmd_vel", "cmd_out_topic": "/cmd_vel_safe"}],
        condition=IfCondition(use_cmd_gate),
    )

    # ---------------------------
    # LaunchDescription
    # ---------------------------
    return LaunchDescription(
        [
            # Primary toggles
            DeclareLaunchArgument(
                "use_realsense",
                default_value="true",
                description="Start realsense2_camera driver (publishes /camera/camera/depth/image_rect_raw).",
            ),
            DeclareLaunchArgument(
                "use_depth_node",
                default_value="true",
                description="Start depth_front_min_node (requires RealSense depth topic).",
            ),
            DeclareLaunchArgument(
                "use_vl53",
                default_value="true",
                description="Start vl53_node (publishes /savo_perception/range/left_m and right_m).",
            ),
            DeclareLaunchArgument(
                "use_ultrasonic",
                default_value="true",
                description="Start ultrasonic_node (publishes /savo_perception/range/front_ultrasonic_m).",
            ),
            DeclareLaunchArgument(
                "use_safety_stop",
                default_value="true",
                description="Start safety_stop_node (publishes /safety/stop and /safety/slowdown_factor).",
            ),
            DeclareLaunchArgument(
                "use_cmd_gate",
                default_value="true",
                description="Start cmd_vel_safety_gate (sub /cmd_vel, pub /cmd_vel_safe).",
            ),
            # RealSense fine controls (defaults match your working command)
            DeclareLaunchArgument("rs_enable_depth", default_value="true"),
            DeclareLaunchArgument("rs_enable_color", default_value="false"),
            DeclareLaunchArgument("rs_enable_infra1", default_value="false"),
            DeclareLaunchArgument("rs_enable_infra2", default_value="false"),
            DeclareLaunchArgument("rs_pointcloud", default_value="false"),
            DeclareLaunchArgument("rs_align_depth", default_value="false"),
            # Actions
            realsense_group,
            depth_front_min_node,
            vl53_node,
            ultrasonic_node,
            safety_stop_node,
            cmd_vel_safety_gate,
        ]
    )
