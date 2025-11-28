#!/usr/bin/env python3
"""
Robot Savo — UI bringup launch file

This launch file starts the Robot Savo UI stack on the Raspberry Pi:

Nodes:
------
1) display_manager_node (required)
   - Package:  savo_ui
   - Executable: display_manager_node.py
   - Name: savo_ui_display
   - Parameters:
       - config/ui_params.yaml (screen layout, colors, fonts, nav view)

2) ui_mode_router_node (optional, enabled by default)
   - Package:  savo_ui
   - Executable: ui_mode_router_node.py
   - Name: savo_ui_mode_router
   - Parameters:
       - robot_id
       - mapping_active_topic
       - idle_status_text
       - mapping_status_text
       - stopped_status_text
       - status_from_reply

3) ui_debug_node (optional, disabled by default)
   - Package:  savo_ui
   - Executable: ui_debug_node.py
   - Name: savo_ui_debug
   - Parameters:
       - robot_id
       - cycle_interval_s
       - enable_fake_mouth
       - mouth_wave_freq_hz

Usage examples:
---------------
# Basic UI bringup (display + mode router)
ros2 launch savo_ui savo_ui_bringup.launch.py \
  robot_id:=robot_savo_pi

# UI bringup without mode router (you publish /savo_ui/mode yourself)
ros2 launch savo_ui savo_ui_bringup.launch.py \
  robot_id:=robot_savo_pi \
  use_mode_router:=false

# UI bringup with debug driver (no full robot stack needed)
ros2 launch savo_ui savo_ui_bringup.launch.py \
  robot_id:=robot_savo_pi \
  use_debug:=true \
  use_mode_router:=false
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ----------------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------------
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_savo_pi",
        description="Robot ID to filter intent results and for logging.",
    )

    use_mode_router_arg = DeclareLaunchArgument(
        "use_mode_router",
        default_value="true",
        description="Whether to start the ui_mode_router_node.",
    )

    use_debug_arg = DeclareLaunchArgument(
        "use_debug",
        default_value="false",
        description=(
            "Whether to start the ui_debug_node (for UI testing without the "
            "full robot stack). WARNING: if enable_fake_mouth is true in the "
            "debug node, it will publish to /savo_speech/mouth_level."
        ),
    )

    ui_params_file_arg = DeclareLaunchArgument(
        "ui_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("savo_ui"),
                "config",
                "ui_params.yaml",
            ]
        ),
        description="Path to the UI parameters YAML file.",
    )

    # You can later add a separate nav_view.yaml if desired; for now we fold
    # navigation settings into ui_params.yaml.

    # ----------------------------------------------------------------------
    # LaunchConfigurations
    # ----------------------------------------------------------------------
    robot_id = LaunchConfiguration("robot_id")
    use_mode_router = LaunchConfiguration("use_mode_router")
    use_debug = LaunchConfiguration("use_debug")
    ui_params_file = LaunchConfiguration("ui_params_file")

    # ----------------------------------------------------------------------
    # display_manager_node
    # ----------------------------------------------------------------------
    display_node = Node(
        package="savo_ui",
        executable="display_manager_node.py",
        name="savo_ui_display",
        output="screen",
        parameters=[ui_params_file],
    )

    # ----------------------------------------------------------------------
    # ui_mode_router_node (optional)
    # ----------------------------------------------------------------------
    mode_router_node = Node(
        package="savo_ui",
        executable="ui_mode_router_node.py",
        name="savo_ui_mode_router",
        output="screen",
        condition=IfCondition(use_mode_router),
        parameters=[
            {
                "robot_id": robot_id,
                # You can override this in higher-level launch files if
                # your mapping package uses a different topic name.
                "mapping_active_topic": "/savo_mapping/mapping_active",
                "idle_status_text": "Hello, I am Robot Savo!",
                "mapping_status_text": "Mapping in progress, please keep distance.",
                "stopped_status_text": "Stopped here.",
                "status_from_reply": True,
            }
        ],
    )

    # ----------------------------------------------------------------------
    # ui_debug_node (optional)
    # ----------------------------------------------------------------------
    debug_node = Node(
        package="savo_ui",
        executable="ui_debug_node.py",
        name="savo_ui_debug",
        output="screen",
        condition=IfCondition(use_debug),
        parameters=[
            {
                "robot_id": robot_id,
                "cycle_interval_s": 10.0,
                "enable_fake_mouth": False,
                "mouth_wave_freq_hz": 1.2,
            }
        ],
    )

    # ----------------------------------------------------------------------
    # Build and return LaunchDescription
    # ----------------------------------------------------------------------
    ld = LaunchDescription()

    # Declare arguments first
    ld.add_action(robot_id_arg)
    ld.add_action(use_mode_router_arg)
    ld.add_action(use_debug_arg)
    ld.add_action(ui_params_file_arg)

    # Add nodes
    ld.add_action(display_node)
    ld.add_action(mode_router_node)
    ld.add_action(debug_node)

    return ld
