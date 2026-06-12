#!/usr/bin/env python3
"""Launch the LiDAR serial-port diagnostic CLI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    json_output = LaunchConfiguration("json")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Preferred RPLIDAR serial port.",
    )

    json_arg = DeclareLaunchArgument(
        "json",
        default_value="false",
        description="Print diagnostic result as JSON.",
    )

    port_check = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "savo_lidar",
            "find_lidar_port_cli.py",
            "--serial-port",
            serial_port,
            "--json",
            json_output,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            serial_port_arg,
            json_arg,
            LogInfo(
                msg=[
                    "Starting Robot Savo LiDAR port diagnostic | serial_port=",
                    serial_port,
                ]
            ),
            port_check,
        ]
    )