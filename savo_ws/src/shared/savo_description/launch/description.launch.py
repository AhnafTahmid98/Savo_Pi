"""Fail-closed Robot SAVO description launch driven by one geometry profile."""

from __future__ import annotations

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be true or false")


def _vector(value) -> str:
    return " ".join(str(item) for item in value)


def _launch_description(context):
    package_share = Path(get_package_share_directory("savo_description"))
    scripts = package_share / "scripts"
    sys.path.insert(0, str(scripts))
    from geometry_profile import (  # pylint: disable=import-outside-toplevel
        canonical_digest,
        load_profile,
        validate_profile,
    )

    profile_path = Path(LaunchConfiguration("geometry_profile").perform(context))
    require_locked = _boolean(
        LaunchConfiguration("require_locked_geometry").perform(context),
        "require_locked_geometry",
    )
    allow_provisional = _boolean(
        LaunchConfiguration("allow_provisional_geometry").perform(context),
        "allow_provisional_geometry",
    )
    profile = load_profile(profile_path)
    validate_profile(
        profile,
        require_locked=require_locked,
        allow_provisional=allow_provisional,
    )

    chassis = profile["chassis"]
    wheels = profile["wheels"]
    mounts = profile["mounts"]
    scalar_arguments = {
        "base_length": chassis["length_m"],
        "base_width": chassis["width_m"],
        "base_height": chassis["height_m"],
        "base_link_z": chassis["base_footprint_to_base_link_z_m"],
        "deck_thickness": chassis["deck_thickness_m"],
        "deck_spacing": chassis["deck_spacing_m"],
        "base_mass": chassis["mass_kg"],
        "deck_mass": chassis["deck_mass_each_kg"],
        "wheel_radius": wheels["radius_m"],
        "wheel_width": wheels["width_m"],
        "wheel_front_x": wheels["front_x_m"],
        "wheel_rear_x": wheels["rear_x_m"],
        "wheel_left_y": wheels["left_y_m"],
        "wheel_right_y": wheels["right_y_m"],
        "wheel_z": wheels["z_m"],
        "wheel_mass": wheels["mass_each_kg"],
    }
    vector_arguments = {
        "lidar_xyz": mounts["lidar"]["xyz_m"],
        "lidar_rpy": mounts["lidar"]["rpy_rad"],
        "imu_xyz": mounts["imu"]["xyz_m"],
        "imu_rpy": mounts["imu"]["rpy_rad"],
        "camera_xyz": mounts["realsense_d435"]["xyz_m"],
        "camera_rpy": mounts["realsense_d435"]["rpy_rad"],
        "tof_left_xyz": mounts["tof_left"]["xyz_m"],
        "tof_left_rpy": mounts["tof_left"]["rpy_rad"],
        "tof_right_xyz": mounts["tof_right"]["xyz_m"],
        "tof_right_rpy": mounts["tof_right"]["rpy_rad"],
        "ultrasonic_xyz": mounts["ultrasonic_front"]["xyz_m"],
        "ultrasonic_rpy": mounts["ultrasonic_front"]["rpy_rad"],
        "display_xyz": mounts["display"]["xyz_m"],
        "display_rpy": mounts["display"]["rpy_rad"],
        "respeaker_xyz": mounts["respeaker"]["xyz_m"],
        "respeaker_rpy": mounts["respeaker"]["rpy_rad"],
        "pantilt_mount_xyz": mounts["pantilt_mount"]["xyz_m"],
        "pantilt_mount_rpy": mounts["pantilt_mount"]["rpy_rad"],
    }

    xacro_path = package_share / "urdf" / "robot_savo.urdf.xacro"
    command = ["xacro ", str(xacro_path)]
    command.extend(
        f" {name}:={value}" for name, value in scalar_arguments.items()
    )
    command.extend(
        f" {name}:='{_vector(value)}'" for name, value in vector_arguments.items()
    )
    command.extend(
        [
            " use_transmissions:=",
            LaunchConfiguration("use_transmissions"),
            " use_gazebo:=",
            LaunchConfiguration("use_gazebo"),
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": Command(command)},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            additional_env={
                "SAVO_GEOMETRY_PROFILE_ID": profile["metadata"]["profile_id"],
                "SAVO_GEOMETRY_SHA256": canonical_digest(profile),
            },
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("savo_description"))
    default_profile = package_share / "config" / "profiles" / "robot_savo_core_v1.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_transmissions", default_value="false"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("geometry_profile", default_value=str(default_profile)),
            DeclareLaunchArgument("require_locked_geometry", default_value="true"),
            DeclareLaunchArgument("allow_provisional_geometry", default_value="false"),
            OpaqueFunction(function=_launch_description),
        ]
    )
