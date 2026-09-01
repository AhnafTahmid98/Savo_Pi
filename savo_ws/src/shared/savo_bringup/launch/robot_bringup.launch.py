"""Role-selecting full Robot Savo launch entry point."""

import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from savo_bringup.launch_contract import as_bool
from savo_bringup.launch_contract import resolve_host_role
from savo_bringup.launch_contract import validate_selection


def _launch(filename: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("savo_bringup"), "launch", filename]
        )
    )


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _common_arguments():
    names = [
        "robot_mode",
        "bringup_profile",
        "d435_voxel_validated",
        "require_locked_geometry",
        "allow_provisional_geometry",
        "use_sim_time",
        "log_level",
    ]
    return {name: LaunchConfiguration(name) for name in names}


def _setup(context):
    requested_role = _value(context, "host_role")
    role = resolve_host_role(
        requested_role,
        socket.gethostname(),
        os.environ.get("SAVO_ROLE"),
    )
    mode = _value(context, "robot_mode")
    profile = _value(context, "bringup_profile")
    validate_selection(
        role,
        mode,
        profile,
        d435_voxel_validated=as_bool(
            _value(context, "d435_voxel_validated")
        ),
        require_locked_geometry=as_bool(
            _value(context, "require_locked_geometry")
        ),
        allow_provisional_geometry=as_bool(
            _value(context, "allow_provisional_geometry")
        ),
    )

    if role == "all" and profile != "bench":
        raise RuntimeError(
            "host_role:=all is reserved for a single-host bench profile"
        )

    actions = [
        LogInfo(
            msg=(
                "Robot Savo distributed bringup selected: "
                f"host_role={role}, requested_role={requested_role}, "
                f"mode={mode}, profile={profile}"
            )
        )
    ]
    common = _common_arguments()

    if role in {"core", "all"}:
        core_arguments = dict(common)
        for name in [
            "map_id",
            "map_output_root",
            "allow_map_overwrite",
            "production_map_root",
            "active_map_contract",
            "locations_database_path",
            "locations_releases_root",
            "locations_create_parent_directories",
            "supervisor_state_path",
            "supervisor_auto_arm",
            "geometry_profile",
            "base_profile",
            "lidar_profile",
            "perception_config_file",
            "control_startup_mode",
            "control_use_backup_escape",
            "control_use_stuck_detector",
            "localization_use_vo",
            "edge_ups_expected",
            "head_enable_tf",
            "head_camera_mode",
            "start_description",
            "start_base",
            "start_lidar",
            "start_perception",
            "start_localization",
            "start_control",
            "start_power",
            "start_supervisor",
            "start_head",
            "start_location_lifecycle",
            "description_start_delay_s",
            "base_start_delay_s",
            "lidar_start_delay_s",
            "perception_start_delay_s",
            "control_start_delay_s",
            "localization_start_delay_s",
            "power_start_delay_s",
            "head_start_delay_s",
            "supervisor_start_delay_s",
            "location_lifecycle_start_delay_s",
            "manual_mapping_start_delay_s",
            "navigation_start_delay_s",
        ]:
            core_arguments[name] = LaunchConfiguration(name)
        core_arguments["readiness_start_delay_s"] = LaunchConfiguration(
            "core_readiness_start_delay_s"
        )
        actions.append(
            IncludeLaunchDescription(
                _launch("core_bringup.launch.py"),
                launch_arguments=core_arguments.items(),
            )
        )

    if role in {"edge", "all"}:
        edge_arguments = dict(common)
        for name in [
            "start_realsense",
            "start_vo",
            "start_obstacle_cloud",
            "enable_observer_color_relay",
            "start_speech",
            "start_ui",
            "start_bridge",
            "start_edge_power",
            "realsense_start_delay_s",
            "camera_support_start_delay_s",
            "vo_start_delay_s",
            "obstacle_cloud_start_delay_s",
            "observer_relay_start_delay_s",
            "bridge_start_delay_s",
            "readiness_start_delay_s",
            "vo_profile",
            "ui_profile",
            "active_map_id",
            "active_map_revision",
        ]:
            key = "start_power" if name == "start_edge_power" else name
            edge_arguments[key] = LaunchConfiguration(name)
        speech_params = _value(context, "speech_params_file")
        if speech_params:
            edge_arguments["speech_params_file"] = (
                PathJoinSubstitution(
                    [
                        FindPackageShare("savo_speech"),
                        "config",
                        "profiles",
                        speech_params,
                    ]
                )
                if speech_params == "edge_real_robot_v1.yaml"
                else speech_params
            )
        actions.append(
            IncludeLaunchDescription(
                _launch("edge_bringup.launch.py"),
                launch_arguments=edge_arguments.items(),
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the role-selecting distributed Robot Savo launch."""
    description_share = FindPackageShare("savo_description")
    perception_share = FindPackageShare("savo_perception")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host_role", default_value="auto"),
            DeclareLaunchArgument("robot_mode", default_value="safe_idle"),
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("d435_voxel_validated", default_value="true"),
            DeclareLaunchArgument("require_locked_geometry", default_value="true"),
            DeclareLaunchArgument(
                "allow_provisional_geometry", default_value="false"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("map_id", default_value="robot_savo_map"),
            DeclareLaunchArgument(
                "map_output_root",
                default_value="/var/lib/robot_savo/maps/sessions",
            ),
            DeclareLaunchArgument("allow_map_overwrite", default_value="false"),
            DeclareLaunchArgument(
                "production_map_root",
                default_value="/var/lib/robot_savo/maps/production",
            ),
            DeclareLaunchArgument("active_map_contract", default_value=""),
            DeclareLaunchArgument(
                "locations_database_path",
                default_value="/var/lib/robot_savo/locations/locations.db",
            ),
            DeclareLaunchArgument(
                "locations_releases_root",
                default_value="/var/lib/robot_savo/locations/releases",
            ),
            DeclareLaunchArgument(
                "locations_create_parent_directories", default_value="false"
            ),
            DeclareLaunchArgument(
                "supervisor_state_path",
                default_value=(
                    "/var/lib/robot_savo/supervisor/system_state.json"
                ),
            ),
            DeclareLaunchArgument("supervisor_auto_arm", default_value="false"),
            DeclareLaunchArgument(
                "geometry_profile",
                default_value=PathJoinSubstitution(
                    [
                        description_share,
                        "config",
                        "profiles",
                        "robot_savo_core_v1.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("base_profile", default_value="real_robot_v1.yaml"),
            DeclareLaunchArgument("lidar_profile", default_value="real_rplidar_a1.yaml"),
            DeclareLaunchArgument(
                "perception_config_file",
                default_value=PathJoinSubstitution(
                    [perception_share, "config", "profiles", "core_real_robot_v1.yaml"]
                ),
            ),
            DeclareLaunchArgument("control_startup_mode", default_value="STOP"),
            DeclareLaunchArgument("control_use_backup_escape", default_value="false"),
            DeclareLaunchArgument("control_use_stuck_detector", default_value="false"),
            DeclareLaunchArgument("localization_use_vo", default_value="true"),
            DeclareLaunchArgument("edge_ups_expected", default_value="false"),
            DeclareLaunchArgument("head_enable_tf", default_value="true"),
            DeclareLaunchArgument("head_camera_mode", default_value="ros"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("start_localization", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument("start_power", default_value="true"),
            DeclareLaunchArgument("start_supervisor", default_value="true"),
            DeclareLaunchArgument("start_head", default_value="true"),
            DeclareLaunchArgument(
                "start_location_lifecycle", default_value="false"
            ),
            DeclareLaunchArgument(
                "description_start_delay_s", default_value="0.0"
            ),
            DeclareLaunchArgument("base_start_delay_s", default_value="3.0"),
            DeclareLaunchArgument("lidar_start_delay_s", default_value="6.0"),
            DeclareLaunchArgument(
                "perception_start_delay_s", default_value="9.0"
            ),
            DeclareLaunchArgument(
                "control_start_delay_s", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "localization_start_delay_s", default_value="17.0"
            ),
            DeclareLaunchArgument("power_start_delay_s", default_value="22.0"),
            DeclareLaunchArgument("head_start_delay_s", default_value="27.0"),
            DeclareLaunchArgument(
                "supervisor_start_delay_s", default_value="33.0"
            ),
            DeclareLaunchArgument(
                "location_lifecycle_start_delay_s", default_value="37.0"
            ),
            DeclareLaunchArgument(
                "manual_mapping_start_delay_s", default_value="40.0"
            ),
            DeclareLaunchArgument(
                "navigation_start_delay_s", default_value="40.0"
            ),
            DeclareLaunchArgument(
                "core_readiness_start_delay_s", default_value="45.0"
            ),
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("start_vo", default_value="true"),
            DeclareLaunchArgument("start_obstacle_cloud", default_value="true"),
            DeclareLaunchArgument(
                "enable_observer_color_relay", default_value="true"
            ),
            DeclareLaunchArgument("start_speech", default_value="false"),
            DeclareLaunchArgument("start_ui", default_value="false"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_edge_power", default_value="true"),
            DeclareLaunchArgument(
                "realsense_start_delay_s", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "camera_support_start_delay_s", default_value="7.0"
            ),
            DeclareLaunchArgument("vo_start_delay_s", default_value="14.0"),
            DeclareLaunchArgument(
                "obstacle_cloud_start_delay_s", default_value="22.0"
            ),
            DeclareLaunchArgument(
                "observer_relay_start_delay_s", default_value="28.0"
            ),
            DeclareLaunchArgument(
                "bridge_start_delay_s", default_value="34.0"
            ),
            DeclareLaunchArgument(
                "readiness_start_delay_s", default_value="40.0"
            ),
            DeclareLaunchArgument("vo_profile", default_value="real_robot_v1"),
            DeclareLaunchArgument("ui_profile", default_value="pi"),
            DeclareLaunchArgument(
                "speech_params_file",
                default_value="edge_real_robot_v1.yaml",
            ),
            DeclareLaunchArgument("active_map_id", default_value=""),
            DeclareLaunchArgument("active_map_revision", default_value="0"),
            OpaqueFunction(function=_setup),
        ]
    )
