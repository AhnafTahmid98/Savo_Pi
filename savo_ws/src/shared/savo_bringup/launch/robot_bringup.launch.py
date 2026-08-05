"""Role-selecting full Robot Savo launch entry point."""

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
    role = _value(context, "host_role")
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
                f"host_role={role}, mode={mode}, profile={profile}"
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
            "head_enable_tf",
            "head_camera_mode",
        ]:
            core_arguments[name] = LaunchConfiguration(name)
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
            "start_speech",
            "start_ui",
            "start_bridge",
            "start_edge_power",
            "vo_profile",
            "ui_profile",
            "active_map_id",
            "active_map_revision",
        ]:
            key = "start_power" if name == "start_edge_power" else name
            edge_arguments[key] = LaunchConfiguration(name)
        speech_params = _value(context, "speech_params_file")
        if speech_params:
            edge_arguments["speech_params_file"] = speech_params
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
            DeclareLaunchArgument("host_role", default_value="core"),
            DeclareLaunchArgument("robot_mode", default_value="safe_idle"),
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("d435_voxel_validated", default_value="false"),
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
            DeclareLaunchArgument("localization_use_vo", default_value="false"),
            DeclareLaunchArgument("head_enable_tf", default_value="false"),
            DeclareLaunchArgument("head_camera_mode", default_value="disabled"),
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("start_vo", default_value="true"),
            DeclareLaunchArgument("start_obstacle_cloud", default_value="false"),
            DeclareLaunchArgument("start_speech", default_value="false"),
            DeclareLaunchArgument("start_ui", default_value="false"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_edge_power", default_value="true"),
            DeclareLaunchArgument("vo_profile", default_value="real_robot_v1"),
            DeclareLaunchArgument("ui_profile", default_value="pi"),
            DeclareLaunchArgument(
                "speech_params_file",
                default_value="",
            ),
            DeclareLaunchArgument("active_map_id", default_value="saved_map"),
            DeclareLaunchArgument("active_map_revision", default_value="1"),
            OpaqueFunction(function=_setup),
        ]
    )
