"""Launch Robot Savo's guarded core-side autonomous mapping stack."""

import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from savo_bringup.autonomous_mapping_profiles import (
    resolve_autonomous_mapping_profile,
    validate_startup_scan360_disabled,
)
from savo_bringup.staged_launch import StartupStageGroup
from savo_bringup.staged_launch import build_staged_sequence
from savo_bringup.startup_timing import CORE_START_DELAYS

_MAP_ID_PATTERN = re.compile(r"[a-z][a-z0-9_]*")


def _validate_arguments(context):
    """Fail before node startup when safety-critical arguments are invalid."""
    initial_scan360_required = LaunchConfiguration(
        "initial_scan360_required"
    ).perform(context)
    try:
        validate_startup_scan360_disabled(initial_scan360_required)
    except ValueError as error:
        raise RuntimeError(str(error)) from error

    profile_name = LaunchConfiguration(
        "autonomous_mapping_profile"
    ).perform(context)
    perception_config_override = LaunchConfiguration(
        "perception_config_file"
    ).perform(context)
    nav_params_override = LaunchConfiguration(
        "nav_params_file"
    ).perform(context)
    try:
        selected_assets = resolve_autonomous_mapping_profile(
            profile_name,
            perception_config_file=perception_config_override,
            nav_params_file=nav_params_override,
        )
    except ValueError as error:
        raise RuntimeError(str(error)) from error

    perception_config_file = Path(
        get_package_share_directory("savo_perception")
    ).joinpath(
        "config", "profiles", selected_assets.perception_config_filename
    ).resolve()
    nav_params_file = Path(
        get_package_share_directory("savo_nav")
    ).joinpath(
        "config", selected_assets.nav_params_filename
    ).resolve()
    if not perception_config_file.is_file():
        raise RuntimeError(
            f"autonomous mapping perception profile missing: {perception_config_file}"
        )
    if not nav_params_file.is_file():
        raise RuntimeError(
            f"autonomous mapping Nav2 profile missing: {nav_params_file}"
        )

    context.launch_configurations["perception_config_file"] = str(
        perception_config_file
    )
    context.launch_configurations["nav_params_file"] = str(nav_params_file)

    map_id = LaunchConfiguration("map_id").perform(context).strip()
    control_mode = (
        LaunchConfiguration("control_startup_mode")
        .perform(context)
        .strip()
        .upper()
    )
    require_locked_geometry = (
        LaunchConfiguration("require_locked_geometry")
        .perform(context)
        .strip()
        .lower()
    )
    allow_provisional_geometry = (
        LaunchConfiguration("allow_provisional_geometry")
        .perform(context)
        .strip()
        .lower()
    )
    geometry_profile = Path(
        LaunchConfiguration("geometry_profile").perform(context)
    ).resolve()
    canonical_geometry_profile = Path(
        get_package_share_directory("savo_description")
    ).joinpath(
        "config", "profiles", "robot_savo_core_v1.yaml"
    ).resolve()
    perception_use_ultrasonic = LaunchConfiguration(
        "perception_use_ultrasonic"
    ).perform(context).strip().lower()
    start_mapping = LaunchConfiguration("start_mapping").perform(
        context
    ).strip().lower() in {"true", "1", "yes", "on"}
    start_navigation = LaunchConfiguration("start_navigation").perform(
        context
    ).strip().lower() in {"true", "1", "yes", "on"}

    if not _MAP_ID_PATTERN.fullmatch(map_id):
        raise RuntimeError(
            "map_id must start with a lowercase letter and contain only "
            "lowercase letters, numbers, and underscores"
        )

    if control_mode != "STOP":
        raise RuntimeError(
            "autonomous mapping must start control in STOP"
        )

    if require_locked_geometry not in {"true", "1", "yes", "on"}:
        raise RuntimeError(
            "autonomous mapping production requires require_locked_geometry:=true"
        )

    if allow_provisional_geometry not in {"false", "0", "no", "off"}:
        raise RuntimeError(
            "autonomous mapping production forbids allow_provisional_geometry:=true"
        )

    if geometry_profile != canonical_geometry_profile:
        raise RuntimeError(
            "autonomous mapping requires the canonical production geometry profile"
        )

    if (
        profile_name.strip() == "core_lidar_mapping_degraded" and
        perception_use_ultrasonic not in {"false", "0", "no", "off"}
    ):
        raise RuntimeError(
            "core_lidar_mapping_degraded requires perception_use_ultrasonic:=false"
        )

    if start_navigation and not start_mapping:
        raise RuntimeError(
            "live-mapping Nav2 requires start_mapping:=true so SLAM owns "
            "the map and map-to-odom transform before Nav2 starts"
        )

    return [
        LogInfo(
            msg=(
                "Robot Savo AM-7/AM-8 launch validated: "
                f"map_id={map_id}, profile={profile_name.strip()}, "
                f"control_startup_mode={control_mode}"
            )
        ),
        LogInfo(
            msg=(
                "Autonomous motion is never started by launch. Send the "
                "typed RunAutonomousMapping action only after readiness "
                "is confirmed; the orchestrator selects control modes."
            )
        ),
    ]


def _python_launch(package: str, filename: str):
    """Return a Python launch-description source from a package share."""
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package), "launch", filename]
        )
    )


def _frontend_launch(package: str, filename: str):
    """Return a declarative launch source from a package share."""
    return FrontendLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package), "launch", filename]
        )
    )


def _stage(delay_argument: str, action):
    """Start one mapping-stack component after its bounded load offset."""
    return TimerAction(
        period=LaunchConfiguration(delay_argument),
        actions=[action],
        cancel_on_shutdown=True,
    )


def _enabled(context, argument: str) -> bool:
    """Resolve a boolean launch argument using ROS launch spellings."""
    return LaunchConfiguration(argument).perform(context).strip().lower() in {
        "true",
        "1",
        "yes",
        "on",
    }


def _build_autonomous_sequence(
    context,
    *,
    readiness_node,
    mapping_foundation_launch,
    navigation_launch,
    mapping_runtime_launch,
    log_level,
):
    """Build the runtime-specific SLAM -> Nav2 -> mapping process chain."""
    if not _enabled(context, "start_mapping"):
        if _enabled(context, "start_navigation"):
            raise RuntimeError(
                "live-mapping Nav2 cannot start without its SLAM foundation"
            )
        return []

    groups = [
        StartupStageGroup(
            name="localization",
            actions=(readiness_node,),
        ),
        StartupStageGroup(
            name="slam_foundation",
            actions=(mapping_foundation_launch,),
        ),
    ]
    if _enabled(context, "start_navigation"):
        groups.extend(
            [
                StartupStageGroup(
                    name="navigation",
                    actions=(navigation_launch,),
                ),
                StartupStageGroup(
                    name="mapping_runtime",
                    actions=(mapping_runtime_launch,),
                ),
            ]
        )

    return build_staged_sequence(
        groups,
        status_topic="/savo_bringup/core/startup_status",
        log_level=log_level,
    )


def generate_launch_description() -> LaunchDescription:
    """Compose the guarded core-side autonomous frontier mapping stack."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    scan360_required = PythonExpression(
        [
            "'",
            LaunchConfiguration("final_scan360_required"),
            "'.lower() in ('true', '1', 'yes', 'on')",
        ]
    )

    description_launch = IncludeLaunchDescription(
        _python_launch("savo_description", "description.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_description")),
        launch_arguments={
            "geometry_profile": LaunchConfiguration("geometry_profile"),
            "require_locked_geometry": LaunchConfiguration(
                "require_locked_geometry"
            ),
            "allow_provisional_geometry": LaunchConfiguration(
                "allow_provisional_geometry"
            ),
            "use_sim_time": use_sim_time,
            "use_transmissions": "false",
            "use_gazebo": "false",
        }.items(),
    )

    base_launch = IncludeLaunchDescription(
        _python_launch("savo_base", "base_bringup.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_base")),
        launch_arguments={
            "profile": LaunchConfiguration("base_profile"),
            "driver_impl": LaunchConfiguration("base_driver_impl"),
            "use_diag_runner": "false",
            "output": "screen",
            "log_level": log_level,
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        _python_launch("savo_lidar", "lidar_mapping_ready.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_lidar")),
        launch_arguments={
            "profile": LaunchConfiguration("lidar_profile"),
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        _python_launch("savo_perception", "perception_bringup.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_perception")),
        launch_arguments={
            "driver_impl": LaunchConfiguration("perception_driver_impl"),
            "config_file": LaunchConfiguration("perception_config_file"),
            "use_ultrasonic": LaunchConfiguration(
                "perception_use_ultrasonic"
            ),
            "use_dashboard": "false",
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        _python_launch("savo_control", "control_bringup.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_control")),
        launch_arguments={
            "startup_mode": LaunchConfiguration("control_startup_mode"),
            "use_backup_escape": LaunchConfiguration(
                "control_use_backup_escape"
            ),
            "use_stuck_detector": LaunchConfiguration(
                "control_use_stuck_detector"
            ),
            "use_control_status": LaunchConfiguration(
                "control_use_control_status"
            ),
            "use_recovery_status": LaunchConfiguration(
                "control_use_recovery_status"
            ),
            "use_rotate_to_heading": scan360_required,
            "use_dashboard": "false",
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        _python_launch(
            "savo_localization",
            "localization_bringup.launch.py",
        ),
        condition=IfCondition(LaunchConfiguration("start_localization")),
        launch_arguments={
            "use_vo": LaunchConfiguration("localization_use_vo"),
            "use_imu": "true",
            "use_wheel_odom": "true",
            "use_ekf": "true",
            "use_health": "true",
        }.items(),
    )

    power_launch = IncludeLaunchDescription(
        _python_launch("savo_power", "power_core.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_power")),
        launch_arguments={
            "use_python_fallback": LaunchConfiguration(
                "power_use_python_fallback"
            ),
            "edge_ups_expected": LaunchConfiguration(
                "edge_ups_expected"
            ),
        }.items(),
    )

    supervisor_launch = IncludeLaunchDescription(
        _python_launch("savo_supervisor", "supervisor.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_supervisor")),
        launch_arguments={
            "robot_mode": "autonomous_mapping",
            "system_state_path": LaunchConfiguration(
                "supervisor_state_path"
            ),
            "auto_arm": LaunchConfiguration("supervisor_auto_arm"),
            "edge_ups_required": LaunchConfiguration("edge_ups_expected"),
            "require_semantic_autonomous_mapping": LaunchConfiguration(
                "start_semantic_interruption"
            ),
        }.items(),
    )

    head_launch = IncludeLaunchDescription(
        _python_launch("savo_head", "head_bringup.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_head")),
        launch_arguments={
            "backend": LaunchConfiguration("head_backend"),
            "use_python_fallback": LaunchConfiguration(
                "head_use_python_fallback"
            ),
            "enable_scan": "true",
            "enable_tf": LaunchConfiguration("head_enable_tf"),
            "enable_status": "true",
            "enable_apriltag_confirm": LaunchConfiguration(
                "head_enable_apriltag_confirm"
            ),
            "center_on_start": "false",
            "center_on_shutdown": "true",
            "camera_mode": LaunchConfiguration("head_camera_mode"),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        _python_launch(
            "savo_nav",
            "live_mapping_navigation.launch.py",
        ),
        condition=IfCondition(LaunchConfiguration("start_navigation")),
        launch_arguments={
            "params_file": LaunchConfiguration("nav_params_file"),
            "readiness_params": LaunchConfiguration(
                "nav_readiness_params"
            ),
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("nav_autostart"),
            "start_readiness": "true",
            "start_startup_readiness": "true",
            "start_goal_gateway": "true",
            "log_level": log_level,
        }.items(),
    )

    location_lifecycle_launch = IncludeLaunchDescription(
        _python_launch("savo_bringup", "location_integration.launch.py"),
        condition=IfCondition(
            LaunchConfiguration("start_location_lifecycle")
        ),
        launch_arguments={
            "log_level": log_level,
            "start_locations": "true",
            "start_supervisor": "false",
            "start_head_observer": "false",
            "start_head_action": "true",
            "start_registration": "true",
            "mapping_local_authority": "true",
            "start_review_gateway": "true",
            "start_navigation": "false",
            "locations_database_path": LaunchConfiguration(
                "locations_database_path"
            ),
            "locations_releases_root": LaunchConfiguration(
                "locations_releases_root"
            ),
            "locations_create_parent_directories": LaunchConfiguration(
                "locations_create_parent_directories"
            ),
        }.items(),
    )

    mapping_common_arguments = {
        "local_health_edge_ups_required": LaunchConfiguration(
            "edge_ups_expected"
        ),
        "map_id": LaunchConfiguration("map_id"),
        "map_output_root": LaunchConfiguration("map_output_root"),
        "allow_map_overwrite": LaunchConfiguration(
            "allow_map_overwrite"
        ),
        "use_sim_time": use_sim_time,
        "slam_autostart": LaunchConfiguration("slam_autostart"),
        "slam_params_file": LaunchConfiguration("slam_params_file"),
        "map_frame": LaunchConfiguration("map_frame"),
        "base_frame": LaunchConfiguration("base_frame"),
        "coverage_enabled": LaunchConfiguration("coverage_enabled"),
        "geometry_profile": LaunchConfiguration("geometry_profile"),
        "require_locked_geometry": LaunchConfiguration(
            "require_locked_geometry"
        ),
        "allow_provisional_geometry": LaunchConfiguration(
            "allow_provisional_geometry"
        ),
        "coverage_params_file": LaunchConfiguration(
            "coverage_params_file"
        ),
        "coverage_profile_file": LaunchConfiguration(
            "coverage_profile_file"
        ),
        "coverage_use_real_robot_profile": LaunchConfiguration(
            "coverage_use_real_robot_profile"
        ),
        "coverage_execution_handoff_params_file": LaunchConfiguration(
            "coverage_execution_handoff_params_file"
        ),
        "coverage_operation_params_file": LaunchConfiguration(
            "coverage_operation_params_file"
        ),
        "initial_scan360_required": "false",
        "initial_head_scan_required": LaunchConfiguration(
            "initial_head_scan_required"
        ),
        "final_scan360_required": LaunchConfiguration(
            "final_scan360_required"
        ),
        "final_head_scan_required": LaunchConfiguration(
            "final_head_scan_required"
        ),
    }

    mapping_foundation_launch = IncludeLaunchDescription(
        _frontend_launch("savo_mapping", "autonomous_mapping.launch.xml"),
        condition=IfCondition(LaunchConfiguration("start_mapping")),
        launch_arguments={
            **mapping_common_arguments,
            "start_mapping_foundation": "true",
            "start_mapping_runtime": "false",
            "semantic_interruption_enabled": LaunchConfiguration(
                "start_semantic_interruption"
            ),
        }.items(),
    )

    mapping_runtime_launch = IncludeLaunchDescription(
        _frontend_launch("savo_mapping", "autonomous_mapping.launch.xml"),
        condition=IfCondition(LaunchConfiguration("start_mapping")),
        launch_arguments={
            **mapping_common_arguments,
            "start_mapping_foundation": "false",
            "start_mapping_runtime": "true",
            "semantic_interruption_enabled": LaunchConfiguration(
                "start_semantic_interruption"
            ),
        }.items(),
    )

    bringup_readiness_params = PathJoinSubstitution(
        [FindPackageShare("savo_bringup"), "config", "core_real_robot.yaml"]
    )
    startup_stages_params = PathJoinSubstitution(
        [FindPackageShare("savo_bringup"), "config", "startup_stages.yaml"]
    )
    readiness_node = Node(
        package="savo_bringup",
        executable="bringup_readiness_node",
        name="bringup_readiness_node",
        output="screen",
        parameters=[
            bringup_readiness_params,
            startup_stages_params,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "robot_mode": "autonomous_mapping",
                "bringup_profile": LaunchConfiguration("bringup_profile"),
                "d435_voxel_validated": ParameterValue(
                    LaunchConfiguration("d435_voxel_validated"),
                    value_type=bool,
                ),
                "require_supervisor": False,
                "require_supervisor_authority": False,
                "require_navigation": ParameterValue(
                    LaunchConfiguration("start_navigation"), value_type=bool
                ),
                "require_mapping": True,
                "require_mapping_runtime": ParameterValue(
                    LaunchConfiguration("start_navigation"), value_type=bool
                ),
                "require_head": False,
                "require_locations": False,
                "require_semantic": False,
                "startup.enabled": True,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    autonomous_sequence = OpaqueFunction(
        function=_build_autonomous_sequence,
        kwargs={
            "readiness_node": readiness_node,
            "mapping_foundation_launch": mapping_foundation_launch,
            "navigation_launch": navigation_launch,
            "mapping_runtime_launch": mapping_runtime_launch,
            "log_level": log_level,
        },
    )

    default_geometry_profile = PathJoinSubstitution(
        [
            FindPackageShare("savo_description"),
            "config",
            "profiles",
            "robot_savo_core_v1.yaml",
        ]
    )
    default_nav_readiness = PathJoinSubstitution(
        [FindPackageShare("savo_nav"), "config", "readiness.yaml"]
    )
    default_slam_params = PathJoinSubstitution(
        [
            FindPackageShare("savo_mapping"),
            "config",
            "slam_toolbox_mapping.yaml",
        ]
    )
    default_coverage_params = PathJoinSubstitution(
        [FindPackageShare("savo_mapping"), "config", "coverage_mapping.yaml"]
    )
    default_coverage_profile = PathJoinSubstitution(
        [
            FindPackageShare("savo_mapping"),
            "config",
            "profiles",
            "coverage_mapping_real_robot.yaml",
        ]
    )
    default_coverage_handoff = PathJoinSubstitution(
        [
            FindPackageShare("savo_mapping"),
            "config",
            "coverage_execution_handoff.yaml",
        ]
    )
    default_coverage_operation = PathJoinSubstitution(
        [
            FindPackageShare("savo_mapping"),
            "config",
            "coverage_operation_orchestrator.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_id",
                default_value="",
                description=(
                    "Required lowercase map/session identifier. The action "
                    "goal must use the same map_id."
                ),
            ),
            DeclareLaunchArgument(
                "autonomous_mapping_profile",
                default_value="production",
                description=(
                    "Atomic autonomous-mapping perception/Nav2 profile: "
                    "production or core_lidar_mapping_degraded."
                ),
            ),
            DeclareLaunchArgument(
                "map_output_root",
                default_value="/var/lib/robot_savo/maps/sessions",
                description="Root directory for committed map sessions.",
            ),
            DeclareLaunchArgument(
                "allow_map_overwrite",
                default_value="false",
                description="Allow replacement of an existing map session.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "bringup_profile", default_value="lidar_only"
            ),
            DeclareLaunchArgument(
                "d435_voxel_validated", default_value="false"
            ),
            DeclareLaunchArgument(
                "supervisor_state_path",
                default_value=(
                    "/var/lib/robot_savo/supervisor/system_state.json"
                ),
            ),
            DeclareLaunchArgument(
                "supervisor_auto_arm", default_value="false"
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument(
                "geometry_profile", default_value=default_geometry_profile
            ),
            DeclareLaunchArgument(
                "require_locked_geometry", default_value="true"
            ),
            DeclareLaunchArgument(
                "allow_provisional_geometry", default_value="false"
            ),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument(
                "start_perception", default_value="true"
            ),
            DeclareLaunchArgument("start_control", default_value="true"),
            DeclareLaunchArgument(
                "start_localization", default_value="true"
            ),
            DeclareLaunchArgument("start_power", default_value="true"),
            DeclareLaunchArgument(
                "start_supervisor", default_value="false"
            ),
            DeclareLaunchArgument("start_head", default_value="false"),
            DeclareLaunchArgument(
                "start_location_lifecycle", default_value="false"
            ),
            DeclareLaunchArgument(
                "start_semantic_interruption", default_value="false"
            ),
            DeclareLaunchArgument(
                "start_navigation", default_value="true"
            ),
            DeclareLaunchArgument("start_mapping", default_value="true"),
            DeclareLaunchArgument(
                "locations_database_path",
                default_value=(
                    "/var/lib/robot_savo/locations/locations.db"
                ),
            ),
            DeclareLaunchArgument(
                "locations_releases_root",
                default_value=(
                    "/var/lib/robot_savo/locations/releases"
                ),
            ),
            DeclareLaunchArgument(
                "locations_create_parent_directories",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "base_profile",
                default_value="real_robot_v1.yaml",
            ),
            DeclareLaunchArgument(
                "base_driver_impl",
                default_value="cpp",
            ),
            DeclareLaunchArgument(
                "lidar_profile",
                default_value="mapping_rplidar_a1.yaml",
            ),
            DeclareLaunchArgument(
                "perception_driver_impl",
                default_value="cpp",
            ),
            DeclareLaunchArgument(
                "perception_config_file",
                default_value="",
                description=(
                    "Internal profile-selected perception asset; direct "
                    "overrides are forbidden."
                ),
            ),
            DeclareLaunchArgument(
                "perception_use_ultrasonic",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "control_startup_mode",
                default_value="STOP",
                description=(
                    "Safe launch default is STOP. The mission orchestrator "
                    "may select NAV/AUTO only after exact authority."
                ),
            ),
            DeclareLaunchArgument(
                "control_use_backup_escape",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "control_use_stuck_detector",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "control_use_control_status",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "control_use_recovery_status",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "localization_use_vo",
                default_value="false",
                description=(
                    "Fuse the validated Edge VO stream during production "
                    "localization."
                ),
            ),
            DeclareLaunchArgument(
                "power_use_python_fallback",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "edge_ups_expected",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "head_backend",
                default_value="pca9685",
            ),
            DeclareLaunchArgument(
                "head_use_python_fallback",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "head_enable_tf",
                default_value="true",
                description=(
                    "Publish the calibrated head TF; fresh pan/tilt state "
                    "remains required."
                ),
            ),
            DeclareLaunchArgument(
                "head_enable_apriltag_confirm",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "head_camera_mode",
                default_value="ros",
                description=(
                    "Use the validated ROS IMX219 camera transport."
                ),
            ),
            DeclareLaunchArgument(
                "nav_params_file", default_value="",
                description=(
                    "Internal profile-selected Nav2 asset; direct overrides "
                    "are forbidden."
                ),
            ),
            DeclareLaunchArgument(
                "nav_readiness_params",
                default_value=default_nav_readiness,
            ),
            DeclareLaunchArgument("nav_autostart", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
            ),
            DeclareLaunchArgument("slam_autostart", default_value="true"),
            DeclareLaunchArgument("coverage_enabled", default_value="false"),
            DeclareLaunchArgument(
                "coverage_params_file", default_value=default_coverage_params
            ),
            DeclareLaunchArgument(
                "coverage_profile_file", default_value=default_coverage_profile
            ),
            DeclareLaunchArgument(
                "coverage_use_real_robot_profile", default_value="true"
            ),
            DeclareLaunchArgument(
                "coverage_execution_handoff_params_file",
                default_value=default_coverage_handoff,
            ),
            DeclareLaunchArgument(
                "coverage_operation_params_file",
                default_value=default_coverage_operation,
            ),
            DeclareLaunchArgument(
                "initial_scan360_required",
                default_value="false",
                description=(
                    "Fixed false for dedicated autonomous mapping; attempts "
                    "to enable startup Scan360 are rejected before startup."
                ),
            ),
            DeclareLaunchArgument(
                "initial_head_scan_required", default_value="false"
            ),
            DeclareLaunchArgument(
                "final_scan360_required", default_value="false"
            ),
            DeclareLaunchArgument(
                "final_head_scan_required", default_value="false"
            ),
            DeclareLaunchArgument(
                "description_start_delay_s",
                default_value=CORE_START_DELAYS["description"],
            ),
            DeclareLaunchArgument(
                "base_start_delay_s",
                default_value=CORE_START_DELAYS["base"],
            ),
            DeclareLaunchArgument(
                "lidar_start_delay_s",
                default_value=CORE_START_DELAYS["lidar"],
            ),
            DeclareLaunchArgument(
                "perception_start_delay_s",
                default_value=CORE_START_DELAYS["perception"],
            ),
            DeclareLaunchArgument(
                "control_start_delay_s",
                default_value=CORE_START_DELAYS["control"],
            ),
            DeclareLaunchArgument(
                "localization_start_delay_s",
                default_value=CORE_START_DELAYS["localization"],
            ),
            DeclareLaunchArgument(
                "power_start_delay_s",
                default_value=CORE_START_DELAYS["power"],
            ),
            DeclareLaunchArgument(
                "head_start_delay_s",
                default_value=CORE_START_DELAYS["head"],
            ),
            DeclareLaunchArgument(
                "supervisor_start_delay_s",
                default_value=CORE_START_DELAYS["supervisor"],
            ),
            DeclareLaunchArgument(
                "location_lifecycle_start_delay_s",
                default_value=CORE_START_DELAYS["location_lifecycle"],
            ),
            DeclareLaunchArgument(
                "navigation_start_delay_s",
                default_value=CORE_START_DELAYS["navigation"],
                description=(
                    "Compatibility timing input; autonomous Nav2 release is "
                    "readiness-gated after SLAM and does not use this delay."
                ),
            ),
            DeclareLaunchArgument(
                "mapping_start_delay_s",
                default_value=CORE_START_DELAYS["mapping"],
            ),
            OpaqueFunction(function=_validate_arguments),
            _stage("description_start_delay_s", description_launch),
            _stage("base_start_delay_s", base_launch),
            _stage("lidar_start_delay_s", lidar_launch),
            _stage("perception_start_delay_s", perception_launch),
            _stage("control_start_delay_s", control_launch),
            _stage("localization_start_delay_s", localization_launch),
            _stage("power_start_delay_s", power_launch),
            _stage("head_start_delay_s", head_launch),
            _stage("supervisor_start_delay_s", supervisor_launch),
            _stage(
                "location_lifecycle_start_delay_s",
                location_lifecycle_launch,
            ),
            _stage("mapping_start_delay_s", autonomous_sequence),
        ]
    )
