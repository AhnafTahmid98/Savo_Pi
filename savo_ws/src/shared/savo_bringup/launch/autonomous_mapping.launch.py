"""Launch Robot Savo's guarded core-side autonomous mapping stack."""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_MAP_ID_PATTERN = re.compile(r"[a-z][a-z0-9_]*")


def _validate_arguments(context):
    """Fail before node startup when safety-critical arguments are invalid."""
    map_id = LaunchConfiguration("map_id").perform(context).strip()
    control_mode = (
        LaunchConfiguration("control_startup_mode")
        .perform(context)
        .strip()
        .upper()
    )

    if not _MAP_ID_PATTERN.fullmatch(map_id):
        raise RuntimeError(
            "map_id must start with a lowercase letter and contain only "
            "lowercase letters, numbers, and underscores"
        )

    if control_mode not in {"STOP", "NAV"}:
        raise RuntimeError("control_startup_mode must be STOP or NAV")

    return [
        LogInfo(
            msg=(
                "Robot Savo AM-5 launch validated: "
                f"map_id={map_id}, control_startup_mode={control_mode}"
            )
        ),
        LogInfo(
            msg=(
                "Autonomous motion is never started by launch. Send the "
                "typed RunAutonomousMapping action only after readiness "
                "and the required control mode are confirmed."
            )
        ),
        LogInfo(
            msg=(
                "savo_description is intentionally external to AM-5 until "
                "AM-0B locks real dimensions, sensor transforms, and STL "
                "meshes. Mapping readiness remains fail-closed on TF."
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


def generate_launch_description() -> LaunchDescription:
    """Compose the guarded core-side autonomous frontier mapping stack."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

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
            "use_dashboard": "false",
            "use_state_publisher": "false",
        }.items(),
    )

    power_launch = IncludeLaunchDescription(
        _python_launch("savo_power", "power_core.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_power")),
        launch_arguments={
            "use_python_fallback": LaunchConfiguration(
                "power_use_python_fallback"
            ),
        }.items(),
    )

    supervisor_launch = IncludeLaunchDescription(
        _python_launch("savo_supervisor", "supervisor.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_supervisor")),
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
            "start_review_gateway": "false",
            "start_navigation": "false",
            "locations_database_path": LaunchConfiguration(
                "locations_database_path"
            ),
            "locations_create_parent_directories": LaunchConfiguration(
                "locations_create_parent_directories"
            ),
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        _frontend_launch("savo_mapping", "autonomous_mapping.launch.xml"),
        condition=IfCondition(LaunchConfiguration("start_mapping")),
        launch_arguments={
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
            "semantic_interruption_enabled": LaunchConfiguration(
                "start_semantic_interruption"
            ),
        }.items(),
    )

    default_perception_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_perception"),
            "config",
            "profiles",
            "core_real_robot_v1.yaml",
        ]
    )
    default_nav_params = PathJoinSubstitution(
        [
            FindPackageShare("savo_nav"),
            "config",
            "nav2",
            "saved_map.yaml",
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
                "map_output_root",
                default_value="~/Savo_Pi/runtime/maps",
                description="Root directory for committed map sessions.",
            ),
            DeclareLaunchArgument(
                "allow_map_overwrite",
                default_value="false",
                description="Allow replacement of an existing map session.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
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
                "start_supervisor", default_value="true"
            ),
            DeclareLaunchArgument("start_head", default_value="true"),
            DeclareLaunchArgument(
                "start_location_lifecycle", default_value="true"
            ),
            DeclareLaunchArgument(
                "start_semantic_interruption", default_value="true"
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
                default_value=default_perception_config,
            ),
            DeclareLaunchArgument(
                "control_startup_mode",
                default_value="STOP",
                description=(
                    "Safe default is STOP. NAV may be selected explicitly "
                    "only during a controlled real-robot test."
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
                "localization_use_vo",
                default_value="false",
                description=(
                    "Use edge VO input only after the edge stream is live."
                ),
            ),
            DeclareLaunchArgument(
                "power_use_python_fallback",
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
                default_value="false",
                description=(
                    "Keep head TF disabled until AM-0B locks the physical "
                    "head and camera transforms."
                ),
            ),
            DeclareLaunchArgument(
                "head_enable_apriltag_confirm",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "head_camera_mode",
                default_value="disabled",
                description=(
                    "Camera transport stays disabled until the real camera "
                    "path is selected for hardware testing."
                ),
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=default_nav_params,
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
            OpaqueFunction(function=_validate_arguments),
            base_launch,
            lidar_launch,
            perception_launch,
            control_launch,
            localization_launch,
            power_launch,
            supervisor_launch,
            head_launch,
            location_lifecycle_launch,
            navigation_launch,
            mapping_launch,
        ]
    )
