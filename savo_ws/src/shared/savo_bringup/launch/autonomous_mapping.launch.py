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
from launch.substitutions import PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from savo_bringup.launch_contract import as_bool
from savo_bringup.staged_launch import StartupStageGroup
from savo_bringup.staged_launch import build_staged_sequence


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

    if control_mode != "STOP":
        raise RuntimeError(
            "staged autonomous mapping must start control in STOP"
        )

    return [
        LogInfo(
            msg=(
                "Robot Savo AM-7/AM-8 launch validated: "
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
    scan360_required = PythonExpression(
        [
            "'",
            LaunchConfiguration("initial_scan360_required"),
            "'.lower() in ('true', '1', 'yes', 'on') or '",
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

    perception_hardware_launch = IncludeLaunchDescription(
        _python_launch("savo_perception", "range_sensors.launch.py"),
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

    perception_safety_launch = IncludeLaunchDescription(
        _python_launch("savo_perception", "safety_bringup.launch.py"),
        condition=IfCondition(LaunchConfiguration("start_perception")),
        launch_arguments={
            "driver_impl": LaunchConfiguration("perception_driver_impl"),
            "config_file": LaunchConfiguration("perception_config_file"),
            "use_ultrasonic": LaunchConfiguration(
                "perception_use_ultrasonic"
            ),
            "use_range_health": "false",
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

    localization_hardware_launch = IncludeLaunchDescription(
        _python_launch(
            "savo_localization",
            "localization_bringup.launch.py",
        ),
        condition=IfCondition(LaunchConfiguration("start_localization")),
        launch_arguments={
            "use_vo": LaunchConfiguration("localization_use_vo"),
            "use_imu": "true",
            "use_wheel_odom": "true",
            "use_ekf": "false",
            "use_health": "false",
        }.items(),
    )

    localization_filter_launch = IncludeLaunchDescription(
        _python_launch(
            "savo_localization",
            "localization_bringup.launch.py",
        ),
        condition=IfCondition(LaunchConfiguration("start_localization")),
        launch_arguments={
            "use_vo": LaunchConfiguration("localization_use_vo"),
            "use_imu": "false",
            "use_wheel_odom": "false",
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
        "initial_scan360_required": LaunchConfiguration(
            "initial_scan360_required"
        ),
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
            "semantic_interruption_enabled": "false",
        }.items(),
    )

    mapping_runtime_launch = IncludeLaunchDescription(
        _frontend_launch("savo_mapping", "autonomous_mapping.launch.xml"),
        condition=IfCondition(LaunchConfiguration("start_mapping")),
        launch_arguments={
            **mapping_common_arguments,
            "start_mapping_foundation": "false",
            "start_mapping_runtime": "true",
            "semantic_interruption_enabled": "false",
        }.items(),
    )

    mapping_semantic_launch = IncludeLaunchDescription(
        _frontend_launch("savo_mapping", "autonomous_mapping.launch.xml"),
        condition=IfCondition(LaunchConfiguration("start_semantic_interruption")),
        launch_arguments={
            **mapping_common_arguments,
            "start_mapping_foundation": "false",
            "start_mapping_runtime": "false",
            "semantic_interruption_enabled": "true",
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
    default_geometry_profile = PathJoinSubstitution(
        [
            FindPackageShare("savo_description"),
            "config",
            "profiles",
            "robot_savo_core_v1.yaml",
        ]
    )
    default_nav_params = PathJoinSubstitution(
        [
            FindPackageShare("savo_nav"),
            "config",
            "nav2_live_mapping.yaml",
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
    startup_stages_config = PathJoinSubstitution(
        [FindPackageShare("savo_bringup"), "config", "startup_stages.yaml"]
    )

    def _build_staged_actions(context):
        enabled = {
            name: as_bool(LaunchConfiguration(name).perform(context))
            for name in (
                "start_description",
                "start_base",
                "start_lidar",
                "start_perception",
                "start_control",
                "start_localization",
                "start_power",
                "start_supervisor",
                "start_navigation",
                "start_mapping",
                "start_head",
                "start_location_lifecycle",
                "start_semantic_interruption",
            )
        }
        coordinator = Node(
            package="savo_bringup",
            executable="bringup_readiness_node",
            name="bringup_readiness_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("savo_bringup"),
                        "config",
                        "core_real_robot.yaml",
                    ]
                ),
                startup_stages_config,
                {
                    "host_role": "core",
                    "robot_mode": "autonomous_mapping",
                    "bringup_profile": LaunchConfiguration(
                        "bringup_profile"
                    ),
                    "d435_voxel_validated": LaunchConfiguration(
                        "d435_voxel_validated"
                    ),
                    "require_locked_geometry": LaunchConfiguration(
                        "require_locked_geometry"
                    ),
                    "allow_provisional_geometry": LaunchConfiguration(
                        "allow_provisional_geometry"
                    ),
                    "startup.enabled": True,
                    "require_geometry": enabled["start_description"],
                    "geometry_policy_validated": True,
                    "require_base": enabled["start_base"],
                    "require_control": enabled["start_control"],
                    "require_safety": enabled["start_perception"],
                    "require_lidar": enabled["start_lidar"],
                    "require_perception": enabled["start_perception"],
                    "require_localization": enabled["start_localization"],
                    "require_power": enabled["start_power"],
                    "require_supervisor": enabled["start_supervisor"],
                    "require_supervisor_authority": False,
                    "require_mapping": enabled["start_mapping"],
                    "mapping_readiness_topic": "/savo_mapping/status",
                    "require_navigation": enabled["start_navigation"],
                    "require_head": enabled["start_head"],
                    "head_status_topic": "/savo_head/dashboard_text",
                    "require_locations": enabled["start_location_lifecycle"],
                    "require_semantic": enabled["start_semantic_interruption"],
                    "require_active_release": False,
                    "require_map_context": False,
                    "require_goal_admission": False,
                    "require_bridge": False,
                    "require_realsense": False,
                    "require_vo": False,
                    "require_speech": False,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )

        groups = [
            StartupStageGroup(
                "infrastructure", (coordinator, description_launch)
            )
        ]
        if any(
            enabled[name]
            for name in (
                "start_power",
                "start_localization",
                "start_perception",
                "start_lidar",
            )
        ):
            groups.append(
                StartupStageGroup(
                    "hardware",
                    (
                        power_launch,
                        localization_hardware_launch,
                        perception_hardware_launch,
                        lidar_launch,
                    ),
                )
            )
        if any(
            enabled[name]
            for name in ("start_base", "start_control", "start_perception")
        ):
            groups.append(
                StartupStageGroup(
                    "motion_safety",
                    (base_launch, control_launch, perception_safety_launch),
                )
            )
        if any(
            enabled[name]
            for name in ("start_localization", "start_perception", "start_lidar")
        ):
            groups.append(StartupStageGroup("sensor_stabilization", ()))
        if enabled["start_localization"]:
            groups.append(
                StartupStageGroup("localization", (localization_filter_launch,))
            )
        if enabled["start_supervisor"]:
            groups.append(StartupStageGroup("supervisor", (supervisor_launch,)))
        if enabled["start_navigation"]:
            groups.append(StartupStageGroup("navigation", (navigation_launch,)))
        if enabled["start_mapping"]:
            groups.append(
                StartupStageGroup("slam_foundation", (mapping_foundation_launch,))
            )
            groups.append(
                StartupStageGroup("mapping_runtime", (mapping_runtime_launch,))
            )
        if enabled["start_head"]:
            groups.append(StartupStageGroup("head", (head_launch,)))
        semantic_location_actions = []
        if enabled["start_location_lifecycle"]:
            semantic_location_actions.append(location_lifecycle_launch)
        if enabled["start_semantic_interruption"]:
            semantic_location_actions.append(mapping_semantic_launch)
        if semantic_location_actions:
            groups.append(
                StartupStageGroup(
                    "semantic_locations", tuple(semantic_location_actions)
                )
            )
        groups.append(StartupStageGroup("complete", ()))
        return build_staged_sequence(
            groups,
            status_topic="/savo_bringup/core/startup_status",
            log_level=log_level,
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
                default_value=default_perception_config,
            ),
            DeclareLaunchArgument(
                "perception_use_ultrasonic",
                default_value="true",
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
                "control_use_control_status",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "control_use_recovery_status",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "localization_use_vo",
                default_value="true",
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
            DeclareLaunchArgument("coverage_enabled", default_value="true"),
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
                "initial_scan360_required", default_value="true"
            ),
            DeclareLaunchArgument(
                "initial_head_scan_required", default_value="true"
            ),
            DeclareLaunchArgument(
                "final_scan360_required", default_value="true"
            ),
            DeclareLaunchArgument(
                "final_head_scan_required", default_value="true"
            ),
            OpaqueFunction(function=_validate_arguments),
            OpaqueFunction(function=_build_staged_actions),
        ]
    )
