"""Launch the production Robot Savo core stack for one explicit robot mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from savo_bringup.launch_contract import as_bool
from savo_bringup.launch_contract import resolve_requirements
from savo_bringup.launch_contract import validate_selection


def _python_launch(package: str, filename: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), "launch", filename])
    )


def _frontend_launch(package: str, filename: str):
    return FrontendLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), "launch", filename])
    )


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _setup(context):
    mode = _value(context, "robot_mode")
    profile = _value(context, "bringup_profile")
    require_locked = as_bool(_value(context, "require_locked_geometry"))
    allow_provisional = as_bool(_value(context, "allow_provisional_geometry"))
    voxel_validated = as_bool(_value(context, "d435_voxel_validated"))

    validate_selection(
        "core",
        mode,
        profile,
        d435_voxel_validated=voxel_validated,
        require_locked_geometry=require_locked,
        allow_provisional_geometry=allow_provisional,
    )
    requirements = resolve_requirements(
        "core",
        mode,
        profile,
        start_bridge=False,
        start_realsense=False,
        start_vo=False,
        start_speech=False,
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    diagnostics_mode = mode == "diagnostics"
    if mode == "autonomous_mapping":
        start_description = True
        start_base = True
        start_lidar = True
        start_perception = True
        start_localization = True
        start_control = True
        start_power = True
        start_supervisor = True
    else:
        start_description = as_bool(_value(context, "start_description"))
        start_base = as_bool(_value(context, "start_base")) and not diagnostics_mode
        start_lidar = as_bool(_value(context, "start_lidar"))
        start_perception = as_bool(_value(context, "start_perception"))
        start_localization = as_bool(_value(context, "start_localization"))
        start_control = as_bool(_value(context, "start_control")) and not diagnostics_mode
        start_power = as_bool(_value(context, "start_power"))
        start_supervisor = (
            as_bool(_value(context, "start_supervisor"))
            and not diagnostics_mode
        )
    actions = [
        LogInfo(
            msg=(
                "Robot Savo core bringup validated: "
                f"mode={mode}, profile={profile}"
            )
        )
    ]

    if mode == "autonomous_mapping":
        nav_params = PathJoinSubstitution(
            [
                FindPackageShare("savo_nav"),
                "config",
                (
                    "nav2_live_mapping_voxel.yaml"
                    if requirements.voxel_layer_enabled
                    else "nav2_live_mapping.yaml"
                ),
            ]
        )
        readiness_params = PathJoinSubstitution(
            [
                FindPackageShare("savo_nav"),
                "config",
                (
                    "readiness_realsense_voxel.yaml"
                    if requirements.voxel_layer_enabled
                    else "readiness.yaml"
                ),
            ]
        )
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_bringup", "autonomous_mapping.launch.py"),
                launch_arguments={
                    "map_id": LaunchConfiguration("map_id"),
                    "map_output_root": LaunchConfiguration("map_output_root"),
                    "allow_map_overwrite": LaunchConfiguration(
                        "allow_map_overwrite"
                    ),
                    "use_sim_time": use_sim_time,
                    "log_level": log_level,
                    "geometry_profile": LaunchConfiguration(
                        "geometry_profile"
                    ),
                    "require_locked_geometry": LaunchConfiguration(
                        "require_locked_geometry"
                    ),
                    "allow_provisional_geometry": LaunchConfiguration(
                        "allow_provisional_geometry"
                    ),
                    "control_startup_mode": LaunchConfiguration(
                        "control_startup_mode"
                    ),
                    "localization_use_vo": LaunchConfiguration(
                        "localization_use_vo"
                    ),
                    "head_enable_tf": LaunchConfiguration("head_enable_tf"),
                    "head_camera_mode": LaunchConfiguration(
                        "head_camera_mode"
                    ),
                    "nav_params_file": nav_params,
                    "nav_readiness_params": readiness_params,
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
        )
    else:
        start_head = as_bool(_value(context, "start_head"))
        start_locations = (
            as_bool(_value(context, "start_location_lifecycle"))
            and not diagnostics_mode
        )

        if start_description:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_description", "description.launch.py"),
                    launch_arguments={
                        "geometry_profile": LaunchConfiguration(
                            "geometry_profile"
                        ),
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
            )
        if start_base:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_base", "base_bringup.launch.py"),
                    launch_arguments={
                        "profile": LaunchConfiguration("base_profile"),
                        "driver_impl": "cpp",
                        "use_diag_runner": "false",
                        "output": "screen",
                        "log_level": log_level,
                    }.items(),
                )
            )
        if start_lidar:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_lidar", "lidar_bringup.launch.py"),
                    launch_arguments={
                        "profile": LaunchConfiguration("lidar_profile")
                    }.items(),
                )
            )
        if start_perception:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch(
                        "savo_perception", "perception_bringup.launch.py"
                    ),
                    launch_arguments={
                        "driver_impl": "cpp",
                        "config_file": LaunchConfiguration(
                            "perception_config_file"
                        ),
                        "use_dashboard": "false",
                    }.items(),
                )
            )
        if start_control:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_control", "control_bringup.launch.py"),
                    launch_arguments={
                        "startup_mode": LaunchConfiguration(
                            "control_startup_mode"
                        ),
                        "use_backup_escape": LaunchConfiguration(
                            "control_use_backup_escape"
                        ),
                        "use_stuck_detector": LaunchConfiguration(
                            "control_use_stuck_detector"
                        ),
                        "use_dashboard": "false",
                    }.items(),
                )
            )
        if start_localization:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch(
                        "savo_localization", "localization_bringup.launch.py"
                    ),
                    launch_arguments={
                        "use_vo": LaunchConfiguration("localization_use_vo"),
                        "use_dashboard": "false",
                        "use_state_publisher": "false",
                    }.items(),
                )
            )
        if start_power:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_power", "power_core.launch.py"),
                    launch_arguments={"use_python_fallback": "false"}.items(),
                )
            )
        if start_supervisor:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_supervisor", "supervisor.launch.py"),
                    launch_arguments={
                        "robot_mode": mode,
                        "system_state_path": LaunchConfiguration(
                            "supervisor_state_path"
                        ),
                        "auto_arm": LaunchConfiguration("supervisor_auto_arm"),
                    }.items(),
                )
            )
        if start_head:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch("savo_head", "head_bringup.launch.py"),
                    launch_arguments={
                        "backend": "pca9685",
                        "use_python_fallback": "false",
                        "enable_scan": "true",
                        "enable_tf": LaunchConfiguration("head_enable_tf"),
                        "enable_status": "true",
                        "enable_apriltag_confirm": "true",
                        "center_on_start": "false",
                        "center_on_shutdown": "true",
                        "camera_mode": LaunchConfiguration("head_camera_mode"),
                    }.items(),
                )
            )
        if start_locations:
            actions.append(
                IncludeLaunchDescription(
                    _python_launch(
                        "savo_bringup", "location_integration.launch.py"
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
            )

        if mode == "manual_mapping":
            actions.append(
                IncludeLaunchDescription(
                    _frontend_launch("savo_mapping", "manual_mapping.launch.xml"),
                    launch_arguments={
                        "map_id": LaunchConfiguration("map_id"),
                        "map_output_root": LaunchConfiguration(
                            "map_output_root"
                        ),
                        "allow_map_overwrite": LaunchConfiguration(
                            "allow_map_overwrite"
                        ),
                        "use_sim_time": use_sim_time,
                        "autostart": "true",
                        "semantic_mapping_enabled": "true",
                    }.items(),
                )
            )
        elif mode == "saved_map_navigation":
            nav_params = PathJoinSubstitution(
                [
                    FindPackageShare("savo_nav"),
                    "config",
                    (
                        "nav2_saved_map_voxel.yaml"
                        if requirements.voxel_layer_enabled
                        else "nav2_saved_map.yaml"
                    ),
                ]
            )
            readiness_params = PathJoinSubstitution(
                [
                    FindPackageShare("savo_nav"),
                    "config",
                    (
                        "readiness_realsense_voxel.yaml"
                        if requirements.voxel_layer_enabled
                        else "readiness.yaml"
                    ),
                ]
            )
            actions.append(
                IncludeLaunchDescription(
                    _python_launch(
                        "savo_nav", "production_navigation.launch.py"
                    ),
                    launch_arguments={
                        "production_map_root": LaunchConfiguration(
                            "production_map_root"
                        ),
                        "active_map_contract": LaunchConfiguration(
                            "active_map_contract"
                        ),
                        "geometry_profile": LaunchConfiguration(
                            "geometry_profile"
                        ),
                        "params_file": nav_params,
                        "readiness_params": readiness_params,
                        "use_sim_time": use_sim_time,
                        "autostart": "true",
                        "start_readiness": "true",
                        "start_goal_gateway": "true",
                        "start_map_context_sync": "true",
                        "log_level": log_level,
                    }.items(),
                )
            )

    actions.append(
        Node(
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
                {
                    "host_role": "core",
                    "robot_mode": mode,
                    "bringup_profile": profile,
                    "d435_voxel_validated": voxel_validated,
                    "require_locked_geometry": require_locked,
                    "allow_provisional_geometry": allow_provisional,
                    "require_geometry": start_description,
                    "geometry_policy_validated": True,
                    "require_base": start_base,
                    "require_control": start_control,
                    "require_safety": start_perception,
                    "require_lidar": start_lidar,
                    "require_perception": start_perception,
                    "require_localization": start_localization,
                    "require_power": start_power,
                    "require_supervisor": (
                        requirements.require_supervisor and start_supervisor
                    ),
                    "require_supervisor_authority": mode not in {
                        "safe_idle",
                        "diagnostics",
                    } and start_supervisor,
                    "require_mapping": requirements.require_mapping,
                    "require_navigation": requirements.require_navigation,
                    "require_active_release": mode == "saved_map_navigation",
                    "active_release_verified": mode == "saved_map_navigation",
                    "require_map_context": mode == "saved_map_navigation",
                    "require_goal_admission": requirements.require_navigation,
                    "require_bridge": False,
                    "require_realsense": False,
                    "require_vo": False,
                    "require_speech": False,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the validated core-host production launch interface."""
    description_share = FindPackageShare("savo_description")
    perception_share = FindPackageShare("savo_perception")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_mode", default_value="safe_idle"),
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("d435_voxel_validated", default_value="false"),
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
            DeclareLaunchArgument("require_locked_geometry", default_value="true"),
            DeclareLaunchArgument(
                "allow_provisional_geometry", default_value="false"
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
                "start_location_lifecycle", default_value="true"
            ),
            OpaqueFunction(function=_setup),
        ]
    )
