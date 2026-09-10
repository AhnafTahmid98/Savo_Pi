"""Launch the production Robot Savo edge stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from savo_bringup.launch_contract import as_bool
from savo_bringup.launch_contract import should_start_obstacle_cloud
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
    voxel_validated = as_bool(_value(context, "d435_voxel_validated"))
    require_locked = as_bool(_value(context, "require_locked_geometry"))
    allow_provisional = as_bool(_value(context, "allow_provisional_geometry"))

    start_realsense = as_bool(_value(context, "start_realsense"))
    start_vo = as_bool(_value(context, "start_vo"))
    enable_observer_color_relay = as_bool(
        _value(context, "enable_observer_color_relay")
    )
    explicit_obstacle_cloud = as_bool(
        _value(context, "start_obstacle_cloud")
    )
    obstacle_cloud_requested = should_start_obstacle_cloud(
        mode,
        profile,
        d435_voxel_validated=voxel_validated,
        explicit_start=explicit_obstacle_cloud,
    )
    start_obstacle_cloud = (
        start_realsense
        and explicit_obstacle_cloud
        and obstacle_cloud_requested
    )
    start_speech = as_bool(_value(context, "start_speech"))
    start_ui = as_bool(_value(context, "start_ui"))
    start_bridge = as_bool(_value(context, "start_bridge"))
    start_power = as_bool(_value(context, "start_power"))

    validate_selection(
        "edge",
        mode,
        profile,
        d435_voxel_validated=voxel_validated,
        require_locked_geometry=require_locked,
        allow_provisional_geometry=allow_provisional,
    )
    if start_vo and not start_realsense:
        raise RuntimeError("start_vo requires start_realsense")
    if explicit_obstacle_cloud and not start_realsense:
        raise RuntimeError("start_obstacle_cloud requires start_realsense")
    if explicit_obstacle_cloud and not voxel_validated:
        raise RuntimeError(
            "start_obstacle_cloud requires d435_voxel_validated:=true"
        )
    actions = [
        LogInfo(
            msg=(
                "Robot Savo edge bringup validated: "
                f"mode={mode}, profile={profile}"
            )
        )
    ]
    if start_realsense:
        if start_obstacle_cloud:
            camera_config_name = "realsense_d435_camera.yaml"
        elif start_vo:
            camera_config_name = "realsense_vo_driver.yaml"
        else:
            camera_config_name = "realsense_minimal.yaml"
        camera_config = PathJoinSubstitution(
            [
                FindPackageShare("savo_realsense"),
                "config",
                camera_config_name,
            ]
        )
        nodes_config = PathJoinSubstitution(
            [
                FindPackageShare("savo_realsense"),
                "config",
                "realsense_d435_nodes.yaml",
            ]
        )
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_realsense", "realsense_bringup.launch.py"),
                launch_arguments={
                    "camera_config_file": camera_config,
                    "nodes_config_file": nodes_config,
                    "use_depth_front_min": "true",
                    "require_vo_health": "false",
                    "require_obstacle_cloud_health": "false",
                    "enable_observer_color_relay": (
                        "true" if enable_observer_color_relay else "false"
                    ),
                    "realsense_start_delay_s": LaunchConfiguration(
                        "realsense_start_delay_s"
                    ),
                    "camera_support_start_delay_s": LaunchConfiguration(
                        "camera_support_start_delay_s"
                    ),
                    "observer_relay_start_delay_s": LaunchConfiguration(
                        "observer_relay_start_delay_s"
                    ),
                }.items(),
            )
        )

    if start_vo:
        actions.append(
            TimerAction(
                period=LaunchConfiguration("vo_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        _python_launch("savo_vo", "vo_bringup.launch.py"),
                        launch_arguments={
                            "implementation": "cpp",
                            "profile": LaunchConfiguration("vo_profile"),
                            "log_level": LaunchConfiguration("log_level"),
                        }.items(),
                    )
                ],
                cancel_on_shutdown=True,
            )
        )

    if start_obstacle_cloud:
        actions.append(
            TimerAction(
                period=LaunchConfiguration("obstacle_cloud_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        _python_launch(
                            "savo_perception",
                            "obstacle_cloud_filter.launch.py",
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time")
                        }.items(),
                    )
                ],
                cancel_on_shutdown=True,
            )
        )

    if start_speech:
        actions.append(
            TimerAction(
                period=LaunchConfiguration("speech_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        _frontend_launch(
                            "savo_speech", "speech_bringup.launch.xml"
                        ),
                        launch_arguments={
                            "params_file": LaunchConfiguration("speech_params_file")
                        }.items(),
                    )
                ],
                cancel_on_shutdown=True,
            )
        )

    if start_ui:
        actions.append(
            TimerAction(
                period=LaunchConfiguration("ui_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        _python_launch("savo_ui", "ui_bringup.launch.py"),
                        launch_arguments={
                            "profile": LaunchConfiguration("ui_profile")
                        }.items(),
                    )
                ],
                cancel_on_shutdown=True,
            )
        )

    if start_bridge:
        actions.append(
            TimerAction(
                period=LaunchConfiguration("bridge_start_delay_s"),
                actions=[
                    IncludeLaunchDescription(
                        _python_launch("savo_bridge", "edge_bridge.launch.py"),
                        launch_arguments={
                            "robot_mode": mode,
                            "active_map_id": LaunchConfiguration("active_map_id"),
                            "active_map_revision": LaunchConfiguration(
                                "active_map_revision"
                            ),
                        }.items(),
                    )
                ],
                cancel_on_shutdown=True,
            )
        )

    if start_power:
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_power", "power_edge.launch.py"),
                launch_arguments={
                    "use_python_fallback": "false",
                    "enable_power_health": "false",
                    "enable_power_dashboard": "false",
                }.items(),
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the validated edge-host production launch interface."""
    speech_share = FindPackageShare("savo_speech")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_mode", default_value="safe_idle"),
            DeclareLaunchArgument("bringup_profile", default_value="lidar_only"),
            DeclareLaunchArgument("d435_voxel_validated", default_value="false"),
            DeclareLaunchArgument("require_locked_geometry", default_value="true"),
            DeclareLaunchArgument(
                "allow_provisional_geometry", default_value="false"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("start_vo", default_value="true"),
            DeclareLaunchArgument("start_obstacle_cloud", default_value="false"),
            DeclareLaunchArgument(
                "enable_observer_color_relay", default_value="false"
            ),
            DeclareLaunchArgument("start_speech", default_value="false"),
            DeclareLaunchArgument("start_ui", default_value="false"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_power", default_value="false"),
            DeclareLaunchArgument(
                "realsense_start_delay_s", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "camera_support_start_delay_s", default_value="10.0"
            ),
            DeclareLaunchArgument("vo_start_delay_s", default_value="22.0"),
            DeclareLaunchArgument(
                "obstacle_cloud_start_delay_s", default_value="34.0"
            ),
            DeclareLaunchArgument(
                "observer_relay_start_delay_s", default_value="40.0"
            ),
            DeclareLaunchArgument(
                "speech_start_delay_s", default_value="46.0"
            ),
            DeclareLaunchArgument(
                "ui_start_delay_s", default_value="50.0"
            ),
            DeclareLaunchArgument(
                "bridge_start_delay_s", default_value="54.0"
            ),
            DeclareLaunchArgument(
                "readiness_start_delay_s", default_value="60.0"
            ),
            DeclareLaunchArgument("vo_profile", default_value="real_robot_v1"),
            DeclareLaunchArgument("ui_profile", default_value="pi"),
            DeclareLaunchArgument(
                "speech_params_file",
                default_value=PathJoinSubstitution(
                    [
                        speech_share,
                        "config",
                        "profiles",
                        "edge_real_robot_v1.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("active_map_id", default_value=""),
            DeclareLaunchArgument("active_map_revision", default_value="0"),
            OpaqueFunction(function=_setup),
        ]
    )
