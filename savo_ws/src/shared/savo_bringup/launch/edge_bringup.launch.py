"""Launch the production Robot Savo edge stack."""

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
    voxel_validated = as_bool(_value(context, "d435_voxel_validated"))
    require_locked = as_bool(_value(context, "require_locked_geometry"))
    allow_provisional = as_bool(_value(context, "allow_provisional_geometry"))

    start_realsense = as_bool(_value(context, "start_realsense"))
    start_vo = as_bool(_value(context, "start_vo"))
    start_obstacle_cloud = (
        as_bool(_value(context, "start_obstacle_cloud"))
        or profile == "lidar_d435_voxel"
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
    if start_obstacle_cloud and not start_realsense:
        raise RuntimeError("start_obstacle_cloud requires start_realsense")
    requirements = resolve_requirements(
        "edge",
        mode,
        profile,
        start_bridge=start_bridge,
        start_realsense=start_realsense,
        start_vo=start_vo,
        start_speech=start_speech,
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
        camera_config = PathJoinSubstitution(
            [
                FindPackageShare("savo_realsense"),
                "config",
                (
                    "realsense_pointcloud_camera.yaml"
                    if requirements.voxel_layer_enabled
                    else "realsense_d435_camera.yaml"
                ),
            ]
        )
        nodes_config = PathJoinSubstitution(
            [
                FindPackageShare("savo_realsense"),
                "config",
                (
                    "realsense_pointcloud_nodes.yaml"
                    if requirements.voxel_layer_enabled
                    else "realsense_d435_nodes.yaml"
                ),
            ]
        )
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_realsense", "realsense_bringup.launch.py"),
                launch_arguments={
                    "camera_config_file": camera_config,
                    "nodes_config_file": nodes_config,
                    "use_depth_front_min": "true",
                }.items(),
            )
        )

    if start_vo:
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_vo", "vo_bringup.launch.py"),
                launch_arguments={
                    "implementation": "cpp",
                    "profile": LaunchConfiguration("vo_profile"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )

    if start_obstacle_cloud:
        actions.append(
            IncludeLaunchDescription(
                _python_launch(
                    "savo_perception", "obstacle_cloud_filter.launch.py"
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }.items(),
            )
        )

    if start_speech:
        actions.append(
            IncludeLaunchDescription(
                _frontend_launch("savo_speech", "speech_bringup.launch.xml"),
                launch_arguments={
                    "params_file": LaunchConfiguration("speech_params_file")
                }.items(),
            )
        )

    if start_ui:
        actions.append(
            IncludeLaunchDescription(
                _python_launch("savo_ui", "ui_bringup.launch.py"),
                launch_arguments={"profile": LaunchConfiguration("ui_profile")}.items(),
            )
        )

    if start_bridge:
        actions.append(
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

    actions.append(
        Node(
            package="savo_bringup",
            executable="bringup_readiness_node",
            name="edge_bringup_readiness_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("savo_bringup"),
                        "config",
                        "edge_real_robot.yaml",
                    ]
                ),
                {
                    "host_role": "edge",
                    "robot_mode": mode,
                    "bringup_profile": profile,
                    "d435_voxel_validated": voxel_validated,
                    "require_locked_geometry": require_locked,
                    "allow_provisional_geometry": allow_provisional,
                    "require_geometry": False,
                    "require_power": start_power,
                    "require_supervisor": False,
                    "require_navigation": False,
                    "require_bridge": requirements.require_bridge,
                    "require_realsense": requirements.require_realsense,
                    "require_vo": requirements.require_vo,
                    "require_speech": requirements.require_speech,
                    "require_ui": start_ui,
                    "require_obstacle_cloud": start_obstacle_cloud,
                },
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
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
            DeclareLaunchArgument("start_speech", default_value="false"),
            DeclareLaunchArgument("start_ui", default_value="false"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_power", default_value="true"),
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
