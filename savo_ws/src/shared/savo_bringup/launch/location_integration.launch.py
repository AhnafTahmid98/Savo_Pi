"""Launch Robot Savo's persistent typed location lifecycle layer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _boolean(name: str) -> ParameterValue:
    """Return a launch configuration interpreted as a ROS boolean."""
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _floating(name: str) -> ParameterValue:
    """Return a launch configuration interpreted as a ROS floating value."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _integer(name: str) -> ParameterValue:
    """Return a launch configuration interpreted as a ROS integer."""
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _critical_exit_handler(node: Node, label: str) -> RegisterEventHandler:
    """Shut down the complete layer when a critical process exits."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                LogInfo(
                    msg=(
                        f"Critical location lifecycle process exited: "
                        f"{label}"
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            f"critical location lifecycle process exited: "
                            f"{label}"
                        )
                    )
                ),
            ],
        )
    )


def generate_launch_description() -> LaunchDescription:
    """Create the typed location lifecycle launch description."""
    log_level = LaunchConfiguration("log_level")

    start_locations = LaunchConfiguration("start_locations")
    start_supervisor = LaunchConfiguration("start_supervisor")
    start_head_observer = LaunchConfiguration("start_head_observer")
    start_head_action = LaunchConfiguration("start_head_action")
    start_registration = LaunchConfiguration("start_registration")
    start_review_gateway = LaunchConfiguration("start_review_gateway")
    start_navigation = LaunchConfiguration("start_navigation")

    locations_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_locations"),
            "config",
            "locations_node.yaml",
        ]
    )
    supervisor_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_supervisor"),
            "config",
            "supervisor.yaml",
        ]
    )
    location_authorization_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_supervisor"),
            "config",
            "location_authorization.yaml",
        ]
    )
    head_observer_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_head"),
            "config",
            "apriltag_semantics.yaml",
        ]
    )
    head_action_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_head"),
            "config",
            "apriltag_confirmation_action.yaml",
        ]
    )
    semantic_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_mapping"),
            "config",
            "semantic_landmarks.yaml",
        ]
    )
    navigation_config = PathJoinSubstitution(
        [
            FindPackageShare("savo_nav"),
            "config",
            "location_navigation.yaml",
        ]
    )

    common_arguments = [
        "--ros-args",
        "--log-level",
        log_level,
    ]

    locations_node = Node(
        package="savo_locations",
        executable="savo_locations_node",
        name="savo_locations",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_locations),
        parameters=[
            locations_config,
            {
                "database_path": LaunchConfiguration(
                    "locations_database_path"
                ),
                "releases_root": LaunchConfiguration(
                    "locations_releases_root"
                ),
                "create_parent_directories": _boolean(
                    "locations_create_parent_directories"
                ),
            },
        ],
        arguments=common_arguments,
    )

    supervisor_node = Node(
        package="savo_supervisor",
        executable="supervisor_node",
        name="savo_supervisor_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_supervisor),
        parameters=[
            supervisor_config,
            location_authorization_config,
            {
                "startup_grace_s": _floating(
                    "supervisor_startup_grace_s"
                ),
                "location_authorization.allow_degraded_motion": _boolean(
                    "supervisor_allow_degraded_motion"
                ),
                "base.enabled": _boolean(
                    "supervisor_base_enabled"
                ),
                "base.required": _boolean(
                    "supervisor_base_required"
                ),
                "control.enabled": _boolean(
                    "supervisor_control_enabled"
                ),
                "control.required": _boolean(
                    "supervisor_control_required"
                ),
                "perception.enabled": _boolean(
                    "supervisor_perception_enabled"
                ),
                "perception.required": _boolean(
                    "supervisor_perception_required"
                ),
                "lidar.enabled": _boolean(
                    "supervisor_lidar_enabled"
                ),
                "lidar.required": _boolean(
                    "supervisor_lidar_required"
                ),
                "power.enabled": _boolean(
                    "supervisor_power_enabled"
                ),
                "power.required": _boolean(
                    "supervisor_power_required"
                ),
                "localization.enabled": _boolean(
                    "supervisor_localization_enabled"
                ),
                "localization.required": _boolean(
                    "supervisor_localization_required"
                ),
            },
        ],
        arguments=common_arguments,
    )

    head_observer_node = Node(
        package="savo_head",
        executable="apriltag_confirm_node",
        name="apriltag_confirm_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_head_observer),
        parameters=[head_observer_config],
        arguments=common_arguments,
    )

    head_action_node = Node(
        package="savo_head",
        executable="apriltag_confirmation_action_node",
        name="apriltag_confirmation_action_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_head_action),
        parameters=[
            head_action_config,
            {
                "minimum_observations": _integer(
                    "head_minimum_observations"
                ),
                "maximum_observation_age_s": _floating(
                    "head_maximum_observation_age_s"
                ),
                "wrong_tag_grace_s": _floating(
                    "head_wrong_tag_grace_s"
                ),
            },
        ],
        arguments=common_arguments,
    )

    registration_node = Node(
        package="savo_mapping",
        executable="mapped_location_registration_node",
        name="mapped_location_registration_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_registration),
        parameters=[
            semantic_config,
            {
                "dependency_wait_timeout_s": _floating(
                    "registration_dependency_wait_timeout_s"
                ),
            },
        ],
        arguments=common_arguments,
    )

    review_gateway_node = Node(
        package="savo_mapping",
        executable="location_review_gateway_node",
        name="location_review_gateway_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_review_gateway),
        parameters=[
            semantic_config,
            {
                "dependency_wait_timeout_s": _floating(
                    "review_dependency_wait_timeout_s"
                ),
                "operation_timeout_s": _floating(
                    "review_operation_timeout_s"
                ),
            },
        ],
        arguments=common_arguments,
    )

    navigation_node = Node(
        package="savo_nav",
        executable="navigate_to_location_node",
        name="navigate_to_location_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_navigation),
        parameters=[
            navigation_config,
            {
                "navigation_action_name": LaunchConfiguration(
                    "navigation_action_name"
                ),
                "dependency_wait_timeout_s": _floating(
                    "navigation_dependency_wait_timeout_s"
                ),
                "authorization_recheck_period_s": _floating(
                    "navigation_authorization_recheck_period_s"
                ),
                "arrival_confirmation_timeout_s": _floating(
                    "arrival_confirmation_timeout_s"
                ),
            },
        ],
        arguments=common_arguments,
    )

    critical_nodes = [
        (locations_node, "savo_locations"),
        (supervisor_node, "savo_supervisor_node"),
        (head_observer_node, "apriltag_confirm_node"),
        (
            head_action_node,
            "apriltag_confirmation_action_node",
        ),
        (
            registration_node,
            "mapped_location_registration_node",
        ),
        (
            review_gateway_node,
            "location_review_gateway_node",
        ),
        (navigation_node, "navigate_to_location_node"),
    ]

    arguments = [
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS logging level for the location lifecycle.",
        ),
        DeclareLaunchArgument("start_locations", default_value="true"),
        DeclareLaunchArgument("start_supervisor", default_value="true"),
        DeclareLaunchArgument("start_head_observer", default_value="true"),
        DeclareLaunchArgument("start_head_action", default_value="true"),
        DeclareLaunchArgument("start_registration", default_value="true"),
        DeclareLaunchArgument("start_review_gateway", default_value="true"),
        DeclareLaunchArgument("start_navigation", default_value="true"),
        DeclareLaunchArgument(
            "locations_database_path",
            default_value="/var/lib/robot_savo/locations/locations.db",
        ),
        DeclareLaunchArgument(
            "locations_releases_root",
            default_value="/var/lib/robot_savo/locations/releases",
        ),
        DeclareLaunchArgument(
            "locations_create_parent_directories",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "supervisor_startup_grace_s",
            default_value="3.0",
        ),
        DeclareLaunchArgument(
            "supervisor_allow_degraded_motion",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "supervisor_base_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_base_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_control_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_control_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_perception_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_perception_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_lidar_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_lidar_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_power_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_power_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_localization_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "supervisor_localization_required",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "head_minimum_observations",
            default_value="5",
        ),
        DeclareLaunchArgument(
            "head_maximum_observation_age_s",
            default_value="0.5",
        ),
        DeclareLaunchArgument(
            "head_wrong_tag_grace_s",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "registration_dependency_wait_timeout_s",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "review_dependency_wait_timeout_s",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "review_operation_timeout_s",
            default_value="5.0",
        ),
        DeclareLaunchArgument(
            "navigation_action_name",
            default_value="/savo_nav/navigation/navigate_to_pose",
        ),
        DeclareLaunchArgument(
            "navigation_dependency_wait_timeout_s",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "navigation_authorization_recheck_period_s",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "arrival_confirmation_timeout_s",
            default_value="15.0",
        ),
    ]

    actions = [
        *arguments,
        locations_node,
        supervisor_node,
        head_observer_node,
        head_action_node,
        registration_node,
        review_gateway_node,
        navigation_node,
    ]
    actions.extend(
        _critical_exit_handler(node, label)
        for node, label in critical_nodes
    )

    return LaunchDescription(actions)
