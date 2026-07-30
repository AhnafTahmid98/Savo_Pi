"""Launch Robot Savo's persistent typed location integration layer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import LogError
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _critical_exit_handler(node: Node, label: str) -> RegisterEventHandler:
    """Shut down the complete layer when a critical process exits."""
    return RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[
                LogError(
                    msg=(
                        f"Critical location integration process exited: "
                        f"{label}"
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            f"critical location integration process exited: "
                            f"{label}"
                        )
                    )
                ),
            ],
        )
    )


def generate_launch_description() -> LaunchDescription:
    """Create the typed location integration launch description."""
    log_level = LaunchConfiguration("log_level")

    start_locations = LaunchConfiguration("start_locations")
    start_supervisor = LaunchConfiguration("start_supervisor")
    start_head_observer = LaunchConfiguration("start_head_observer")
    start_head_action = LaunchConfiguration("start_head_action")
    start_registration = LaunchConfiguration("start_registration")
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
    registration_config = PathJoinSubstitution(
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
        parameters=[locations_config],
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
        parameters=[head_action_config],
        arguments=common_arguments,
    )

    registration_node = Node(
        package="savo_mapping",
        executable="mapped_location_registration_node",
        name="mapped_location_registration_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_registration),
        parameters=[registration_config],
        arguments=common_arguments,
    )

    navigation_node = Node(
        package="savo_nav",
        executable="navigate_to_location_node",
        name="navigate_to_location_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_navigation),
        parameters=[navigation_config],
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
        (navigation_node, "navigate_to_location_node"),
    ]

    actions = [
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS logging level for the location layer.",
        ),
        DeclareLaunchArgument(
            "start_locations",
            default_value="true",
            description="Start the persistent location registry.",
        ),
        DeclareLaunchArgument(
            "start_supervisor",
            default_value="true",
            description="Start deterministic location authorization.",
        ),
        DeclareLaunchArgument(
            "start_head_observer",
            default_value="true",
            description=(
                "Start the existing AprilTag observer that publishes "
                "typed observations."
            ),
        ),
        DeclareLaunchArgument(
            "start_head_action",
            default_value="true",
            description="Start typed AprilTag confirmation action.",
        ),
        DeclareLaunchArgument(
            "start_registration",
            default_value="true",
            description="Start mapped-location registration action.",
        ),
        DeclareLaunchArgument(
            "start_navigation",
            default_value="true",
            description="Start semantic navigation action.",
        ),
        locations_node,
        supervisor_node,
        head_observer_node,
        head_action_node,
        registration_node,
        navigation_node,
    ]

    actions.extend(
        _critical_exit_handler(node, label)
        for node, label in critical_nodes
    )

    return LaunchDescription(actions)
