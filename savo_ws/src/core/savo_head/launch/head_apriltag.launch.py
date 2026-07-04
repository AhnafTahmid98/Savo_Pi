from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_nodes(context):
    use_python_fallback = _as_bool(LaunchConfiguration("use_python_fallback").perform(context))
    enable_tf = _as_bool(LaunchConfiguration("enable_tf").perform(context))
    enable_status = _as_bool(LaunchConfiguration("enable_status").perform(context))

    executable_suffix = "_py" if use_python_fallback else ""

    apriltag_params = {
        "enabled": _as_bool(LaunchConfiguration("enabled").perform(context)),
        "family": LaunchConfiguration("family").perform(context),
        "min_stable_frames": int(LaunchConfiguration("min_stable_frames").perform(context)),
        "min_detection_confidence": float(
            LaunchConfiguration("min_detection_confidence").perform(context)
        ),
        "max_detection_distance_m": float(
            LaunchConfiguration("max_detection_distance_m").perform(context)
        ),
        "max_detection_age_s": float(
            LaunchConfiguration("max_detection_age_s").perform(context)
        ),
        "require_robot_pose": _as_bool(
            LaunchConfiguration("require_robot_pose").perform(context)
        ),
        "require_tf_available": _as_bool(
            LaunchConfiguration("require_tf_available").perform(context)
        ),
        "require_robot_stationary": _as_bool(
            LaunchConfiguration("require_robot_stationary").perform(context)
        ),
        "require_localization_ok": _as_bool(
            LaunchConfiguration("require_localization_ok").perform(context)
        ),
        "require_lidar_map_pose": _as_bool(
            LaunchConfiguration("require_lidar_map_pose").perform(context)
        ),
        "require_semantic_label": _as_bool(
            LaunchConfiguration("require_semantic_label").perform(context)
        ),
        "allow_unknown_tags": _as_bool(
            LaunchConfiguration("allow_unknown_tags").perform(context)
        ),
        "publish_rejections": _as_bool(
            LaunchConfiguration("publish_rejections").perform(context)
        ),
        "registered_tag_ids_csv": LaunchConfiguration("registered_tag_ids_csv").perform(context),
        "tag_param_prefix": LaunchConfiguration("tag_param_prefix").perform(context),
    }

    nodes = [
        Node(
            package="savo_head",
            executable=f"apriltag_confirm_node{executable_suffix}",
            name="apriltag_confirm_node",
            output="screen",
            parameters=[apriltag_params],
        )
    ]

    if enable_status:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_status_node{executable_suffix}",
                name="head_status_node",
                output="screen",
            )
        )

    if enable_tf:
        nodes.append(
            Node(
                package="savo_head",
                executable=f"head_tf_node{executable_suffix}",
                name="head_tf_node",
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_python_fallback",
                default_value="false",
                description="Use Python fallback AprilTag confirmation node instead of C++ default.",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Enable AprilTag semantic confirmation.",
            ),
            DeclareLaunchArgument(
                "enable_tf",
                default_value="false",
                description="Start head TF node together with AprilTag confirmation.",
            ),
            DeclareLaunchArgument(
                "enable_status",
                default_value="true",
                description="Start head status node together with AprilTag confirmation.",
            ),
            DeclareLaunchArgument(
                "family",
                default_value="tag36h11",
                description="AprilTag family used by detections.",
            ),
            DeclareLaunchArgument(
                "min_stable_frames",
                default_value="5",
                description="Minimum stable frames before semantic confirmation.",
            ),
            DeclareLaunchArgument(
                "min_detection_confidence",
                default_value="0.70",
                description="Minimum detection confidence.",
            ),
            DeclareLaunchArgument(
                "max_detection_distance_m",
                default_value="3.0",
                description="Maximum accepted tag distance in meters.",
            ),
            DeclareLaunchArgument(
                "max_detection_age_s",
                default_value="0.50",
                description="Maximum accepted detection age in seconds.",
            ),
            DeclareLaunchArgument(
                "require_robot_pose",
                default_value="true",
                description="Require robot pose snapshot before confirmation.",
            ),
            DeclareLaunchArgument(
                "require_tf_available",
                default_value="true",
                description="Require TF evidence before confirmation.",
            ),
            DeclareLaunchArgument(
                "require_robot_stationary",
                default_value="true",
                description="Require robot to be stationary before confirmation.",
            ),
            DeclareLaunchArgument(
                "require_localization_ok",
                default_value="true",
                description="Require localization quality before confirmation.",
            ),
            DeclareLaunchArgument(
                "require_lidar_map_pose",
                default_value="true",
                description="Require LiDAR/map pose evidence before confirmation.",
            ),
            DeclareLaunchArgument(
                "require_semantic_label",
                default_value="true",
                description="Require a semantic label such as A201.",
            ),
            DeclareLaunchArgument(
                "allow_unknown_tags",
                default_value="false",
                description="Allow tags not listed in registered_tag_ids_csv.",
            ),
            DeclareLaunchArgument(
                "publish_rejections",
                default_value="true",
                description="Publish rejected confirmation attempts for diagnostics.",
            ),
            DeclareLaunchArgument(
                "registered_tag_ids_csv",
                default_value="",
                description="Comma-separated registered tag IDs, for example: 12,13,14.",
            ),
            DeclareLaunchArgument(
                "tag_param_prefix",
                default_value="tag_",
                description="Prefix for future per-tag parameters.",
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )
