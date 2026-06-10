"""ROS topic names used by savo_vo."""

COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
COLOR_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC = "/camera/camera/depth/camera_info"

VO_ODOM_TOPIC = "/vo/odom"
VO_ODOM_RAW_TOPIC = "/vo/odom/raw"
VO_STATUS_TOPIC = "/vo/status"
VO_HEALTH_TOPIC = "/vo/health"
VO_TRACKING_QUALITY_TOPIC = "/vo/tracking_quality"
VO_RESET_TOPIC = "/vo/reset"

DIAGNOSTICS_TOPIC = "/diagnostics"

__all__ = [
    "COLOR_IMAGE_TOPIC",
    "COLOR_CAMERA_INFO_TOPIC",
    "DEPTH_IMAGE_TOPIC",
    "DEPTH_CAMERA_INFO_TOPIC",
    "VO_ODOM_TOPIC",
    "VO_ODOM_RAW_TOPIC",
    "VO_STATUS_TOPIC",
    "VO_HEALTH_TOPIC",
    "VO_TRACKING_QUALITY_TOPIC",
    "VO_RESET_TOPIC",
    "DIAGNOSTICS_TOPIC",
]