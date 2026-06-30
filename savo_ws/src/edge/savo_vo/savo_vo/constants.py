# Copyright 2026 Ahnaf Tahmid

PACKAGE_NAME = "savo_vo"

# RealSense topics are consumed through ROS only.
# savo_vo must not import or link against savo_realsense code.
DEFAULT_COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_COLOR_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

DEFAULT_DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw"
DEFAULT_DEPTH_CAMERA_INFO_TOPIC = "/camera/camera/depth/camera_info"

DEFAULT_VO_ODOM_TOPIC = "/vo/odom"
DEFAULT_VO_ODOM_RAW_TOPIC = "/vo/odom/raw"
DEFAULT_VO_STATUS_TOPIC = "/vo/status"
DEFAULT_VO_HEALTH_TOPIC = "/vo/health"
DEFAULT_VO_TRACKING_QUALITY_TOPIC = "/vo/tracking_quality"
DEFAULT_VO_RESET_TOPIC = "/vo/reset"

DEFAULT_DIAGNOSTICS_TOPIC = "/diagnostics"

DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_CAMERA_FRAME = "camera_color_optical_frame"

DEFAULT_STALE_TIMEOUT_S = 0.50
DEFAULT_MAX_IMAGE_DELAY_S = 0.15
DEFAULT_MAX_DEPTH_DELAY_S = 0.15

DEFAULT_MIN_FEATURES = 80
DEFAULT_GOOD_FEATURES_TARGET = 300
DEFAULT_MAX_FEATURES = 800

DEFAULT_MIN_TRACKING_QUALITY = 0.35
DEFAULT_MAX_TRANSLATION_JUMP_M = 0.30
DEFAULT_MAX_ROTATION_JUMP_RAD = 0.35

DEFAULT_PUBLISH_TF = False
DEFAULT_PUBLISH_DIAGNOSTICS = True
