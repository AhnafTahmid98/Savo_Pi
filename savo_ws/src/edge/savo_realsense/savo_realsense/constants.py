# Copyright 2026 Ahnaf Tahmid
PACKAGE_NAME = "savo_realsense"

# RealSense ROS 2 wrapper uses a nested namespace by default.
DEFAULT_CAMERA_NAMESPACE = "/camera/camera"

DEFAULT_COLOR_IMAGE_TOPIC = f"{DEFAULT_CAMERA_NAMESPACE}/color/image_raw"
DEFAULT_COLOR_INFO_TOPIC = f"{DEFAULT_CAMERA_NAMESPACE}/color/camera_info"

DEFAULT_DEPTH_IMAGE_TOPIC = f"{DEFAULT_CAMERA_NAMESPACE}/depth/image_rect_raw"
DEFAULT_DEPTH_INFO_TOPIC = f"{DEFAULT_CAMERA_NAMESPACE}/depth/camera_info"

DEFAULT_POINTCLOUD_TOPIC = f"{DEFAULT_CAMERA_NAMESPACE}/depth/color/points"

DEFAULT_CAMERA_STATUS_TOPIC = "/realsense/status"
DEFAULT_DIAGNOSTICS_TOPIC = "/diagnostics"

DEFAULT_BASE_FRAME = "base_link"
DEFAULT_CAMERA_LINK_FRAME = "camera_link"
DEFAULT_COLOR_OPTICAL_FRAME = "camera_color_optical_frame"
DEFAULT_DEPTH_OPTICAL_FRAME = "camera_depth_optical_frame"

# 0.50s gives the camera stream some tolerance without hiding real dropouts.
DEFAULT_STALE_TIMEOUT_S = 0.50
DEFAULT_STATUS_HZ = 2.0

DEFAULT_EXPECTED_COLOR_HZ = 30.0
DEFAULT_EXPECTED_DEPTH_HZ = 30.0
DEFAULT_EXPECTED_CAMERA_INFO_HZ = 30.0
DEFAULT_EXPECTED_POINTCLOUD_HZ = 10.0

# Matches the valid depth window used by Robot Savo near-field perception.
DEFAULT_MIN_VALID_DEPTH_M = 0.02
DEFAULT_MAX_VALID_DEPTH_M = 3.00
DEFAULT_FRONT_DEPTH_PERCENTILE = 10.0

# Intel USB vendor id.
DEFAULT_USB_VENDOR_ID = "8086"
DEFAULT_USB_PRODUCT_HINT = "RealSense"
