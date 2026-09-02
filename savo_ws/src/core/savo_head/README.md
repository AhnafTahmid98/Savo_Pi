# Robot Savo Pi Camera 2 NoIR on Ubuntu

Robot Savo Pi Camera 2 NoIR on Ubuntu is validated with GStreamer libcamerasrc.
OpenCV VideoCapture is not validated yet.
rpicam/libcamera command-based streaming was tested and did not work on this Ubuntu setup.
Do not replace the validated libcamerasrc backend without a separate hardware validation test

Production camera health is intentionally lightweight. ROS 2 gscam publishes
`CameraInfo` with each successfully pulled image, so
`head_camera_status_node` measures freshness, cadence, frame identity,
resolution metadata, and calibration from `/savo_head/camera/camera_info`.
It checks `/savo_head/camera/image_raw` publisher presence through the ROS graph
without subscribing to image or compressed-image payloads. Missing publishers,
missing/stale metadata, and invalid timestamps remain fail-closed after the
configured startup grace period. The configured `rgb8` encoding is a static
launch/config contract because encoding cannot be observed without consuming
image payloads.
