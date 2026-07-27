# Robot Savo — one camera-derived AprilTag observation.
#
# This message contains detector evidence only.
# It does not:
# - create or approve a semantic location;
# - declare navigation success;
# - command the head or robot base.

# Timestamp of the camera frame and camera optical frame ID.
# The normal production frame is:
#   head_camera_optical_frame
std_msgs/Header header

# Detector implementation and AprilTag identity.
string detector_name
string family
int32 tag_id

# Physical black-square edge length used for pose estimation, in metres.
#
# This must match the measured printed black-square size.
# Zero means that no metric pose was calculated.
float64 tag_size_m

# Monotonically increasing observation sequence assigned by the detector.
# This helps the confirmation node reject duplicate observations.
uint64 observation_sequence

# Dimensions of the image used for this detection.
uint32 image_width
uint32 image_height

# Four detected image-space corners:
#
# [x0, y0,
#  x1, y1,
#  x2, y2,
#  x3, y3]
#
# Units are pixels. Corner ordering must remain consistent for every
# observation produced by the detector.
float32[8] image_corners_xy

# Detector quality evidence.
#
# detection_quality:
#   Normalized detector quality in the range 0.0 to 1.0.
#
# decision_margin:
#   Raw AprilTag decision-separation metric.
#
# hamming_distance:
#   Number of corrected code bits.
float32 detection_quality
float32 decision_margin
uint8 hamming_distance

# Pose of the AprilTag relative to header.frame_id.
#
# The confirmation node must still validate:
# - timestamp freshness;
# - repeated-observation stability;
# - head settling;
# - robot motion;
# - TF availability;
# - localization health.
bool pose_valid
geometry_msgs/PoseWithCovariance pose

# Detector-specific pose-estimation error.
# Lower values represent a better estimate.
float32 pose_error
