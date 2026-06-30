#pragma once

namespace savo_vo
{
namespace constants
{

constexpr const char * kPackageName = "savo_vo";

constexpr const char * kColorImageTopic = "/camera/camera/color/image_raw";
constexpr const char * kColorCameraInfoTopic = "/camera/camera/color/camera_info";
constexpr const char * kDepthImageTopic = "/camera/camera/depth/image_rect_raw";
constexpr const char * kDepthCameraInfoTopic = "/camera/camera/depth/camera_info";

constexpr const char * kVoOdomTopic = "/vo/odom";
constexpr const char * kVoOdomRawTopic = "/vo/odom/raw";
constexpr const char * kVoStatusTopic = "/vo/status";
constexpr const char * kVoHealthTopic = "/vo/health";
constexpr const char * kVoTrackingQualityTopic = "/vo/tracking_quality";
constexpr const char * kDiagnosticsTopic = "/diagnostics";

constexpr const char * kOdomFrame = "odom";
constexpr const char * kBaseFrame = "base_link";
constexpr const char * kCameraFrame = "camera_color_optical_frame";

constexpr const char * kColorImageTopicParam = "color_image_topic";
constexpr const char * kColorCameraInfoTopicParam = "color_camera_info_topic";
constexpr const char * kDepthImageTopicParam = "depth_image_topic";
constexpr const char * kDepthCameraInfoTopicParam = "depth_camera_info_topic";

constexpr const char * kOdomTopicParam = "odom_topic";
constexpr const char * kOdomRawTopicParam = "odom_raw_topic";
constexpr const char * kStatusTopicParam = "status_topic";
constexpr const char * kHealthTopicParam = "health_topic";
constexpr const char * kTrackingQualityTopicParam = "tracking_quality_topic";
constexpr const char * kDiagnosticsTopicParam = "diagnostics_topic";

constexpr const char * kOdomFrameParam = "odom_frame";
constexpr const char * kBaseFrameParam = "base_frame";
constexpr const char * kCameraFrameParam = "camera_frame";

constexpr const char * kStaleTimeoutSParam = "stale_timeout_s";
constexpr const char * kDiagnosticsRateHzParam = "diagnostics_rate_hz";
constexpr const char * kMinTrackingQualityParam = "min_tracking_quality";
constexpr const char * kPublishTfParam = "publish_tf";

constexpr double kDefaultStaleTimeoutS = 0.50;
constexpr double kDefaultDiagnosticsRateHz = 2.0;
constexpr double kDefaultMinTrackingQuality = 0.35;

constexpr bool kDefaultPublishTf = false;

}  // namespace constants
}  // namespace savo_vo
