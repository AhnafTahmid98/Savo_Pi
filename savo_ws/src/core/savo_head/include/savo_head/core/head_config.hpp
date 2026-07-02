#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "savo_head/core/frame_contract.hpp"
#include "savo_head/core/scan_pattern.hpp"
#include "savo_head/core/topic_contract.hpp"
#include "savo_head/drivers/pantilt_driver.hpp"

namespace savo_head
{

inline constexpr const char * kHardwareProfileRobotSavoCoreV1 = "robot_savo_core_v1";

inline constexpr const char * kCameraBackendGstreamerLibcameraSrc = "gstreamer_libcamerasrc";
inline constexpr const char * kCameraBackendOpenCvUnvalidated = "opencv_videocapture_unvalidated";

inline constexpr int kCameraWidthDefault = 640;
inline constexpr int kCameraHeightDefault = 480;
inline constexpr int kCameraFpsDefault = 30;
inline constexpr int kCameraUdpPortDefault = 5000;
inline constexpr int kCameraBitrateKbpsDefault = 2000;

struct HeadHardwareConfig
{
  std::string hardware_profile{kHardwareProfileRobotSavoCoreV1};
  std::string backend{kHeadBackendPca9685};

  int i2c_bus{kI2cBusDefault};
  int pca9685_address{kPca9685AddressDefault};
  double pwm_frequency_hz{kPca9685PwmFrequencyHzDefault};

  HeadServoCalibration calibration{};

  bool center_on_start{false};
  bool center_on_shutdown{true};
  bool stop_on_watchdog_timeout{true};

  double control_hz{kControlHz};
  double status_hz{kStatusHz};
  double watchdog_timeout_s{kWatchdogTimeoutS};

  [[nodiscard]] HeadHardwareConfig normalized() const
  {
    HeadHardwareConfig out = *this;

    if (out.hardware_profile.empty()) {
      out.hardware_profile = kHardwareProfileRobotSavoCoreV1;
    }

    if (!valid_head_backend(out.backend)) {
      out.backend = kHeadBackendDryrun;
    }

    if (out.i2c_bus < 0) {
      out.i2c_bus = kI2cBusDefault;
    }

    if (out.pca9685_address <= 0 || out.pca9685_address > 0x7f) {
      out.pca9685_address = kPca9685AddressDefault;
    }

    if (out.pwm_frequency_hz <= 0.0 || !std::isfinite(out.pwm_frequency_hz)) {
      out.pwm_frequency_hz = kPca9685PwmFrequencyHzDefault;
    }

    out.calibration = out.calibration.normalized();

    out.control_hz = std::max(1.0, out.control_hz);
    out.status_hz = std::max(0.1, out.status_hz);
    out.watchdog_timeout_s = std::max(0.05, out.watchdog_timeout_s);

    return out;
  }

  [[nodiscard]] PanTiltDriverConfig driver_config() const
  {
    const auto item = normalized();

    PanTiltDriverConfig config;
    config.backend = item.backend;
    config.i2c_bus = item.i2c_bus;
    config.pca9685_address = item.pca9685_address;
    config.pwm_frequency_hz = item.pwm_frequency_hz;
    config.calibration = item.calibration;
    config.center_on_open = item.center_on_start;
    config.center_on_close = false;

    return config.normalized();
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (hardware_profile.empty()) {
      errors.emplace_back("hardware_profile must not be empty");
    }

    if (!valid_head_backend(backend)) {
      errors.emplace_back("backend must be pca9685 or dryrun");
    }

    if (i2c_bus < 0) {
      errors.emplace_back("i2c_bus must be non-negative");
    }

    if (pca9685_address <= 0 || pca9685_address > 0x7f) {
      errors.emplace_back("pca9685_address must be a valid 7-bit I2C address");
    }

    if (pwm_frequency_hz <= 0.0 || !std::isfinite(pwm_frequency_hz)) {
      errors.emplace_back("pwm_frequency_hz must be finite and positive");
    }

    for (const auto & error : calibration.validation_errors()) {
      errors.emplace_back("calibration: " + error);
    }

    if (control_hz <= 0.0 || !std::isfinite(control_hz)) {
      errors.emplace_back("control_hz must be finite and positive");
    }

    if (status_hz <= 0.0 || !std::isfinite(status_hz)) {
      errors.emplace_back("status_hz must be finite and positive");
    }

    if (watchdog_timeout_s <= 0.0 || !std::isfinite(watchdog_timeout_s)) {
      errors.emplace_back("watchdog_timeout_s must be finite and positive");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct HeadCameraConfig
{
  std::string camera_backend{kCameraBackendGstreamerLibcameraSrc};

  int width{kCameraWidthDefault};
  int height{kCameraHeightDefault};
  int fps{kCameraFpsDefault};
  std::string format{"I420"};

  std::string gst_source{"libcamerasrc"};
  std::string gst_encoder{"x264enc"};
  std::string gst_payloader{"rtph264pay"};
  std::string gst_sink{"udpsink"};

  std::string udp_host{};
  int udp_port{kCameraUdpPortDefault};
  int bitrate_kbps{kCameraBitrateKbpsDefault};

  bool publish_ros_image{false};
  bool stream_udp{true};

  [[nodiscard]] HeadCameraConfig normalized() const
  {
    HeadCameraConfig out = *this;

    if (out.camera_backend != kCameraBackendGstreamerLibcameraSrc &&
      out.camera_backend != kCameraBackendOpenCvUnvalidated)
    {
      out.camera_backend = kCameraBackendGstreamerLibcameraSrc;
    }

    out.width = std::max(1, out.width);
    out.height = std::max(1, out.height);
    out.fps = std::max(1, out.fps);

    if (out.format.empty()) {
      out.format = "I420";
    }

    if (out.gst_source.empty()) {
      out.gst_source = "libcamerasrc";
    }

    if (out.gst_encoder.empty()) {
      out.gst_encoder = "x264enc";
    }

    if (out.gst_payloader.empty()) {
      out.gst_payloader = "rtph264pay";
    }

    if (out.gst_sink.empty()) {
      out.gst_sink = "udpsink";
    }

    out.udp_port = std::clamp(out.udp_port, 1, 65535);
    out.bitrate_kbps = std::max(100, out.bitrate_kbps);

    return out;
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (camera_backend != kCameraBackendGstreamerLibcameraSrc &&
      camera_backend != kCameraBackendOpenCvUnvalidated)
    {
      errors.emplace_back("camera_backend is unsupported");
    }

    if (width <= 0) {
      errors.emplace_back("camera width must be positive");
    }

    if (height <= 0) {
      errors.emplace_back("camera height must be positive");
    }

    if (fps <= 0) {
      errors.emplace_back("camera fps must be positive");
    }

    if (format.empty()) {
      errors.emplace_back("camera format must not be empty");
    }

    if (gst_source.empty()) {
      errors.emplace_back("gst_source must not be empty");
    }

    if (udp_port <= 0 || udp_port > 65535) {
      errors.emplace_back("udp_port must be in range 1..65535");
    }

    if (bitrate_kbps <= 0) {
      errors.emplace_back("bitrate_kbps must be positive");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct HeadAprilTagConfig
{
  bool enabled{true};
  std::string family{"tag36h11"};

  int min_stable_frames{5};
  double min_detection_confidence{0.70};
  double max_detection_distance_m{3.0};
  double max_detection_age_s{0.50};

  bool require_tf_available{true};
  bool require_robot_stationary{true};
  double max_robot_linear_speed_mps{0.03};
  double max_robot_angular_speed_radps{0.05};

  bool require_localization_ok{true};
  double max_pose_covariance_xy{0.25};
  double max_yaw_covariance{0.20};

  bool require_lidar_map_pose{true};
  bool require_semantic_label{true};
  bool save_as_summon_point_by_default{false};
  bool allow_unknown_tags{false};
  bool publish_rejections{true};
  bool require_robot_pose{true};

  std::string registered_tag_ids_csv{};
  std::string tag_param_prefix{"tag_"};

  [[nodiscard]] HeadAprilTagConfig normalized() const
  {
    HeadAprilTagConfig out = *this;

    if (out.family.empty()) {
      out.family = "tag36h11";
    }

    out.min_stable_frames = std::max(1, out.min_stable_frames);
    out.min_detection_confidence = std::clamp(out.min_detection_confidence, 0.0, 1.0);
    out.max_detection_distance_m = std::max(0.01, out.max_detection_distance_m);
    out.max_detection_age_s = std::max(0.01, out.max_detection_age_s);

    out.max_robot_linear_speed_mps = std::max(0.0, out.max_robot_linear_speed_mps);
    out.max_robot_angular_speed_radps = std::max(0.0, out.max_robot_angular_speed_radps);

    out.max_pose_covariance_xy = std::max(0.0, out.max_pose_covariance_xy);
    out.max_yaw_covariance = std::max(0.0, out.max_yaw_covariance);

    if (out.tag_param_prefix.empty()) {
      out.tag_param_prefix = "tag_";
    }

    return out;
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (family.empty()) {
      errors.emplace_back("AprilTag family must not be empty");
    }

    if (min_stable_frames <= 0) {
      errors.emplace_back("min_stable_frames must be positive");
    }

    if (min_detection_confidence < 0.0 || min_detection_confidence > 1.0 ||
      !std::isfinite(min_detection_confidence))
    {
      errors.emplace_back("min_detection_confidence must be in range 0..1");
    }

    if (max_detection_distance_m <= 0.0 || !std::isfinite(max_detection_distance_m)) {
      errors.emplace_back("max_detection_distance_m must be finite and positive");
    }

    if (max_detection_age_s <= 0.0 || !std::isfinite(max_detection_age_s)) {
      errors.emplace_back("max_detection_age_s must be finite and positive");
    }

    if (max_robot_linear_speed_mps < 0.0 || !std::isfinite(max_robot_linear_speed_mps)) {
      errors.emplace_back("max_robot_linear_speed_mps must be finite and non-negative");
    }

    if (max_robot_angular_speed_radps < 0.0 || !std::isfinite(max_robot_angular_speed_radps)) {
      errors.emplace_back("max_robot_angular_speed_radps must be finite and non-negative");
    }

    if (max_pose_covariance_xy < 0.0 || !std::isfinite(max_pose_covariance_xy)) {
      errors.emplace_back("max_pose_covariance_xy must be finite and non-negative");
    }

    if (max_yaw_covariance < 0.0 || !std::isfinite(max_yaw_covariance)) {
      errors.emplace_back("max_yaw_covariance must be finite and non-negative");
    }

    if (tag_param_prefix.empty()) {
      errors.emplace_back("tag_param_prefix must not be empty");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct HeadDiagnosticsConfig
{
  bool enabled{true};
  double status_publish_hz{kStatusHz};

  double state_stale_timeout_s{0.50};
  double scan_stale_timeout_s{1.00};
  double camera_stream_stale_timeout_s{2.00};
  double apriltag_detection_stale_timeout_s{1.00};

  bool require_pan_tilt_for_ok{true};
  bool require_scan_for_ok{false};
  bool require_camera_for_ok{false};
  bool require_apriltag_for_ok{false};

  [[nodiscard]] HeadDiagnosticsConfig normalized() const
  {
    HeadDiagnosticsConfig out = *this;

    out.status_publish_hz = std::max(0.1, out.status_publish_hz);
    out.state_stale_timeout_s = std::max(0.05, out.state_stale_timeout_s);
    out.scan_stale_timeout_s = std::max(0.05, out.scan_stale_timeout_s);
    out.camera_stream_stale_timeout_s = std::max(0.05, out.camera_stream_stale_timeout_s);
    out.apriltag_detection_stale_timeout_s =
      std::max(0.05, out.apriltag_detection_stale_timeout_s);

    return out;
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (status_publish_hz <= 0.0 || !std::isfinite(status_publish_hz)) {
      errors.emplace_back("status_publish_hz must be finite and positive");
    }

    if (state_stale_timeout_s <= 0.0 || !std::isfinite(state_stale_timeout_s)) {
      errors.emplace_back("state_stale_timeout_s must be finite and positive");
    }

    if (scan_stale_timeout_s <= 0.0 || !std::isfinite(scan_stale_timeout_s)) {
      errors.emplace_back("scan_stale_timeout_s must be finite and positive");
    }

    if (camera_stream_stale_timeout_s <= 0.0 ||
      !std::isfinite(camera_stream_stale_timeout_s))
    {
      errors.emplace_back("camera_stream_stale_timeout_s must be finite and positive");
    }

    if (apriltag_detection_stale_timeout_s <= 0.0 ||
      !std::isfinite(apriltag_detection_stale_timeout_s))
    {
      errors.emplace_back("apriltag_detection_stale_timeout_s must be finite and positive");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct HeadConfig
{
  HeadHardwareConfig hardware{};
  HeadTopicContract topics{};
  HeadQosContract qos{};
  HeadFrameContract frames{};
  ScanProfile scan{};
  HeadCameraConfig camera{};
  HeadAprilTagConfig apriltag{};
  HeadDiagnosticsConfig diagnostics{};

  [[nodiscard]] HeadConfig normalized() const
  {
    HeadConfig out = *this;
    out.hardware = out.hardware.normalized();
    out.scan = out.scan.normalized();
    out.camera = out.camera.normalized();
    out.apriltag = out.apriltag.normalized();
    out.diagnostics = out.diagnostics.normalized();
    return out;
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    append_errors(errors, "hardware", hardware.validation_errors());
    append_errors(errors, "topics", topics.validation_errors());
    append_errors(errors, "qos", qos.validation_errors());
    append_errors(errors, "frames", frames.validation_errors());
    append_errors(errors, "scan", scan.validation_errors());
    append_errors(errors, "camera", camera.validation_errors());
    append_errors(errors, "apriltag", apriltag.validation_errors());
    append_errors(errors, "diagnostics", diagnostics.validation_errors());

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

private:
  static void append_errors(
    std::vector<std::string> & target,
    const std::string & source,
    const std::vector<std::string> & errors)
  {
    for (const auto & error : errors) {
      target.emplace_back(source + ": " + error);
    }
  }
};

[[nodiscard]] inline HeadConfig default_head_config()
{
  return HeadConfig{}.normalized();
}

}  // namespace savo_head
