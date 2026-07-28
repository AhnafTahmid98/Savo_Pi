#include "savo_head/core/camera_health.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace savo_head
{

namespace
{

CameraHealthResult error_result(std::string reason)
{
  return CameraHealthResult{
    CameraHealthLevel::kError,
    "ERROR",
    std::move(reason),
    false,
    false
  };
}

CameraHealthResult warning_result(
  std::string reason,
  bool stream_healthy,
  bool ready_for_pose_estimation = false)
{
  return CameraHealthResult{
    CameraHealthLevel::kWarn,
    "WARN",
    std::move(reason),
    stream_healthy,
    ready_for_pose_estimation
  };
}

CameraHealthResult ok_result()
{
  return CameraHealthResult{
    CameraHealthLevel::kOk,
    "OK",
    "camera_ready",
    true,
    true
  };
}

double non_negative_age(double now_s, double receipt_s)
{
  if (
    !std::isfinite(now_s) ||
    !std::isfinite(receipt_s) ||
    receipt_s <= 0.0)
  {
    return 0.0;
  }

  return std::max(0.0, now_s - receipt_s);
}

}  // namespace

std::vector<std::string> CameraHealthConfig::validation_errors() const
{
  std::vector<std::string> errors;

  if (expected_width == 0U) {
    errors.emplace_back("expected_width must be positive");
  }

  if (expected_height == 0U) {
    errors.emplace_back("expected_height must be positive");
  }

  if (expected_frame_id.empty()) {
    errors.emplace_back("expected_frame_id must not be empty");
  }

  if (expected_encoding.empty()) {
    errors.emplace_back("expected_encoding must not be empty");
  }

  if (!std::isfinite(startup_grace_s) || startup_grace_s < 0.0) {
    errors.emplace_back(
      "startup_grace_s must be finite and non-negative");
  }

  if (
    !std::isfinite(image_stale_timeout_s) ||
    image_stale_timeout_s <= 0.0)
  {
    errors.emplace_back(
      "image_stale_timeout_s must be finite and positive");
  }

  if (
    !std::isfinite(camera_info_stale_timeout_s) ||
    camera_info_stale_timeout_s <= 0.0)
  {
    errors.emplace_back(
      "camera_info_stale_timeout_s must be finite and positive");
  }

  if (
    !std::isfinite(min_frame_rate_hz) ||
    min_frame_rate_hz < 0.0)
  {
    errors.emplace_back(
      "min_frame_rate_hz must be finite and non-negative");
  }

  if (min_frame_samples == 0U) {
    errors.emplace_back("min_frame_samples must be positive");
  }

  return errors;
}

bool CameraHealthConfig::valid() const
{
  return validation_errors().empty();
}

const char * to_string(CameraHealthLevel level)
{
  switch (level) {
    case CameraHealthLevel::kOk:
      return "OK";

    case CameraHealthLevel::kWarn:
      return "WARN";

    case CameraHealthLevel::kError:
      return "ERROR";
  }

  return "ERROR";
}

CameraHealthResult evaluate_camera_health(
  const CameraHealthConfig & config,
  const CameraHealthSnapshot & snapshot)
{
  if (!config.valid()) {
    return error_result("invalid_configuration");
  }

  if (
    !std::isfinite(snapshot.now_s) ||
    !std::isfinite(snapshot.start_time_s))
  {
    return error_result("invalid_clock_state");
  }

  const auto startup_age_s =
    std::max(0.0, snapshot.now_s - snapshot.start_time_s);

  /*
   * During startup, missing streams are warnings. After the startup
   * grace period, missing streams become errors.
   */
  if (!snapshot.image_seen) {
    if (startup_age_s <= config.startup_grace_s) {
      return warning_result("waiting_for_image", false);
    }

    return error_result("image_not_received");
  }

  if (!snapshot.camera_info_seen) {
    if (startup_age_s <= config.startup_grace_s) {
      return warning_result("waiting_for_camera_info", false);
    }

    return error_result("camera_info_not_received");
  }

  /*
   * A camera stream that previously existed but stopped updating is
   * considered unhealthy.
   */
  if (
    non_negative_age(
      snapshot.now_s,
      snapshot.image_receipt_time_s) >
    config.image_stale_timeout_s)
  {
    return error_result("image_stale");
  }

  if (
    non_negative_age(
      snapshot.now_s,
      snapshot.camera_info_receipt_time_s) >
    config.camera_info_stale_timeout_s)
  {
    return error_result("camera_info_stale");
  }

  if (!snapshot.image_timestamp_monotonic) {
    return error_result("image_timestamp_not_monotonic");
  }

  if (!snapshot.camera_info_timestamp_monotonic) {
    return error_result("camera_info_timestamp_not_monotonic");
  }

  if (!snapshot.image_data_valid) {
    return error_result("image_data_invalid");
  }

  /*
   * Validate Image and CameraInfo dimensions against both the configured
   * camera profile and each other.
   */
  if (config.strict_resolution) {
    if (
      snapshot.image_width != config.expected_width ||
      snapshot.image_height != config.expected_height)
    {
      return error_result("image_resolution_mismatch");
    }

    if (
      snapshot.camera_info_width != config.expected_width ||
      snapshot.camera_info_height != config.expected_height)
    {
      return error_result("camera_info_resolution_mismatch");
    }
  }

  if (
    snapshot.image_width != snapshot.camera_info_width ||
    snapshot.image_height != snapshot.camera_info_height)
  {
    return error_result("image_camera_info_resolution_disagree");
  }

  /*
   * The Image and CameraInfo messages must refer to the same optical
   * frame.
   */
  if (snapshot.image_frame_id != snapshot.camera_info_frame_id) {
    return error_result("image_camera_info_frame_disagree");
  }

  if (
    config.strict_frame_id &&
    snapshot.image_frame_id != config.expected_frame_id)
  {
    return error_result("camera_frame_mismatch");
  }

  if (
    config.strict_encoding &&
    snapshot.image_encoding != config.expected_encoding)
  {
    return error_result("image_encoding_mismatch");
  }

  /*
   * Do not judge the measured frame rate until enough samples exist.
   */
  if (snapshot.frames_received < config.min_frame_samples) {
    return warning_result(
      "collecting_frame_rate",
      true,
      false);
  }

  if (
    config.min_frame_rate_hz > 0.0 &&
    snapshot.frame_rate_hz < config.min_frame_rate_hz)
  {
    return warning_result(
      "low_frame_rate",
      true,
      false);
  }

  /*
   * A live uncalibrated camera is stream-healthy but cannot yet produce
   * trustworthy metric AprilTag poses.
   */
  if (
    config.require_calibration_for_pose &&
    !snapshot.camera_calibrated)
  {
    return warning_result(
      "camera_uncalibrated",
      true,
      false);
  }

  return ok_result();
}

std::string camera_health_status_text(
  const CameraHealthResult & result,
  const CameraHealthSnapshot & snapshot)
{
  std::ostringstream stream;

  stream
    << "status=" << result.status
    << " reason=" << result.reason
    << " stream_healthy="
    << (result.stream_healthy ? "true" : "false")
    << " ready_for_pose_estimation="
    << (result.ready_for_pose_estimation ? "true" : "false")
    << " image_seen="
    << (snapshot.image_seen ? "true" : "false")
    << " camera_info_seen="
    << (snapshot.camera_info_seen ? "true" : "false")
    << " calibrated="
    << (snapshot.camera_calibrated ? "true" : "false")
    << " frame_rate_hz="
    << snapshot.frame_rate_hz
    << " frames_received="
    << snapshot.frames_received
    << " frame_id="
    << snapshot.image_frame_id
    << " resolution="
    << snapshot.image_width
    << "x"
    << snapshot.image_height
    << " encoding="
    << snapshot.image_encoding;

  return stream.str();
}

}  // namespace savo_head
