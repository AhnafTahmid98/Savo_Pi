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
    !std::isfinite(metadata_stale_timeout_s) ||
    metadata_stale_timeout_s <= 0.0)
  {
    errors.emplace_back(
      "metadata_stale_timeout_s must be finite and positive");
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

  if (!snapshot.image_publisher_present) {
    if (startup_age_s <= config.startup_grace_s) {
      return warning_result("waiting_for_image_publisher", false);
    }
    return error_result("image_publisher_not_present");
  }

  /* CameraInfo is frame-coupled to each successfully published gscam Image. */
  if (!snapshot.stream_metadata_seen) {
    if (startup_age_s <= config.startup_grace_s) {
      return warning_result("waiting_for_stream_metadata", false);
    }
    return error_result("stream_metadata_not_received");
  }

  if (
    non_negative_age(snapshot.now_s, snapshot.metadata_receipt_time_s) >
    config.metadata_stale_timeout_s)
  {
    return error_result("stream_metadata_stale");
  }

  if (!snapshot.metadata_timestamp_monotonic) {
    return error_result("stream_metadata_timestamp_not_monotonic");
  }

  const bool metadata_dimensions_missing =
    snapshot.metadata_width == 0U &&
    snapshot.metadata_height == 0U;

  if ((snapshot.metadata_width == 0U) != (snapshot.metadata_height == 0U)) {
    return error_result("stream_metadata_dimensions_invalid");
  }

  if (metadata_dimensions_missing && snapshot.camera_calibrated) {
    return error_result("stream_metadata_dimensions_invalid");
  }

  /* Uncalibrated CameraInfo may report 0x0; config is the documented fallback. */
  if (
    config.strict_resolution &&
    !metadata_dimensions_missing &&
    (snapshot.metadata_width != config.expected_width ||
    snapshot.metadata_height != config.expected_height))
  {
    return error_result("stream_metadata_resolution_mismatch");
  }

  if (
    config.strict_frame_id &&
    snapshot.metadata_frame_id != config.expected_frame_id)
  {
    return error_result("camera_frame_mismatch");
  }

  if (snapshot.frames_received < config.min_frame_samples) {
    return warning_result("collecting_frame_rate", true, false);
  }

  if (
    config.min_frame_rate_hz > 0.0 &&
    snapshot.frame_rate_hz < config.min_frame_rate_hz)
  {
    return warning_result("low_frame_rate", true, false);
  }

  if (config.require_calibration_for_pose && !snapshot.camera_calibrated) {
    return warning_result("camera_uncalibrated", true, false);
  }

  return ok_result();
}

std::string camera_health_status_text(
  const CameraHealthConfig & config,
  const CameraHealthResult & result,
  const CameraHealthSnapshot & snapshot)
{
  std::ostringstream stream;

  stream
    << "status=" << result.status
    << " reason=" << result.reason
    << " stream_healthy=" << (result.stream_healthy ? "true" : "false")
    << " ready_for_pose_estimation="
    << (result.ready_for_pose_estimation ? "true" : "false")
    << " image_publisher_present="
    << (snapshot.image_publisher_present ? "true" : "false")
    << " stream_metadata_seen="
    << (snapshot.stream_metadata_seen ? "true" : "false")
    << " camera_info_seen="
    << (snapshot.stream_metadata_seen ? "true" : "false")
    << " calibrated=" << (snapshot.camera_calibrated ? "true" : "false")
    << " frame_rate_hz=" << snapshot.frame_rate_hz
    << " frames_received=" << snapshot.frames_received
    << " frame_id=" << snapshot.metadata_frame_id
    << " metadata_resolution=" << snapshot.metadata_width
    << "x" << snapshot.metadata_height
    << " configured_resolution=" << config.expected_width
    << "x" << config.expected_height
    << " configured_encoding=" << config.expected_encoding
    << " encoding_verification=static_config";

  return stream.str();
}

}  // namespace savo_head
