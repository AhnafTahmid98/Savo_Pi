// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#include "savo_head/core/apriltag_detector_config.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace savo_head
{

bool is_supported_apriltag_family(const std::string_view family)
{
  return family == kProductionAprilTagFamily;
}

bool is_allowed_apriltag_id(
  const AprilTagDetectorConfig & config,
  const std::int64_t tag_id)
{
  if (tag_id < 0) {
    return false;
  }

  if (config.allowed_tag_ids.empty()) {
    return true;
  }

  return std::find(
    config.allowed_tag_ids.begin(),
    config.allowed_tag_ids.end(),
    tag_id) != config.allowed_tag_ids.end();
}

double normalized_detection_quality(
  const double decision_margin,
  const double full_quality_decision_margin)
{
  if (!std::isfinite(decision_margin) || decision_margin <= 0.0 ||
    !std::isfinite(full_quality_decision_margin) ||
    full_quality_decision_margin <= 0.0)
  {
    return 0.0;
  }

  return std::clamp(decision_margin / full_quality_decision_margin, 0.0, 1.0);
}

std::vector<std::string> validate_apriltag_detector_config(
  const AprilTagDetectorConfig & config)
{
  std::vector<std::string> errors;

  if (!is_supported_apriltag_family(config.family)) {
    errors.emplace_back("unsupported_apriltag_family");
  }

  if (!std::isfinite(config.tag_size_m) || config.tag_size_m <= 0.0) {
    errors.emplace_back("invalid_tag_size_m");
  }

  if (config.image_topic.empty()) {
    errors.emplace_back("image_topic_empty");
  }
  if (config.camera_info_topic.empty()) {
    errors.emplace_back("camera_info_topic_empty");
  }
  if (config.observation_topic.empty()) {
    errors.emplace_back("observation_topic_empty");
  }
  if (config.diagnostics_topic.empty()) {
    errors.emplace_back("diagnostics_topic_empty");
  }
  if (config.camera_optical_frame.empty()) {
    errors.emplace_back("camera_optical_frame_empty");
  }
  if (config.detector_name.empty()) {
    errors.emplace_back("detector_name_empty");
  }

  if (config.maximum_hamming_distance < 0 ||
    config.maximum_hamming_distance > 3)
  {
    errors.emplace_back("invalid_maximum_hamming_distance");
  }

  if (!std::isfinite(config.minimum_decision_margin) ||
    config.minimum_decision_margin < 0.0)
  {
    errors.emplace_back("invalid_minimum_decision_margin");
  }

  if (!std::isfinite(config.full_quality_decision_margin) ||
    config.full_quality_decision_margin <= 0.0)
  {
    errors.emplace_back("invalid_full_quality_decision_margin");
  }

  if (!std::isfinite(config.maximum_detection_distance_m) ||
    config.maximum_detection_distance_m <= 0.0)
  {
    errors.emplace_back("invalid_maximum_detection_distance_m");
  }

  if (config.detector_threads <= 0 || config.detector_threads > 8) {
    errors.emplace_back("invalid_detector_threads");
  }

  if (!std::isfinite(config.quad_decimate) || config.quad_decimate < 1.0) {
    errors.emplace_back("invalid_quad_decimate");
  }

  if (!std::isfinite(config.quad_sigma) || config.quad_sigma < 0.0) {
    errors.emplace_back("invalid_quad_sigma");
  }

  if (!std::isfinite(config.decode_sharpening) ||
    config.decode_sharpening < 0.0 || config.decode_sharpening > 1.0)
  {
    errors.emplace_back("invalid_decode_sharpening");
  }

  if (!std::isfinite(config.camera_info_max_age_s) ||
    config.camera_info_max_age_s < 0.0)
  {
    errors.emplace_back("invalid_camera_info_max_age_s");
  }

  if (!std::isfinite(config.maximum_processing_latency_ms) ||
    config.maximum_processing_latency_ms <= 0.0)
  {
    errors.emplace_back("invalid_maximum_processing_latency_ms");
  }

  if (!std::isfinite(config.camera_stale_timeout_s) ||
    config.camera_stale_timeout_s <= 0.0)
  {
    errors.emplace_back("invalid_camera_stale_timeout_s");
  }

  if (!std::isfinite(config.diagnostics_rate_hz) ||
    config.diagnostics_rate_hz <= 0.0)
  {
    errors.emplace_back("invalid_diagnostics_rate_hz");
  }

  if (!std::isfinite(config.position_variance_m2) ||
    config.position_variance_m2 <= 0.0)
  {
    errors.emplace_back("invalid_position_variance_m2");
  }

  if (!std::isfinite(config.orientation_variance_rad2) ||
    config.orientation_variance_rad2 <= 0.0)
  {
    errors.emplace_back("invalid_orientation_variance_rad2");
  }

  std::set<std::int64_t> unique_ids;
  for (const auto tag_id : config.allowed_tag_ids) {
    if (tag_id < 0) {
      errors.emplace_back("negative_allowed_tag_id");
      break;
    }
    if (!unique_ids.insert(tag_id).second) {
      errors.emplace_back("duplicate_allowed_tag_id");
      break;
    }
  }

  return errors;
}

}  // namespace savo_head
