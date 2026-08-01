// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace savo_head
{

inline constexpr std::string_view kProductionAprilTagFamily{"tag36h11"};
inline constexpr double kInitialAprilTagSizeM = 0.140;

struct AprilTagDetectorConfig
{
  bool enabled{true};
  std::string family{std::string{kProductionAprilTagFamily}};
  double tag_size_m{kInitialAprilTagSizeM};
  bool pose_estimation_enabled{true};

  std::string image_topic{"/savo_head/camera/image_raw"};
  std::string camera_info_topic{"/savo_head/camera/camera_info"};
  std::string observation_topic{"/savo_head/apriltag/observations"};
  std::string diagnostics_topic{"/savo_head/apriltag/detector_status"};
  std::string camera_optical_frame{"pi_camera_optical_frame"};
  std::string detector_name{"apriltag3_cpp"};

  std::vector<std::int64_t> allowed_tag_ids{};
  int maximum_hamming_distance{1};
  double minimum_decision_margin{0.0};
  double full_quality_decision_margin{100.0};
  double maximum_detection_distance_m{3.0};

  int detector_threads{2};
  double quad_decimate{2.0};
  double quad_sigma{0.0};
  bool refine_edges{true};
  double decode_sharpening{0.25};

  double camera_info_max_age_s{1.0};
  bool require_matching_camera_info_stamp{false};
  double maximum_processing_latency_ms{250.0};
  double camera_stale_timeout_s{1.0};
  double diagnostics_rate_hz{2.0};

  double position_variance_m2{0.0025};
  double orientation_variance_rad2{0.01};
};

[[nodiscard]] bool is_supported_apriltag_family(std::string_view family);

[[nodiscard]] bool is_allowed_apriltag_id(
  const AprilTagDetectorConfig & config,
  std::int64_t tag_id);

[[nodiscard]] double normalized_detection_quality(
  double decision_margin,
  double full_quality_decision_margin);

[[nodiscard]] std::vector<std::string> validate_apriltag_detector_config(
  const AprilTagDetectorConfig & config);

}  // namespace savo_head
