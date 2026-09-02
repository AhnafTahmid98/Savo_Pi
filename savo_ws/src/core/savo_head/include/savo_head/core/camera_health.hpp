#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_head
{

enum class CameraHealthLevel
{
  kOk,
  kWarn,
  kError
};

struct CameraHealthConfig
{
  std::uint32_t expected_width{640U};
  std::uint32_t expected_height{480U};

  std::string expected_frame_id{"pi_camera_optical_frame"};
  std::string expected_encoding{"rgb8"};

  double startup_grace_s{3.0};
  double metadata_stale_timeout_s{2.0};
  double min_frame_rate_hz{10.0};

  std::uint64_t min_frame_samples{5U};

  bool require_calibration_for_pose{true};
  bool strict_frame_id{true};
  bool strict_resolution{true};

  [[nodiscard]] std::vector<std::string> validation_errors() const;
  [[nodiscard]] bool valid() const;
};

struct CameraHealthSnapshot
{
  double start_time_s{0.0};
  double now_s{0.0};

  bool image_publisher_present{false};
  bool stream_metadata_seen{false};
  double metadata_receipt_time_s{0.0};
  bool metadata_timestamp_monotonic{true};

  std::uint32_t metadata_width{0U};
  std::uint32_t metadata_height{0U};
  std::string metadata_frame_id{};

  bool camera_calibrated{false};

  std::uint64_t frames_received{0U};
  double frame_rate_hz{0.0};
};

struct CameraHealthResult
{
  CameraHealthLevel level{CameraHealthLevel::kError};

  std::string status{"ERROR"};
  std::string reason{"not_evaluated"};

  bool stream_healthy{false};
  bool ready_for_pose_estimation{false};
};

[[nodiscard]] const char * to_string(CameraHealthLevel level);

[[nodiscard]] CameraHealthResult evaluate_camera_health(
  const CameraHealthConfig & config,
  const CameraHealthSnapshot & snapshot);

[[nodiscard]] std::string camera_health_status_text(
  const CameraHealthConfig & config,
  const CameraHealthResult & result,
  const CameraHealthSnapshot & snapshot);

}  // namespace savo_head
