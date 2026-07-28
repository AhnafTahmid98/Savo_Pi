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
  double image_stale_timeout_s{2.0};
  double camera_info_stale_timeout_s{2.0};
  double min_frame_rate_hz{10.0};

  std::uint64_t min_frame_samples{5U};

  bool require_calibration_for_pose{true};
  bool strict_frame_id{true};
  bool strict_resolution{true};
  bool strict_encoding{true};

  [[nodiscard]] std::vector<std::string> validation_errors() const;
  [[nodiscard]] bool valid() const;
};

struct CameraHealthSnapshot
{
  double start_time_s{0.0};
  double now_s{0.0};

  bool image_seen{false};
  bool camera_info_seen{false};

  double image_receipt_time_s{0.0};
  double camera_info_receipt_time_s{0.0};

  bool image_timestamp_monotonic{true};
  bool camera_info_timestamp_monotonic{true};
  bool image_data_valid{false};

  std::uint32_t image_width{0U};
  std::uint32_t image_height{0U};

  std::uint32_t camera_info_width{0U};
  std::uint32_t camera_info_height{0U};

  std::string image_frame_id{};
  std::string camera_info_frame_id{};
  std::string image_encoding{};

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
  const CameraHealthResult & result,
  const CameraHealthSnapshot & snapshot);

}  // namespace savo_head
