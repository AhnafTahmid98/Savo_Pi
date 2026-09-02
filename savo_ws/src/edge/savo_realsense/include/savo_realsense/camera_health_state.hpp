#pragma once

#include <string>
#include <vector>

namespace savo_realsense
{

struct HealthSignal
{
  bool required{false};
  bool seen{false};
  bool healthy{false};
  double last_update_s{0.0};
};

struct CameraHealthEvaluation
{
  bool ok{false};
  bool depth_signal_ok{false};
  bool vo_health_ok{false};
  bool obstacle_cloud_ok{false};
  bool color_ok{false};
  bool color_info_ok{false};
  bool depth_ok{false};
  bool depth_info_ok{false};
  bool aligned_depth_ok{false};
  bool pointcloud_ok{false};
  std::vector<std::string> failures;
};

bool health_text_is_ok(const std::string & text);
bool signal_is_fresh(
  const HealthSignal & signal, double now_s, double stale_timeout_s);
bool signal_is_ok(
  const HealthSignal & signal, double now_s, double stale_timeout_s);

CameraHealthEvaluation evaluate_camera_health(
  const HealthSignal & depth_signal,
  const HealthSignal & vo_health,
  const HealthSignal & obstacle_cloud,
  double now_s,
  double stale_timeout_s);

std::string camera_health_message(const CameraHealthEvaluation & evaluation);

}  // namespace savo_realsense
