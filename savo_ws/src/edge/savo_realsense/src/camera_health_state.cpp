#include "savo_realsense/camera_health_state.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace savo_realsense
{
namespace
{

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

void append_failure(
  std::vector<std::string> & failures,
  const char * name,
  const HealthSignal & signal,
  const double now_s,
  const double stale_timeout_s)
{
  if (!signal.required || signal_is_ok(signal, now_s, stale_timeout_s)) {
    return;
  }

  std::string reason;
  if (!signal.seen) {
    reason = "not_seen";
  } else if (!signal_is_fresh(signal, now_s, stale_timeout_s)) {
    reason = "stale";
  } else {
    reason = "unhealthy";
  }
  failures.push_back(std::string(name) + ":" + reason);
}

}  // namespace

bool health_text_is_ok(const std::string & text)
{
  const std::string normalized = lower_copy(text);
  return normalized == "ok" || normalized.rfind("ok:", 0U) == 0U;
}

bool signal_is_fresh(
  const HealthSignal & signal,
  const double now_s,
  const double stale_timeout_s)
{
  if (!signal.seen) {
    return false;
  }
  const double age_s = std::max(0.0, now_s - signal.last_update_s);
  return age_s <= std::max(0.0, stale_timeout_s);
}

bool signal_is_ok(
  const HealthSignal & signal,
  const double now_s,
  const double stale_timeout_s)
{
  return !signal.required ||
         (signal_is_fresh(signal, now_s, stale_timeout_s) && signal.healthy);
}

CameraHealthEvaluation evaluate_camera_health(
  const HealthSignal & depth_signal,
  const HealthSignal & vo_health,
  const HealthSignal & obstacle_cloud,
  const double now_s,
  const double stale_timeout_s)
{
  CameraHealthEvaluation evaluation;
  evaluation.depth_signal_ok = signal_is_ok(depth_signal, now_s, stale_timeout_s);
  evaluation.vo_health_ok = signal_is_ok(vo_health, now_s, stale_timeout_s);
  evaluation.obstacle_cloud_ok = signal_is_ok(obstacle_cloud, now_s, stale_timeout_s);

  // Preserve the established public fields as lightweight pipeline projections.
  evaluation.color_ok = evaluation.vo_health_ok;
  evaluation.color_info_ok = evaluation.vo_health_ok;
  evaluation.depth_ok = evaluation.depth_signal_ok;
  evaluation.depth_info_ok = evaluation.depth_signal_ok;
  evaluation.aligned_depth_ok = evaluation.vo_health_ok;
  evaluation.pointcloud_ok = evaluation.obstacle_cloud_ok;

  append_failure(
    evaluation.failures, "depth_front", depth_signal, now_s, stale_timeout_s);
  append_failure(evaluation.failures, "vo", vo_health, now_s, stale_timeout_s);
  append_failure(
    evaluation.failures, "obstacle_cloud", obstacle_cloud, now_s, stale_timeout_s);
  evaluation.ok = evaluation.failures.empty();
  return evaluation;
}

std::string camera_health_message(const CameraHealthEvaluation & evaluation)
{
  if (evaluation.ok) {
    return "RealSense streams OK";
  }

  std::ostringstream stream;
  stream << "Unhealthy streams: ";
  for (std::size_t index = 0; index < evaluation.failures.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << evaluation.failures[index];
  }
  return stream.str();
}

}  // namespace savo_realsense
