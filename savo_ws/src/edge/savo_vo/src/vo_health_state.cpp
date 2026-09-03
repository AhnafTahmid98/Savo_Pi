#include "savo_vo/vo_health_state.hpp"

#include <algorithm>
#include <cctype>

namespace savo_vo
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

bool contains(const std::string & value, const char * token)
{
  return value.find(token) != std::string::npos;
}

}  // namespace

std::string evaluate_vo_health(
  const VOHealthState & state,
  const double now_s,
  const double stale_timeout_s)
{
  const double timeout_s = std::max(0.0, stale_timeout_s);
  const bool status_fresh = state.has_status &&
    std::max(0.0, now_s - state.last_status_time_s) <= timeout_s;
  const bool odom_fresh = state.has_odom &&
    std::max(0.0, now_s - state.last_odom_time_s) <= timeout_s;

  // A fresh semantic status is authoritative, including while the last
  // accepted odometry sample is still inside its freshness window.
  if (status_fresh) {
    const std::string normalized = lower_copy(state.status_text);
    if (normalized.empty() || normalized == "status empty") {
      return "error: visual odometry status empty";
    }
    if (contains(normalized, "error")) {
      return "error: " + state.status_text;
    }
    if (contains(normalized, "waiting")) {
      return "waiting: " + state.status_text;
    }
    if (contains(normalized, "lost") ||
      contains(normalized, "rejected") ||
      contains(normalized, "degraded"))
    {
      return "degraded: " + state.status_text;
    }

    if (!state.has_odom) {
      return "stale: visual odometry not received";
    }
    if (!odom_fresh) {
      return "stale: visual odometry timeout";
    }
    return "ok: " + state.status_text;
  }

  // Status and odometry are independent ROS channels.  Missing auxiliary
  // status telemetry must remain visible, but it must not make fresh accepted
  // odometry non-operational.
  if (odom_fresh) {
    return state.has_status ?
      "ok: odometry fresh; status channel stale" :
      "ok: odometry fresh; status channel not received; telemetry degraded";
  }

  if (!state.has_odom) {
    return state.has_status ?
      "stale: visual odometry not received; status channel stale" :
      "stale: visual odometry status and odometry not received";
  }

  return state.has_status ?
    "stale: visual odometry timeout; status channel stale" :
    "stale: visual odometry timeout; status channel not received";
}

}  // namespace savo_vo
