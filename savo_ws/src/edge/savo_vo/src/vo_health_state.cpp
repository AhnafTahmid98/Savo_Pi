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
  if (!state.has_status) {
    return "stale: visual odometry status not received";
  }

  const double status_age_s = std::max(0.0, now_s - state.last_status_time_s);
  if (status_age_s > stale_timeout_s) {
    return "stale: visual odometry status timeout";
  }

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

  const double odom_age_s = std::max(0.0, now_s - state.last_odom_time_s);
  if (odom_age_s > stale_timeout_s) {
    return "stale: visual odometry timeout";
  }

  return "ok: " + state.status_text;
}

}  // namespace savo_vo
