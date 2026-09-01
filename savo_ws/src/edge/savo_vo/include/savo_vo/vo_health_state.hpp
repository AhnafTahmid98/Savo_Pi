#pragma once

#include <string>

namespace savo_vo
{

struct VOHealthState
{
  bool has_status{false};
  double last_status_time_s{0.0};
  std::string status_text;
  bool has_odom{false};
  double last_odom_time_s{0.0};
};

std::string evaluate_vo_health(
  const VOHealthState & state,
  double now_s,
  double stale_timeout_s);

}  // namespace savo_vo
