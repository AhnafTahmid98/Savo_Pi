#ifndef SAVO_PERCEPTION__RANGE_HEALTH_POLICY_HPP_
#define SAVO_PERCEPTION__RANGE_HEALTH_POLICY_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "savo_perception/perception_types.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC RequiredRangeHealth
{
  bool ok{true};
  SensorStatus status{SensorStatus::kOk};
  std::vector<std::string> stale_required_sensors;
  std::vector<std::string> error_required_sensors;
};

inline RequiredRangeHealth evaluate_required_range_health(
  const std::vector<SensorHealth> & health,
  const std::vector<std::string> & required_sensors,
  const std::vector<std::string> & below_minimum_rate_sensors = {})
{
  RequiredRangeHealth result;

  for (const auto & required_name : required_sensors) {
    const auto item = std::find_if(
      health.begin(), health.end(),
      [&required_name](const SensorHealth & candidate) {
        return candidate.sensor_name == required_name;
      });

    if (item == health.end()) {
      result.error_required_sensors.push_back(required_name);
      continue;
    }

    if (item->stale) {
      result.stale_required_sensors.push_back(required_name);
      continue;
    }

    const bool rate_below_minimum = std::find(
      below_minimum_rate_sensors.begin(),
      below_minimum_rate_sensors.end(),
      required_name) != below_minimum_rate_sensors.end();
    if (item->status == SensorStatus::kError || rate_below_minimum) {
      result.error_required_sensors.push_back(required_name);
    }
  }

  result.ok = result.stale_required_sensors.empty() &&
    result.error_required_sensors.empty();
  if (!result.error_required_sensors.empty()) {
    result.status = SensorStatus::kError;
  } else if (!result.stale_required_sensors.empty()) {
    result.status = SensorStatus::kStale;
  } else {
    result.status = SensorStatus::kOk;
  }

  return result;
}

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__RANGE_HEALTH_POLICY_HPP_
