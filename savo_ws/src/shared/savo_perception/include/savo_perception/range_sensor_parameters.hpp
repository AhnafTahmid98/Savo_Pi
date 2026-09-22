#pragma once

#include <stdexcept>
#include <string>
#include <vector>

namespace savo_perception
{

inline constexpr char kNoRequiredSensorSentinel[] = "__none__";

inline std::vector<std::string> normalize_required_sensor_names(
  const std::vector<std::string> & configured_names)
{
  const std::vector<std::string> sentinel_only{kNoRequiredSensorSentinel};
  if (configured_names == sentinel_only) {
    return {};
  }

  for (const auto & name : configured_names) {
    if (name == kNoRequiredSensorSentinel) {
      throw std::invalid_argument(
              "__none__ must be the only configured value for required_sensors");
    }
  }

  return configured_names;
}

}  // namespace savo_perception
