#ifndef SAVO_POWER__BATTERY_READING_HPP_
#define SAVO_POWER__BATTERY_READING_HPP_

#include "savo_power/power_state.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace savo_power
{

enum class BatterySource
{
  CORE_UPS,
  EDGE_UPS,
  BASE_BATTERY,
  UNKNOWN
};

inline constexpr std::string_view to_string(BatterySource source)
{
  switch (source) {
    case BatterySource::CORE_UPS:
      return "core_ups";
    case BatterySource::EDGE_UPS:
      return "edge_ups";
    case BatterySource::BASE_BATTERY:
      return "base_battery";
    case BatterySource::UNKNOWN:
      return "unknown";
  }

  return "unknown";
}

struct BatteryReading
{
  BatterySource source{BatterySource::UNKNOWN};
  PowerState state{PowerState::UNKNOWN};

  bool ok{false};

  std::optional<double> voltage_v{};
  std::optional<double> capacity_pct{};
  std::optional<double> soc_pct{};

  std::string error_message{};
};

inline bool has_voltage(const BatteryReading & reading)
{
  return reading.voltage_v.has_value();
}

inline bool has_capacity(const BatteryReading & reading)
{
  return reading.capacity_pct.has_value();
}

inline bool has_soc(const BatteryReading & reading)
{
  return reading.soc_pct.has_value();
}

inline constexpr bool is_valid_percentage(double value)
{
  return value >= 0.0 && value <= 100.0;
}

inline bool has_valid_capacity(const BatteryReading & reading)
{
  return reading.capacity_pct.has_value() && is_valid_percentage(*reading.capacity_pct);
}

inline bool has_valid_soc(const BatteryReading & reading)
{
  return reading.soc_pct.has_value() && is_valid_percentage(*reading.soc_pct);
}

inline bool reading_has_error(const BatteryReading & reading)
{
  return !reading.ok ||
         reading.state == PowerState::ERROR ||
         !reading.error_message.empty();
}

inline BatteryReading make_error_reading(
  BatterySource source,
  std::string error_message)
{
  BatteryReading reading;
  reading.source = source;
  reading.state = PowerState::ERROR;
  reading.ok = false;
  reading.error_message = std::move(error_message);
  return reading;
}

inline BatteryReading make_unknown_reading(BatterySource source)
{
  BatteryReading reading;
  reading.source = source;
  reading.state = PowerState::UNKNOWN;
  reading.ok = false;
  return reading;
}

}  // namespace savo_power

#endif  // SAVO_POWER__BATTERY_READING_HPP_
