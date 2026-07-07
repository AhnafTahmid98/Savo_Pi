#ifndef SAVO_POWER__POWER_TYPES_HPP_
#define SAVO_POWER__POWER_TYPES_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/power_state.hpp"
#include "savo_power/visibility_control.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace savo_power
{

enum class PowerHealthLevel
{
  OK,
  WARN,
  ERROR,
  UNKNOWN
};

inline constexpr std::string_view to_string(PowerHealthLevel level)
{
  switch (level) {
    case PowerHealthLevel::OK:
      return "OK";
    case PowerHealthLevel::WARN:
      return "WARN";
    case PowerHealthLevel::ERROR:
      return "ERROR";
    case PowerHealthLevel::UNKNOWN:
      return "UNKNOWN";
  }

  return "UNKNOWN";
}

inline constexpr PowerHealthLevel health_level_from_power_state(PowerState state)
{
  switch (state) {
    case PowerState::OK:
    case PowerState::FULL:
    case PowerState::CHARGING:
      return PowerHealthLevel::OK;

    case PowerState::LOW:
      return PowerHealthLevel::WARN;

    case PowerState::CRITICAL:
    case PowerState::ERROR:
    case PowerState::STALE:
    case PowerState::UNKNOWN:
      return PowerHealthLevel::ERROR;
  }

  return PowerHealthLevel::UNKNOWN;
}

inline constexpr bool is_ok_health_level(PowerHealthLevel level)
{
  return level == PowerHealthLevel::OK;
}

inline constexpr bool is_warning_health_level(PowerHealthLevel level)
{
  return level == PowerHealthLevel::WARN;
}

inline constexpr bool is_error_health_level(PowerHealthLevel level)
{
  return level == PowerHealthLevel::ERROR ||
         level == PowerHealthLevel::UNKNOWN;
}

struct PowerSourceStatus
{
  BatterySource source{BatterySource::UNKNOWN};
  PowerState state{PowerState::UNKNOWN};

  bool expected{true};
  bool seen{false};
  bool stale{false};

  double age_s{0.0};

  std::string text{};
};

struct PowerStatusSummary
{
  PowerState overall_state{PowerState::UNKNOWN};
  PowerHealthLevel health_level{PowerHealthLevel::UNKNOWN};

  PowerSourceStatus core_ups{};
  PowerSourceStatus edge_ups{};
  PowerSourceStatus base_battery{};

  std::string status_text{};
  std::string dashboard_text{};
};

inline constexpr bool source_needs_attention(const PowerSourceStatus & status)
{
  return status.expected &&
         (!status.seen || status.stale || is_fault_state(status.state));
}

inline constexpr bool status_needs_attention(const PowerStatusSummary & summary)
{
  return is_error_health_level(summary.health_level) ||
         is_warning_health_level(summary.health_level) ||
         source_needs_attention(summary.core_ups) ||
         source_needs_attention(summary.edge_ups) ||
         source_needs_attention(summary.base_battery);
}

inline PowerSourceStatus make_missing_source_status(BatterySource source)
{
  PowerSourceStatus status;
  status.source = source;
  status.state = PowerState::UNKNOWN;
  status.expected = true;
  status.seen = false;
  status.stale = false;
  status.text = "missing";
  return status;
}

inline PowerSourceStatus make_not_expected_source_status(BatterySource source)
{
  PowerSourceStatus status;
  status.source = source;
  status.state = PowerState::OK;
  status.expected = false;
  status.seen = false;
  status.stale = false;
  status.text = "not expected";
  return status;
}

inline PowerSourceStatus make_source_status_from_reading(
  const BatteryReading & reading)
{
  PowerSourceStatus status;
  status.source = reading.source;
  status.state = reading.state;
  status.expected = true;
  status.seen = true;
  status.stale = false;
  status.text = std::string(to_string(reading.state));
  return status;
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_TYPES_HPP_
