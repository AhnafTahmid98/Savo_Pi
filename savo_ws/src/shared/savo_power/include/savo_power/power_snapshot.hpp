#ifndef SAVO_POWER__POWER_SNAPSHOT_HPP_
#define SAVO_POWER__POWER_SNAPSHOT_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/power_state.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace savo_power
{

struct PowerSnapshot
{
  std::optional<BatteryReading> core_ups{};
  std::optional<BatteryReading> edge_ups{};
  std::optional<BatteryReading> base_battery{};

  PowerState overall_state{PowerState::UNKNOWN};

  bool core_present{false};
  bool edge_present{false};
  bool base_present{false};

  bool shutdown_requested{false};

  std::string summary{};
};

inline bool has_core_ups(const PowerSnapshot & snapshot)
{
  return snapshot.core_ups.has_value();
}

inline bool has_edge_ups(const PowerSnapshot & snapshot)
{
  return snapshot.edge_ups.has_value();
}

inline bool has_base_battery(const PowerSnapshot & snapshot)
{
  return snapshot.base_battery.has_value();
}

inline bool has_any_reading(const PowerSnapshot & snapshot)
{
  return has_core_ups(snapshot) ||
         has_edge_ups(snapshot) ||
         has_base_battery(snapshot);
}

inline bool has_all_expected_readings(const PowerSnapshot & snapshot)
{
  return (!snapshot.core_present || has_core_ups(snapshot)) &&
         (!snapshot.edge_present || has_edge_ups(snapshot)) &&
         (!snapshot.base_present || has_base_battery(snapshot));
}

inline bool snapshot_has_fault(const PowerSnapshot & snapshot)
{
  if (is_fault_state(snapshot.overall_state)) {
    return true;
  }

  if (snapshot.core_ups.has_value() && reading_has_error(*snapshot.core_ups)) {
    return true;
  }

  if (snapshot.edge_ups.has_value() && reading_has_error(*snapshot.edge_ups)) {
    return true;
  }

  if (
    snapshot.base_battery.has_value() &&
    reading_has_error(*snapshot.base_battery))
  {
    return true;
  }

  return false;
}

inline constexpr PowerState more_severe_state(PowerState left, PowerState right)
{
  if (left == PowerState::CRITICAL || right == PowerState::CRITICAL) {
    return PowerState::CRITICAL;
  }

  if (left == PowerState::ERROR || right == PowerState::ERROR) {
    return PowerState::ERROR;
  }

  if (left == PowerState::STALE || right == PowerState::STALE) {
    return PowerState::STALE;
  }

  if (left == PowerState::LOW || right == PowerState::LOW) {
    return PowerState::LOW;
  }

  if (left == PowerState::UNKNOWN || right == PowerState::UNKNOWN) {
    return PowerState::UNKNOWN;
  }

  if (left == PowerState::CHARGING || right == PowerState::CHARGING) {
    return PowerState::CHARGING;
  }

  if (left == PowerState::FULL && right == PowerState::FULL) {
    return PowerState::FULL;
  }

  return PowerState::OK;
}

inline PowerState compute_overall_state(const PowerSnapshot & snapshot)
{
  if (!has_any_reading(snapshot)) {
    return PowerState::UNKNOWN;
  }

  PowerState state{PowerState::OK};

  if (snapshot.core_ups.has_value()) {
    state = more_severe_state(state, snapshot.core_ups->state);
  }

  if (snapshot.edge_ups.has_value()) {
    state = more_severe_state(state, snapshot.edge_ups->state);
  }

  if (snapshot.base_battery.has_value()) {
    state = more_severe_state(state, snapshot.base_battery->state);
  }

  if (!has_all_expected_readings(snapshot)) {
    state = more_severe_state(state, PowerState::UNKNOWN);
  }

  return state;
}

inline void update_overall_state(PowerSnapshot & snapshot)
{
  snapshot.overall_state = compute_overall_state(snapshot);
}

inline PowerSnapshot make_empty_snapshot()
{
  PowerSnapshot snapshot;
  snapshot.overall_state = PowerState::UNKNOWN;
  snapshot.summary = "No power readings available";
  return snapshot;
}

inline constexpr std::string_view source_label(BatterySource source)
{
  switch (source) {
    case BatterySource::CORE_UPS:
      return "Core UPS";
    case BatterySource::EDGE_UPS:
      return "Edge UPS";
    case BatterySource::BASE_BATTERY:
      return "Base battery";
    case BatterySource::UNKNOWN:
      return "Unknown";
  }

  return "Unknown";
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_SNAPSHOT_HPP_
