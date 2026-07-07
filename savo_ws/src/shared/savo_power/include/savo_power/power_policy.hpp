#ifndef SAVO_POWER__POWER_POLICY_HPP_
#define SAVO_POWER__POWER_POLICY_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/constants.hpp"
#include "savo_power/power_state.hpp"

namespace savo_power
{

struct PowerPolicyThresholds
{
  double ups_low_voltage_v{constants::kUpsLowVoltageDefault};
  double ups_critical_voltage_v{constants::kUpsCriticalVoltageDefault};

  double base_empty_voltage_v{constants::kBaseBatteryEmptyVoltageDefault};
  double base_low_voltage_v{constants::kBaseBatteryLowVoltageDefault};
  double base_full_voltage_v{constants::kBaseBatteryFullVoltageDefault};

  double base_low_soc_pct{constants::kBaseBatteryLowSocDefault};
  double base_full_soc_pct{constants::kBaseBatteryFullSocDefault};

  double full_capacity_pct{constants::kBaseBatteryFullSocDefault};

  bool automatic_shutdown_enabled{constants::kAutomaticShutdownEnabledDefault};
};

inline bool has_driver_error(const BatteryReading & reading)
{
  if (reading.state == PowerState::ERROR) {
    return true;
  }

  if (!reading.error_message.empty()) {
    return true;
  }

  if (!reading.ok && reading.state != PowerState::UNKNOWN) {
    return true;
  }

  return false;
}

inline PowerState evaluate_ups_state(
  const BatteryReading & reading,
  const PowerPolicyThresholds & thresholds = PowerPolicyThresholds{})
{
  if (has_driver_error(reading)) {
    return PowerState::ERROR;
  }

  if (!reading.voltage_v.has_value()) {
    return PowerState::UNKNOWN;
  }

  const double voltage = *reading.voltage_v;

  if (voltage <= thresholds.ups_critical_voltage_v) {
    return PowerState::CRITICAL;
  }

  if (voltage <= thresholds.ups_low_voltage_v) {
    return PowerState::LOW;
  }

  if (
    reading.capacity_pct.has_value() &&
    is_valid_percentage(*reading.capacity_pct) &&
    *reading.capacity_pct >= thresholds.full_capacity_pct)
  {
    return PowerState::FULL;
  }

  return PowerState::OK;
}

inline PowerState evaluate_base_battery_state(
  const BatteryReading & reading,
  const PowerPolicyThresholds & thresholds = PowerPolicyThresholds{})
{
  if (has_driver_error(reading)) {
    return PowerState::ERROR;
  }

  const bool has_voltage_reading = reading.voltage_v.has_value();
  const bool has_soc_reading =
    reading.soc_pct.has_value() && is_valid_percentage(*reading.soc_pct);

  if (!has_voltage_reading && !has_soc_reading) {
    return PowerState::UNKNOWN;
  }

  if (has_voltage_reading && *reading.voltage_v <= thresholds.base_empty_voltage_v) {
    return PowerState::CRITICAL;
  }

  if (has_voltage_reading && *reading.voltage_v <= thresholds.base_low_voltage_v) {
    return PowerState::LOW;
  }

  if (has_soc_reading && *reading.soc_pct <= thresholds.base_low_soc_pct) {
    return PowerState::LOW;
  }

  if (has_voltage_reading && *reading.voltage_v >= thresholds.base_full_voltage_v) {
    return PowerState::FULL;
  }

  if (has_soc_reading && *reading.soc_pct >= thresholds.base_full_soc_pct) {
    return PowerState::FULL;
  }

  return PowerState::OK;
}

inline PowerState evaluate_power_state(
  const BatteryReading & reading,
  const PowerPolicyThresholds & thresholds = PowerPolicyThresholds{})
{
  switch (reading.source) {
    case BatterySource::CORE_UPS:
    case BatterySource::EDGE_UPS:
      return evaluate_ups_state(reading, thresholds);

    case BatterySource::BASE_BATTERY:
      return evaluate_base_battery_state(reading, thresholds);

    case BatterySource::UNKNOWN:
      break;
  }

  if (has_driver_error(reading)) {
    return PowerState::ERROR;
  }

  return PowerState::UNKNOWN;
}

inline BatteryReading apply_power_policy(
  BatteryReading reading,
  const PowerPolicyThresholds & thresholds = PowerPolicyThresholds{})
{
  reading.state = evaluate_power_state(reading, thresholds);
  reading.ok = reading.state != PowerState::ERROR &&
               reading.state != PowerState::UNKNOWN &&
               reading.state != PowerState::STALE;
  return reading;
}

inline bool should_request_shutdown(
  PowerState state,
  const PowerPolicyThresholds & thresholds = PowerPolicyThresholds{})
{
  return thresholds.automatic_shutdown_enabled &&
         state == PowerState::CRITICAL;
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_POLICY_HPP_
