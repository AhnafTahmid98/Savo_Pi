#ifndef SAVO_POWER__POWER_FORMAT_HPP_
#define SAVO_POWER__POWER_FORMAT_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/power_snapshot.hpp"
#include "savo_power/power_state.hpp"

#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace savo_power
{

inline std::string format_double(double value, int precision = 2)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

inline std::string format_voltage(const std::optional<double> & voltage_v)
{
  if (!voltage_v.has_value()) {
    return "n/a V";
  }

  return format_double(*voltage_v, 2) + " V";
}

inline std::string format_percent(const std::optional<double> & percent)
{
  if (!percent.has_value()) {
    return "n/a%";
  }

  return format_double(*percent, 1) + "%";
}

inline std::string format_power_state(PowerState state)
{
  return std::string(to_string(state));
}

inline std::string format_battery_source(BatterySource source)
{
  return std::string(source_label(source));
}

inline std::string format_battery_reading_line(const BatteryReading & reading)
{
  std::ostringstream out;

  out << format_battery_source(reading.source)
      << " "
      << format_power_state(reading.state)
      << ": "
      << format_voltage(reading.voltage_v);

  if (reading.capacity_pct.has_value()) {
    out << ", capacity " << format_percent(reading.capacity_pct);
  }

  if (reading.soc_pct.has_value()) {
    out << ", SoC " << format_percent(reading.soc_pct);
  }

  if (!reading.ok && reading.error_message.empty()) {
    out << ", not ok";
  }

  if (!reading.error_message.empty()) {
    out << ", error: " << reading.error_message;
  }

  return out.str();
}

inline std::string format_optional_reading_line(
  const std::optional<BatteryReading> & reading,
  BatterySource expected_source)
{
  if (!reading.has_value()) {
    return format_battery_source(expected_source) + " missing";
  }

  return format_battery_reading_line(*reading);
}

inline std::string format_snapshot_summary(const PowerSnapshot & snapshot)
{
  std::ostringstream out;

  out << "Overall power: "
      << format_power_state(snapshot.overall_state);

  if (snapshot.shutdown_requested) {
    out << " — shutdown requested";
  }

  return out.str();
}

inline std::string format_snapshot_multiline(const PowerSnapshot & snapshot)
{
  std::ostringstream out;

  out << format_snapshot_summary(snapshot) << "\n";

  if (snapshot.core_present || snapshot.core_ups.has_value()) {
    out << format_optional_reading_line(
      snapshot.core_ups,
      BatterySource::CORE_UPS
    ) << "\n";
  }

  if (snapshot.edge_present || snapshot.edge_ups.has_value()) {
    out << format_optional_reading_line(
      snapshot.edge_ups,
      BatterySource::EDGE_UPS
    ) << "\n";
  }

  if (snapshot.base_present || snapshot.base_battery.has_value()) {
    out << format_optional_reading_line(
      snapshot.base_battery,
      BatterySource::BASE_BATTERY
    ) << "\n";
  }

  if (!snapshot.summary.empty()) {
    out << "Summary: " << snapshot.summary << "\n";
  }

  return out.str();
}

inline std::string make_power_dashboard_text(PowerSnapshot snapshot)
{
  update_overall_state(snapshot);
  return format_snapshot_multiline(snapshot);
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_FORMAT_HPP_
