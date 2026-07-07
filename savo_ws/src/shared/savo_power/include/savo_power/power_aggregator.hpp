#ifndef SAVO_POWER__POWER_AGGREGATOR_HPP_
#define SAVO_POWER__POWER_AGGREGATOR_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/power_state.hpp"
#include "savo_power/power_types.hpp"
#include "savo_power/visibility_control.hpp"

#include <string>

namespace savo_power
{

struct PowerAggregatorConfig
{
  bool core_ups_expected{true};
  bool edge_ups_expected{true};
  bool base_battery_expected{true};

  double stale_timeout_s{5.0};
};

struct TimedPowerSourceInput
{
  BatterySource source{BatterySource::UNKNOWN};
  bool seen{false};
  PowerState state{PowerState::UNKNOWN};
  double age_s{0.0};
  std::string text{};
};

struct PowerAggregatorInputs
{
  TimedPowerSourceInput core_ups{};
  TimedPowerSourceInput edge_ups{};
  TimedPowerSourceInput base_battery{};
};

class SAVO_POWER_PUBLIC PowerAggregator
{
public:
  explicit PowerAggregator(
    PowerAggregatorConfig config = PowerAggregatorConfig{});

  const PowerAggregatorConfig & config() const;

  void set_config(const PowerAggregatorConfig & config);

  PowerStatusSummary aggregate(
    const PowerAggregatorInputs & inputs) const;

  PowerState overall_state(
    const PowerAggregatorInputs & inputs) const;

private:
  PowerSourceStatus make_source_status(
    const TimedPowerSourceInput & input,
    BatterySource expected_source,
    bool expected) const;

  static PowerState more_severe(
    PowerState left,
    PowerState right);

  static std::string make_status_text(
    const PowerStatusSummary & summary);

  static std::string make_dashboard_text(
    const PowerStatusSummary & summary);

  PowerAggregatorConfig config_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_AGGREGATOR_HPP_
