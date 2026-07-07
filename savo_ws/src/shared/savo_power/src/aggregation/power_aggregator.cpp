#include "savo_power/power_aggregator.hpp"

#include <sstream>
#include <string>

namespace savo_power
{

PowerAggregator::PowerAggregator(PowerAggregatorConfig config)
: config_(config)
{
}

const PowerAggregatorConfig & PowerAggregator::config() const
{
  return config_;
}

void PowerAggregator::set_config(const PowerAggregatorConfig & config)
{
  config_ = config;
}

PowerStatusSummary PowerAggregator::aggregate(
  const PowerAggregatorInputs & inputs) const
{
  PowerStatusSummary summary;

  summary.core_ups = make_source_status(
    inputs.core_ups,
    BatterySource::CORE_UPS,
    config_.core_ups_expected);

  summary.edge_ups = make_source_status(
    inputs.edge_ups,
    BatterySource::EDGE_UPS,
    config_.edge_ups_expected);

  summary.base_battery = make_source_status(
    inputs.base_battery,
    BatterySource::BASE_BATTERY,
    config_.base_battery_expected);

  summary.overall_state = more_severe(
    more_severe(summary.core_ups.state, summary.edge_ups.state),
    summary.base_battery.state);

  summary.health_level =
    health_level_from_power_state(summary.overall_state);

  summary.status_text = make_status_text(summary);
  summary.dashboard_text = make_dashboard_text(summary);

  return summary;
}

PowerState PowerAggregator::overall_state(
  const PowerAggregatorInputs & inputs) const
{
  return aggregate(inputs).overall_state;
}

PowerSourceStatus PowerAggregator::make_source_status(
  const TimedPowerSourceInput & input,
  BatterySource expected_source,
  bool expected) const
{
  if (!expected) {
    return make_not_expected_source_status(expected_source);
  }

  if (!input.seen) {
    return make_missing_source_status(expected_source);
  }

  PowerSourceStatus status;
  status.source = expected_source;
  status.expected = true;
  status.seen = true;
  status.age_s = input.age_s;
  status.stale = input.age_s > config_.stale_timeout_s;

  if (status.stale) {
    status.state = PowerState::STALE;
    status.text = "stale";
    return status;
  }

  if (
    input.source != BatterySource::UNKNOWN &&
    input.source != expected_source)
  {
    status.state = PowerState::ERROR;
    status.text = "source mismatch";
    return status;
  }

  status.state = input.state;
  status.text = input.text.empty() ?
    std::string(to_string(status.state)) :
    input.text;

  return status;
}

PowerState PowerAggregator::more_severe(
  PowerState left,
  PowerState right)
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

std::string PowerAggregator::make_status_text(
  const PowerStatusSummary & summary)
{
  std::ostringstream out;

  out << "overall=" << std::string(to_string(summary.overall_state))
      << " health=" << std::string(to_string(summary.health_level))
      << " core=" << std::string(to_string(summary.core_ups.state))
      << " edge=" << std::string(to_string(summary.edge_ups.state))
      << " base=" << std::string(to_string(summary.base_battery.state));

  return out.str();
}

std::string PowerAggregator::make_dashboard_text(
  const PowerStatusSummary & summary)
{
  std::ostringstream out;

  out << "Overall power: "
      << std::string(to_string(summary.overall_state))
      << "\n";

  out << "Health: "
      << std::string(to_string(summary.health_level))
      << "\n";

  out << "Core UPS: "
      << summary.core_ups.text
      << "\n";

  out << "Edge UPS: "
      << summary.edge_ups.text
      << "\n";

  out << "Base battery: "
      << summary.base_battery.text
      << "\n";

  return out.str();
}

}  // namespace savo_power
