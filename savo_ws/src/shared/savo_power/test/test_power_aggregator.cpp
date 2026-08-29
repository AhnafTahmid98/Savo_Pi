#include "gtest/gtest.h"

#include "savo_power/power_aggregator.hpp"

namespace
{

savo_power::TimedPowerSourceInput source_input(
  const savo_power::BatterySource source,
  const savo_power::PowerState state = savo_power::PowerState::OK)
{
  savo_power::TimedPowerSourceInput input;
  input.source = source;
  input.seen = true;
  input.state = state;
  input.text = std::string(savo_power::to_string(state));
  return input;
}

savo_power::PowerAggregatorInputs healthy_required_inputs()
{
  savo_power::PowerAggregatorInputs inputs;
  inputs.core_ups = source_input(savo_power::BatterySource::CORE_UPS);
  inputs.base_battery = source_input(savo_power::BatterySource::BASE_BATTERY);
  return inputs;
}

TEST(PowerAggregatorDefaults, RequiresCoreAndBaseButNotEdge)
{
  const savo_power::PowerAggregatorConfig config;

  EXPECT_TRUE(config.core_ups_expected);
  EXPECT_FALSE(config.edge_ups_expected);
  EXPECT_TRUE(config.base_battery_expected);
}

TEST(PowerAggregatorOptionalEdge, MissingEdgeKeepsHealthyAggregate)
{
  const savo_power::PowerAggregator aggregator;
  const auto summary = aggregator.aggregate(healthy_required_inputs());

  EXPECT_EQ(summary.overall_state, savo_power::PowerState::OK);
  EXPECT_EQ(summary.health_level, savo_power::PowerHealthLevel::OK);
  EXPECT_FALSE(summary.edge_ups.expected);
  EXPECT_FALSE(summary.edge_ups.seen);
}

TEST(PowerAggregatorOptionalEdge, EdgeErrorDoesNotChangeAggregateSeverity)
{
  auto inputs = healthy_required_inputs();
  inputs.edge_ups = source_input(
    savo_power::BatterySource::EDGE_UPS,
    savo_power::PowerState::ERROR);

  const savo_power::PowerAggregator aggregator;
  const auto summary = aggregator.aggregate(inputs);

  EXPECT_EQ(summary.overall_state, savo_power::PowerState::OK);
  EXPECT_EQ(summary.health_level, savo_power::PowerHealthLevel::OK);
  EXPECT_FALSE(summary.edge_ups.expected);
  EXPECT_TRUE(summary.edge_ups.seen);
  EXPECT_EQ(summary.edge_ups.state, savo_power::PowerState::ERROR);
}

TEST(PowerAggregatorRequiredEdge, MissingEdgePreservesFailureBehavior)
{
  savo_power::PowerAggregatorConfig config;
  config.edge_ups_expected = true;

  const savo_power::PowerAggregator aggregator(config);
  const auto summary = aggregator.aggregate(healthy_required_inputs());

  EXPECT_EQ(summary.overall_state, savo_power::PowerState::UNKNOWN);
  EXPECT_EQ(summary.health_level, savo_power::PowerHealthLevel::ERROR);
  EXPECT_TRUE(summary.edge_ups.expected);
  EXPECT_FALSE(summary.edge_ups.seen);
}

TEST(PowerAggregatorRequiredEdge, EdgeErrorPreservesFailureBehavior)
{
  savo_power::PowerAggregatorConfig config;
  config.edge_ups_expected = true;
  auto inputs = healthy_required_inputs();
  inputs.edge_ups = source_input(
    savo_power::BatterySource::EDGE_UPS,
    savo_power::PowerState::ERROR);

  const savo_power::PowerAggregator aggregator(config);
  const auto summary = aggregator.aggregate(inputs);

  EXPECT_EQ(summary.overall_state, savo_power::PowerState::ERROR);
  EXPECT_EQ(summary.health_level, savo_power::PowerHealthLevel::ERROR);
}

TEST(PowerAggregatorRequiredCore, MissingOrErrorStillFailsWithOptionalEdge)
{
  const savo_power::PowerAggregator aggregator;

  auto missing = healthy_required_inputs();
  missing.core_ups = {};
  EXPECT_EQ(
    aggregator.aggregate(missing).health_level,
    savo_power::PowerHealthLevel::ERROR);

  auto error = healthy_required_inputs();
  error.core_ups = source_input(
    savo_power::BatterySource::CORE_UPS,
    savo_power::PowerState::ERROR);
  EXPECT_EQ(
    aggregator.aggregate(error).overall_state,
    savo_power::PowerState::ERROR);
}

TEST(PowerAggregatorRequiredBase, MissingOrErrorStillFailsWithOptionalEdge)
{
  const savo_power::PowerAggregator aggregator;

  auto missing = healthy_required_inputs();
  missing.base_battery = {};
  EXPECT_EQ(
    aggregator.aggregate(missing).health_level,
    savo_power::PowerHealthLevel::ERROR);

  auto error = healthy_required_inputs();
  error.base_battery = source_input(
    savo_power::BatterySource::BASE_BATTERY,
    savo_power::PowerState::ERROR);
  EXPECT_EQ(
    aggregator.aggregate(error).overall_state,
    savo_power::PowerState::ERROR);
}

}  // namespace
