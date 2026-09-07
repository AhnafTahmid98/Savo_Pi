#include <chrono>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "savo_perception/perception_types.hpp"
#include "savo_perception/range_fusion.hpp"

namespace savo_perception
{
namespace
{

RangeSample sample(
  const std::string & name,
  const double distance_m,
  const bool valid = true)
{
  auto value = make_range_sample(name, distance_m, valid, "test");
  return value;
}

RangeSample invalid_sample(const std::string & name)
{
  return make_invalid_range_sample(name, "invalid_distance", "test");
}

RangeSnapshot clear_snapshot()
{
  RangeSnapshot snapshot;
  snapshot.depth_front = sample("depth_front", 1.5);
  snapshot.tof_left = sample("tof_left", 0.6);
  snapshot.tof_right = sample("tof_right", 0.6);
  snapshot.ultrasonic_front = sample("ultrasonic_front", 1.0);
  return snapshot;
}

TEST(RangeFusionTest, LeftRequiredTofInvalidStops)
{
  auto snapshot = clear_snapshot();
  snapshot.tof_left = invalid_sample("tof_left");

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_TRUE(result.decision.stop_required);
  EXPECT_EQ(result.decision.reason, "required_sensor_invalid");
}

TEST(RangeFusionTest, RightRequiredTofInvalidStops)
{
  auto snapshot = clear_snapshot();
  snapshot.tof_right = invalid_sample("tof_right");

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_TRUE(result.decision.stop_required);
  EXPECT_EQ(result.decision.reason, "required_sensor_invalid");
}

TEST(RangeFusionTest, RequiredTofStaleStops)
{
  auto snapshot = clear_snapshot();
  snapshot.tof_left.stamp = std::chrono::steady_clock::now() - std::chrono::seconds(2);

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_TRUE(result.decision.stop_required);
  EXPECT_EQ(result.decision.reason, "required_sensor_stale");
}

TEST(RangeFusionTest, HealthyRequiredTofsDoNotCauseRequiredSensorStop)
{
  const auto result = fuse_range_snapshot(clear_snapshot(), RangeFusionConfig{});

  EXPECT_FALSE(result.decision.stop_required);
  EXPECT_EQ(result.decision.reason, "clear");
}

TEST(RangeFusionTest, DisabledUltrasonicIsCompletelyIgnored)
{
  auto snapshot = clear_snapshot();
  snapshot.ultrasonic_front = invalid_sample("ultrasonic_front");
  snapshot.ultrasonic_front.stamp =
    std::chrono::steady_clock::now() - std::chrono::seconds(2);
  auto config = RangeFusionConfig{};
  config.use_ultrasonic = false;
  config.required_sensors.push_back("ultrasonic_front");

  const auto result = fuse_range_snapshot(snapshot, config);

  EXPECT_FALSE(result.decision.stop_required);
  EXPECT_FALSE(contains_sensor_name(result.stale_sensors, "ultrasonic_front"));
  EXPECT_FALSE(contains_sensor_name(result.invalid_sensors, "ultrasonic_front"));
  EXPECT_FALSE(result.ultrasonic_front_distance_m.has_value());
}

TEST(RangeFusionTest, OptionalInvalidUltrasonicDoesNotBecomeRequiredFailure)
{
  auto snapshot = clear_snapshot();
  snapshot.ultrasonic_front = invalid_sample("ultrasonic_front");

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_FALSE(result.decision.stop_required);
  EXPECT_NE(result.decision.reason, "required_sensor_invalid");
  EXPECT_TRUE(contains_sensor_name(result.invalid_sensors, "ultrasonic_front"));
}

TEST(RangeFusionTest, OptionalStaleUltrasonicDoesNotBecomeRequiredFailure)
{
  auto snapshot = clear_snapshot();
  snapshot.ultrasonic_front.stamp =
    std::chrono::steady_clock::now() - std::chrono::seconds(2);

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_FALSE(result.decision.stop_required);
  EXPECT_NE(result.decision.reason, "required_sensor_stale");
  EXPECT_TRUE(contains_sensor_name(result.stale_sensors, "ultrasonic_front"));
}

TEST(RangeFusionTest, EnabledCloseUltrasonicStops)
{
  auto snapshot = clear_snapshot();
  snapshot.ultrasonic_front = sample("ultrasonic_front", 0.05);

  const auto result = fuse_range_snapshot(snapshot, RangeFusionConfig{});

  EXPECT_TRUE(result.decision.stop_required);
  EXPECT_EQ(result.decision.reason, "front_stop_zone");
}

TEST(RangeFusionTest, NonFiniteAndNonPositiveDistancesAreInvalid)
{
  EXPECT_FALSE(is_valid_distance(std::numeric_limits<double>::quiet_NaN(), 0.0, 10.0));
  EXPECT_FALSE(is_valid_distance(std::numeric_limits<double>::infinity(), 0.0, 10.0));
  EXPECT_FALSE(is_valid_distance(-std::numeric_limits<double>::infinity(), 0.0, 10.0));
  EXPECT_FALSE(is_valid_distance(0.0, 0.01, 10.0));
  EXPECT_FALSE(is_valid_distance(-0.1, 0.01, 10.0));
  EXPECT_TRUE(is_valid_distance(0.5, 0.01, 10.0));
}

TEST(RangeFusionTest, ContradictoryInfiniteSampleCannotReportValidHealth)
{
  const auto contradictory = sample(
    "tof_left",
    std::numeric_limits<double>::infinity());

  const auto health = make_sensor_health(contradictory, 1.0);

  EXPECT_FALSE(health.valid);
  EXPECT_FALSE(health.ok);
  EXPECT_FALSE(health.last_distance_m.has_value());
  EXPECT_EQ(health.status, SensorStatus::kError);
}

}  // namespace
}  // namespace savo_perception
