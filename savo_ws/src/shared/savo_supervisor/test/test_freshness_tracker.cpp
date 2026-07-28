#include <gtest/gtest.h>

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/freshness_tracker.hpp"

namespace
{

rclcpp::Time steady_time(double seconds)
{
  return rclcpp::Time(
    static_cast<int64_t>(seconds * 1e9),
    RCL_STEADY_TIME);
}

builtin_interfaces::msg::Time message_stamp(
  int32_t seconds,
  uint32_t nanoseconds = 0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = seconds;
  stamp.nanosec = nanoseconds;
  return stamp;
}

}  // namespace

using savo_supervisor::FreshnessTracker;

TEST(FreshnessTracker, DisabledSnapshot)
{
  FreshnessTracker tracker;
  tracker.mark_disabled();

  const auto result =
    tracker.snapshot(steady_time(1.0), 1.0);

  EXPECT_TRUE(result.disabled);
  EXPECT_FALSE(result.received);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "disabled");
}

TEST(FreshnessTracker, NeverReceivedIsMissing)
{
  FreshnessTracker tracker;

  const auto result =
    tracker.snapshot(steady_time(1.0), 1.0);

  EXPECT_FALSE(result.received);
  EXPECT_TRUE(result.stale);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "never received");
}

TEST(FreshnessTracker, FreshMessageIsValid)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(1.5), 1.0);

  EXPECT_TRUE(result.received);
  EXPECT_FALSE(result.stale);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.age_s, 0.5);
  EXPECT_EQ(result.recovery_count, 0u);
}

TEST(FreshnessTracker, ExactTimeoutBoundaryIsFresh)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(2.0), 1.0);

  EXPECT_FALSE(result.stale);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.age_s, 1.0);
}

TEST(FreshnessTracker, MessageAfterTimeoutIsStale)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(2.001), 1.0);

  EXPECT_TRUE(result.stale);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "message stale");
}

TEST(FreshnessTracker, InitialAcquisitionIsNotRecovery)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  EXPECT_EQ(tracker.recovery_count(), 0u);
}

TEST(FreshnessTracker, StaleStreamCanRecover)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  const auto stale =
    tracker.snapshot(steady_time(3.0), 1.0);

  ASSERT_TRUE(stale.stale);

  tracker.observe_message(
    steady_time(3.1),
    message_stamp(3, 100000000),
    false,
    "");

  const auto recovered =
    tracker.snapshot(steady_time(3.2), 1.0);

  EXPECT_TRUE(recovered.valid);
  EXPECT_FALSE(recovered.stale);
  EXPECT_EQ(recovered.recovery_count, 1u);
}

TEST(FreshnessTracker, MalformedStreamCanRecover)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    true,
    "bad JSON");

  auto result =
    tracker.snapshot(steady_time(1.1), 1.0);

  ASSERT_TRUE(result.malformed);
  ASSERT_FALSE(result.valid);
  EXPECT_EQ(result.malformed_count, 1u);
  EXPECT_EQ(result.recovery_count, 0u);

  tracker.observe_message(
    steady_time(1.2),
    message_stamp(1, 200000000),
    false,
    "");

  result =
    tracker.snapshot(steady_time(1.3), 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.malformed_count, 1u);
  EXPECT_EQ(result.recovery_count, 1u);
}

TEST(FreshnessTracker, HeaderTimestampRegressionIsDetected)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(10),
    false,
    "");

  tracker.observe_message(
    steady_time(1.1),
    message_stamp(9),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(1.2), 1.0);

  EXPECT_TRUE(result.timestamp_fault);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.detail,
    "message timestamp regressed");
}

TEST(FreshnessTracker, HeaderTimestampRegressionCanRecover)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(10),
    false,
    "");

  tracker.observe_message(
    steady_time(1.1),
    message_stamp(9),
    false,
    "");

  tracker.observe_message(
    steady_time(1.2),
    message_stamp(11),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(1.3), 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.timestamp_fault);
  EXPECT_EQ(result.recovery_count, 1u);
}

TEST(FreshnessTracker, InvalidHeaderTimestampIsRejected)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1, 1000000000u),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(1.1), 1.0);

  EXPECT_TRUE(result.timestamp_fault);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "invalid message timestamp");
}

TEST(FreshnessTracker, ReceiveTimeRegressionIsDetected)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(10.0),
    message_stamp(10),
    false,
    "");

  tracker.observe_message(
    steady_time(9.0),
    message_stamp(11),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(9.1), 1.0);

  EXPECT_TRUE(result.time_regression);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "receive time regressed");
}

TEST(FreshnessTracker, ReceiveTimeRegressionCanRecover)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(10.0),
    message_stamp(10),
    false,
    "");

  tracker.observe_message(
    steady_time(9.0),
    message_stamp(11),
    false,
    "");

  tracker.observe_message(
    steady_time(9.2),
    message_stamp(12),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(9.3), 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.time_regression);
  EXPECT_EQ(result.recovery_count, 1u);
}

TEST(FreshnessTracker, CurrentTimeRegressionIsRejected)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(10.0),
    message_stamp(10),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(9.0), 1.0);

  EXPECT_TRUE(result.time_regression);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "current time regressed");
}

TEST(FreshnessTracker, InvalidTimeoutIsRejected)
{
  FreshnessTracker tracker;

  tracker.observe_message(
    steady_time(1.0),
    message_stamp(1),
    false,
    "");

  const auto result =
    tracker.snapshot(steady_time(1.1), 0.0);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.detail, "invalid timeout");
}
