#include <gtest/gtest.h>

#include "savo_vo/vo_health_state.hpp"

namespace
{

constexpr double kNow = 10.0;
constexpr double kTimeout = 0.5;

savo_vo::VOHealthState fresh_state(const std::string & status)
{
  savo_vo::VOHealthState state;
  state.has_status = true;
  state.last_status_time_s = 9.8;
  state.status_text = status;
  return state;
}

TEST(VOHealthState, MissingOrStaleStatusFailsClosed)
{
  EXPECT_EQ(
    savo_vo::evaluate_vo_health({}, kNow, kTimeout),
    "stale: visual odometry status not received");

  auto state = fresh_state("tracking normal");
  state.last_status_time_s = 9.0;
  state.has_odom = true;
  state.last_odom_time_s = 9.9;
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "stale: visual odometry status timeout");
}

TEST(VOHealthState, FreshWaitingStatusDoesNotBecomeFakeOdomTimeout)
{
  const auto state = fresh_state("waiting_for_reference: first RGB-D frame stored");
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "waiting: waiting_for_reference: first RGB-D frame stored");
}

TEST(VOHealthState, FreshRejectedLostAndDegradedStatusesAreDegraded)
{
  for (const std::string status : {
      "rejected: invalid RGB-D frame interval; reference reseeded",
      "tracking lost",
      "tracking degraded"})
  {
    const auto state = fresh_state(status);
    EXPECT_EQ(
      savo_vo::evaluate_vo_health(state, kNow, kTimeout),
      "degraded: " + status);
  }
}

TEST(VOHealthState, FreshErrorStatusIsError)
{
  const auto state = fresh_state("error: failed to convert synchronized RGB-D images");
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "error: error: failed to convert synchronized RGB-D images");
}

TEST(VOHealthState, HealthyTrackingRequiresFreshOdometry)
{
  auto state = fresh_state("tracking normal");
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "stale: visual odometry not received");

  state.has_odom = true;
  state.last_odom_time_s = 9.0;
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "stale: visual odometry timeout");

  state.last_odom_time_s = 9.8;
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "ok: tracking normal");
}

}  // namespace
