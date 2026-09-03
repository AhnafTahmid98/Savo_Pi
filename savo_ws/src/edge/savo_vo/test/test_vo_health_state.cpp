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

void add_fresh_odom(savo_vo::VOHealthState & state)
{
  state.has_odom = true;
  state.last_odom_time_s = 9.8;
}

TEST(VOHealthState, MissingStatusAndMissingOdometryIsStale)
{
  EXPECT_EQ(
    savo_vo::evaluate_vo_health({}, kNow, kTimeout),
    "stale: visual odometry status and odometry not received");
}

TEST(VOHealthState, StaleStatusAndStaleOdometryIsStale)
{
  auto state = fresh_state("tracking normal");
  state.last_status_time_s = 9.0;
  state.has_odom = true;
  state.last_odom_time_s = 9.0;
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "stale: visual odometry timeout; status channel stale");
}

TEST(VOHealthState, MissingStatusAndFreshOdometryIsOperationalWithTelemetryWarning)
{
  savo_vo::VOHealthState state;
  add_fresh_odom(state);
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "ok: odometry fresh; status channel not received; telemetry degraded");
}

TEST(VOHealthState, StaleStatusAndFreshOdometryIsOperationalWithTelemetryWarning)
{
  auto state = fresh_state("tracking normal");
  state.last_status_time_s = 9.0;
  add_fresh_odom(state);
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "ok: odometry fresh; status channel stale");
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
    auto state = fresh_state(status);
    add_fresh_odom(state);
    EXPECT_EQ(
      savo_vo::evaluate_vo_health(state, kNow, kTimeout),
      "degraded: " + status);
  }
}

TEST(VOHealthState, FreshErrorStatusIsError)
{
  auto state = fresh_state("error: failed to convert synchronized RGB-D images");
  add_fresh_odom(state);
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "error: error: failed to convert synchronized RGB-D images");
}

TEST(VOHealthState, FreshHealthyStatusAndFreshOdometryIsOk)
{
  auto state = fresh_state("tracking normal");
  add_fresh_odom(state);
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "ok: tracking normal");
}

TEST(VOHealthState, FreshHealthyStatusRequiresFreshOdometry)
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
}

TEST(VOHealthState, FreshStatusAtTimeoutBoundaryRemainsAuthoritative)
{
  auto state = fresh_state("error: tracking failed");
  state.last_status_time_s = kNow - kTimeout;
  add_fresh_odom(state);
  EXPECT_EQ(
    savo_vo::evaluate_vo_health(state, kNow, kTimeout),
    "error: error: tracking failed");
}

}  // namespace
