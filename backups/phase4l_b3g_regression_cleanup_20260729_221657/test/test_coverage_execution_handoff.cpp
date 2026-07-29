#include "savo_mapping/coverage_execution_handoff.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace
{

using savo_mapping::coverage::CoverageExecutionHandoffPolicy;
using savo_mapping::coverage::make_coverage_mission_id;
using savo_mapping::coverage::validate_coverage_execution_handoff_policy;

TEST(CoverageExecutionHandoffTest, AcceptsDefaultPolicy)
{
  EXPECT_TRUE(
    validate_coverage_execution_handoff_policy(
      CoverageExecutionHandoffPolicy{}).empty());
}

TEST(CoverageExecutionHandoffTest, RejectsInvalidTimeouts)
{
  auto policy = CoverageExecutionHandoffPolicy{};
  policy.server_wait_timeout_sec = 0.0;
  EXPECT_EQ(
    validate_coverage_execution_handoff_policy(policy),
    "coverage_handoff_server_wait_timeout_invalid");

  policy = CoverageExecutionHandoffPolicy{};
  policy.goal_response_timeout_sec =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_coverage_execution_handoff_policy(policy),
    "coverage_handoff_goal_response_timeout_invalid");

  policy = CoverageExecutionHandoffPolicy{};
  policy.cancel_timeout_sec = -1.0;
  EXPECT_EQ(
    validate_coverage_execution_handoff_policy(policy),
    "coverage_handoff_cancel_timeout_invalid");

  policy = CoverageExecutionHandoffPolicy{};
  policy.execution_timeout_sec = -0.1;
  EXPECT_EQ(
    validate_coverage_execution_handoff_policy(policy),
    "coverage_handoff_execution_timeout_invalid");
}

TEST(CoverageExecutionHandoffTest, RejectsZeroWaypointLimit)
{
  auto policy = CoverageExecutionHandoffPolicy{};
  policy.maximum_waypoints = 0U;
  EXPECT_EQ(
    validate_coverage_execution_handoff_policy(policy),
    "coverage_handoff_maximum_waypoints_invalid");
}

TEST(CoverageExecutionHandoffTest, BuildsStableMissionId)
{
  EXPECT_EQ(
    make_coverage_mission_id(
      "coverage", 7U, 123456),
    "coverage-7-123456");
}

TEST(CoverageExecutionHandoffTest, RejectsInvalidMissionIdInputs)
{
  EXPECT_TRUE(
    make_coverage_mission_id(
      "", 1U, 10).empty());
  EXPECT_TRUE(
    make_coverage_mission_id(
      "coverage mission", 1U, 10).empty());
  EXPECT_TRUE(
    make_coverage_mission_id(
      "coverage", 0U, 10).empty());
  EXPECT_TRUE(
    make_coverage_mission_id(
      "coverage", 1U, -1).empty());
}

}  // namespace
