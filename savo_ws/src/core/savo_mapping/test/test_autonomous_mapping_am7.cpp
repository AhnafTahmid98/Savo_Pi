#include "savo_mapping/autonomous_mapping_am7.hpp"

#include <gtest/gtest.h>

namespace
{

using savo_mapping::autonomous::PlanarPose;
using savo_mapping::autonomous::CoveragePlanCorrelation;
using savo_mapping::autonomous::ReturnActionLifecycle;
using savo_mapping::autonomous::evaluate_planar_proximity;
using savo_mapping::autonomous::evaluate_coverage_plan_correlation;
using savo_mapping::autonomous::parse_coverage_operation_status;
using savo_mapping::autonomous::parse_coverage_planner_status;
using savo_mapping::autonomous::return_goal_may_be_executing;

}  // namespace

TEST(AutonomousMappingAm7, ParsesFreshCoveragePlanStatus)
{
  const auto status = parse_coverage_planner_status(
    "{\"state\":\"plan_ready\",\"reason\":\"coverage_node_plan_ready\","
    "\"map_valid\":true,\"map_fresh\":true,\"map_sequence\":7,"
    "\"request_generation\":4,\"reset_generation\":2,"
    "\"plan_sequence\":3,\"waypoint_count\":42}");

  ASSERT_TRUE(status.parsed);
  EXPECT_TRUE(status.planning_complete);
  EXPECT_TRUE(status.plan_valid);
  EXPECT_EQ(status.plan_generation, 3U);
  EXPECT_EQ(status.map_generation, 7U);
  EXPECT_EQ(status.request_generation, 4U);
  EXPECT_EQ(status.reset_generation, 2U);
  EXPECT_EQ(status.waypoint_count, 42U);
}

TEST(AutonomousMappingAm7, EmptyPlanRequiresExplicitNoop)
{
  const auto malformed = parse_coverage_planner_status(
    "{\"state\":\"plan_ready\",\"map_valid\":true,\"map_fresh\":true,"
    "\"plan_sequence\":2,\"map_sequence\":4,\"waypoint_count\":0}");
  EXPECT_FALSE(malformed.plan_valid);

  const auto noop = parse_coverage_planner_status(
    "{\"state\":\"plan_ready\",\"map_valid\":true,\"map_fresh\":true,"
    "\"plan_sequence\":3,\"map_sequence\":4,\"waypoint_count\":0,"
    "\"explicit_noop\":true}");
  EXPECT_TRUE(noop.plan_valid);
  EXPECT_TRUE(noop.explicit_noop);
}

TEST(AutonomousMappingAm7, ParsesCorrelatedCoverageProgress)
{
  const auto status = parse_coverage_operation_status(
    "{\"state\":\"executing\",\"supervisor_authorized\":true,"
    "\"candidate_valid\":true,\"candidate_generation\":8,"
    "\"feedback_received\":true,\"feedback_sequence\":12,"
    "\"feedback_age_s\":0.25,"
    "\"mission_id\":\"coverage-8-1\",\"terminal_state\":\"\","
    "\"result_reason\":\"executing\",\"latest_feedback\":"
    "\"{\\\"mission_id\\\":\\\"coverage-8-1\\\","
    "\\\"current_waypoint\\\":3,\\\"completed_waypoints\\\":2,"
    "\\\"total_waypoints\\\":10,\\\"completion_ratio\\\":0.2,"
    "\\\"remaining_distance_m\\\":4.5}\"}");

  ASSERT_TRUE(status.parsed);
  EXPECT_TRUE(status.supervisor_authorized);
  EXPECT_EQ(status.candidate_generation, 8U);
  EXPECT_TRUE(status.feedback_received);
  EXPECT_EQ(status.feedback_sequence, 12U);
  EXPECT_DOUBLE_EQ(status.feedback_age_s, 0.25);
  EXPECT_EQ(status.current_waypoint, 3U);
  EXPECT_EQ(status.completed_waypoints, 2U);
  EXPECT_DOUBLE_EQ(status.remaining_distance_m, 4.5);
}

TEST(AutonomousMappingAm7, ReturnLifecycleSeparatesPendingFromActive)
{
  EXPECT_TRUE(return_goal_may_be_executing(
      ReturnActionLifecycle::GoalRequestPending));
  EXPECT_TRUE(return_goal_may_be_executing(
      ReturnActionLifecycle::AcceptedActive));
  EXPECT_TRUE(return_goal_may_be_executing(
      ReturnActionLifecycle::CancelPending));
  EXPECT_FALSE(return_goal_may_be_executing(ReturnActionLifecycle::Idle));
  EXPECT_FALSE(return_goal_may_be_executing(ReturnActionLifecycle::Terminal));
}

TEST(AutonomousMappingAm7, CorrelatesPlanToCurrentResetRequestAndMap)
{
  const CoveragePlanCorrelation expected{4U, 3U, 7U, 20U, true};
  const auto current = parse_coverage_planner_status(
    "{\"state\":\"plan_ready\",\"reason\":\"current\","
    "\"map_valid\":true,\"map_fresh\":true,"
    "\"request_generation\":4,\"reset_generation\":3,"
    "\"plan_sequence\":8,\"map_sequence\":20,"
    "\"waypoint_count\":5}");
  EXPECT_TRUE(evaluate_coverage_plan_correlation(current, expected).accepted);

  auto stale_request = current;
  stale_request.request_generation = 3U;
  EXPECT_EQ(
    evaluate_coverage_plan_correlation(stale_request, expected).reason,
    "coverage_plan_request_generation_stale");

  auto stale_reset = current;
  stale_reset.reset_generation = 2U;
  EXPECT_EQ(
    evaluate_coverage_plan_correlation(stale_reset, expected).reason,
    "coverage_plan_reset_generation_stale");

  auto previous_plan = current;
  previous_plan.plan_generation = 7U;
  EXPECT_EQ(
    evaluate_coverage_plan_correlation(previous_plan, expected).reason,
    "coverage_plan_generation_stale");

  auto stale_map = current;
  stale_map.map_generation = 19U;
  EXPECT_EQ(
    evaluate_coverage_plan_correlation(stale_map, expected).reason,
    "coverage_plan_map_generation_stale");
}

TEST(AutonomousMappingAm7, AcceptsCorrelatedExplicitNoopPlan)
{
  const auto noop = parse_coverage_planner_status(
    "{\"state\":\"plan_ready\",\"reason\":\"valid_noop\","
    "\"map_valid\":true,\"map_fresh\":true,"
    "\"request_generation\":2,\"reset_generation\":5,"
    "\"plan_sequence\":9,\"map_sequence\":12,"
    "\"waypoint_count\":0,\"explicit_noop\":true}");
  const CoveragePlanCorrelation expected{2U, 5U, 8U, 12U, true};
  EXPECT_TRUE(evaluate_coverage_plan_correlation(noop, expected).accepted);
}

TEST(AutonomousMappingAm7, MalformedNestedFeedbackPreservesOperationState)
{
  const auto status = parse_coverage_operation_status(
    "{\"state\":\"approval_pending\",\"supervisor_authorized\":true,"
    "\"candidate_valid\":true,\"candidate_generation\":9,"
    "\"approval_pending\":true,\"latest_feedback\":\"not-json\"}");

  ASSERT_TRUE(status.parsed);
  EXPECT_EQ(status.state, "approval_pending");
  EXPECT_TRUE(status.approval_pending);
  EXPECT_EQ(status.candidate_generation, 9U);
}

TEST(AutonomousMappingAm7, EvaluatesReturnNearStartTolerance)
{
  const PlanarPose start{1.0, 2.0, 0.0, 1.0};
  const PlanarPose near{1.2, 2.1, 0.0, 1.0};
  const auto accepted = evaluate_planar_proximity(
    start, near, 0.35, false, 0.1);
  EXPECT_TRUE(accepted.valid);
  EXPECT_TRUE(accepted.within_tolerance);

  const PlanarPose far{1.5, 2.0, 0.0, 1.0};
  const auto rejected = evaluate_planar_proximity(
    start, far, 0.35, false, 0.1);
  EXPECT_TRUE(rejected.valid);
  EXPECT_FALSE(rejected.within_tolerance);
}
