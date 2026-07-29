#include "savo_mapping/coverage_mission.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace
{

using savo_mapping::coverage::CoverageMission;
using savo_mapping::coverage::CoverageMissionPlan;
using savo_mapping::coverage::CoverageMissionState;
using savo_mapping::coverage::kNoWaypoint;

CoverageMissionPlan make_plan(
  const std::string & mission_id = "coverage-1",
  const std::uint32_t total_waypoints = 4U,
  const double total_distance_m = 10.0,
  const std::int64_t created_at_ns = 10)
{
  return CoverageMissionPlan{
    mission_id,
    total_waypoints,
    total_distance_m,
    created_at_ns};
}

TEST(CoverageMissionTest, StartsIdle)
{
  const CoverageMission mission;

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kIdle);

  EXPECT_EQ(mission.reason(), "idle");
  EXPECT_EQ(mission.sequence(), 0U);
  EXPECT_EQ(mission.started_at_ns(), -1);
  EXPECT_TRUE(mission.plan().mission_id.empty());
  EXPECT_EQ(
    mission.progress().current_waypoint,
    kNoWaypoint);
}

TEST(CoverageMissionTest, LoadsValidPlan)
{
  CoverageMission mission;

  const auto result =
    mission.load_plan(make_plan());

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.previous, CoverageMissionState::kIdle);
  EXPECT_EQ(
    result.current,
    CoverageMissionState::kPlanReady);

  EXPECT_EQ(mission.plan().mission_id, "coverage-1");
  EXPECT_EQ(mission.plan().total_waypoints, 4U);
  EXPECT_DOUBLE_EQ(
    mission.progress().remaining_distance_m,
    10.0);

  EXPECT_EQ(mission.reason(), "plan_loaded");
  EXPECT_EQ(mission.sequence(), 1U);
}

TEST(CoverageMissionTest, RejectsInvalidPlans)
{
  CoverageMission mission;

  auto plan = make_plan();
  plan.mission_id.clear();

  EXPECT_FALSE(
    mission.load_plan(plan).accepted);

  plan = make_plan();
  plan.total_waypoints = 0U;

  EXPECT_FALSE(
    mission.load_plan(plan).accepted);

  plan = make_plan();
  plan.total_distance_m = -1.0;

  EXPECT_FALSE(
    mission.load_plan(plan).accepted);

  plan = make_plan();
  plan.total_distance_m =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(
    mission.load_plan(plan).accepted);

  plan = make_plan();
  plan.created_at_ns = -1;

  EXPECT_FALSE(
    mission.load_plan(plan).accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kIdle);

  EXPECT_EQ(mission.sequence(), 0U);
}

TEST(CoverageMissionTest, DispatchesAndStartsExecution)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  const auto dispatch =
    mission.request_dispatch(20);

  ASSERT_TRUE(dispatch.accepted);
  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kAwaitingDispatch);

  const auto accepted =
    mission.mark_accepted(30);

  ASSERT_TRUE(accepted.accepted);
  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kExecuting);

  EXPECT_EQ(mission.started_at_ns(), 30);
  EXPECT_EQ(mission.reason(), "execution_accepted");
}

TEST(CoverageMissionTest, UpdatesProgress)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  const auto result =
    mission.update_progress(
    "coverage-1",
    1U,
    1U,
    2.5,
    40);

  ASSERT_TRUE(result.accepted);

  const auto & progress = mission.progress();

  EXPECT_EQ(progress.current_waypoint, 1U);
  EXPECT_EQ(progress.completed_waypoints, 1U);
  EXPECT_DOUBLE_EQ(
    progress.completed_distance_m,
    2.5);

  EXPECT_DOUBLE_EQ(
    progress.remaining_distance_m,
    7.5);

  EXPECT_NEAR(
    progress.completion_ratio,
    0.25,
    1.0e-12);

  EXPECT_EQ(progress.latest_event_ns, 40);
}

TEST(CoverageMissionTest, RejectsWrongMissionFeedback)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  const auto result =
    mission.update_progress(
    "wrong-mission",
    1U,
    1U,
    2.5,
    40);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "mission_id_mismatch");

  EXPECT_EQ(
    mission.progress().completed_waypoints,
    0U);
}

TEST(CoverageMissionTest, RejectsStaleAndRegressingProgress)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  ASSERT_TRUE(
    mission.update_progress(
      "coverage-1",
      2U,
      2U,
      5.0,
      40).accepted);

  EXPECT_FALSE(
    mission.update_progress(
      "coverage-1",
      2U,
      2U,
      5.0,
      40).accepted);

  const auto waypoint_regression =
    mission.update_progress(
    "coverage-1",
    1U,
    1U,
    6.0,
    50);

  EXPECT_FALSE(waypoint_regression.accepted);
  EXPECT_EQ(
    waypoint_regression.reason,
    "completed_waypoints_regressed");

  const auto distance_regression =
    mission.update_progress(
    "coverage-1",
    2U,
    2U,
    4.0,
    51);

  EXPECT_FALSE(distance_regression.accepted);
  EXPECT_EQ(
    distance_regression.reason,
    "completed_distance_regressed");
}


TEST(CoverageMissionTest, CancelsBeforeDispatchAcceptance)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  const auto result =
    mission.request_cancel(
    20,
    "operator_canceled");

  ASSERT_TRUE(result.accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kCanceled);

  EXPECT_EQ(
    mission.reason(),
    "operator_canceled");
}

TEST(CoverageMissionTest, ActiveCancelRequiresTerminalResult)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  ASSERT_TRUE(
    mission.request_cancel(
      40,
      "operator_canceled").accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kCanceling);

  ASSERT_TRUE(
    mission.mark_canceled(
      50,
      "backend_canceled").accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kCanceled);
}


TEST(CoverageMissionTest, SuccessCompletesAllProgress)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  ASSERT_TRUE(
    mission.update_progress(
      "coverage-1",
      2U,
      2U,
      5.0,
      40).accepted);

  ASSERT_TRUE(
    mission.mark_succeeded(50).accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kSucceeded);

  EXPECT_EQ(
    mission.progress().current_waypoint,
    kNoWaypoint);

  EXPECT_EQ(
    mission.progress().completed_waypoints,
    4U);

  EXPECT_DOUBLE_EQ(
    mission.progress().completion_ratio,
    1.0);

  EXPECT_DOUBLE_EQ(
    mission.progress().remaining_distance_m,
    0.0);
}

TEST(CoverageMissionTest, ZeroDistanceUsesWaypointRatio)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(
      make_plan(
        "coverage-zero",
        4U,
        0.0,
        10)).accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_accepted(30).accepted);

  ASSERT_TRUE(
    mission.update_progress(
      "coverage-zero",
      2U,
      2U,
      0.0,
      40).accepted);

  EXPECT_DOUBLE_EQ(
    mission.progress().completion_ratio,
    0.5);

  EXPECT_DOUBLE_EQ(
    mission.progress().remaining_distance_m,
    0.0);
}

TEST(CoverageMissionTest, ResetsOnlyTerminalMission)
{
  CoverageMission mission;

  ASSERT_TRUE(
    mission.load_plan(make_plan()).accepted);

  EXPECT_FALSE(mission.reset().accepted);

  ASSERT_TRUE(
    mission.request_dispatch(20).accepted);

  ASSERT_TRUE(
    mission.mark_rejected(
      30,
      "nav_rejected").accepted);

  const auto reset = mission.reset();

  ASSERT_TRUE(reset.accepted);

  EXPECT_EQ(
    reset.previous,
    CoverageMissionState::kRejected);

  EXPECT_EQ(
    reset.current,
    CoverageMissionState::kIdle);

  EXPECT_EQ(reset.mission_id, "coverage-1");

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kIdle);

  EXPECT_TRUE(mission.plan().mission_id.empty());
  EXPECT_EQ(mission.reason(), "idle");
  EXPECT_EQ(mission.started_at_ns(), -1);
}

TEST(CoverageMissionTest, RejectsInvalidTransitions)
{
  CoverageMission mission;

  EXPECT_FALSE(
    mission.mark_accepted(10).accepted);

  EXPECT_FALSE(
    mission.request_dispatch(11).accepted);

  EXPECT_FALSE(
    mission.mark_succeeded(12).accepted);

  EXPECT_EQ(
    mission.state(),
    CoverageMissionState::kIdle);

  EXPECT_EQ(mission.sequence(), 0U);
}

}  // namespace
