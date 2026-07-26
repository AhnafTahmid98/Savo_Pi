// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "savo_nav/goal_gateway.hpp"

namespace
{

using Authority =
  savo_nav::MapToOdomAuthority;

using GatewayState =
  savo_nav::GoalGatewayState;

using MapMode =
  savo_nav::NavigationMapMode;

using ReadinessState =
  savo_nav::NavigationReadinessState;

using Source =
  savo_nav::GoalSource;

savo_nav::GoalValidationRequest MakeRequest(
  const std::string & goal_id,
  const Source source,
  const std::uint64_t sequence)
{
  savo_nav::GoalValidationRequest request;

  request.context.goal_id = goal_id;
  request.context.source = source;
  request.context.target_frame = "map";
  request.context.map_id = "campus-main";
  request.context.sequence = sequence;

  request.pose.x = 1.0;
  request.pose.y = 2.0;
  request.pose.yaw = 0.5;

  request.map_context.mode =
    MapMode::kSavedMap;

  request.map_context.authority =
    Authority::kAmcl;

  request.map_context.map_id =
    "campus-main";

  request.map_context.frame_id = "map";
  request.map_context.revision = 1;
  request.map_context.available = true;

  request.map_context.localization_ready =
    true;

  request.map_context.mapping_active =
    false;

  request.readiness.state =
    ReadinessState::kReady;

  request.readiness.goal_acceptance_allowed =
    true;

  request.readiness.reason = "ready";

  return request;
}

TEST(GoalGatewayTest, StartsIdle)
{
  const savo_nav::GoalGateway gateway;

  const auto snapshot = gateway.Snapshot();

  EXPECT_EQ(
    snapshot.state,
    GatewayState::kIdle);

  EXPECT_FALSE(gateway.HasActiveGoal());
}

TEST(GoalGatewayTest, AdmitsValidGoal)
{
  savo_nav::GoalGateway gateway;

  const auto decision = gateway.Admit(
    MakeRequest(
      "goal-1",
      Source::kNavigation,
      1));

  EXPECT_TRUE(decision.accepted);

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kReserved);
}

TEST(GoalGatewayTest, RejectsInvalidGoal)
{
  savo_nav::GoalGateway gateway;

  auto request = MakeRequest(
    "goal-1",
    Source::kNavigation,
    1);

  request.readiness.state =
    ReadinessState::kBlocked;

  request.readiness.goal_acceptance_allowed =
    false;

  const auto decision =
    gateway.Admit(request);

  EXPECT_FALSE(decision.accepted);

  EXPECT_EQ(
    decision.validation_code,
    savo_nav::ValidationCode::kNotReady);

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kIdle);
}

TEST(GoalGatewayTest, ForwardsAndActivates)
{
  savo_nav::GoalGateway gateway;

  ASSERT_TRUE(
    gateway.Admit(
      MakeRequest(
        "goal-1",
        Source::kNavigation,
        1)).accepted);

  EXPECT_TRUE(
    gateway.MarkForwarding(
      "goal-1",
      1));

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kForwarding);

  EXPECT_TRUE(
    gateway.MarkAcceptedByNav2(
      "goal-1",
      1));

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kActive);
}

TEST(GoalGatewayTest, EnforcesSingleGoal)
{
  savo_nav::GoalGateway gateway;

  ASSERT_TRUE(
    gateway.Admit(
      MakeRequest(
        "goal-1",
        Source::kNavigation,
        1)).accepted);

  const auto second = gateway.Admit(
    MakeRequest(
      "goal-2",
      Source::kExploration,
      1));

  EXPECT_FALSE(second.accepted);

  EXPECT_EQ(
    second.arbitration_code,
    savo_nav::GoalArbitrationCode::kBusy);
}

TEST(GoalGatewayTest, CancelKeepsOwnershipUntilAck)
{
  savo_nav::GoalGateway gateway;

  ASSERT_TRUE(
    gateway.Admit(
      MakeRequest(
        "goal-1",
        Source::kNavigation,
        1)).accepted);

  ASSERT_TRUE(
    gateway.MarkForwarding(
      "goal-1",
      1));

  ASSERT_TRUE(
    gateway.MarkAcceptedByNav2(
      "goal-1",
      1));

  const auto cancel =
    gateway.RequestCancel(
    "goal-1",
    1,
    "client_cancel");

  EXPECT_TRUE(cancel.accepted);
  EXPECT_TRUE(gateway.HasActiveGoal());
  EXPECT_TRUE(gateway.CancelRequested());

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kCanceling);

  const auto acknowledged =
    gateway.AcknowledgeCancellation(
    "goal-1",
    1);

  EXPECT_TRUE(acknowledged.accepted);
  EXPECT_FALSE(gateway.HasActiveGoal());

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kIdle);
}

TEST(GoalGatewayTest, LateSuccessAfterCancelCompletes)
{
  savo_nav::GoalGateway gateway;

  ASSERT_TRUE(
    gateway.Admit(
      MakeRequest(
        "goal-1",
        Source::kNavigation,
        1)).accepted);

  ASSERT_TRUE(
    gateway.MarkForwarding(
      "goal-1",
      1));

  ASSERT_TRUE(
    gateway.MarkAcceptedByNav2(
      "goal-1",
      1));

  ASSERT_TRUE(
    gateway.RequestCancel(
      "goal-1",
      1,
      "client_cancel").accepted);

  const auto completed =
    gateway.Complete(
    "goal-1",
    1,
    "late_success");

  EXPECT_TRUE(completed.accepted);
  EXPECT_FALSE(gateway.HasActiveGoal());

  EXPECT_EQ(
    gateway.Snapshot().state,
    GatewayState::kIdle);
}

TEST(GoalGatewayTest, ConvertsEveryState)
{
  EXPECT_EQ(
    savo_nav::GoalGateway::ToString(
      GatewayState::kIdle),
    "idle");

  EXPECT_EQ(
    savo_nav::GoalGateway::ToString(
      GatewayState::kReserved),
    "reserved");

  EXPECT_EQ(
    savo_nav::GoalGateway::ToString(
      GatewayState::kForwarding),
    "forwarding");

  EXPECT_EQ(
    savo_nav::GoalGateway::ToString(
      GatewayState::kActive),
    "active");

  EXPECT_EQ(
    savo_nav::GoalGateway::ToString(
      GatewayState::kCanceling),
    "canceling");
}

}  // namespace
