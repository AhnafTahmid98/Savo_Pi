// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_supervisor/location_authorization_policy.hpp"

namespace
{

savo_supervisor::SupervisorState ready_state()
{
  savo_supervisor::SupervisorState state;
  state.lifecycle = savo_supervisor::Lifecycle::RUNNING;
  state.health = savo_supervisor::AggregateHealth::OK;
  state.safety = savo_supervisor::SafetyObservation::CLEAR;
  state.ready = true;
  return state;
}

savo_supervisor::LocationAuthorizationRequest navigation_request()
{
  savo_supervisor::LocationAuthorizationRequest request;
  request.operation =
    savo_supervisor::LocationOperation::kNavigateToLocation;
  request.request_id = "nav-a201-1";
  request.actor_id = "operator";
  request.location_id = "A201";
  request.map_id = "campus_main";
  request.map_revision = 7U;
  request.motion_required = true;
  return request;
}

TEST(LocationAuthorizationPolicy, AuthorizesReadyHealthyNavigation)
{
  savo_supervisor::LocationAuthorizationEvaluator evaluator;
  const auto decision = evaluator.Evaluate(
    navigation_request(), ready_state());
  EXPECT_TRUE(decision.authorized);
}

TEST(LocationAuthorizationPolicy, RejectsSafetyStop)
{
  savo_supervisor::LocationAuthorizationEvaluator evaluator;
  auto state = ready_state();
  state.safety = savo_supervisor::SafetyObservation::STOPPED;
  const auto decision = evaluator.Evaluate(
    navigation_request(), state);
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(
    decision.code,
    savo_supervisor::LocationAuthorizationCode::kSafetyBlocked);
}

TEST(LocationAuthorizationPolicy, RejectsDegradedMotionByDefault)
{
  savo_supervisor::LocationAuthorizationEvaluator evaluator;
  auto state = ready_state();
  state.health = savo_supervisor::AggregateHealth::DEGRADED;
  const auto decision = evaluator.Evaluate(
    navigation_request(), state);
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(
    decision.code,
    savo_supervisor::LocationAuthorizationCode::kHealthBlocked);
}

}  // namespace
