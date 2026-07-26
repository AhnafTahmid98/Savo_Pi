// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "savo_nav/goal_validator.hpp"

namespace
{

using Authority = savo_nav::MapToOdomAuthority;
using Mode = savo_nav::NavigationMapMode;
using Source = savo_nav::GoalSource;
using State = savo_nav::NavigationReadinessState;

savo_nav::GoalValidationRequest MakeValidRequest()
{
  savo_nav::GoalValidationRequest request;

  request.context.goal_id = "navigation-42";
  request.context.source = Source::kNavigation;
  request.context.target_frame = "map";
  request.context.map_id = "campus-main";
  request.context.sequence = 42;

  request.pose.x = 2.0;
  request.pose.y = -1.5;
  request.pose.yaw = 0.5;

  request.map_context.mode = Mode::kSavedMap;
  request.map_context.authority = Authority::kAmcl;
  request.map_context.map_id = "campus-main";
  request.map_context.frame_id = "map";
  request.map_context.revision = 3;
  request.map_context.available = true;
  request.map_context.localization_ready = true;
  request.map_context.mapping_active = false;

  request.readiness.state = State::kReady;
  request.readiness.goal_acceptance_allowed = true;
  request.readiness.reason =
    "all_required_navigation_dependencies_ready";

  return request;
}

TEST(GoalValidatorTest, AcceptsValidSavedMapGoal)
{
  const auto validation =
    savo_nav::GoalValidator::Validate(
    MakeValidRequest());

  EXPECT_TRUE(validation.IsValid());
}

TEST(GoalValidatorTest, RejectsNonfinitePosition)
{
  auto request = MakeValidRequest();

  request.pose.x =
    std::numeric_limits<double>::quiet_NaN();

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidPose);
}

TEST(GoalValidatorTest, RejectsCoordinateLimit)
{
  auto request = MakeValidRequest();
  request.pose.x = 1001.0;

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kCoordinateOutOfBounds);
}

TEST(GoalValidatorTest, RejectsUnnormalizedYaw)
{
  auto request = MakeValidRequest();
  request.pose.yaw = 4.0;

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidPose);
}

TEST(GoalValidatorTest, RejectsUnavailableMap)
{
  auto request = MakeValidRequest();

  request.map_context = {};

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kMapUnavailable);
}

TEST(GoalValidatorTest, RejectsNotReadyState)
{
  auto request = MakeValidRequest();

  request.readiness.state =
    State::kWaitingForLocalization;

  request.readiness.goal_acceptance_allowed = false;

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kNotReady);
}

TEST(GoalValidatorTest, RejectsSavedMapMismatch)
{
  auto request = MakeValidRequest();
  request.context.map_id = "wrong-map";

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kMapMismatch);
}

TEST(GoalValidatorTest, RejectsMissingSavedMapId)
{
  auto request = MakeValidRequest();
  request.context.map_id.clear();

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kMissingMapId);
}

TEST(GoalValidatorTest, RejectsCancelFlagOnNewGoal)
{
  auto request = MakeValidRequest();
  request.context.cancel_requested = true;

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kCancellationRequested);
}

TEST(GoalValidatorTest, RejectsInvalidSourceValue)
{
  auto request = MakeValidRequest();

  request.context.source =
    static_cast<Source>(255);

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidState);
}

TEST(GoalValidatorTest, AcceptsLiveMappingWithoutMapId)
{
  auto request = MakeValidRequest();

  request.context.source = Source::kExploration;
  request.context.map_id.clear();

  request.map_context.mode = Mode::kLiveMapping;

  request.map_context.authority =
    Authority::kSlamToolbox;

  request.map_context.map_id.clear();
  request.map_context.mapping_active = true;

  const auto validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_TRUE(validation.IsValid());
}

TEST(GoalValidatorTest, DegradedRequiresExplicitPolicy)
{
  auto request = MakeValidRequest();

  request.readiness.state = State::kDegraded;
  request.readiness.goal_acceptance_allowed = true;

  const auto default_validation =
    savo_nav::GoalValidator::Validate(request);

  EXPECT_EQ(
    default_validation.code,
    savo_nav::ValidationCode::kNotReady);

  savo_nav::GoalValidationPolicy policy;
  policy.allow_degraded_readiness = true;

  const auto allowed_validation =
    savo_nav::GoalValidator::Validate(
    request,
    policy);

  EXPECT_TRUE(allowed_validation.IsValid());
}

TEST(GoalValidatorTest, RejectsInvalidPolicy)
{
  auto request = MakeValidRequest();

  savo_nav::GoalValidationPolicy policy;
  policy.max_abs_coordinate_m = 0.0;

  const auto validation =
    savo_nav::GoalValidator::Validate(
    request,
    policy);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidCombination);
}

}  // namespace
