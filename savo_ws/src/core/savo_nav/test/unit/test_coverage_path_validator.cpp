// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <limits>

#include "gtest/gtest.h"

#include "savo_nav/coverage_path_validator.hpp"

namespace
{

using Authority = savo_nav::MapToOdomAuthority;
using Mode = savo_nav::NavigationMapMode;
using Point = savo_nav::CoveragePathPoint;
using Policy = savo_nav::CoveragePathValidationPolicy;
using Request = savo_nav::CoveragePathValidationRequest;
using State = savo_nav::NavigationReadinessState;
using ValidationCode = savo_nav::ValidationCode;

Point MakePoint(
  const double x,
  const double y)
{
  Point point;

  point.x = x;
  point.y = y;
  point.frame_id = "map";

  return point;
}

Request MakeValidRequest()
{
  Request request;

  request.contract_version = 1;
  request.mission_id = "coverage-1";
  request.path_frame = "map";

  request.points = {
    MakePoint(0.0, 0.0),
    MakePoint(3.0, 4.0)
  };

  request.execution_timeout_seconds = 300.0;

  request.map_context.mode = Mode::kSavedMap;
  request.map_context.authority = Authority::kAmcl;
  request.map_context.map_id = "campus-main";
  request.map_context.frame_id = "map";
  request.map_context.revision = 1;
  request.map_context.available = true;
  request.map_context.localization_ready = true;
  request.map_context.mapping_active = false;

  request.readiness.state = State::kReady;
  request.readiness.goal_acceptance_allowed = true;
  request.readiness.reason = "ready";

  return request;
}

TEST(CoveragePathValidatorTest, AcceptsValidPath)
{
  const auto result =
    savo_nav::CoveragePathValidator::Validate(
    MakeValidRequest());

  EXPECT_TRUE(result.IsValid());
  EXPECT_EQ(
    result.validation.reason,
    "coverage_path_valid");

  EXPECT_DOUBLE_EQ(
    result.total_distance_m,
    5.0);
}

TEST(CoveragePathValidatorTest, AcceptsConfiguredDefaultTimeout)
{
  auto request = MakeValidRequest();
  request.execution_timeout_seconds = 0.0;

  EXPECT_TRUE(
    savo_nav::CoveragePathValidator::Validate(
      request).IsValid());
}

TEST(CoveragePathValidatorTest, RejectsContractMismatch)
{
  auto request = MakeValidRequest();
  request.contract_version = 2;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsEmptyMissionId)
{
  auto request = MakeValidRequest();
  request.mission_id.clear();

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kEmptyIdentifier);
}

TEST(CoveragePathValidatorTest, RejectsWhitespaceMissionId)
{
  auto request = MakeValidRequest();
  request.mission_id = "coverage mission";

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidIdentifier);
}

TEST(CoveragePathValidatorTest, RejectsMissionIdLengthLimit)
{
  auto request = MakeValidRequest();

  Policy policy;
  policy.maximum_mission_id_length = 4;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(
    request,
    policy);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidIdentifier);
}

TEST(CoveragePathValidatorTest, RejectsNonMapPathFrame)
{
  auto request = MakeValidRequest();
  request.path_frame = "odom";

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidFrame);
}

TEST(CoveragePathValidatorTest, RejectsEmptyPath)
{
  auto request = MakeValidRequest();
  request.points.clear();

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsWaypointLimit)
{
  auto request = MakeValidRequest();

  Policy policy;
  policy.maximum_waypoints = 1;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(
    request,
    policy);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsWaypointFrameMismatch)
{
  auto request = MakeValidRequest();
  request.points[1].frame_id = "odom";

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidFrame);
}

TEST(CoveragePathValidatorTest, RejectsNonfiniteWaypoint)
{
  auto request = MakeValidRequest();

  request.points[1].x =
    std::numeric_limits<double>::quiet_NaN();

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidPose);
}

TEST(CoveragePathValidatorTest, RejectsCoordinateLimit)
{
  auto request = MakeValidRequest();
  request.points[1].x = 1001.0;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kCoordinateOutOfBounds);
}

TEST(CoveragePathValidatorTest, RejectsNonPlanarOrientation)
{
  auto request = MakeValidRequest();
  request.points[1].orientation_x = 0.1;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidPose);
}

TEST(CoveragePathValidatorTest, RejectsUnnormalizedOrientation)
{
  auto request = MakeValidRequest();
  request.points[1].orientation_w = 0.5;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidPose);
}

TEST(CoveragePathValidatorTest, RejectsDistanceLimit)
{
  auto request = MakeValidRequest();

  Policy policy;
  policy.maximum_total_distance_m = 4.0;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(
    request,
    policy);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsUnavailableMap)
{
  auto request = MakeValidRequest();
  request.map_context = {};

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kMapUnavailable);
}

TEST(CoveragePathValidatorTest, RejectsBlockedReadiness)
{
  auto request = MakeValidRequest();

  request.readiness.state = State::kBlocked;
  request.readiness.goal_acceptance_allowed = false;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kNotReady);
}

TEST(CoveragePathValidatorTest, AllowsExplicitDegradedPolicy)
{
  auto request = MakeValidRequest();

  request.readiness.state = State::kDegraded;
  request.readiness.goal_acceptance_allowed = true;

  Policy policy;
  policy.allow_degraded_readiness = true;

  EXPECT_TRUE(
    savo_nav::CoveragePathValidator::Validate(
      request,
      policy).IsValid());
}

TEST(CoveragePathValidatorTest, RejectsNegativeTimeout)
{
  auto request = MakeValidRequest();
  request.execution_timeout_seconds = -1.0;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsTimeoutAboveLimit)
{
  auto request = MakeValidRequest();
  request.execution_timeout_seconds = 4000.0;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(request);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

TEST(CoveragePathValidatorTest, RejectsInvalidPolicy)
{
  const auto request = MakeValidRequest();

  Policy policy;
  policy.maximum_waypoints = 0;

  const auto result =
    savo_nav::CoveragePathValidator::Validate(
    request,
    policy);

  EXPECT_EQ(
    result.validation.code,
    ValidationCode::kInvalidCombination);
}

}  // namespace
