#include "savo_mapping/scan360_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace
{

namespace scan360 =
  savo_mapping::scan360;

}  // namespace

TEST(
  Scan360PlannerContract,
  DirectionStringsAreStable)
{
  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::RotationDirection::
      CounterClockwise)},
    "counter_clockwise");

  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::RotationDirection::
      Clockwise)},
    "clockwise");
}

TEST(
  Scan360PlannerContract,
  FullCounterClockwisePlanUsesExactFinalTarget)
{
  scan360::Scan360PlanOptions options;

  options.step_angle_rad =
    scan360::kPi / 2.0;

  const auto result =
    scan360::make_plan(
    0.25,
    options);

  ASSERT_TRUE(result.accepted);
  ASSERT_EQ(result.plan.targets.size(), 4U);

  EXPECT_NEAR(
    result.plan.targets[0].
    relative_yaw_rad,
    scan360::kPi / 2.0,
    1.0e-9);

  EXPECT_NEAR(
    result.plan.targets.back().
    relative_yaw_rad,
    scan360::kTwoPi,
    1.0e-9);

  EXPECT_TRUE(
    result.plan.targets.back().
    final_target);

  EXPECT_NEAR(
    result.plan.targets.back().
    normalized_yaw_rad,
    scan360::normalize_yaw(0.25),
    1.0e-9);
}

TEST(
  Scan360PlannerContract,
  ClockwisePlanUsesNegativeRelativeYaw)
{
  scan360::Scan360PlanOptions options;

  options.step_angle_rad =
    scan360::kPi;

  options.direction =
    scan360::RotationDirection::
    Clockwise;

  const auto result =
    scan360::make_plan(
    -0.4,
    options);

  ASSERT_TRUE(result.accepted);
  ASSERT_EQ(result.plan.targets.size(), 2U);

  EXPECT_NEAR(
    result.plan.targets[0].
    relative_yaw_rad,
    -scan360::kPi,
    1.0e-9);

  EXPECT_NEAR(
    result.plan.targets[1].
    relative_yaw_rad,
    -scan360::kTwoPi,
    1.0e-9);
}

TEST(
  Scan360PlannerContract,
  NonDivisibleStepStillEndsAtRequestedSweep)
{
  scan360::Scan360PlanOptions options;

  options.sweep_angle_rad = 1.0;
  options.step_angle_rad = 0.3;

  const auto result =
    scan360::make_plan(
    0.0,
    options);

  ASSERT_TRUE(result.accepted);
  ASSERT_EQ(result.plan.targets.size(), 4U);

  EXPECT_NEAR(
    result.plan.targets.back().
    relative_yaw_rad,
    1.0,
    1.0e-9);

  EXPECT_TRUE(
    result.plan.targets.back().
    final_target);
}

TEST(
  Scan360PlannerContract,
  InvalidInputsAreRejected)
{
  scan360::Scan360PlanOptions options;

  options.sweep_angle_rad = 0.0;

  auto result =
    scan360::make_plan(
    0.0,
    options);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
    result.reason,
    "sweep_angle_out_of_range");

  options.sweep_angle_rad =
    scan360::kTwoPi;

  options.step_angle_rad = 0.0;

  result =
    scan360::make_plan(
    0.0,
    options);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
    result.reason,
    "step_angle_out_of_range");

  result =
    scan360::make_plan(
    std::nan(""),
    scan360::Scan360PlanOptions{});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(
    result.reason,
    "start_yaw_not_finite");
}

TEST(
  Scan360PlannerContract,
  NormalizeYawUsesRosYawRange)
{
  EXPECT_NEAR(
    scan360::normalize_yaw(
      scan360::kTwoPi + 0.2),
    0.2,
    1.0e-9);

  EXPECT_NEAR(
    scan360::normalize_yaw(
      -scan360::kTwoPi - 0.2),
    -0.2,
    1.0e-9);

  EXPECT_NEAR(
    scan360::normalize_yaw(
      scan360::kPi),
    -scan360::kPi,
    1.0e-9);
}
