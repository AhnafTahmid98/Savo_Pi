// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <limits>

#include "gtest/gtest.h"

#include "savo_nav/coverage_progress_tracker.hpp"

namespace
{

savo_nav::CoveragePathPoint Point(
  const double x,
  const double y)
{
  savo_nav::CoveragePathPoint point;
  point.x = x;
  point.y = y;
  return point;
}

TEST(CoverageProgressTrackerTest, RejectsEmptyPath)
{
  savo_nav::CoverageProgressTracker tracker;
  EXPECT_FALSE(tracker.Configure({}));
}

TEST(CoverageProgressTrackerTest, StartsConfiguredPath)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(3.0, 4.0)}));

  ASSERT_TRUE(tracker.Start(10.0));

  EXPECT_EQ(tracker.Snapshot().total_waypoints, 2U);
  EXPECT_DOUBLE_EQ(
    tracker.Snapshot().remaining_distance_m,
    5.0);
}

TEST(CoverageProgressTrackerTest, CalculatesProgress)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
    {
      Point(0.0, 0.0),
      Point(3.0, 4.0),
      Point(6.0, 8.0)
    }));

  ASSERT_TRUE(tracker.Start(0.0));
  ASSERT_TRUE(tracker.UpdateRemainingDistance(5.0, 5.0));

  EXPECT_DOUBLE_EQ(
    tracker.Snapshot().completion_ratio,
    0.5);

  EXPECT_EQ(
    tracker.Snapshot().completed_waypoints,
    2U);
}

TEST(CoverageProgressTrackerTest, ProgressDoesNotRegress)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(10.0, 0.0)}));

  ASSERT_TRUE(tracker.Start(0.0));
  ASSERT_TRUE(tracker.UpdateRemainingDistance(4.0, 2.0));
  ASSERT_TRUE(tracker.UpdateRemainingDistance(7.0, 3.0));

  EXPECT_DOUBLE_EQ(
    tracker.Snapshot().remaining_distance_m,
    4.0);
}

TEST(CoverageProgressTrackerTest, RejectsClockReversal)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(1.0, 0.0)}));

  ASSERT_TRUE(tracker.Start(5.0));

  EXPECT_FALSE(
    tracker.UpdateRemainingDistance(0.5, 4.0));
}

TEST(CoverageProgressTrackerTest, RejectsInvalidDistance)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(1.0, 0.0)}));

  ASSERT_TRUE(tracker.Start(0.0));

  EXPECT_FALSE(
    tracker.UpdateRemainingDistance(
      std::numeric_limits<double>::quiet_NaN(),
      1.0));

  EXPECT_FALSE(
    tracker.UpdateRemainingDistance(-1.0, 1.0));
}

TEST(CoverageProgressTrackerTest, SuccessCompletesPath)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(2.0, 0.0)}));

  ASSERT_TRUE(tracker.Start(0.0));
  ASSERT_TRUE(tracker.MarkSucceeded(4.0));

  EXPECT_EQ(
    tracker.Snapshot().completed_waypoints,
    2U);

  EXPECT_DOUBLE_EQ(
    tracker.Snapshot().completion_ratio,
    1.0);
}

TEST(CoverageProgressTrackerTest, HandlesZeroDistancePath)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(1.0, 1.0), Point(1.0, 1.0)}));

  ASSERT_TRUE(tracker.Start(0.0));
  ASSERT_TRUE(tracker.UpdateRemainingDistance(0.0, 1.0));

  EXPECT_DOUBLE_EQ(
    tracker.Snapshot().completion_ratio,
    1.0);
}

TEST(CoverageProgressTrackerTest, ResetReturnsToIdle)
{
  savo_nav::CoverageProgressTracker tracker;

  ASSERT_TRUE(tracker.Configure(
      {Point(0.0, 0.0), Point(1.0, 0.0)}));

  tracker.Reset();

  EXPECT_FALSE(tracker.IsConfigured());
  EXPECT_FALSE(tracker.IsStarted());
}

}  // namespace
