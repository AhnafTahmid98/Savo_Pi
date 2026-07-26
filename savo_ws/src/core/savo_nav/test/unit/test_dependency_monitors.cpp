// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "savo_nav/localization_monitor.hpp"
#include "savo_nav/nav2_health_monitor.hpp"
#include "savo_nav/safety_state_monitor.hpp"

namespace
{

TEST(LocalizationMonitorTest, TracksFilteredOdometry)
{
  savo_nav::LocalizationMonitor monitor(1.0);

  EXPECT_FALSE(
    monitor.GetSnapshot(5.0).odometry_fresh);

  monitor.MarkOdometryReceived(5.0);

  EXPECT_TRUE(
    monitor.GetSnapshot(5.5).odometry_fresh);

  EXPECT_FALSE(
    monitor.GetSnapshot(6.1).odometry_fresh);
}

TEST(SafetyStateMonitorTest, RequiresBothSafetyTopics)
{
  savo_nav::SafetyStateMonitor monitor(1.0);

  monitor.UpdateStop(false, 5.0);

  EXPECT_FALSE(
    monitor.GetSnapshot(5.1).state_fresh);

  monitor.UpdateSlowdown(1.0, 5.0);

  const auto snapshot = monitor.GetSnapshot(5.1);

  EXPECT_TRUE(snapshot.state_fresh);
  EXPECT_FALSE(snapshot.stop_active);
  EXPECT_TRUE(snapshot.slowdown_valid);
  EXPECT_DOUBLE_EQ(snapshot.slowdown_factor, 1.0);
}

TEST(SafetyStateMonitorTest, DetectsInvalidSlowdown)
{
  savo_nav::SafetyStateMonitor monitor(1.0);

  monitor.UpdateStop(false, 5.0);
  monitor.UpdateSlowdown(1.5, 5.0);

  const auto snapshot = monitor.GetSnapshot(5.1);

  EXPECT_TRUE(snapshot.state_fresh);
  EXPECT_FALSE(snapshot.slowdown_valid);
}

TEST(SafetyStateMonitorTest, PreservesSafetyStop)
{
  savo_nav::SafetyStateMonitor monitor(1.0);

  monitor.UpdateStop(true, 5.0);
  monitor.UpdateSlowdown(0.0, 5.0);

  const auto snapshot = monitor.GetSnapshot(5.1);

  EXPECT_TRUE(snapshot.stop_active);
  EXPECT_DOUBLE_EQ(snapshot.slowdown_factor, 0.0);
}

TEST(Nav2HealthMonitorTest, TracksServerAndCostmaps)
{
  savo_nav::Nav2HealthMonitor monitor(2.0);

  monitor.UpdateActionServerAvailable(true);
  monitor.MarkGlobalCostmapReceived(5.0);
  monitor.MarkLocalCostmapReceived(5.0);

  const auto snapshot = monitor.GetSnapshot(6.0);

  EXPECT_TRUE(snapshot.action_server_available);
  EXPECT_TRUE(snapshot.global_costmap_fresh);
  EXPECT_TRUE(snapshot.local_costmap_fresh);
}

TEST(Nav2HealthMonitorTest, CostmapsExpire)
{
  savo_nav::Nav2HealthMonitor monitor(1.0);

  monitor.MarkGlobalCostmapReceived(5.0);
  monitor.MarkLocalCostmapReceived(5.0);

  const auto snapshot = monitor.GetSnapshot(6.1);

  EXPECT_FALSE(snapshot.global_costmap_fresh);
  EXPECT_FALSE(snapshot.local_costmap_fresh);
}

}  // namespace
