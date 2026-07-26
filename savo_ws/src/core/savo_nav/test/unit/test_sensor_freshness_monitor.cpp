// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_nav/sensor_freshness_monitor.hpp"

namespace
{

TEST(SensorFreshnessMonitorTest, RejectsInvalidTimeout)
{
  EXPECT_THROW(
    savo_nav::SensorFreshnessMonitor(0.0),
    std::invalid_argument);

  EXPECT_THROW(
    savo_nav::SensorFreshnessMonitor(-1.0),
    std::invalid_argument);
}

TEST(SensorFreshnessMonitorTest, StartsWithoutData)
{
  const savo_nav::SensorFreshnessMonitor monitor(1.0);

  EXPECT_FALSE(monitor.HasReceived());
  EXPECT_FALSE(monitor.IsFresh(1.0));
}

TEST(SensorFreshnessMonitorTest, MarksAndExpiresData)
{
  savo_nav::SensorFreshnessMonitor monitor(1.0);

  monitor.MarkReceived(10.0);

  EXPECT_TRUE(monitor.HasReceived());
  EXPECT_TRUE(monitor.IsFresh(10.5));
  EXPECT_TRUE(monitor.IsFresh(11.0));
  EXPECT_FALSE(monitor.IsFresh(11.01));
}

TEST(SensorFreshnessMonitorTest, RejectsClockReversal)
{
  savo_nav::SensorFreshnessMonitor monitor(1.0);

  monitor.MarkReceived(10.0);

  EXPECT_FALSE(monitor.IsFresh(9.0));
}

TEST(SensorFreshnessMonitorTest, ResetRemovesReceipt)
{
  savo_nav::SensorFreshnessMonitor monitor(1.0);

  monitor.MarkReceived(10.0);
  monitor.Reset();

  EXPECT_FALSE(monitor.HasReceived());
  EXPECT_FALSE(monitor.IsFresh(10.0));
}

}  // namespace
