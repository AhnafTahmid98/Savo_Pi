#include <gtest/gtest.h>

#include <chrono>

#include "savo_realsense/camera_monitor_common.hpp"

TEST(CameraMonitorCommon, CameraRateQualityUsesFifteenFpsContract)
{
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(7.99, 8.0, 12.0, 14.0),
    "BELOW_MINIMUM");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(8.0, 8.0, 12.0, 14.0),
    "MINIMUM");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(12.0, 8.0, 12.0, 14.0),
    "GOOD");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(14.0, 8.0, 12.0, 14.0),
    "EXCELLENT");
}

TEST(CameraMonitorCommon, FreshOneHertzStreamIsNotHealthy)
{
  savo_realsense::StreamStatus status;
  status.seen = true;
  status.stale = false;
  status.rate_hz = 1.0;
  status.minimum_hz = 8.0;
  status.rate_quality = savo_realsense::classify_rate_quality(
    status.rate_hz, status.minimum_hz, 12.0, 14.0);

  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.rate_quality, "BELOW_MINIMUM");
}

TEST(CameraMonitorCommon, ObstacleCloudUsesEightHertzContract)
{
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(2.99, 3.0, 5.0, 7.0),
    "BELOW_MINIMUM");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(3.0, 3.0, 5.0, 7.0),
    "MINIMUM");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(5.0, 3.0, 5.0, 7.0),
    "GOOD");
  EXPECT_EQ(
    savo_realsense::classify_rate_quality(7.0, 3.0, 5.0, 7.0),
    "EXCELLENT");
}

TEST(CameraMonitorCommon, RateAndFreshnessUseMonotonicElapsedTime)
{
  savo_realsense::RateTracker tracker(4U);
  const auto start = savo_realsense::RateTracker::TimePoint{};
  tracker.tick(start);
  tracker.tick(start + std::chrono::milliseconds(100));
  tracker.tick(start + std::chrono::milliseconds(200));

  EXPECT_NEAR(tracker.rate_hz(), 10.0, 1.0e-9);
  EXPECT_NEAR(
    tracker.last_age_s(start + std::chrono::milliseconds(350)),
    0.15,
    1.0e-9);
}
