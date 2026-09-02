#include <gtest/gtest.h>

#include <string>

#include "savo_realsense/camera_health_state.hpp"

namespace
{

using savo_realsense::HealthSignal;

HealthSignal signal(
  const bool required,
  const bool seen,
  const bool healthy,
  const double last_update_s)
{
  return HealthSignal{required, seen, healthy, last_update_s};
}

TEST(CameraHealthState, HealthyRequiredSignalsPreservePublicContract)
{
  const auto evaluation = savo_realsense::evaluate_camera_health(
    signal(true, true, true, 9.8),
    signal(true, true, true, 9.8),
    signal(true, true, true, 9.8),
    10.0,
    0.75);

  EXPECT_TRUE(evaluation.ok);
  EXPECT_TRUE(evaluation.depth_signal_ok);
  EXPECT_TRUE(evaluation.vo_health_ok);
  EXPECT_TRUE(evaluation.obstacle_cloud_ok);
  EXPECT_TRUE(evaluation.color_ok);
  EXPECT_TRUE(evaluation.color_info_ok);
  EXPECT_TRUE(evaluation.depth_ok);
  EXPECT_TRUE(evaluation.depth_info_ok);
  EXPECT_TRUE(evaluation.aligned_depth_ok);
  EXPECT_TRUE(evaluation.pointcloud_ok);
  EXPECT_EQ(savo_realsense::camera_health_message(evaluation), "RealSense streams OK");
}

TEST(CameraHealthState, MissingRequiredDepthSignalFailsClosed)
{
  const auto evaluation = savo_realsense::evaluate_camera_health(
    signal(true, false, false, 0.0),
    signal(false, false, false, 0.0),
    signal(false, false, false, 0.0),
    10.0,
    0.75);

  EXPECT_FALSE(evaluation.ok);
  EXPECT_FALSE(evaluation.depth_signal_ok);
  ASSERT_EQ(evaluation.failures.size(), 1U);
  EXPECT_EQ(evaluation.failures.front(), "depth_front:not_seen");
}

TEST(CameraHealthState, StaleRequiredVoSignalFailsClosed)
{
  const auto evaluation = savo_realsense::evaluate_camera_health(
    signal(false, false, false, 0.0),
    signal(true, true, true, 9.0),
    signal(false, false, false, 0.0),
    10.0,
    0.5);

  EXPECT_FALSE(evaluation.ok);
  EXPECT_FALSE(evaluation.vo_health_ok);
  ASSERT_EQ(evaluation.failures.size(), 1U);
  EXPECT_EQ(evaluation.failures.front(), "vo:stale");
}

TEST(CameraHealthState, UnhealthyRequiredObstacleCloudFailsClosed)
{
  const auto evaluation = savo_realsense::evaluate_camera_health(
    signal(false, false, false, 0.0),
    signal(false, false, false, 0.0),
    signal(true, true, false, 9.8),
    10.0,
    0.75);

  EXPECT_FALSE(evaluation.ok);
  EXPECT_FALSE(evaluation.obstacle_cloud_ok);
  ASSERT_EQ(evaluation.failures.size(), 1U);
  EXPECT_EQ(evaluation.failures.front(), "obstacle_cloud:unhealthy");
}

TEST(CameraHealthState, DisabledOptionalSignalsDoNotCreateFalseFailures)
{
  const auto evaluation = savo_realsense::evaluate_camera_health(
    signal(false, false, false, 0.0),
    signal(false, false, false, 0.0),
    signal(false, false, false, 0.0),
    10.0,
    0.75);

  EXPECT_TRUE(evaluation.ok);
  EXPECT_TRUE(evaluation.depth_signal_ok);
  EXPECT_TRUE(evaluation.vo_health_ok);
  EXPECT_TRUE(evaluation.obstacle_cloud_ok);
  EXPECT_TRUE(evaluation.failures.empty());
}

TEST(CameraHealthState, VoHealthTextOnlyAcceptsOkStates)
{
  EXPECT_TRUE(savo_realsense::health_text_is_ok("ok"));
  EXPECT_TRUE(savo_realsense::health_text_is_ok("OK: tracking accepted=true"));
  EXPECT_FALSE(savo_realsense::health_text_is_ok("waiting: no odometry"));
  EXPECT_FALSE(savo_realsense::health_text_is_ok("degraded: tracking rejected"));
  EXPECT_FALSE(savo_realsense::health_text_is_ok("error"));
  EXPECT_FALSE(savo_realsense::health_text_is_ok(""));
}

}  // namespace
