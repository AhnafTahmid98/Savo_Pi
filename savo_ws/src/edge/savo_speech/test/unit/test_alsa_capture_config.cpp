#include "gtest/gtest.h"

#include "savo_speech/drivers/alsa_capture_stream.hpp"

TEST(AlsaCaptureConfig, DefaultConfigurationNeedsDeviceName)
{
  const savo_speech::drivers::AlsaCaptureConfig config;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaCaptureConfig, AcceptsValidConfiguration)
{
  savo_speech::drivers::AlsaCaptureConfig config;

  config.device_name = "default";

  EXPECT_TRUE(config.is_valid());
}

TEST(AlsaCaptureConfig, RejectsInvalidPeriod)
{
  savo_speech::drivers::AlsaCaptureConfig config;

  config.device_name = "default";
  config.period_frames = 0U;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaCaptureConfig, RejectsInvalidPeriodCount)
{
  savo_speech::drivers::AlsaCaptureConfig config;

  config.device_name = "default";
  config.periods = 1U;

  EXPECT_FALSE(config.is_valid());

  config.periods = 33U;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaCaptureConfig, RejectsInvalidRecoveryLimit)
{
  savo_speech::drivers::AlsaCaptureConfig config;

  config.device_name = "default";
  config.maximum_recovery_attempts = 0U;

  EXPECT_FALSE(config.is_valid());

  config.maximum_recovery_attempts = 101U;

  EXPECT_FALSE(config.is_valid());
}
