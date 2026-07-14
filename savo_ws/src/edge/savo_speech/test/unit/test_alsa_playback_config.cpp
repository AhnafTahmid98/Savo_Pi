#include "gtest/gtest.h"

#include "savo_speech/drivers/alsa_playback_stream.hpp"

TEST(AlsaPlaybackConfig, DefaultConfigurationNeedsDeviceName)
{
  const savo_speech::drivers::AlsaPlaybackConfig config;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaPlaybackConfig, AcceptsValidConfiguration)
{
  savo_speech::drivers::AlsaPlaybackConfig config;

  config.device_name = "default";

  EXPECT_TRUE(config.is_valid());
}

TEST(AlsaPlaybackConfig, RejectsInvalidAudioFormat)
{
  savo_speech::drivers::AlsaPlaybackConfig config;

  config.device_name = "default";
  config.requested_format.sample_rate_hz = 0U;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaPlaybackConfig, RejectsInvalidPeriod)
{
  savo_speech::drivers::AlsaPlaybackConfig config;

  config.device_name = "default";
  config.period_frames = 0U;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaPlaybackConfig, RejectsInvalidPeriodCount)
{
  savo_speech::drivers::AlsaPlaybackConfig config;

  config.device_name = "default";
  config.periods = 1U;

  EXPECT_FALSE(config.is_valid());

  config.periods = 33U;

  EXPECT_FALSE(config.is_valid());
}

TEST(AlsaPlaybackConfig, RejectsInvalidRecoveryLimit)
{
  savo_speech::drivers::AlsaPlaybackConfig config;

  config.device_name = "default";
  config.maximum_recovery_attempts = 0U;

  EXPECT_FALSE(config.is_valid());

  config.maximum_recovery_attempts = 101U;

  EXPECT_FALSE(config.is_valid());
}
