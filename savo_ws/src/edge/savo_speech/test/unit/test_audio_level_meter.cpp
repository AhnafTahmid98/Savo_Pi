#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_level_meter.hpp"

TEST(AudioLevelMeter, EmptyInputProducesZeroLevel)
{
  const std::vector<std::int16_t> samples;

  const auto result =
    savo_speech::audio::AudioLevelMeter::measure(samples);

  EXPECT_DOUBLE_EQ(result.rms, 0.0);
  EXPECT_DOUBLE_EQ(result.peak, 0.0);
  EXPECT_FALSE(result.clipping);
  EXPECT_EQ(result.sample_count, 0U);
}

TEST(AudioLevelMeter, MeasuresNormalizedPeakAndRms)
{
  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(0),
    static_cast<std::int16_t>(16384),
    static_cast<std::int16_t>(-16384)};

  const auto result =
    savo_speech::audio::AudioLevelMeter::measure(samples);

  EXPECT_NEAR(result.peak, 0.5, 1.0e-9);
  EXPECT_NEAR(result.rms, 0.4082482904, 1.0e-9);
  EXPECT_FALSE(result.clipping);
  EXPECT_EQ(result.sample_count, 3U);
}

TEST(AudioLevelMeter, DetectsNegativeFullScaleClipping)
{
  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(-32768)};

  const auto result =
    savo_speech::audio::AudioLevelMeter::measure(samples);

  EXPECT_DOUBLE_EQ(result.peak, 1.0);
  EXPECT_DOUBLE_EQ(result.rms, 1.0);
  EXPECT_TRUE(result.clipping);
}

TEST(AudioLevelMeter, RejectsInvalidClippingThreshold)
{
  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(0)};

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::AudioLevelMeter::measure(samples, 0U)),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::AudioLevelMeter::measure(samples, 32769U)),
    std::invalid_argument);
}
