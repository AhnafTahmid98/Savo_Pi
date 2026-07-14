#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_ring_buffer.hpp"

TEST(AudioRingBuffer, RejectsZeroCapacity)
{
  EXPECT_THROW(
    savo_speech::audio::AudioRingBuffer{0U},
    std::invalid_argument);
}

TEST(AudioRingBuffer, StoresPartialBufferChronologically)
{
  savo_speech::audio::AudioRingBuffer buffer{5U};

  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2),
    static_cast<std::int16_t>(3)};

  buffer.append(samples);

  EXPECT_EQ(buffer.snapshot(), samples);
  EXPECT_EQ(buffer.size(), 3U);
  EXPECT_FALSE(buffer.full());
}

TEST(AudioRingBuffer, KeepsNewestSamplesAfterWrap)
{
  savo_speech::audio::AudioRingBuffer buffer{4U};

  const std::vector<std::int16_t> first{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2),
    static_cast<std::int16_t>(3)};

  const std::vector<std::int16_t> second{
    static_cast<std::int16_t>(4),
    static_cast<std::int16_t>(5),
    static_cast<std::int16_t>(6)};

  buffer.append(first);
  buffer.append(second);

  const std::vector<std::int16_t> expected{
    static_cast<std::int16_t>(3),
    static_cast<std::int16_t>(4),
    static_cast<std::int16_t>(5),
    static_cast<std::int16_t>(6)};

  EXPECT_EQ(buffer.snapshot(), expected);
  EXPECT_TRUE(buffer.full());
}

TEST(AudioRingBuffer, KeepsTailOfOversizedAppend)
{
  savo_speech::audio::AudioRingBuffer buffer{3U};

  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2),
    static_cast<std::int16_t>(3),
    static_cast<std::int16_t>(4),
    static_cast<std::int16_t>(5)};

  buffer.append(samples);

  const std::vector<std::int16_t> expected{
    static_cast<std::int16_t>(3),
    static_cast<std::int16_t>(4),
    static_cast<std::int16_t>(5)};

  EXPECT_EQ(buffer.snapshot(), expected);
}

TEST(AudioRingBuffer, ClearRemovesSamples)
{
  savo_speech::audio::AudioRingBuffer buffer{4U};

  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2)};

  buffer.append(samples);
  buffer.clear();

  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_EQ(buffer.capacity(), 4U);
  EXPECT_TRUE(buffer.snapshot().empty());
}

TEST(AudioRingBuffer, ReportsLifetimeStatistics)
{
  savo_speech::audio::AudioRingBuffer buffer{4U};

  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2),
    static_cast<std::int16_t>(3),
    static_cast<std::int16_t>(4),
    static_cast<std::int16_t>(5),
    static_cast<std::int16_t>(6)};

  buffer.append(samples);

  const auto statistics = buffer.statistics();

  EXPECT_EQ(statistics.size_samples, 4U);
  EXPECT_EQ(statistics.capacity_samples, 4U);
  EXPECT_EQ(statistics.total_samples_written, 6U);
  EXPECT_EQ(statistics.overwritten_samples, 2U);

  buffer.reset_statistics();

  const auto reset_statistics = buffer.statistics();

  EXPECT_EQ(reset_statistics.total_samples_written, 0U);
  EXPECT_EQ(reset_statistics.overwritten_samples, 0U);
  EXPECT_EQ(reset_statistics.size_samples, 4U);
}
