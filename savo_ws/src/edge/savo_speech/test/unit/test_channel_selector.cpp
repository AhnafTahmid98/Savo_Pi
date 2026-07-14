#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/channel_selector.hpp"

TEST(ChannelSelector, ExtractsFirstChannel)
{
  const std::vector<std::int16_t> interleaved{
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(11),
    static_cast<std::int16_t>(21),
    static_cast<std::int16_t>(12),
    static_cast<std::int16_t>(22)};

  const auto output =
    savo_speech::audio::ChannelSelector::extract(
    interleaved,
    2U,
    0U);

  const std::vector<std::int16_t> expected{
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(11),
    static_cast<std::int16_t>(12)};

  EXPECT_EQ(output, expected);
}

TEST(ChannelSelector, ExtractsSecondChannel)
{
  const std::vector<std::int16_t> interleaved{
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(11),
    static_cast<std::int16_t>(21)};

  const auto output =
    savo_speech::audio::ChannelSelector::extract(
    interleaved,
    2U,
    1U);

  const std::vector<std::int16_t> expected{
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(21)};

  EXPECT_EQ(output, expected);
}

TEST(ChannelSelector, RejectsInvalidChannelCount)
{
  const std::vector<std::int16_t> samples;

  EXPECT_THROW(
    savo_speech::audio::ChannelSelector::extract(
      samples,
      0U,
      0U),
    std::invalid_argument);
}

TEST(ChannelSelector, RejectsInvalidChannelIndex)
{
  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2)};

  EXPECT_THROW(
    savo_speech::audio::ChannelSelector::extract(
      samples,
      2U,
      2U),
    std::out_of_range);
}

TEST(ChannelSelector, RejectsIncompleteInterleavedData)
{
  const std::vector<std::int16_t> samples{
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2),
    static_cast<std::int16_t>(3)};

  EXPECT_THROW(
    savo_speech::audio::ChannelSelector::extract(
      samples,
      2U,
      0U),
    std::invalid_argument);
}
