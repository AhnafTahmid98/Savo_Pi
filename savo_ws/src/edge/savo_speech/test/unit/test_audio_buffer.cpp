#include <chrono>
#include <cstdint>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"

TEST(AudioBuffer, ReportsFramesAndDuration)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  audio.interleaved_samples.resize(
    16000U,
    static_cast<std::int16_t>(0));

  EXPECT_TRUE(audio.is_consistent());
  EXPECT_EQ(audio.frame_count(), 16000U);
  EXPECT_EQ(audio.duration(), std::chrono::seconds{1});
}

TEST(AudioBuffer, MultichannelDurationUsesFrameCount)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    48000U,
    2U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  audio.interleaved_samples.resize(
    48000U,
    static_cast<std::int16_t>(0));

  EXPECT_TRUE(audio.is_consistent());
  EXPECT_EQ(audio.frame_count(), 24000U);
  EXPECT_EQ(audio.duration(), std::chrono::milliseconds{500});
}

TEST(AudioBuffer, EmptyBufferIsNotConsistent)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  EXPECT_TRUE(audio.empty());
  EXPECT_FALSE(audio.is_consistent());
  EXPECT_EQ(audio.duration(), std::chrono::nanoseconds{0});
}
