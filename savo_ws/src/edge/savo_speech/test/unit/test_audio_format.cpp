#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/audio_frame.hpp"

TEST(AudioFormat, DefaultFormatIsInvalid)
{
  const savo_speech::audio::AudioFormat format;

  EXPECT_FALSE(format.is_valid());
  EXPECT_EQ(format.bytes_per_frame(), 0U);
}

TEST(AudioFormat, ValidMonoSpeechFormat)
{
  const savo_speech::audio::AudioFormat format{
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  EXPECT_TRUE(format.is_valid());
  EXPECT_EQ(format.bytes_per_frame(), 2U);
}

TEST(AudioFormat, ValidSixChannelRespeakerFormat)
{
  const savo_speech::audio::AudioFormat format{
    16000U,
    6U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  EXPECT_TRUE(format.is_valid());
  EXPECT_EQ(format.bytes_per_frame(), 12U);
}

TEST(AudioFrame, ReportsFrameCountAndConsistency)
{
  savo_speech::audio::AudioFrame frame;

  frame.format = {
    16000U,
    2U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  frame.interleaved_samples = {
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(11),
    static_cast<std::int16_t>(21)};

  EXPECT_TRUE(frame.is_consistent());
  EXPECT_EQ(frame.frame_count(), 2U);
}

TEST(AudioFrame, RejectsIncompleteInterleavedFrame)
{
  savo_speech::audio::AudioFrame frame;

  frame.format = {
    16000U,
    2U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  frame.interleaved_samples = {
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(11)};

  EXPECT_FALSE(frame.is_consistent());
}
