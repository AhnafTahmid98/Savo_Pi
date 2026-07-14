#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/audio/capture_pipeline.hpp"

namespace
{

[[nodiscard]] savo_speech::audio::AudioFrame make_stereo_frame(
  const std::uint64_t sequence,
  const std::int16_t left,
  const std::int16_t right)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;
  frame.captured_at =
    savo_speech::audio::AudioFrame::Clock::now();

  frame.format = {
    16000U,
    2U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    left,
    right,
    static_cast<std::int16_t>(left + 1),
    static_cast<std::int16_t>(right + 1)};

  return frame;
}

}  // namespace

TEST(CapturePipeline, RejectsInvalidConfiguration)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.pre_roll_samples = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::CapturePipeline(config, gate)),
    std::invalid_argument);
}

TEST(CapturePipeline, SelectsConfiguredChannelAndQueuesMono)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.selected_channel = 1U;
  config.pre_roll_samples = 8U;
  config.queue_capacity_frames = 4U;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  EXPECT_EQ(
    pipeline.process(make_stereo_frame(1U, 10, 20)),
    savo_speech::audio::CapturePipelineResult::Accepted);

  const auto output = pipeline.try_pop();

  ASSERT_TRUE(output.has_value());

  EXPECT_EQ(output->sequence, 1U);
  EXPECT_EQ(output->format.channels, 1U);

  ASSERT_EQ(output->interleaved_samples.size(), 2U);
  EXPECT_EQ(output->interleaved_samples[0], 20);
  EXPECT_EQ(output->interleaved_samples[1], 21);
}

TEST(CapturePipeline, BuildsChronologicalMonoPreRoll)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.selected_channel = 0U;
  config.pre_roll_samples = 3U;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  static_cast<void>(
    pipeline.process(make_stereo_frame(1U, 10, 20)));

  static_cast<void>(
    pipeline.process(make_stereo_frame(2U, 30, 40)));

  const auto pre_roll = pipeline.pre_roll_snapshot();

  ASSERT_TRUE(pre_roll.is_consistent());
  ASSERT_EQ(pre_roll.interleaved_samples.size(), 3U);

  EXPECT_EQ(pre_roll.interleaved_samples[0], 11);
  EXPECT_EQ(pre_roll.interleaved_samples[1], 30);
  EXPECT_EQ(pre_roll.interleaved_samples[2], 31);
}

TEST(CapturePipeline, GatingFlushesQueueAndPreRollOnce)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.pre_roll_samples = 8U;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  static_cast<void>(
    pipeline.process(make_stereo_frame(1U, 10, 20)));

  ASSERT_TRUE(pipeline.try_pop().has_value());

  static_cast<void>(
    pipeline.process(make_stereo_frame(2U, 30, 40)));

  gate.begin_playback();

  EXPECT_EQ(
    pipeline.process(make_stereo_frame(3U, 50, 60)),
    savo_speech::audio::CapturePipelineResult::
    DroppedWhileGated);

  EXPECT_FALSE(pipeline.try_pop().has_value());
  EXPECT_TRUE(
    pipeline.pre_roll_snapshot().interleaved_samples.empty());

  static_cast<void>(
    pipeline.process(make_stereo_frame(4U, 70, 80)));

  const auto statistics = pipeline.statistics();

  EXPECT_EQ(statistics.gated_frames, 2U);
  EXPECT_EQ(statistics.gate_flushes, 1U);
}

TEST(CapturePipeline, ResumesAfterPostPlaybackHold)
{
  savo_speech::audio::MicrophoneGateConfig gate_config;
  gate_config.post_playback_hold =
    std::chrono::milliseconds{100};

  savo_speech::audio::MicrophoneGate gate{gate_config};

  savo_speech::audio::CapturePipelineConfig config;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  const auto start =
    savo_speech::audio::MicrophoneGate::Clock::now();

  gate.begin_playback();
  gate.end_playback(start);

  EXPECT_EQ(
    pipeline.process(
      make_stereo_frame(1U, 10, 20),
      start + std::chrono::milliseconds{50}),
    savo_speech::audio::CapturePipelineResult::
    DroppedWhileGated);

  EXPECT_EQ(
    pipeline.process(
      make_stereo_frame(2U, 30, 40),
      start + std::chrono::milliseconds{101}),
    savo_speech::audio::CapturePipelineResult::Accepted);
}

TEST(CapturePipeline, ReportsDropOldestQueueOverflow)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.queue_capacity_frames = 1U;
  config.queue_overflow_policy =
    savo_speech::audio::QueueOverflowPolicy::DropOldest;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  EXPECT_EQ(
    pipeline.process(make_stereo_frame(1U, 10, 20)),
    savo_speech::audio::CapturePipelineResult::Accepted);

  EXPECT_EQ(
    pipeline.process(make_stereo_frame(2U, 30, 40)),
    savo_speech::audio::CapturePipelineResult::
    AcceptedAfterDroppingOldest);

  const auto output = pipeline.try_pop();

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->sequence, 2U);

  const auto statistics = pipeline.statistics();

  EXPECT_EQ(statistics.accepted_frames, 2U);
  EXPECT_EQ(statistics.queue_drop_events, 1U);
}

TEST(CapturePipeline, RejectsUnavailableChannel)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipelineConfig config;
  config.selected_channel = 2U;

  savo_speech::audio::CapturePipeline pipeline{
    config,
    gate};

  EXPECT_EQ(
    pipeline.process(make_stereo_frame(1U, 10, 20)),
    savo_speech::audio::CapturePipelineResult::
    RejectedChannelUnavailable);

  EXPECT_EQ(pipeline.statistics().channel_errors, 1U);
}
