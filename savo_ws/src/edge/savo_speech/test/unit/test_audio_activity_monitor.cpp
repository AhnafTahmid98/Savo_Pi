#include <cstdint>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_activity_monitor.hpp"

namespace
{

[[nodiscard]] savo_speech::audio::AudioFrame
make_mono_frame()
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = 1U;
  frame.captured_at =
    savo_speech::audio::AudioFrame::Clock::now();

  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    0,
    1000,
    -1000,
    32767};

  return frame;
}

}  // namespace

TEST(AudioActivityMonitor, RecordsAudioLevels)
{
  savo_speech::audio::AudioActivityMonitor monitor;

  monitor.process(make_mono_frame());

  const auto snapshot = monitor.snapshot();

  EXPECT_EQ(snapshot.frames_processed, 1U);
  EXPECT_EQ(snapshot.samples_processed, 4U);

  EXPECT_GT(snapshot.last_rms, 0.0);
  EXPECT_GT(snapshot.last_peak, 0.0);

  EXPECT_EQ(
    snapshot.maximum_peak,
    snapshot.last_peak);
}

TEST(AudioActivityMonitor, RejectsMultichannelFrame)
{
  savo_speech::audio::AudioActivityMonitor monitor;

  auto frame = make_mono_frame();

  frame.format.channels = 2U;
  frame.interleaved_samples = {
    1,
    2,
    3,
    4};

  EXPECT_THROW(
    monitor.process(frame),
    std::invalid_argument);
}

TEST(AudioActivityMonitor, ResetClearsStatistics)
{
  savo_speech::audio::AudioActivityMonitor monitor;

  monitor.process(make_mono_frame());
  monitor.reset();

  const auto snapshot = monitor.snapshot();

  EXPECT_EQ(snapshot.frames_processed, 0U);
  EXPECT_EQ(snapshot.samples_processed, 0U);
  EXPECT_DOUBLE_EQ(snapshot.last_rms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.last_peak, 0.0);
}
