#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_controller.hpp"
#include "savo_speech/audio/playback_sink.hpp"

namespace
{

class FakePlaybackSink final :
  public savo_speech::audio::PlaybackSink
{
public:
  explicit FakePlaybackSink(
    const savo_speech::audio::AudioFormat format)
  : format_{format}
  {
  }

  void open() override
  {
    open_ = true;
  }

  void close() noexcept override
  {
    open_ = false;
  }

  [[nodiscard]] bool is_open() const noexcept override
  {
    return open_;
  }

  [[nodiscard]] savo_speech::audio::AudioFormat
  format() const noexcept override
  {
    return format_;
  }

  void write(
    const std::span<const std::int16_t> samples) override
  {
    ++write_calls;

    written_samples.insert(
      written_samples.end(),
      samples.begin(),
      samples.end());

    if (on_write) {
      on_write();
    }
  }

  void drain() override
  {
    ++drain_calls;
  }

  void stop() noexcept override
  {
    ++stop_calls;
  }

  bool open_{true};

  savo_speech::audio::AudioFormat format_;

  std::vector<std::int16_t> written_samples;

  std::size_t write_calls{0U};
  std::size_t drain_calls{0U};
  std::size_t stop_calls{0U};

  std::function<void()> on_write;
};

[[nodiscard]] savo_speech::audio::AudioBuffer
make_audio(
  const std::size_t frame_count,
  const std::uint16_t channels = 1U)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    channels,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  const std::size_t sample_count =
    frame_count * static_cast<std::size_t>(channels);

  audio.interleaved_samples.reserve(sample_count);

  for (std::size_t index = 0U;
    index < sample_count;
    ++index)
  {
    audio.interleaved_samples.push_back(
      static_cast<std::int16_t>(index));
  }

  return audio;
}

}  // namespace

TEST(PlaybackController, RejectsZeroChunkSize)
{
  savo_speech::audio::MicrophoneGate gate;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::PlaybackController(gate, 0U)),
    std::invalid_argument);
}

TEST(PlaybackController, PlaysAudioInBoundedChunks)
{
  savo_speech::audio::MicrophoneGateConfig gate_config;
  gate_config.post_playback_hold =
    std::chrono::seconds{5};

  savo_speech::audio::MicrophoneGate gate{gate_config};

  auto audio = make_audio(10U);
  FakePlaybackSink sink{audio.format};

  savo_speech::audio::PlaybackController controller{
    gate,
    4U};

  const auto result = controller.play(audio, sink);

  EXPECT_EQ(
    result.outcome,
    savo_speech::audio::PlaybackOutcome::Completed);

  EXPECT_EQ(result.frames_submitted, 10U);
  EXPECT_EQ(result.chunks_submitted, 3U);

  EXPECT_EQ(sink.write_calls, 3U);
  EXPECT_EQ(sink.drain_calls, 1U);
  EXPECT_EQ(sink.stop_calls, 0U);

  EXPECT_EQ(
    sink.written_samples,
    audio.interleaved_samples);

  const auto gate_state = gate.state();

  EXPECT_TRUE(gate_state.gated);
  EXPECT_EQ(
    gate_state.reason,
    savo_speech::audio::MicrophoneGateReason::
    PostPlaybackHold);
}

TEST(PlaybackController, CancelsBetweenChunks)
{
  savo_speech::audio::MicrophoneGate gate;

  auto audio = make_audio(12U);
  FakePlaybackSink sink{audio.format};

  savo_speech::audio::PlaybackController controller{
    gate,
    4U};

  sink.on_write = [&controller, &sink]() {
      if (sink.write_calls == 1U) {
        controller.request_cancel();
      }
    };

  const auto result = controller.play(audio, sink);

  EXPECT_EQ(
    result.outcome,
    savo_speech::audio::PlaybackOutcome::Cancelled);

  EXPECT_EQ(result.frames_submitted, 4U);
  EXPECT_EQ(result.chunks_submitted, 1U);

  EXPECT_EQ(sink.write_calls, 1U);
  EXPECT_EQ(sink.drain_calls, 0U);
  EXPECT_EQ(sink.stop_calls, 1U);
}

TEST(PlaybackController, RejectsClosedSink)
{
  savo_speech::audio::MicrophoneGate gate;

  auto audio = make_audio(4U);
  FakePlaybackSink sink{audio.format};

  sink.close();

  savo_speech::audio::PlaybackController controller{
    gate,
    4U};

  EXPECT_THROW(
    static_cast<void>(controller.play(audio, sink)),
    std::logic_error);
}

TEST(PlaybackController, RejectsFormatMismatch)
{
  savo_speech::audio::MicrophoneGate gate;

  auto audio = make_audio(4U);

  auto different_format = audio.format;
  different_format.sample_rate_hz = 48000U;

  FakePlaybackSink sink{different_format};

  savo_speech::audio::PlaybackController controller{
    gate,
    4U};

  EXPECT_THROW(
    static_cast<void>(controller.play(audio, sink)),
    std::invalid_argument);
}
