#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/vad/vad_processor.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeVadBackend final :
  public savo_speech::vad::VadBackend
{
public:
  [[nodiscard]]
  savo_speech::vad::VadBackendResult analyze(
    const savo_speech::audio::AudioFrame &) override
  {
    if (throw_on_analyze_) {
      throw std::runtime_error{
              "scripted VAD backend failure"};
    }

    if (results_.empty()) {
      return {};
    }

    const auto result = results_.front();
    results_.pop_front();

    return result;
  }

  void reset() noexcept override
  {
    results_.clear();
    throw_on_analyze_ = false;

    ++reset_calls_;
  }

  void push(const double speech_score)
  {
    results_.push_back(
      savo_speech::vad::VadBackendResult{
        speech_score});
  }

  void set_throw_on_analyze(
    const bool enabled) noexcept
  {
    throw_on_analyze_ = enabled;
  }

  [[nodiscard]] std::uint64_t reset_calls()
  const noexcept
  {
    return reset_calls_;
  }

private:
  std::deque<
    savo_speech::vad::VadBackendResult>
    results_{};

  bool throw_on_analyze_{false};

  std::uint64_t reset_calls_{0U};
};

[[nodiscard]] savo_speech::audio::AudioFrame
make_frame(
  const std::uint64_t sequence,
  const std::chrono::milliseconds offset = 0ms)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;

  frame.captured_at =
    savo_speech::audio::AudioFrame::Clock::time_point{} +
    offset;

  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    100,
    -100,
    200,
    -200};

  return frame;
}

[[nodiscard]]
savo_speech::vad::VadProcessorConfig
make_config()
{
  savo_speech::vad::VadProcessorConfig config;

  config.speech_start_threshold = 0.65;
  config.speech_end_threshold = 0.35;

  config.required_start_frames = 2U;
  config.required_end_frames = 2U;

  config.event_queue_capacity = 4U;

  return config;
}

}  // namespace

TEST(VadProcessor, RejectsInvalidConfiguration)
{
  FakeVadBackend backend;

  auto config = make_config();
  config.speech_end_threshold =
    config.speech_start_threshold;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::VadProcessor{
        backend,
        config}),
    std::invalid_argument);

  config = make_config();
  config.required_start_frames = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::VadProcessor{
        backend,
        config}),
    std::invalid_argument);

  config = make_config();
  config.event_queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::VadProcessor{
        backend,
        config}),
    std::invalid_argument);
}

TEST(VadProcessor, EmitsStartAfterConsecutiveSpeechFrames)
{
  FakeVadBackend backend;

  backend.push(0.80);
  backend.push(0.90);

  savo_speech::vad::VadProcessor processor{
    backend,
    make_config()};

  processor.process(make_frame(1U, 1000ms));

  EXPECT_FALSE(
    processor.try_pop_event().has_value());

  processor.process(make_frame(2U, 1020ms));

  const auto event =
    processor.try_pop_event();

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    savo_speech::vad::VadEventType::
    SpeechStarted);

  EXPECT_EQ(event->event_id, 1U);
  EXPECT_EQ(event->segment_id, 1U);
  EXPECT_EQ(event->frame_sequence, 2U);
  EXPECT_DOUBLE_EQ(event->speech_score, 0.90);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.state,
    savo_speech::vad::VadState::Speech);

  EXPECT_EQ(snapshot.active_segment_id, 1U);
}

TEST(VadProcessor, EmitsEndAfterConsecutiveSilenceFrames)
{
  FakeVadBackend backend;

  backend.push(0.80);
  backend.push(0.90);
  backend.push(0.20);
  backend.push(0.10);

  savo_speech::vad::VadProcessor processor{
    backend,
    make_config()};

  processor.process(make_frame(1U, 1000ms));
  processor.process(make_frame(2U, 1020ms));

  ASSERT_TRUE(
    processor.try_pop_event().has_value());

  processor.process(make_frame(3U, 1040ms));

  EXPECT_FALSE(
    processor.try_pop_event().has_value());

  processor.process(make_frame(4U, 1060ms));

  const auto event =
    processor.try_pop_event();

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    savo_speech::vad::VadEventType::
    SpeechEnded);

  EXPECT_EQ(event->event_id, 2U);
  EXPECT_EQ(event->segment_id, 1U);
  EXPECT_EQ(event->frame_sequence, 4U);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.state,
    savo_speech::vad::VadState::Silence);

  EXPECT_EQ(snapshot.active_segment_id, 0U);
}

TEST(VadProcessor, HysteresisPreservesSpeechState)
{
  FakeVadBackend backend;

  backend.push(0.80);
  backend.push(0.20);
  backend.push(0.50);

  auto config = make_config();

  config.required_start_frames = 1U;
  config.required_end_frames = 2U;

  savo_speech::vad::VadProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U));

  ASSERT_TRUE(
    processor.try_pop_event().has_value());

  processor.process(make_frame(2U));
  processor.process(make_frame(3U));

  EXPECT_FALSE(
    processor.try_pop_event().has_value());

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.state,
    savo_speech::vad::VadState::Speech);

  EXPECT_EQ(
    snapshot.statistics.end_debounce_resets,
    1U);
}

TEST(VadProcessor, DoesNotEmitDuplicateTransitions)
{
  FakeVadBackend backend;

  backend.push(0.90);
  backend.push(0.95);
  backend.push(0.10);
  backend.push(0.05);

  auto config = make_config();

  config.required_start_frames = 1U;
  config.required_end_frames = 1U;

  savo_speech::vad::VadProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U));
  processor.process(make_frame(2U));
  processor.process(make_frame(3U));
  processor.process(make_frame(4U));

  const auto first =
    processor.try_pop_event();

  const auto second =
    processor.try_pop_event();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(
    first->type,
    savo_speech::vad::VadEventType::
    SpeechStarted);

  EXPECT_EQ(
    second->type,
    savo_speech::vad::VadEventType::
    SpeechEnded);

  EXPECT_FALSE(
    processor.try_pop_event().has_value());

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.speech_started_events,
    1U);

  EXPECT_EQ(
    snapshot.statistics.speech_ended_events,
    1U);
}

TEST(VadProcessor, DropsOldestEventWhenQueueIsFull)
{
  FakeVadBackend backend;

  backend.push(0.90);
  backend.push(0.10);
  backend.push(0.90);

  auto config = make_config();

  config.required_start_frames = 1U;
  config.required_end_frames = 1U;
  config.event_queue_capacity = 2U;

  savo_speech::vad::VadProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U));
  processor.process(make_frame(2U));
  processor.process(make_frame(3U));

  const auto first =
    processor.try_pop_event();

  const auto second =
    processor.try_pop_event();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->event_id, 2U);
  EXPECT_EQ(second->event_id, 3U);

  EXPECT_EQ(first->segment_id, 1U);
  EXPECT_EQ(second->segment_id, 2U);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.queue_overflows,
    1U);

  EXPECT_EQ(
    snapshot.statistics.events_dropped,
    1U);
}

TEST(VadProcessor, ResetReturnsProcessorToSilence)
{
  FakeVadBackend backend;

  backend.push(0.90);

  auto config = make_config();
  config.required_start_frames = 1U;

  savo_speech::vad::VadProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U));

  ASSERT_EQ(
    processor.snapshot().state,
    savo_speech::vad::VadState::Speech);

  processor.reset();

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.state,
    savo_speech::vad::VadState::Silence);

  EXPECT_EQ(snapshot.queued_events, 0U);
  EXPECT_EQ(snapshot.active_segment_id, 0U);
  EXPECT_EQ(snapshot.statistics.frames_processed, 0U);
  EXPECT_FALSE(snapshot.last_event.has_value());
  EXPECT_TRUE(snapshot.last_error.empty());

  EXPECT_EQ(backend.reset_calls(), 1U);
}

TEST(VadProcessor, PropagatesTimestampSequenceAndSegment)
{
  FakeVadBackend backend;

  backend.push(0.90);
  backend.push(0.10);

  auto config = make_config();

  config.required_start_frames = 1U;
  config.required_end_frames = 1U;

  savo_speech::vad::VadProcessor processor{
    backend,
    config};

  const auto start_frame =
    make_frame(42U, 1234ms);

  const auto end_frame =
    make_frame(43U, 1254ms);

  processor.process(start_frame);
  processor.process(end_frame);

  const auto started =
    processor.try_pop_event();

  const auto ended =
    processor.try_pop_event();

  ASSERT_TRUE(started.has_value());
  ASSERT_TRUE(ended.has_value());

  EXPECT_EQ(started->frame_sequence, 42U);
  EXPECT_EQ(started->occurred_at, start_frame.captured_at);
  EXPECT_EQ(started->segment_id, 1U);

  EXPECT_EQ(ended->frame_sequence, 43U);
  EXPECT_EQ(ended->occurred_at, end_frame.captured_at);
  EXPECT_EQ(ended->segment_id, 1U);
}

TEST(VadProcessor, RejectsInvalidBackendScore)
{
  FakeVadBackend backend;

  backend.push(
    std::numeric_limits<double>::quiet_NaN());

  savo_speech::vad::VadProcessor processor{
    backend,
    make_config()};

  EXPECT_THROW(
    processor.process(make_frame(1U)),
    std::runtime_error);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.backend_failures,
    1U);

  EXPECT_EQ(
    snapshot.statistics.invalid_backend_results,
    1U);

  EXPECT_NE(
    snapshot.last_error.find("[0, 1]"),
    std::string::npos);
}

TEST(VadProcessor, BackendFailureIsPropagated)
{
  FakeVadBackend backend;

  backend.set_throw_on_analyze(true);

  savo_speech::vad::VadProcessor processor{
    backend,
    make_config()};

  EXPECT_THROW(
    processor.process(make_frame(1U)),
    std::runtime_error);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.backend_failures,
    1U);

  EXPECT_NE(
    snapshot.last_error.find(
      "scripted VAD backend failure"),
    std::string::npos);
}
