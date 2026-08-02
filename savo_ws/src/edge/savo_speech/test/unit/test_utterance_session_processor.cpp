// Copyright 2026 Ahnaf Tahmid
#include <chrono>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "savo_speech/session/utterance_session_processor.hpp"
#include "savo_speech/vad/vad_backend.hpp"
#include "savo_speech/wake_word/wake_word_backend.hpp"

namespace
{

using namespace std::chrono_literals;

using SessionProcessor =
  savo_speech::session::
  UtteranceSessionProcessor;

using SessionState =
  savo_speech::session::
  UtteranceSessionState;

using CompletionReason =
  savo_speech::session::
  UtteranceCompletionReason;

using CancellationReason =
  savo_speech::session::
  UtteranceCancellationReason;

class ScriptedWakeWordBackend final
  : public savo_speech::wake_word::WakeWordBackend
{
public:
  [[nodiscard]]
  savo_speech::wake_word::WakeWordBackendResult
  analyze(
    const savo_speech::audio::AudioFrame &) override
  {
    if (results_.empty()) {
      return {};
    }

    auto result =
      std::move(results_.front());

    results_.pop_front();

    return result;
  }

  void reset() noexcept override
  {
    results_.clear();
  }

  void push_detection(
    std::string phrase = "hey savo",
    const double confidence = 0.90)
  {
    savo_speech::wake_word::
    WakeWordBackendResult result;

    result.detected = true;
    result.phrase = std::move(phrase);
    result.confidence = confidence;

    results_.push_back(
      std::move(result));
  }

private:
  std::deque<
    savo_speech::wake_word::
    WakeWordBackendResult> results_{};
};

class ScriptedVadBackend final
  : public savo_speech::vad::VadBackend
{
public:
  [[nodiscard]]
  savo_speech::vad::VadBackendResult
  analyze(
    const savo_speech::audio::AudioFrame &) override
  {
    if (results_.empty()) {
      savo_speech::vad::VadBackendResult result;
      result.speech_score = 0.50;
      return result;
    }

    const auto result = results_.front();
    results_.pop_front();

    return result;
  }

  void reset() noexcept override
  {
    results_.clear();
  }

  void push_score(const double score)
  {
    savo_speech::vad::VadBackendResult result;
    result.speech_score = score;

    results_.push_back(result);
  }

private:
  std::deque<
    savo_speech::vad::VadBackendResult>
  results_{};
};

[[nodiscard]]
savo_speech::wake_word::WakeWordProcessorConfig
make_wake_config()
{
  savo_speech::wake_word::
  WakeWordProcessorConfig config;

  config.confidence_threshold = 0.65;
  config.required_consecutive_detections = 1U;
  config.cooldown = 0ms;
  config.event_queue_capacity = 8U;

  return config;
}

[[nodiscard]]
savo_speech::vad::VadProcessorConfig
make_vad_config()
{
  savo_speech::vad::VadProcessorConfig config;

  config.speech_start_threshold = 0.70;
  config.speech_end_threshold = 0.30;

  config.required_start_frames = 1U;
  config.required_end_frames = 1U;

  config.event_queue_capacity = 8U;

  return config;
}

[[nodiscard]]
savo_speech::session::UtteranceSessionConfig
make_session_config()
{
  savo_speech::session::
  UtteranceSessionConfig config;

  config.pre_roll_duration = 1ms;
  config.speech_start_timeout = 100ms;
  config.maximum_utterance_duration = 1000ms;
  config.completed_queue_capacity = 4U;

  return config;
}

[[nodiscard]]
savo_speech::audio::AudioFrame::Clock::time_point
time_at(const std::int64_t milliseconds)
{
  return
    savo_speech::audio::AudioFrame::
    Clock::time_point{} +
    std::chrono::milliseconds{
      milliseconds};
}

[[nodiscard]]
savo_speech::audio::AudioFrame
make_frame(
  const std::uint64_t sequence,
  const std::int16_t sample,
  const std::int64_t milliseconds)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;
  frame.captured_at = time_at(milliseconds);

  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    sample,
    sample,
    sample,
    sample};

  return frame;
}

struct Harness
{
  explicit Harness(
    savo_speech::session::
    UtteranceSessionConfig session_config =
    make_session_config())
  : wake_processor{
      wake_backend,
      make_wake_config()},
    vad_processor{
      vad_backend,
      make_vad_config()},
    session_processor{
      wake_processor,
      vad_processor,
      std::move(session_config)}
  {
  }

  void process_all(
    const savo_speech::audio::AudioFrame & frame)
  {
    wake_processor.process(frame);
    vad_processor.process(frame);
    session_processor.process(frame);
  }

  ScriptedWakeWordBackend wake_backend;
  ScriptedVadBackend vad_backend;

  savo_speech::wake_word::WakeWordProcessor
    wake_processor;

  savo_speech::vad::VadProcessor
    vad_processor;

  SessionProcessor session_processor;
};

void start_session(
  Harness & harness)
{
  harness.wake_backend.push_detection();
  harness.vad_backend.push_score(0.90);

  harness.process_all(
    make_frame(1U, 100, 0));
}

}  // namespace

TEST(
  UtteranceSessionProcessor,
  RoutesWakeAndVadStartFromSameFrame)
{
  Harness harness;

  start_session(harness);

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Recording);

  EXPECT_EQ(
    snapshot.statistics.frames_completed,
    1U);

  EXPECT_EQ(
    snapshot.statistics.wake_events_drained,
    1U);

  EXPECT_EQ(
    snapshot.statistics.wake_events_accepted,
    1U);

  EXPECT_EQ(
    snapshot.statistics.vad_events_drained,
    1U);

  EXPECT_EQ(
    snapshot.statistics.vad_events_accepted,
    1U);
}

TEST(
  UtteranceSessionProcessor,
  SpeechEndIncludesCurrentFrameAndCompletes)
{
  Harness harness;

  start_session(harness);

  harness.vad_backend.push_score(0.10);

  harness.process_all(
    make_frame(2U, 200, 20));

  const auto completed =
    harness.session_processor.
    try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  const std::vector<std::int16_t> expected{
    100, 100, 100, 100,
    200, 200, 200, 200};

  EXPECT_EQ(
    completed->audio.interleaved_samples,
    expected);

  EXPECT_EQ(
    completed->completion_reason,
    CompletionReason::SpeechEnded);

  EXPECT_EQ(
    harness.session_processor.
    snapshot().session.state,
    SessionState::Idle);
}

TEST(
  UtteranceSessionProcessor,
  RejectsVadStartWithoutWake)
{
  Harness harness;

  harness.vad_backend.push_score(0.90);

  harness.process_all(
    make_frame(1U, 100, 0));

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Idle);

  EXPECT_EQ(
    snapshot.statistics.vad_events_drained,
    1U);

  EXPECT_EQ(
    snapshot.statistics.vad_events_rejected,
    1U);
}

TEST(
  UtteranceSessionProcessor,
  ProcessesEventBeforeTimeoutAtSameTimestamp)
{
  Harness harness;

  harness.wake_backend.push_detection();

  harness.process_all(
    make_frame(1U, 100, 0));

  ASSERT_EQ(
    harness.session_processor.
    snapshot().session.state,
    SessionState::Armed);

  harness.vad_backend.push_score(0.90);

  harness.process_all(
    make_frame(2U, 200, 100));

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Recording);

  EXPECT_EQ(
    snapshot.session.statistics.
    speech_start_timeouts,
    0U);
}

TEST(
  UtteranceSessionProcessor,
  DrainsQueuedEventsInDeterministicOrder)
{
  Harness harness;

  const auto first =
    make_frame(1U, 100, 0);

  const auto second =
    make_frame(2U, 200, 20);

  harness.wake_backend.push_detection(
    "hey savo");

  harness.wake_processor.process(first);

  harness.wake_backend.push_detection(
    "hi savo");

  harness.wake_processor.process(second);

  harness.vad_backend.push_score(0.90);
  harness.vad_processor.process(first);

  harness.vad_backend.push_score(0.10);
  harness.vad_processor.process(second);

  harness.session_processor.process(second);

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.wake_events_drained,
    2U);

  EXPECT_EQ(
    snapshot.statistics.wake_events_accepted,
    1U);

  EXPECT_EQ(
    snapshot.statistics.wake_events_rejected,
    1U);

  EXPECT_EQ(
    snapshot.statistics.vad_events_drained,
    2U);

  EXPECT_EQ(
    snapshot.statistics.vad_events_accepted,
    2U);

  EXPECT_EQ(
    snapshot.session.statistics.
    utterances_completed,
    1U);
}

TEST(
  UtteranceSessionProcessor,
  ForwardsCompletedUtteranceWait)
{
  Harness harness;

  start_session(harness);

  harness.vad_backend.push_score(0.10);

  harness.process_all(
    make_frame(2U, 200, 20));

  const auto completed =
    harness.session_processor.
    wait_completed_for(0ms);

  ASSERT_TRUE(completed.has_value());

  EXPECT_EQ(completed->utterance_id, 1U);
}

TEST(
  UtteranceSessionProcessor,
  ResetDiscardsQueuedEventsAndSessionState)
{
  Harness harness;

  const auto frame =
    make_frame(1U, 100, 0);

  harness.wake_backend.push_detection();
  harness.wake_processor.process(frame);

  harness.vad_backend.push_score(0.90);
  harness.vad_processor.process(frame);

  ASSERT_EQ(
    harness.wake_processor.
    snapshot().queued_events,
    1U);

  ASSERT_EQ(
    harness.vad_processor.
    snapshot().queued_events,
    1U);

  harness.session_processor.reset();

  auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(snapshot.pending_wake_events, 0U);
  EXPECT_EQ(snapshot.pending_vad_events, 0U);

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Idle);

  EXPECT_EQ(
    snapshot.statistics.frames_received,
    0U);

  harness.session_processor.process(
    make_frame(2U, 200, 20));

  snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Idle);

  EXPECT_EQ(
    snapshot.statistics.frames_completed,
    1U);
}

TEST(
  UtteranceSessionProcessor,
  RecordsFrameFailureAndRethrows)
{
  Harness harness;

  const auto frame =
    make_frame(1U, 100, 0);

  harness.session_processor.process(frame);

  EXPECT_THROW(
    harness.session_processor.process(frame),
    std::invalid_argument);

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.frames_received,
    2U);

  EXPECT_EQ(
    snapshot.statistics.frames_completed,
    1U);

  EXPECT_EQ(
    snapshot.statistics.frames_failed,
    1U);

  EXPECT_FALSE(snapshot.last_error.empty());
}

TEST(
  UtteranceSessionProcessor,
  ForwardsExplicitCancellation)
{
  Harness harness;

  start_session(harness);

  EXPECT_TRUE(
    harness.session_processor.cancel(
      CancellationReason::
      ExplicitCancellation));

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(
    snapshot.session.state,
    SessionState::Idle);

  EXPECT_EQ(
    snapshot.session.
    last_cancellation_reason,
    CancellationReason::
    ExplicitCancellation);
}

TEST(
  UtteranceSessionProcessor,
  MaximumDurationCompletesAfterEventDrain)
{
  auto config = make_session_config();

  config.maximum_utterance_duration =
    100ms;

  Harness harness{config};

  start_session(harness);

  harness.session_processor.process(
    make_frame(2U, 200, 100));

  const auto completed =
    harness.session_processor.
    try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  EXPECT_EQ(
    completed->completion_reason,
    CompletionReason::
    MaximumDurationReached);
}

TEST(
  UtteranceSessionProcessor,
  SnapshotReportsPendingUpstreamEvents)
{
  Harness harness;

  const auto frame =
    make_frame(1U, 100, 0);

  harness.wake_backend.push_detection();
  harness.wake_processor.process(frame);

  harness.vad_backend.push_score(0.90);
  harness.vad_processor.process(frame);

  const auto snapshot =
    harness.session_processor.snapshot();

  EXPECT_EQ(snapshot.pending_wake_events, 1U);
  EXPECT_EQ(snapshot.pending_vad_events, 1U);

  EXPECT_EQ(
    snapshot.statistics.frames_received,
    0U);
}
