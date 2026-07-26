#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/wav_reader.hpp"
#include "savo_speech/session/completed_utterance.hpp"
#include "savo_speech/session/completed_utterance_source.hpp"
#include "savo_speech/session/completed_utterance_worker.hpp"
#include "savo_speech/session/utterance_session_event.hpp"

namespace
{

using namespace std::chrono_literals;

class ScriptedCompletedUtteranceSource final :
  public savo_speech::session::
  CompletedUtteranceSource
{
public:
  [[nodiscard]]
  std::optional<
    savo_speech::session::CompletedUtterance>
  wait_completed_for(
    const std::chrono::milliseconds timeout) override
  {
    std::unique_lock lock{mutex_};

    condition_.wait_for(
      lock,
      timeout,
      [this]() {
        return
          fail_waits_ ||
          !utterances_.empty();
      });

    if (fail_waits_) {
      throw std::runtime_error{failure_message_};
    }

    if (utterances_.empty()) {
      return std::nullopt;
    }

    auto utterance =
      std::move(utterances_.front());

    utterances_.pop_front();

    return utterance;
  }

  void push(
    savo_speech::session::CompletedUtterance
      utterance)
  {
    {
      const std::scoped_lock lock{mutex_};

      utterances_.push_back(
        std::move(utterance));
    }

    condition_.notify_one();
  }

  void fail_waits(std::string message)
  {
    {
      const std::scoped_lock lock{mutex_};

      failure_message_ = std::move(message);
      fail_waits_ = true;
    }

    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<
    savo_speech::session::CompletedUtterance>
    utterances_;

  bool fail_waits_{false};

  std::string failure_message_{
    "scripted completed-utterance source failure"};
};

template<typename Predicate>
[[nodiscard]] bool wait_until(
  Predicate predicate,
  const std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + timeout;

  while (
    std::chrono::steady_clock::now() < deadline)
  {
    if (predicate()) {
      return true;
    }

    std::this_thread::sleep_for(1ms);
  }

  return predicate();
}

[[nodiscard]]
savo_speech::session::CompletedUtterance
make_completed_utterance(
  const std::uint64_t utterance_id,
  const std::size_t sample_count = 32U)
{
  using Completed =
    savo_speech::session::CompletedUtterance;

  Completed utterance;

  utterance.utterance_id = utterance_id;

  utterance.wake_event_id =
    utterance_id + 100U;

  utterance.wake_frame_sequence = 10U;

  utterance.wake_detected_at =
    Completed::Clock::time_point{100ms};

  utterance.wake_phrase = "savo";
  utterance.wake_confidence = 0.91;

  utterance.vad_segment_id =
    utterance_id + 200U;

  utterance.speech_start_frame_sequence = 11U;
  utterance.speech_end_frame_sequence = 13U;

  utterance.speech_started_at =
    Completed::Clock::time_point{120ms};

  utterance.speech_ended_at =
    Completed::Clock::time_point{180ms};

  utterance.first_audio_frame_sequence = 9U;
  utterance.last_audio_frame_sequence = 13U;

  utterance.audio_started_at =
    Completed::Clock::time_point{80ms};

  utterance.audio_ended_at =
    Completed::Clock::time_point{180ms};

  utterance.completed_at =
    Completed::Clock::time_point{200ms};

  utterance.completion_reason =
    savo_speech::session::
    UtteranceCompletionReason::SpeechEnded;

  utterance.audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  utterance.audio.interleaved_samples.resize(
    sample_count,
    static_cast<std::int16_t>(250));

  utterance.pre_roll_samples = 4U;

  utterance.sequence_gap_count = 2U;
  utterance.missing_audio_frames = 3U;

  return utterance;
}

[[nodiscard]]
savo_speech::session::
CompletedUtteranceWorkerConfig
make_worker_config()
{
  savo_speech::session::
  CompletedUtteranceWorkerConfig config;

  config.output_queue_capacity = 4U;
  config.source_wait_timeout = 5ms;
  config.maximum_wav_bytes =
    2U * 1024U * 1024U;

  return config;
}

}  // namespace

TEST(
  CompletedUtteranceWorker,
  RejectsInvalidConfiguration)
{
  ScriptedCompletedUtteranceSource source;

  auto config = make_worker_config();
  config.output_queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::session::
      CompletedUtteranceWorker{
        source,
        config}),
    std::invalid_argument);

  config = make_worker_config();
  config.source_wait_timeout = 0ms;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::session::
      CompletedUtteranceWorker{
        source,
        config}),
    std::invalid_argument);

  config = make_worker_config();
  config.maximum_wav_bytes = 43U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::session::
      CompletedUtteranceWorker{
        source,
        config}),
    std::invalid_argument);
}

TEST(
  CompletedUtteranceWorker,
  SerializesCompletedUtteranceAndPreservesMetadata)
{
  ScriptedCompletedUtteranceSource source;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    make_worker_config()};

  ASSERT_TRUE(worker.start());

  auto completed = make_completed_utterance(7U);

  completed.completion_reason =
    savo_speech::session::
    UtteranceCompletionReason::
    MaximumDurationReached;

  source.push(std::move(completed));

  const auto serialized =
    worker.wait_serialized_for(500ms);

  ASSERT_TRUE(serialized.has_value());
  ASSERT_TRUE(serialized->is_valid());

  EXPECT_EQ(serialized->utterance_id, 7U);
  EXPECT_EQ(serialized->wake_event_id, 107U);
  EXPECT_EQ(serialized->wake_phrase, "savo");
  EXPECT_DOUBLE_EQ(
    serialized->wake_confidence,
    0.91);

  EXPECT_EQ(serialized->vad_segment_id, 207U);

  EXPECT_EQ(
    serialized->completion_reason,
    savo_speech::session::
    UtteranceCompletionReason::
    MaximumDurationReached);

  EXPECT_EQ(serialized->sample_count, 32U);
  EXPECT_EQ(serialized->pre_roll_samples, 4U);

  EXPECT_EQ(serialized->sequence_gap_count, 2U);
  EXPECT_EQ(serialized->missing_audio_frames, 3U);

  const auto decoded =
    savo_speech::audio::WavReader::decode(
    serialized->wav_bytes);

  EXPECT_EQ(decoded.format, serialized->audio_format);
  EXPECT_EQ(decoded.interleaved_samples.size(), 32U);

  for (const std::int16_t sample :
    decoded.interleaved_samples)
  {
    EXPECT_EQ(sample, 250);
  }

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(
    snapshot.statistics.utterances_received,
    1U);

  EXPECT_EQ(
    snapshot.statistics.utterances_serialized,
    1U);

  EXPECT_EQ(
    snapshot.last_seen_utterance_id,
    std::optional<std::uint64_t>{7U});

  worker.stop();

  EXPECT_FALSE(worker.running());
}

TEST(
  CompletedUtteranceWorker,
  RejectsInvalidCompletedUtteranceWithoutFaulting)
{
  ScriptedCompletedUtteranceSource source;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    make_worker_config()};

  ASSERT_TRUE(worker.start());

  auto invalid = make_completed_utterance(1U);

  invalid.audio.interleaved_samples.clear();

  source.push(std::move(invalid));

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().
          statistics.utterances_received >= 1U;
      }));

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(
    snapshot.statistics.invalid_utterances,
    1U);

  EXPECT_EQ(
    snapshot.state,
    savo_speech::session::
    CompletedUtteranceWorkerState::Running);

  EXPECT_FALSE(
    worker.try_pop_serialized().has_value());

  worker.stop();
}

TEST(
  CompletedUtteranceWorker,
  RejectsDuplicateAndOutOfOrderIds)
{
  ScriptedCompletedUtteranceSource source;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    make_worker_config()};

  ASSERT_TRUE(worker.start());

  source.push(make_completed_utterance(10U));

  const auto first =
    worker.wait_serialized_for(500ms);

  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->utterance_id, 10U);

  source.push(make_completed_utterance(10U));
  source.push(make_completed_utterance(9U));
  source.push(make_completed_utterance(11U));

  const auto second =
    worker.wait_serialized_for(500ms);

  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->utterance_id, 11U);

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().
          statistics.utterances_received >= 4U;
      }));

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(
    snapshot.statistics.
    duplicate_or_out_of_order_ids,
    2U);

  EXPECT_EQ(
    snapshot.statistics.utterances_serialized,
    2U);

  EXPECT_EQ(
    snapshot.last_seen_utterance_id,
    std::optional<std::uint64_t>{11U});

  worker.stop();
}

TEST(
  CompletedUtteranceWorker,
  EnforcesMaximumWavByteLimit)
{
  ScriptedCompletedUtteranceSource source;

  auto config = make_worker_config();

  config.maximum_wav_bytes = 44U;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    config};

  ASSERT_TRUE(worker.start());

  source.push(make_completed_utterance(1U, 8U));

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().
          statistics.wav_size_limit_rejections >= 1U;
      }));

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(
    snapshot.statistics.utterances_received,
    1U);

  EXPECT_EQ(
    snapshot.statistics.wav_size_limit_rejections,
    1U);

  EXPECT_EQ(
    snapshot.statistics.utterances_serialized,
    0U);

  EXPECT_FALSE(
    worker.try_pop_serialized().has_value());

  worker.stop();
}

TEST(
  CompletedUtteranceWorker,
  RejectsWhenOutputQueueIsFull)
{
  ScriptedCompletedUtteranceSource source;

  auto config = make_worker_config();

  config.output_queue_capacity = 1U;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    config};

  ASSERT_TRUE(worker.start());

  source.push(make_completed_utterance(1U));

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().output_queue.size == 1U;
      }));

  source.push(make_completed_utterance(2U));

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().
          statistics.utterances_received >= 2U;
      }));

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(
    snapshot.statistics.utterances_serialized,
    1U);

  EXPECT_EQ(
    snapshot.statistics.output_queue_rejections,
    1U);

  EXPECT_EQ(snapshot.output_queue.rejected_full, 1U);

  const auto queued =
    worker.try_pop_serialized();

  ASSERT_TRUE(queued.has_value());
  EXPECT_EQ(queued->utterance_id, 1U);

  worker.stop();
}

TEST(
  CompletedUtteranceWorker,
  SourceFailureTransitionsWorkerToFaulted)
{
  ScriptedCompletedUtteranceSource source;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    make_worker_config()};

  source.fail_waits(
    "scripted completed source failure");

  ASSERT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.snapshot().state ==
          savo_speech::session::
          CompletedUtteranceWorkerState::Faulted;
      }));

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(snapshot.statistics.faults, 1U);

  EXPECT_NE(
    snapshot.last_error.find(
      "scripted completed source failure"),
    std::string::npos);

  worker.stop();

  EXPECT_EQ(
    worker.snapshot().state,
    savo_speech::session::
    CompletedUtteranceWorkerState::Stopped);
}

TEST(
  CompletedUtteranceWorker,
  CanStopAndRestart)
{
  ScriptedCompletedUtteranceSource source;

  savo_speech::session::CompletedUtteranceWorker
    worker{
    source,
    make_worker_config()};

  ASSERT_TRUE(worker.start());

  source.push(make_completed_utterance(1U));

  const auto first =
    worker.wait_serialized_for(500ms);

  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->utterance_id, 1U);

  worker.stop();

  EXPECT_FALSE(worker.running());

  ASSERT_TRUE(worker.start());

  source.push(make_completed_utterance(2U));

  const auto second =
    worker.wait_serialized_for(500ms);

  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->utterance_id, 2U);

  worker.stop();

  const auto snapshot = worker.snapshot();

  EXPECT_EQ(snapshot.statistics.starts, 2U);
  EXPECT_EQ(snapshot.statistics.stops, 2U);
  EXPECT_EQ(
    snapshot.statistics.utterances_serialized,
    2U);
}
