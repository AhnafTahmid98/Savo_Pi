// Copyright 2026 Ahnaf Tahmid
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_speech/session/completed_utterance_worker.hpp"

#include "savo_speech/audio/wav_writer.hpp"

namespace savo_speech::session
{

namespace
{

[[nodiscard]] SerializedUtterance serialize(
  CompletedUtterance utterance)
{
  SerializedUtterance result;

  result.utterance_id = utterance.utterance_id;

  result.wake_event_id = utterance.wake_event_id;
  result.wake_frame_sequence =
    utterance.wake_frame_sequence;

  result.wake_detected_at =
    utterance.wake_detected_at;

  result.wake_phrase =
    std::move(utterance.wake_phrase);

  result.wake_confidence =
    utterance.wake_confidence;

  result.vad_segment_id =
    utterance.vad_segment_id;

  result.speech_start_frame_sequence =
    utterance.speech_start_frame_sequence;

  result.speech_end_frame_sequence =
    utterance.speech_end_frame_sequence;

  result.speech_started_at =
    utterance.speech_started_at;

  result.speech_ended_at =
    utterance.speech_ended_at;

  result.first_audio_frame_sequence =
    utterance.first_audio_frame_sequence;

  result.last_audio_frame_sequence =
    utterance.last_audio_frame_sequence;

  result.audio_started_at =
    utterance.audio_started_at;

  result.audio_ended_at =
    utterance.audio_ended_at;

  result.completed_at =
    utterance.completed_at;

  result.completion_reason =
    utterance.completion_reason;

  result.audio_format =
    utterance.audio.format;

  result.sample_count =
    utterance.audio.interleaved_samples.size();

  result.pre_roll_samples =
    utterance.pre_roll_samples;

  result.sequence_gap_count =
    utterance.sequence_gap_count;

  result.missing_audio_frames =
    utterance.missing_audio_frames;

  result.wav_bytes =
    audio::WavWriter::encode(utterance.audio);

  return result;
}

}  // namespace

CompletedUtteranceWorker::CompletedUtteranceWorker(
  CompletedUtteranceSource & source,
  const CompletedUtteranceWorkerConfig config)
: source_{source},
  config_{config},
  output_queue_{config.output_queue_capacity}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid completed-utterance worker configuration"};
  }
}

CompletedUtteranceWorker::~CompletedUtteranceWorker()
{
  try {
    stop();
  } catch (...) {
  }
}

bool CompletedUtteranceWorker::start()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == CompletedUtteranceWorkerState::Starting ||
      state_ == CompletedUtteranceWorkerState::Running ||
      state_ == CompletedUtteranceWorkerState::Stopping)
    {
      return false;
    }

    state_ = CompletedUtteranceWorkerState::Starting;
    current_utterance_id_.reset();
    last_error_.clear();
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  {
    const std::scoped_lock lock{mutex_};

    state_ = CompletedUtteranceWorkerState::Running;

    ++statistics_.starts;
  }

  try {
    thread_ = std::jthread{
      [this](const std::stop_token stop_token) {
        run(stop_token);
      }};
  } catch (const std::exception & exception) {
    fault(exception.what());
    throw;
  } catch (...) {
    fault(
      "unknown completed-utterance worker "
      "start failure");

    throw;
  }

  return true;
}

void CompletedUtteranceWorker::stop()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == CompletedUtteranceWorkerState::Stopped &&
      !thread_.joinable())
    {
      return;
    }

    state_ = CompletedUtteranceWorkerState::Stopping;
  }

  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }

  {
    const std::scoped_lock lock{mutex_};

    current_utterance_id_.reset();

    state_ = CompletedUtteranceWorkerState::Stopped;

    ++statistics_.stops;
  }
}

bool CompletedUtteranceWorker::running() const noexcept
{
  const std::scoped_lock lock{mutex_};

  return
    state_ == CompletedUtteranceWorkerState::Running;
}

std::optional<SerializedUtterance>
CompletedUtteranceWorker::try_pop_serialized()
{
  return output_queue_.try_pop();
}

std::optional<SerializedUtterance>
CompletedUtteranceWorker::wait_serialized_for(
  const std::chrono::milliseconds timeout)
{
  return output_queue_.wait_pop_for(timeout);
}

void CompletedUtteranceWorker::clear_output() noexcept
{
  output_queue_.clear();
}

CompletedUtteranceWorkerSnapshot
CompletedUtteranceWorker::snapshot() const
{
  CompletedUtteranceWorkerSnapshot result;

  {
    const std::scoped_lock lock{mutex_};

    result.state = state_;
    result.statistics = statistics_;

    result.current_utterance_id =
      current_utterance_id_;

    result.last_seen_utterance_id =
      last_seen_utterance_id_;

    result.last_error = last_error_;
  }

  result.output_queue =
    output_queue_.statistics();

  return result;
}

void CompletedUtteranceWorker::run(
  const std::stop_token stop_token)
{
  try {
    while (!stop_token.stop_requested()) {
      std::optional<CompletedUtterance> completed =
        source_.wait_completed_for(
        config_.source_wait_timeout);

      if (stop_token.stop_requested()) {
        return;
      }

      if (!completed.has_value()) {
        const std::scoped_lock lock{mutex_};

        ++statistics_.source_wait_timeouts;

        continue;
      }

      process_completed(std::move(*completed));
    }
  } catch (const std::exception & exception) {
    fault(exception.what());
  } catch (...) {
    fault(
      "unknown completed-utterance worker "
      "runtime failure");
  }
}

void CompletedUtteranceWorker::process_completed(
  CompletedUtterance utterance)
{
  {
    const std::scoped_lock lock{mutex_};

    ++statistics_.utterances_received;

    if (
      utterance.utterance_id == 0U ||
      !utterance.audio.is_consistent())
    {
      ++statistics_.invalid_utterances;

      last_error_ =
        "invalid completed utterance";

      return;
    }

    if (
      last_seen_utterance_id_.has_value() &&
      utterance.utterance_id <=
      *last_seen_utterance_id_)
    {
      ++statistics_.duplicate_or_out_of_order_ids;

      last_error_ =
        "duplicate or out-of-order utterance ID";

      return;
    }

    last_seen_utterance_id_ =
      utterance.utterance_id;

    current_utterance_id_ =
      utterance.utterance_id;
  }

  SerializedUtterance serialized;

  try {
    serialized = serialize(std::move(utterance));
  } catch (const std::exception & exception) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.encoding_failures;

      last_error_ = exception.what();
    }

    clear_current();

    return;
  } catch (...) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.encoding_failures;

      last_error_ =
        "unknown completed-utterance encoding failure";
    }

    clear_current();

    return;
  }

  if (serialized.wav_bytes.size() > config_.maximum_wav_bytes) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.wav_size_limit_rejections;

      last_error_ =
        "serialized WAV exceeds configured byte limit";
    }

    clear_current();

    return;
  }

  if (!serialized.is_valid()) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.encoding_failures;

      last_error_ =
        "serialized utterance failed validation";
    }

    clear_current();

    return;
  }

  const SerializedUtterancePushResult result =
    output_queue_.push(std::move(serialized));

  {
    const std::scoped_lock lock{mutex_};

    if (
      result ==
      SerializedUtterancePushResult::Accepted)
    {
      ++statistics_.utterances_serialized;
      last_error_.clear();
    } else {
      ++statistics_.output_queue_rejections;

      last_error_ =
        "serialized output queue rejected utterance: " +
        std::string{to_string(result)};
    }
  }

  clear_current();
}

void CompletedUtteranceWorker::fault(
  std::string error) noexcept
{
  const std::scoped_lock lock{mutex_};

  current_utterance_id_.reset();

  state_ = CompletedUtteranceWorkerState::Faulted;

  last_error_ = std::move(error);

  ++statistics_.faults;
}

void CompletedUtteranceWorker::clear_current() noexcept
{
  const std::scoped_lock lock{mutex_};

  current_utterance_id_.reset();
}

}  // namespace savo_speech::session
