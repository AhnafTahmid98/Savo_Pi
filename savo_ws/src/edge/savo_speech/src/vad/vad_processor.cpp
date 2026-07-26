#include "savo_speech/vad/vad_processor.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::vad
{

bool VadProcessorConfig::is_valid() const noexcept
{
  return
    std::isfinite(speech_start_threshold) &&
    std::isfinite(speech_end_threshold) &&
    speech_start_threshold >= 0.0 &&
    speech_start_threshold <= 1.0 &&
    speech_end_threshold >= 0.0 &&
    speech_end_threshold <= 1.0 &&
    speech_end_threshold < speech_start_threshold &&
    required_start_frames >= 1U &&
    required_start_frames <= 1000U &&
    required_end_frames >= 1U &&
    required_end_frames <= 1000U &&
    event_queue_capacity >= 1U &&
    event_queue_capacity <= 1024U;
}

VadProcessor::VadProcessor(
  VadBackend & backend,
  VadProcessorConfig config)
: backend_{backend},
  config_{std::move(config)}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid VAD processor configuration"};
  }
}

void VadProcessor::process(
  const audio::AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    throw std::invalid_argument{
            "VAD processor received an invalid audio frame"};
  }

  if (frame.format.channels != 1U) {
    throw std::invalid_argument{
            "VAD processor requires mono audio"};
  }

  VadBackendResult result;

  try {
    result = backend_.analyze(frame);
  } catch (const std::exception & exception) {
    const std::string error =
      "VAD backend failed: " +
      std::string{exception.what()};

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      ++statistics_.backend_failures;

      statistics_.last_frame_sequence =
        frame.sequence;

      last_error_ = error;

      reset_debounce_locked();
    }

    throw std::runtime_error{error};
  } catch (...) {
    const std::string error =
      "VAD backend failed with an unknown exception";

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      ++statistics_.backend_failures;

      statistics_.last_frame_sequence =
        frame.sequence;

      last_error_ = error;

      reset_debounce_locked();
    }

    throw std::runtime_error{error};
  }

  if (
    !std::isfinite(result.speech_score) ||
    result.speech_score < 0.0 ||
    result.speech_score > 1.0)
  {
    const std::string error =
      "VAD backend returned a speech score "
      "outside the range [0, 1]";

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      ++statistics_.backend_failures;
      ++statistics_.invalid_backend_results;

      statistics_.last_frame_sequence =
        frame.sequence;

      last_error_ = error;

      reset_debounce_locked();
    }

    throw std::runtime_error{error};
  }

  bool event_enqueued{false};

  std::unique_lock lock{mutex_};

  ++statistics_.frames_processed;

  statistics_.last_frame_sequence =
    frame.sequence;

  last_speech_score_ =
    result.speech_score;

  last_error_.clear();

  if (state_ == VadState::Silence) {
    consecutive_end_frames_ = 0U;

    if (
      result.speech_score >=
      config_.speech_start_threshold)
    {
      ++statistics_.start_candidate_frames;
      ++consecutive_start_frames_;

      if (
        consecutive_start_frames_ >=
        config_.required_start_frames)
      {
        enqueue_transition_locked(
          VadEventType::SpeechStarted,
          frame,
          result.speech_score);

        event_enqueued = true;
      }
    } else {
      if (consecutive_start_frames_ > 0U) {
        ++statistics_.start_debounce_resets;
      }

      consecutive_start_frames_ = 0U;
    }
  } else {
    consecutive_start_frames_ = 0U;

    if (
      result.speech_score <=
      config_.speech_end_threshold)
    {
      ++statistics_.end_candidate_frames;
      ++consecutive_end_frames_;

      if (
        consecutive_end_frames_ >=
        config_.required_end_frames)
      {
        enqueue_transition_locked(
          VadEventType::SpeechEnded,
          frame,
          result.speech_score);

        event_enqueued = true;
      }
    } else {
      if (consecutive_end_frames_ > 0U) {
        ++statistics_.end_debounce_resets;
      }

      consecutive_end_frames_ = 0U;
    }
  }

  lock.unlock();

  if (event_enqueued) {
    condition_.notify_one();
  }
}

std::optional<VadEvent>
VadProcessor::try_pop_event()
{
  const std::scoped_lock lock{mutex_};

  if (events_.empty()) {
    return std::nullopt;
  }

  VadEvent event =
    std::move(events_.front());

  events_.pop_front();

  return event;
}

std::optional<VadEvent>
VadProcessor::wait_event_for(
  const std::chrono::milliseconds timeout)
{
  if (timeout < std::chrono::milliseconds{0}) {
    throw std::invalid_argument{
            "VAD event wait timeout must not be negative"};
  }

  std::unique_lock lock{mutex_};

  const bool available = condition_.wait_for(
    lock,
    timeout,
    [this]() {
      return !events_.empty();
    });

  if (!available || events_.empty()) {
    return std::nullopt;
  }

  VadEvent event =
    std::move(events_.front());

  events_.pop_front();

  return event;
}

VadProcessorSnapshot
VadProcessor::snapshot() const
{
  const std::scoped_lock lock{mutex_};

  VadProcessorSnapshot snapshot;

  snapshot.state = state_;
  snapshot.statistics = statistics_;

  snapshot.queued_events = events_.size();

  snapshot.consecutive_start_frames =
    consecutive_start_frames_;

  snapshot.consecutive_end_frames =
    consecutive_end_frames_;

  snapshot.active_segment_id =
    active_segment_id_;

  snapshot.last_speech_score =
    last_speech_score_;

  snapshot.last_event =
    last_event_;

  snapshot.last_error =
    last_error_;

  return snapshot;
}

void VadProcessor::reset() noexcept
{
  {
    const std::scoped_lock lock{mutex_};

    events_.clear();

    state_ = VadState::Silence;

    statistics_ =
      VadProcessorStatistics{};

    active_segment_id_ = 0U;

    last_speech_score_ = 0.0;

    last_event_.reset();
    last_error_.clear();

    reset_debounce_locked();
  }

  backend_.reset();
}

void VadProcessor::reset_debounce_locked() noexcept
{
  consecutive_start_frames_ = 0U;
  consecutive_end_frames_ = 0U;
}

void VadProcessor::enqueue_transition_locked(
  const VadEventType type,
  const audio::AudioFrame & frame,
  const double speech_score)
{
  VadEvent event;

  event.event_id =
    statistics_.last_event_id + 1U;

  event.frame_sequence =
    frame.sequence;

  event.occurred_at =
    frame.captured_at;

  event.type = type;
  event.speech_score = speech_score;

  if (type == VadEventType::SpeechStarted) {
    ++statistics_.last_segment_id;

    active_segment_id_ =
      statistics_.last_segment_id;

    event.segment_id =
      active_segment_id_;

    state_ = VadState::Speech;

    ++statistics_.speech_started_events;
  } else {
    event.segment_id =
      active_segment_id_;

    state_ = VadState::Silence;

    ++statistics_.speech_ended_events;

    active_segment_id_ = 0U;
  }

  statistics_.last_event_id =
    event.event_id;

  last_event_ = event;

  reset_debounce_locked();

  enqueue_event_locked(
    std::move(event));
}

void VadProcessor::enqueue_event_locked(
  VadEvent event)
{
  if (events_.size() >= config_.event_queue_capacity) {
    ++statistics_.queue_overflows;
    ++statistics_.events_dropped;

    events_.pop_front();
  }

  events_.push_back(
    std::move(event));
}

}  // namespace savo_speech::vad
