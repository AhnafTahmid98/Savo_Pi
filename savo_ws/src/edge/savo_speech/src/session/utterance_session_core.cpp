// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_speech/session/utterance_session_core.hpp"

namespace savo_speech::session
{

namespace
{

[[nodiscard]] bool valid_duration(
  const std::chrono::milliseconds value,
  const std::chrono::milliseconds maximum) noexcept
{
  return
    value > std::chrono::milliseconds{0} &&
    value <= maximum;
}

[[nodiscard]] bool valid_probability(
  const double value) noexcept
{
  return
    std::isfinite(value) &&
    value >= 0.0 &&
    value <= 1.0;
}

}  // namespace

bool UtteranceSessionConfig::is_valid() const noexcept
{
  return
    valid_duration(
      pre_roll_duration,
      std::chrono::milliseconds{10000}) &&
    valid_duration(
      speech_start_timeout,
      std::chrono::milliseconds{60000}) &&
    valid_duration(
      maximum_utterance_duration,
      std::chrono::milliseconds{300000}) &&
    completed_queue_capacity >= 1U &&
    completed_queue_capacity <= 1024U;
}

UtteranceSessionCore::UtteranceSessionCore(
  UtteranceSessionConfig config)
: config_{std::move(config)}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid utterance-session configuration"};
  }
}

void UtteranceSessionCore::process_audio_frame(
  const audio::AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    const std::scoped_lock lock{mutex_};
    last_error_ =
      "utterance-session core received an invalid audio frame";

    throw std::invalid_argument{last_error_};
  }

  if (frame.format.channels != 1U) {
    const std::scoped_lock lock{mutex_};
    last_error_ =
      "utterance-session core requires mono audio";

    throw std::invalid_argument{last_error_};
  }

  bool notify{false};

  std::unique_lock lock{mutex_};

  if (!audio_format_.has_value()) {
    initialize_audio_format_locked(frame.format);
  } else if (!same_audio_format_locked(frame.format)) {
    ++statistics_.audio_format_mismatches;

    last_error_ =
      "utterance-session audio format changed";

    throw std::invalid_argument{last_error_};
  }

  if (last_audio_sequence_.has_value()) {
    if (frame.sequence == *last_audio_sequence_) {
      ++statistics_.duplicate_audio_frames;

      last_error_ =
        "duplicate utterance-session audio frame";

      throw std::invalid_argument{last_error_};
    }

    if (frame.sequence < *last_audio_sequence_) {
      ++statistics_.out_of_order_audio_frames;

      last_error_ =
        "out-of-order utterance-session audio frame";

      throw std::invalid_argument{last_error_};
    }

    if (
      last_audio_captured_at_.has_value() &&
      frame.captured_at < *last_audio_captured_at_)
    {
      ++statistics_.out_of_order_audio_timestamps;

      last_error_ =
        "out-of-order utterance-session audio timestamp";

      throw std::invalid_argument{last_error_};
    }

    if (frame.sequence > *last_audio_sequence_ + 1U) {
      ++statistics_.sequence_gaps;

      statistics_.missing_audio_frames +=
        frame.sequence -
        *last_audio_sequence_ -
        1U;
    }
  }

  ++statistics_.audio_frames_processed;

  statistics_.audio_samples_processed +=
    static_cast<std::uint64_t>(
    frame.interleaved_samples.size());

  statistics_.last_audio_frame_sequence =
    frame.sequence;

  if (state_ == UtteranceSessionState::Recording) {
    append_active_frame_locked(frame);
  }

  append_pre_roll_locked(frame);

  last_audio_sequence_ = frame.sequence;
  last_audio_captured_at_ = frame.captured_at;

  last_error_.clear();

  if (
    state_ == UtteranceSessionState::Recording &&
    pending_speech_end_.has_value() &&
    frame.sequence >=
    pending_speech_end_->frame_sequence)
  {
    const auto end_event =
      *pending_speech_end_;

    pending_speech_end_.reset();

    notify = finalize_locked(
      UtteranceCompletionReason::SpeechEnded,
      end_event.occurred_at,
      end_event.frame_sequence);
  }

  lock.unlock();

  if (notify) {
    condition_.notify_one();
  }
}

bool UtteranceSessionCore::handle_wake_word_event(
  const wake_word::WakeWordEvent & event)
{
  if (
    event.event_id == 0U ||
    event.phrase.empty() ||
    !valid_probability(event.confidence))
  {
    throw std::invalid_argument{
            "invalid wake-word event"};
  }

  const std::scoped_lock lock{mutex_};

  ++statistics_.wake_events_received;

  if (
    event.event_id <=
    statistics_.last_wake_event_id)
  {
    ++statistics_.stale_wake_events;
    return false;
  }

  statistics_.last_wake_event_id =
    event.event_id;

  if (state_ != UtteranceSessionState::Idle) {
    ++statistics_.wake_events_ignored_while_active;
    return false;
  }

  active_wake_event_ = event;

  // The wake phrase is only an activation trigger. The current
  // audio frame has already entered pre-roll before wake events
  // are consumed, so discard the wake frame and all pre-wake
  // audio here. Only post-wake audio may become user utterance
  // audio.
  if (pre_roll_buffer_) {
    pre_roll_buffer_->clear();
  }
  pre_roll_spans_.clear();

  state_ = UtteranceSessionState::Armed;

  last_cancellation_reason_ =
    UtteranceCancellationReason::None;

  ++statistics_.wake_events_accepted;
  ++statistics_.sessions_armed;

  last_error_.clear();

  return true;
}

bool UtteranceSessionCore::handle_vad_event(
  const vad::VadEvent & event)
{
  if (
    event.event_id == 0U ||
    event.segment_id == 0U ||
    !valid_probability(event.speech_score))
  {
    throw std::invalid_argument{
            "invalid VAD event"};
  }

  bool notify{false};

  std::unique_lock lock{mutex_};

  ++statistics_.vad_events_received;

  if (
    event.event_id <=
    statistics_.last_vad_event_id)
  {
    ++statistics_.stale_vad_events;
    return false;
  }

  statistics_.last_vad_event_id =
    event.event_id;

  if (event.type == vad::VadEventType::SpeechStarted) {
    if (
      state_ != UtteranceSessionState::Armed ||
      !active_wake_event_.has_value())
    {
      ++statistics_.invalid_vad_transitions;
      return false;
    }

    if (
      event.frame_sequence <=
      active_wake_event_->frame_sequence ||
      event.occurred_at <
      active_wake_event_->detected_at)
    {
      ++statistics_.stale_vad_events;
      return false;
    }

    start_recording_locked(event);

    ++statistics_.vad_events_accepted;

    return true;
  }

  if (
    state_ != UtteranceSessionState::Recording ||
    !active_speech_start_.has_value())
  {
    ++statistics_.invalid_vad_transitions;
    return false;
  }

  if (
    pending_speech_end_.has_value() ||
    event.segment_id !=
    active_speech_start_->segment_id)
  {
    ++statistics_.invalid_vad_transitions;
    return false;
  }

  if (
    event.frame_sequence <
    active_speech_start_->frame_sequence ||
    event.occurred_at <
    active_speech_start_->occurred_at)
  {
    ++statistics_.stale_vad_events;
    return false;
  }

  ++statistics_.vad_events_accepted;

  if (
    last_audio_sequence_.has_value() &&
    *last_audio_sequence_ >= event.frame_sequence)
  {
    notify = finalize_locked(
      UtteranceCompletionReason::SpeechEnded,
      event.occurred_at,
      event.frame_sequence);
  } else {
    pending_speech_end_ = event;
  }

  lock.unlock();

  if (notify) {
    condition_.notify_one();
  }

  return true;
}

void UtteranceSessionCore::advance_time(
  const Clock::time_point now)
{
  bool notify{false};

  std::unique_lock lock{mutex_};

  if (
    last_advanced_time_.has_value() &&
    now < *last_advanced_time_)
  {
    last_error_ =
      "utterance-session time moved backwards";

    throw std::invalid_argument{last_error_};
  }

  last_advanced_time_ = now;

  notify = check_timeouts_locked(now);

  lock.unlock();

  if (notify) {
    condition_.notify_one();
  }
}

bool UtteranceSessionCore::cancel(
  const UtteranceCancellationReason reason)
{
  if (
    reason == UtteranceCancellationReason::None)
  {
    throw std::invalid_argument{
            "cancellation reason must not be none"};
  }

  const std::scoped_lock lock{mutex_};

  return cancel_locked(reason);
}

std::optional<CompletedUtterance>
UtteranceSessionCore::try_pop_completed()
{
  const std::scoped_lock lock{mutex_};

  if (completed_utterances_.empty()) {
    return std::nullopt;
  }

  CompletedUtterance utterance =
    std::move(completed_utterances_.front());

  completed_utterances_.pop_front();

  return utterance;
}

std::optional<CompletedUtterance>
UtteranceSessionCore::wait_completed_for(
  const std::chrono::milliseconds timeout)
{
  if (timeout < std::chrono::milliseconds{0}) {
    throw std::invalid_argument{
            "completed-utterance wait timeout must not be negative"};
  }

  std::unique_lock lock{mutex_};

  const bool available = condition_.wait_for(
    lock,
    timeout,
    [this]() {
      return !completed_utterances_.empty();
    });

  if (!available || completed_utterances_.empty()) {
    return std::nullopt;
  }

  CompletedUtterance utterance =
    std::move(completed_utterances_.front());

  completed_utterances_.pop_front();

  return utterance;
}

UtteranceSessionSnapshot
UtteranceSessionCore::snapshot() const
{
  const std::scoped_lock lock{mutex_};

  UtteranceSessionSnapshot snapshot;

  snapshot.state = state_;
  snapshot.statistics = statistics_;

  snapshot.audio_format_initialized =
    audio_format_.has_value();

  snapshot.pre_roll_samples =
    pre_roll_buffer_ ?
    pre_roll_buffer_->size() :
    0U;

  snapshot.active_audio_samples =
    active_samples_.size();

  snapshot.queued_completed_utterances =
    completed_utterances_.size();

  snapshot.active_utterance_id =
    active_utterance_id_;

  snapshot.active_vad_segment_id =
    active_speech_start_.has_value() ?
    active_speech_start_->segment_id :
    0U;

  snapshot.active_wake_phrase =
    active_wake_event_.has_value() ?
    active_wake_event_->phrase :
    std::string{};

  snapshot.pending_speech_end =
    pending_speech_end_.has_value();

  snapshot.last_cancellation_reason =
    last_cancellation_reason_;

  snapshot.last_completion_reason =
    last_completion_reason_;

  snapshot.last_error = last_error_;

  return snapshot;
}

void UtteranceSessionCore::reset() noexcept
{
  const std::scoped_lock lock{mutex_};

  state_ = UtteranceSessionState::Idle;

  statistics_ =
    UtteranceSessionStatistics{};

  audio_format_.reset();
  pre_roll_buffer_.reset();
  pre_roll_spans_.clear();

  last_audio_sequence_.reset();
  last_audio_captured_at_.reset();
  last_advanced_time_.reset();

  clear_active_session_locked();

  completed_utterances_.clear();

  last_cancellation_reason_ =
    UtteranceCancellationReason::None;

  last_completion_reason_.reset();

  last_error_.clear();
}

void UtteranceSessionCore::initialize_audio_format_locked(
  const audio::AudioFormat & format)
{
  const std::uint64_t sample_rate =
    static_cast<std::uint64_t>(
    format.sample_rate_hz);

  const std::uint64_t milliseconds =
    static_cast<std::uint64_t>(
    config_.pre_roll_duration.count());

  const std::uint64_t capacity_samples =
    std::max<std::uint64_t>(
    1U,
    sample_rate * milliseconds / 1000U);

  if (
    capacity_samples >
    static_cast<std::uint64_t>(
      std::numeric_limits<std::size_t>::max()))
  {
    throw std::overflow_error{
            "pre-roll sample capacity exceeds size_t"};
  }

  audio_format_ = format;

  pre_roll_buffer_ =
    std::make_unique<audio::AudioRingBuffer>(
    static_cast<std::size_t>(
      capacity_samples));
}

bool UtteranceSessionCore::same_audio_format_locked(
  const audio::AudioFormat & format) const noexcept
{
  if (!audio_format_.has_value()) {
    return false;
  }

  return
    audio_format_->sample_rate_hz ==
    format.sample_rate_hz &&
    audio_format_->channels ==
    format.channels;
}

void UtteranceSessionCore::append_pre_roll_locked(
  const audio::AudioFrame & frame)
{
  if (!pre_roll_buffer_) {
    return;
  }

  const std::size_t capacity =
    pre_roll_buffer_->capacity();

  const std::size_t sample_count =
    frame.interleaved_samples.size();

  if (sample_count >= capacity) {
    pre_roll_spans_.clear();

    pre_roll_spans_.push_back(
      FrameSpan{
        frame.sequence,
        capacity,
        frame.captured_at});

    pre_roll_buffer_->append(
      frame.interleaved_samples);

    return;
  }

  const std::size_t current_size =
    pre_roll_buffer_->size();

  std::size_t overflow =
    current_size + sample_count > capacity ?
    current_size + sample_count - capacity :
    0U;

  while (
    overflow > 0U &&
    !pre_roll_spans_.empty())
  {
    if (
      pre_roll_spans_.front().sample_count <=
      overflow)
    {
      overflow -=
        pre_roll_spans_.front().sample_count;

      pre_roll_spans_.pop_front();
    } else {
      pre_roll_spans_.front().sample_count -=
        overflow;

      overflow = 0U;
    }
  }

  pre_roll_spans_.push_back(
    FrameSpan{
      frame.sequence,
      sample_count,
      frame.captured_at});

  pre_roll_buffer_->append(
    frame.interleaved_samples);
}

void UtteranceSessionCore::start_recording_locked(
  const vad::VadEvent & event)
{
  state_ = UtteranceSessionState::Recording;

  active_speech_start_ = event;
  pending_speech_end_.reset();

  active_utterance_id_ =
    statistics_.last_utterance_id + 1U;

  statistics_.last_utterance_id =
    active_utterance_id_;

  active_samples_ =
    pre_roll_buffer_ ?
    pre_roll_buffer_->snapshot() :
    std::vector<std::int16_t>{};

  active_spans_ = pre_roll_spans_;

  active_pre_roll_samples_ = 0U;

  for (const FrameSpan & span : active_spans_) {
    if (span.sequence < event.frame_sequence) {
      active_pre_roll_samples_ +=
        span.sample_count;
    }
  }

  ++statistics_.recordings_started;

  last_cancellation_reason_ =
    UtteranceCancellationReason::None;

  last_error_.clear();
}

void UtteranceSessionCore::append_active_frame_locked(
  const audio::AudioFrame & frame)
{
  active_samples_.insert(
    active_samples_.end(),
    frame.interleaved_samples.begin(),
    frame.interleaved_samples.end());

  active_spans_.push_back(
    FrameSpan{
      frame.sequence,
      frame.interleaved_samples.size(),
      frame.captured_at});
}

void UtteranceSessionCore::trim_active_after_sequence_locked(
  const std::uint64_t sequence)
{
  while (
    !active_spans_.empty() &&
    active_spans_.back().sequence > sequence)
  {
    const std::size_t remove_count =
      active_spans_.back().sample_count;

    if (remove_count >= active_samples_.size()) {
      active_samples_.clear();
    } else {
      active_samples_.resize(
        active_samples_.size() -
        remove_count);
    }

    active_spans_.pop_back();
  }
}

bool UtteranceSessionCore::finalize_locked(
  const UtteranceCompletionReason reason,
  const Clock::time_point completed_at,
  const std::uint64_t speech_end_frame_sequence)
{
  if (
    reason ==
    UtteranceCompletionReason::SpeechEnded)
  {
    trim_active_after_sequence_locked(
      speech_end_frame_sequence);
  }

  if (
    active_samples_.empty() ||
    active_spans_.empty() ||
    !audio_format_.has_value() ||
    !active_wake_event_.has_value() ||
    !active_speech_start_.has_value())
  {
    static_cast<void>(
      cancel_locked(
        UtteranceCancellationReason::
        NoAudioCaptured));

    return false;
  }

  CompletedUtterance utterance;

  utterance.utterance_id =
    active_utterance_id_;

  utterance.wake_event_id =
    active_wake_event_->event_id;

  utterance.wake_frame_sequence =
    active_wake_event_->frame_sequence;

  utterance.wake_detected_at =
    active_wake_event_->detected_at;

  utterance.wake_phrase =
    active_wake_event_->phrase;

  utterance.wake_confidence =
    active_wake_event_->confidence;

  utterance.vad_segment_id =
    active_speech_start_->segment_id;

  utterance.speech_start_frame_sequence =
    active_speech_start_->frame_sequence;

  utterance.speech_end_frame_sequence =
    speech_end_frame_sequence;

  utterance.speech_started_at =
    active_speech_start_->occurred_at;

  utterance.speech_ended_at =
    completed_at;

  utterance.first_audio_frame_sequence =
    active_spans_.front().sequence;

  utterance.last_audio_frame_sequence =
    active_spans_.back().sequence;

  utterance.audio_started_at =
    active_spans_.front().captured_at;

  utterance.audio_ended_at =
    active_spans_.back().captured_at;

  utterance.completed_at = completed_at;
  utterance.completion_reason = reason;

  utterance.audio.format =
    *audio_format_;

  utterance.audio.interleaved_samples =
    std::move(active_samples_);

  utterance.pre_roll_samples =
    std::min(
    active_pre_roll_samples_,
    utterance.audio.interleaved_samples.size());

  const auto [gap_count, missing_frames] =
    active_gap_metadata_locked();

  utterance.sequence_gap_count =
    gap_count;

  utterance.missing_audio_frames =
    missing_frames;

  ++statistics_.utterances_completed;

  if (
    reason ==
    UtteranceCompletionReason::SpeechEnded)
  {
    ++statistics_.speech_ended_completions;
  } else {
    ++statistics_.maximum_duration_completions;
  }

  last_completion_reason_ = reason;
  last_cancellation_reason_ =
    UtteranceCancellationReason::None;

  enqueue_completed_locked(
    std::move(utterance));

  clear_active_session_locked();

  return true;
}

bool UtteranceSessionCore::check_timeouts_locked(
  const Clock::time_point now)
{
  if (
    state_ == UtteranceSessionState::Armed &&
    active_wake_event_.has_value())
  {
    const auto deadline =
      active_wake_event_->detected_at +
      config_.speech_start_timeout;

    if (now >= deadline) {
      static_cast<void>(
        cancel_locked(
          UtteranceCancellationReason::
          SpeechStartTimeout));

      return false;
    }
  }

  if (
    state_ == UtteranceSessionState::Recording &&
    active_speech_start_.has_value())
  {
    const auto deadline =
      active_speech_start_->occurred_at +
      config_.maximum_utterance_duration;

    if (now >= deadline) {
      const std::uint64_t end_sequence =
        active_spans_.empty() ?
        active_speech_start_->frame_sequence :
        active_spans_.back().sequence;

      return finalize_locked(
        UtteranceCompletionReason::
        MaximumDurationReached,
        now,
        end_sequence);
    }
  }

  return false;
}

bool UtteranceSessionCore::cancel_locked(
  const UtteranceCancellationReason reason)
{
  if (state_ == UtteranceSessionState::Idle) {
    return false;
  }

  ++statistics_.sessions_canceled;

  switch (reason) {
    case UtteranceCancellationReason::
      SpeechStartTimeout:
      ++statistics_.speech_start_timeouts;
      break;

    case UtteranceCancellationReason::
      ExplicitCancellation:
      ++statistics_.explicit_cancellations;
      break;

    case UtteranceCancellationReason::
      NoAudioCaptured:
      ++statistics_.no_audio_cancellations;
      break;

    case UtteranceCancellationReason::None:
      break;
  }

  last_cancellation_reason_ = reason;

  clear_active_session_locked();

  return true;
}

void UtteranceSessionCore::clear_active_session_locked() noexcept
{
  state_ = UtteranceSessionState::Idle;

  active_wake_event_.reset();
  active_speech_start_.reset();
  pending_speech_end_.reset();

  active_utterance_id_ = 0U;

  active_samples_.clear();
  active_spans_.clear();

  active_pre_roll_samples_ = 0U;
}

void UtteranceSessionCore::enqueue_completed_locked(
  CompletedUtterance utterance)
{
  if (
    completed_utterances_.size() >=
    config_.completed_queue_capacity)
  {
    ++statistics_.completed_queue_overflows;
    ++statistics_.completed_utterances_dropped;

    completed_utterances_.pop_front();
  }

  completed_utterances_.push_back(
    std::move(utterance));
}

std::pair<std::uint64_t, std::uint64_t>
UtteranceSessionCore::active_gap_metadata_locked() const noexcept
{
  std::uint64_t gap_count{0U};
  std::uint64_t missing_frames{0U};

  if (active_spans_.size() < 2U) {
    return {gap_count, missing_frames};
  }

  auto previous = active_spans_.begin();
  auto current = previous;
  ++current;

  for (
    ;
    current != active_spans_.end();
    ++previous, ++current)
  {
    if (
      current->sequence >
      previous->sequence + 1U)
    {
      ++gap_count;

      missing_frames +=
        current->sequence -
        previous->sequence -
        1U;
    }
  }

  return {gap_count, missing_frames};
}

}  // namespace savo_speech::session
