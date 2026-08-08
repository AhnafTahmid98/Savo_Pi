// Copyright 2026 Ahnaf Tahmid
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_speech/session/utterance_session_processor.hpp"

namespace savo_speech::session
{

namespace
{

struct DrainedEventCounts
{
  std::uint64_t wake_drained{0U};
  std::uint64_t wake_accepted{0U};
  std::uint64_t wake_rejected{0U};

  std::uint64_t vad_drained{0U};
  std::uint64_t vad_accepted{0U};
  std::uint64_t vad_rejected{0U};
};

}  // namespace

UtteranceSessionProcessor::UtteranceSessionProcessor(
  wake_word::WakeWordProcessor & wake_word_processor,
  vad::VadProcessor & vad_processor,
  UtteranceSessionConfig session_config)
: wake_word_processor_{wake_word_processor},
  vad_processor_{vad_processor},
  session_core_{std::move(session_config)}
{
}

void UtteranceSessionProcessor::process(
  const audio::AudioFrame & frame)
{
  const std::scoped_lock operation_lock{
    operation_mutex_};

  {
    const std::scoped_lock lock{
      statistics_mutex_};

    ++statistics_.frames_received;

    statistics_.last_frame_sequence =
      frame.sequence;
  }

  DrainedEventCounts counts;

  try {
    // The current frame must enter the pre-roll or recording
    // buffer before its wake-word and VAD events are consumed.
    session_core_.process_audio_frame(frame);

    while (true) {
      auto event =
        wake_word_processor_.try_pop_event();

      if (!event.has_value()) {
        break;
      }

      ++counts.wake_drained;

      if (
        session_core_.handle_wake_word_event(
          *event))
      {
        ++counts.wake_accepted;
      } else {
        ++counts.wake_rejected;
      }
    }

    while (true) {
      auto event =
        vad_processor_.try_pop_event();

      if (!event.has_value()) {
        break;
      }

      ++counts.vad_drained;

      if (
        session_core_.handle_vad_event(
          *event))
      {
        ++counts.vad_accepted;
      } else {
        ++counts.vad_rejected;
      }
    }

    // Events from this frame are intentionally handled before
    // evaluating a deadline at the same timestamp.
    session_core_.advance_time(
      frame.captured_at);
  } catch (const std::exception & exception) {
    const std::scoped_lock lock{
      statistics_mutex_};

    statistics_.wake_events_drained +=
      counts.wake_drained;

    statistics_.wake_events_accepted +=
      counts.wake_accepted;

    statistics_.wake_events_rejected +=
      counts.wake_rejected;

    statistics_.vad_events_drained +=
      counts.vad_drained;

    statistics_.vad_events_accepted +=
      counts.vad_accepted;

    statistics_.vad_events_rejected +=
      counts.vad_rejected;

    ++statistics_.frames_failed;

    last_error_ = exception.what();

    throw;
  } catch (...) {
    const std::string error =
      "utterance-session processor failed with "
      "an unknown exception";

    {
      const std::scoped_lock lock{
        statistics_mutex_};

      statistics_.wake_events_drained +=
        counts.wake_drained;

      statistics_.wake_events_accepted +=
        counts.wake_accepted;

      statistics_.wake_events_rejected +=
        counts.wake_rejected;

      statistics_.vad_events_drained +=
        counts.vad_drained;

      statistics_.vad_events_accepted +=
        counts.vad_accepted;

      statistics_.vad_events_rejected +=
        counts.vad_rejected;

      ++statistics_.frames_failed;

      last_error_ = error;
    }

    throw std::runtime_error{error};
  }

  const std::scoped_lock lock{
    statistics_mutex_};

  statistics_.wake_events_drained +=
    counts.wake_drained;

  statistics_.wake_events_accepted +=
    counts.wake_accepted;

  statistics_.wake_events_rejected +=
    counts.wake_rejected;

  statistics_.vad_events_drained +=
    counts.vad_drained;

  statistics_.vad_events_accepted +=
    counts.vad_accepted;

  statistics_.vad_events_rejected +=
    counts.vad_rejected;

  ++statistics_.frames_completed;

  last_error_.clear();
}

bool UtteranceSessionProcessor::cancel(
  const UtteranceCancellationReason reason)
{
  const std::scoped_lock lock{
    operation_mutex_};

  return session_core_.cancel(reason);
}

bool UtteranceSessionProcessor::begin_follow_up(
  const UtteranceSessionCore::Clock::time_point now)
{
  const std::scoped_lock lock{
    operation_mutex_};

  return session_core_.begin_follow_up(now);
}

std::optional<CompletedUtterance>
UtteranceSessionProcessor::try_pop_completed()
{
  return session_core_.try_pop_completed();
}

std::optional<CompletedUtterance>
UtteranceSessionProcessor::wait_completed_for(
  const std::chrono::milliseconds timeout)
{
  return session_core_.wait_completed_for(
    timeout);
}

UtteranceSessionProcessorSnapshot
UtteranceSessionProcessor::snapshot() const
{
  UtteranceSessionProcessorSnapshot snapshot;

  {
    const std::scoped_lock lock{
      statistics_mutex_};

    snapshot.statistics = statistics_;
    snapshot.last_error = last_error_;
  }

  snapshot.session =
    session_core_.snapshot();

  snapshot.pending_wake_events =
    wake_word_processor_.snapshot().
    queued_events;

  snapshot.pending_vad_events =
    vad_processor_.snapshot().
    queued_events;

  return snapshot;
}

void UtteranceSessionProcessor::reset() noexcept
{
  const std::scoped_lock operation_lock{
    operation_mutex_};

  // Discard queued events so detections produced before the reset
  // cannot arm a new post-reset session.
  while (
    wake_word_processor_.
    try_pop_event().has_value())
  {
  }

  while (
    vad_processor_.
    try_pop_event().has_value())
  {
  }

  session_core_.reset();

  const std::scoped_lock statistics_lock{
    statistics_mutex_};

  statistics_ =
    UtteranceSessionProcessorStatistics{};

  last_error_.clear();
}

}  // namespace savo_speech::session
