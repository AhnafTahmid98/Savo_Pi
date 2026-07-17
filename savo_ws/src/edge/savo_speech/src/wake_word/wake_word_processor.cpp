#include "savo_speech/wake_word/wake_word_processor.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::wake_word
{

bool WakeWordProcessorConfig::is_valid() const noexcept
{
  return
    std::isfinite(confidence_threshold) &&
    confidence_threshold >= 0.0 &&
    confidence_threshold <= 1.0 &&
    required_consecutive_detections >= 1U &&
    required_consecutive_detections <= 100U &&
    cooldown >= std::chrono::milliseconds{0} &&
    cooldown <= std::chrono::seconds{30} &&
    event_queue_capacity >= 1U &&
    event_queue_capacity <= 1024U;
}

WakeWordProcessor::WakeWordProcessor(
  WakeWordBackend & backend,
  WakeWordProcessorConfig config)
: backend_{backend},
  config_{std::move(config)}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid wake-word processor configuration"};
  }
}

void WakeWordProcessor::process(
  const audio::AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    throw std::invalid_argument{
            "wake-word processor received an invalid frame"};
  }

  if (frame.format.channels != 1U) {
    throw std::invalid_argument{
            "wake-word processor requires mono audio"};
  }

  WakeWordBackendResult result;

  try {
    result = backend_.analyze(frame);
  } catch (const std::exception & exception) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      ++statistics_.backend_failures;

      statistics_.last_frame_sequence =
        frame.sequence;

      last_error_ =
        "wake-word backend failed: " +
        std::string{exception.what()};

      reset_candidate_locked();
    }

    throw std::runtime_error{last_error_};
  } catch (...) {
    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      ++statistics_.backend_failures;

      statistics_.last_frame_sequence =
        frame.sequence;

      last_error_ =
        "wake-word backend failed with an "
        "unknown exception";

      reset_candidate_locked();
    }

    throw std::runtime_error{last_error_};
  }

  std::unique_lock lock{mutex_};

  ++statistics_.frames_processed;

  statistics_.last_frame_sequence =
    frame.sequence;

  if (!result.detected) {
    if (consecutive_detections_ > 0U) {
      ++statistics_.debounce_resets;
    }

    reset_candidate_locked();
    return;
  }

  ++statistics_.backend_detections;

  if (
    result.phrase.empty() ||
    !std::isfinite(result.confidence))
  {
    last_error_ =
      "wake-word backend returned an invalid detection";

    reset_candidate_locked();

    throw std::runtime_error{last_error_};
  }

  if (result.confidence < config_.confidence_threshold) {
    ++statistics_.below_threshold;

    if (consecutive_detections_ > 0U) {
      ++statistics_.debounce_resets;
    }

    reset_candidate_locked();
    return;
  }

  if (
    candidate_phrase_.empty() ||
    candidate_phrase_ != result.phrase)
  {
    if (consecutive_detections_ > 0U) {
      ++statistics_.debounce_resets;
    }

    candidate_phrase_ = result.phrase;
    candidate_confidence_ = result.confidence;
    consecutive_detections_ = 1U;
  } else {
    ++consecutive_detections_;

    if (result.confidence > candidate_confidence_) {
      candidate_confidence_ = result.confidence;
    }
  }

  if (
    consecutive_detections_ <
    config_.required_consecutive_detections)
  {
    return;
  }

  if (
    last_accepted_at_.has_value() &&
    frame.captured_at <
    last_accepted_at_.value() + config_.cooldown)
  {
    ++statistics_.cooldown_suppressed;

    reset_candidate_locked();
    return;
  }

  WakeWordEvent event;

  event.event_id =
    statistics_.last_event_id + 1U;

  event.frame_sequence = frame.sequence;
  event.detected_at = frame.captured_at;

  event.phrase = candidate_phrase_;
  event.confidence = candidate_confidence_;

  enqueue_event_locked(event);

  ++statistics_.accepted_detections;

  statistics_.last_event_id =
    event.event_id;

  last_accepted_at_ = event.detected_at;

  last_detected_phrase_ = event.phrase;
  last_detected_confidence_ = event.confidence;

  last_error_.clear();

  reset_candidate_locked();

  lock.unlock();
  condition_.notify_one();
}

std::optional<WakeWordEvent>
WakeWordProcessor::try_pop_event()
{
  const std::scoped_lock lock{mutex_};

  if (events_.empty()) {
    return std::nullopt;
  }

  WakeWordEvent event =
    std::move(events_.front());

  events_.pop_front();

  return event;
}

std::optional<WakeWordEvent>
WakeWordProcessor::wait_event_for(
  const std::chrono::milliseconds timeout)
{
  if (timeout < std::chrono::milliseconds{0}) {
    throw std::invalid_argument{
            "wake-word event wait timeout must not "
            "be negative"};
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

  WakeWordEvent event =
    std::move(events_.front());

  events_.pop_front();

  return event;
}

WakeWordProcessorSnapshot
WakeWordProcessor::snapshot() const
{
  const std::scoped_lock lock{mutex_};

  WakeWordProcessorSnapshot snapshot;

  snapshot.statistics = statistics_;

  snapshot.queued_events = events_.size();

  snapshot.consecutive_detections =
    consecutive_detections_;

  snapshot.candidate_phrase =
    candidate_phrase_;

  snapshot.candidate_confidence =
    candidate_confidence_;

  snapshot.last_detected_phrase =
    last_detected_phrase_;

  snapshot.last_detected_confidence =
    last_detected_confidence_;

  snapshot.last_error = last_error_;

  return snapshot;
}

void WakeWordProcessor::reset() noexcept
{
  {
    const std::scoped_lock lock{mutex_};

    events_.clear();

    statistics_ =
      WakeWordProcessorStatistics{};

    last_accepted_at_.reset();

    last_detected_phrase_.clear();
    last_detected_confidence_ = 0.0;

    last_error_.clear();

    reset_candidate_locked();
  }

  backend_.reset();
}

void WakeWordProcessor::reset_candidate_locked() noexcept
{
  consecutive_detections_ = 0U;

  candidate_phrase_.clear();
  candidate_confidence_ = 0.0;
}

void WakeWordProcessor::enqueue_event_locked(
  WakeWordEvent event)
{
  if (events_.size() >= config_.event_queue_capacity) {
    ++statistics_.queue_overflows;
    ++statistics_.events_dropped;

    events_.pop_front();
  }

  events_.push_back(std::move(event));
}

}  // namespace savo_speech::wake_word
