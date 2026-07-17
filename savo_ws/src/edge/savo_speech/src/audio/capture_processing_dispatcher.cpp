#include "savo_speech/audio/capture_processing_dispatcher.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::audio
{

CaptureProcessingDispatcher::
CaptureProcessingDispatcher(
  CapturedFrameSource & source,
  CapturedAudioProcessor & processor,
  const CaptureProcessingConfig config)
: source_{source},
  processor_{processor},
  config_{config}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid capture-processing configuration"};
  }
}

CaptureProcessingDispatcher::
~CaptureProcessingDispatcher()
{
  stop();
}

bool CaptureProcessingDispatcher::start()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == CaptureProcessingState::Starting ||
      state_ == CaptureProcessingState::Running ||
      state_ == CaptureProcessingState::Stopping)
    {
      return false;
    }

    state_ = CaptureProcessingState::Starting;
    last_error_.clear();
    last_processed_at_.reset();
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  try {
    {
      const std::scoped_lock lock{mutex_};

      state_ = CaptureProcessingState::Running;
      ++statistics_.starts;
    }

    thread_ = std::jthread{
      [this](const std::stop_token stop_token) {
        run(stop_token);
      }};
  } catch (const std::exception & exception) {
    record_source_failure(exception.what());
    throw;
  } catch (...) {
    record_source_failure(
      "unknown capture-processing startup failure");

    throw;
  }

  return true;
}

void CaptureProcessingDispatcher::stop() noexcept
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == CaptureProcessingState::Stopped &&
      !thread_.joinable())
    {
      return;
    }

    if (state_ != CaptureProcessingState::Faulted) {
      state_ = CaptureProcessingState::Stopping;
    }
  }

  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }

  {
    const std::scoped_lock lock{mutex_};

    state_ = CaptureProcessingState::Stopped;
    ++statistics_.stops;
  }
}

CaptureProcessingState
CaptureProcessingDispatcher::state() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return state_;
}

bool CaptureProcessingDispatcher::running() const noexcept
{
  return state() == CaptureProcessingState::Running;
}

CaptureProcessingStatistics
CaptureProcessingDispatcher::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return statistics_;
}

CaptureProcessingHealthSnapshot
CaptureProcessingDispatcher::health_snapshot() const
{
  const auto now = Clock::now();

  const std::scoped_lock lock{mutex_};

  CaptureProcessingHealthSnapshot snapshot;

  snapshot.state = state_;
  snapshot.statistics = statistics_;
  snapshot.last_error = last_error_;

  if (last_processed_at_.has_value()) {
    const auto age =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_processed_at_.value());

    snapshot.last_frame_age_ms = age.count();

    snapshot.fresh =
      age <= config_.freshness_timeout;
  }

  snapshot.ready =
    state_ == CaptureProcessingState::Running &&
    statistics_.frames_processed > 0U &&
    snapshot.fresh;

  return snapshot;
}

std::string
CaptureProcessingDispatcher::last_error() const
{
  const std::scoped_lock lock{mutex_};
  return last_error_;
}

void CaptureProcessingDispatcher::run(
  const std::stop_token stop_token)
{
  while (!stop_token.stop_requested()) {
    std::optional<AudioFrame> frame;

    try {
      frame = source_.wait_captured_frame_for(
        config_.wait_timeout);
    } catch (const std::exception & exception) {
      if (stop_token.stop_requested()) {
        return;
      }

      record_source_failure(exception.what());
      return;
    } catch (...) {
      if (stop_token.stop_requested()) {
        return;
      }

      record_source_failure(
        "unknown captured-frame source failure");

      return;
    }

    if (!frame.has_value()) {
      const std::scoped_lock lock{mutex_};
      ++statistics_.wait_timeouts;
      continue;
    }

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_received;

      if (statistics_.last_sequence != 0U) {
        if (
          frame->sequence <=
          statistics_.last_sequence)
        {
          ++statistics_.out_of_order_frames;
        } else if (
          frame->sequence >
          statistics_.last_sequence + 1U)
        {
          statistics_.missing_sequence_frames +=
            frame->sequence -
            statistics_.last_sequence -
            1U;
        }
      }

      statistics_.last_sequence = frame->sequence;
    }

    try {
      processor_.process(frame.value());

      const std::scoped_lock lock{mutex_};

      ++statistics_.frames_processed;
      last_processed_at_ = Clock::now();
    } catch (const std::exception & exception) {
      record_processor_failure(exception.what());

      if (config_.fault_on_processor_error) {
        return;
      }
    } catch (...) {
      record_processor_failure(
        "unknown captured-audio processor failure");

      if (config_.fault_on_processor_error) {
        return;
      }
    }
  }
}

void CaptureProcessingDispatcher::record_source_failure(
  const std::string & error) noexcept
{
  const std::scoped_lock lock{mutex_};

  state_ = CaptureProcessingState::Faulted;
  last_error_ = error;

  ++statistics_.source_failures;
  ++statistics_.faults;
}

void CaptureProcessingDispatcher::
record_processor_failure(
  const std::string & error) noexcept
{
  const std::scoped_lock lock{mutex_};

  last_error_ = error;

  ++statistics_.processor_failures;

  if (config_.fault_on_processor_error) {
    state_ = CaptureProcessingState::Faulted;
    ++statistics_.faults;
  }
}

}  // namespace savo_speech::audio
