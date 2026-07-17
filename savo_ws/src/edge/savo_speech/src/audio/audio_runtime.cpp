#include "savo_speech/audio/audio_runtime.hpp"

#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::audio
{

AudioRuntime::AudioRuntime(
  CaptureSource & capture_source,
  PlaybackSink & playback_sink,
  AudioRuntimeConfig config)
: capture_source_{capture_source},
  playback_sink_{playback_sink},
  config_{std::move(config)},
  microphone_gate_{config_.microphone_gate},
  capture_pipeline_{
    config_.capture_pipeline,
    microphone_gate_},
  capture_worker_{
    capture_source_,
    capture_pipeline_},
  playback_worker_{
    playback_sink_,
    microphone_gate_,
    config_.playback_worker}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid audio-runtime configuration"};
  }

  // Keep captured frames blocked until both workers are healthy.
  microphone_gate_.set_shutdown_gate(true);
}

AudioRuntime::~AudioRuntime()
{
  stop();
}

bool AudioRuntime::start()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == AudioRuntimeState::Starting ||
      state_ == AudioRuntimeState::Running ||
      state_ == AudioRuntimeState::Stopping)
    {
      return false;
    }

    state_ = AudioRuntimeState::Starting;
    last_error_.clear();
  }

  microphone_gate_.set_shutdown_gate(true);
  capture_pipeline_.clear();

  while (
    playback_worker_.try_pop_completion().has_value())
  {
  }

  bool playback_started{false};

  try {
    const bool playback_result =
      playback_worker_.start();

    if (!playback_result) {
      throw std::runtime_error{
              "playback worker was already active"};
    }

    playback_started = true;

    const bool capture_result =
      capture_worker_.start();

    if (!capture_result) {
      throw std::runtime_error{
              "capture worker was already active"};
    }

    microphone_gate_.set_shutdown_gate(false);

    {
      const std::scoped_lock lock{mutex_};

      state_ = AudioRuntimeState::Running;
      ++statistics_.starts;
    }

    return true;
  } catch (const std::exception & exception) {
    capture_worker_.stop();

    try {
      if (playback_started) {
        playback_worker_.stop();
      }
    } catch (...) {
    }

    capture_pipeline_.clear();
    microphone_gate_.set_shutdown_gate(true);

    record_start_failure(
      exception.what(),
      playback_started);

    throw;
  } catch (...) {
    capture_worker_.stop();

    try {
      if (playback_started) {
        playback_worker_.stop();
      }
    } catch (...) {
    }

    capture_pipeline_.clear();
    microphone_gate_.set_shutdown_gate(true);

    record_start_failure(
      "unknown audio-runtime startup failure",
      playback_started);

    throw;
  }
}

void AudioRuntime::stop() noexcept
{
  {
    const std::scoped_lock lock{mutex_};

    if (state_ == AudioRuntimeState::Stopped) {
      return;
    }

    state_ = AudioRuntimeState::Stopping;
  }

  microphone_gate_.set_shutdown_gate(true);

  std::string shutdown_error;

  try {
    playback_worker_.stop();
  } catch (const std::exception & exception) {
    shutdown_error = exception.what();
  } catch (...) {
    shutdown_error =
      "unknown playback-worker shutdown failure";
  }

  capture_worker_.stop();
  capture_pipeline_.clear();

  {
    const std::scoped_lock lock{mutex_};

    ++statistics_.stops;

    if (shutdown_error.empty()) {
      state_ = AudioRuntimeState::Stopped;
    } else {
      state_ = AudioRuntimeState::Faulted;
      last_error_ = shutdown_error;
    }
  }
}

PlaybackEnqueueResult AudioRuntime::enqueue_playback(
  PlaybackRequest request)
{
  {
    const std::scoped_lock lock{mutex_};
    ++statistics_.playback_enqueue_attempts;
  }

  if (state() != AudioRuntimeState::Running) {
    const std::scoped_lock lock{mutex_};
    ++statistics_.playback_enqueue_rejected;

    return PlaybackEnqueueResult::RejectedNotRunning;
  }

  const PlaybackEnqueueResult result =
    playback_worker_.enqueue(std::move(request));

  {
    const std::scoped_lock lock{mutex_};

    if (result == PlaybackEnqueueResult::Accepted) {
      ++statistics_.playback_enqueue_accepted;
    } else {
      ++statistics_.playback_enqueue_rejected;
    }
  }

  return result;
}

bool AudioRuntime::cancel_current_playback() noexcept
{
  return playback_worker_.cancel_current();
}

std::size_t AudioRuntime::cancel_pending_playback()
{
  return playback_worker_.cancel_pending();
}

void AudioRuntime::cancel_all_playback()
{
  playback_worker_.cancel_all();
}

std::optional<PlaybackCompletion>
AudioRuntime::try_pop_playback_completion()
{
  return playback_worker_.try_pop_completion();
}

std::optional<PlaybackCompletion>
AudioRuntime::wait_playback_completion_for(
  const std::chrono::milliseconds timeout)
{
  return playback_worker_.wait_completion_for(timeout);
}

std::optional<AudioFrame>
AudioRuntime::try_pop_captured_frame()
{
  return capture_pipeline_.try_pop();
}

std::optional<AudioFrame>
AudioRuntime::wait_captured_frame_for(
  const std::chrono::milliseconds timeout)
{
  return capture_pipeline_.wait_pop_for(timeout);
}

AudioBuffer AudioRuntime::pre_roll_snapshot() const
{
  return capture_pipeline_.pre_roll_snapshot();
}

AudioRuntimeState AudioRuntime::state() const noexcept
{
  AudioRuntimeState configured_state;

  {
    const std::scoped_lock lock{mutex_};
    configured_state = state_;
  }

  if (configured_state != AudioRuntimeState::Running) {
    return configured_state;
  }

  const CaptureWorkerState capture_state =
    capture_worker_.state();

  const PlaybackWorkerState playback_state =
    playback_worker_.state();

  if (
    capture_state != CaptureWorkerState::Running ||
    playback_state != PlaybackWorkerState::Running)
  {
    return AudioRuntimeState::Faulted;
  }

  return AudioRuntimeState::Running;
}

bool AudioRuntime::ready() const noexcept
{
  return state() == AudioRuntimeState::Running;
}

AudioRuntimeStatistics
AudioRuntime::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return statistics_;
}

AudioRuntimeHealthSnapshot
AudioRuntime::health_snapshot() const
{
  AudioRuntimeHealthSnapshot snapshot;

  snapshot.state = state();

  snapshot.capture_worker_state =
    capture_worker_.state();

  snapshot.playback_worker_state =
    playback_worker_.state();

  snapshot.ready =
    snapshot.state == AudioRuntimeState::Running &&
    snapshot.capture_worker_state ==
    CaptureWorkerState::Running &&
    snapshot.playback_worker_state ==
    PlaybackWorkerState::Running;

  snapshot.microphone_gate =
    microphone_gate_.state();

  snapshot.capture_worker_statistics =
    capture_worker_.statistics();

  snapshot.capture_pipeline_statistics =
    capture_pipeline_.statistics();

  snapshot.capture_queue_statistics =
    capture_pipeline_.queue_statistics();

  snapshot.pre_roll_statistics =
    capture_pipeline_.pre_roll_statistics();

  snapshot.playback_worker_statistics =
    playback_worker_.statistics();

  snapshot.runtime_statistics = statistics();

  snapshot.pending_playback_requests =
    playback_worker_.pending_requests();

  snapshot.current_playback_request_id =
    playback_worker_.current_request_id();

  snapshot.last_error = last_error();

  return snapshot;
}

std::string AudioRuntime::last_error() const
{
  {
    const std::scoped_lock lock{mutex_};

    if (!last_error_.empty()) {
      return last_error_;
    }
  }

  const std::string capture_error =
    capture_worker_.last_error();

  if (!capture_error.empty()) {
    return capture_error;
  }

  const std::string playback_error =
    playback_worker_.last_error();

  if (!playback_error.empty()) {
    return playback_error;
  }

  if (state() == AudioRuntimeState::Faulted) {
    return "audio worker left the running state";
  }

  return {};
}

void AudioRuntime::record_start_failure(
  const std::string & error,
  const bool playback_was_started) noexcept
{
  const std::scoped_lock lock{mutex_};

  state_ = AudioRuntimeState::Faulted;
  last_error_ = error;

  ++statistics_.start_failures;

  if (playback_was_started) {
    ++statistics_.startup_rollbacks;
  }
}

}  // namespace savo_speech::audio
