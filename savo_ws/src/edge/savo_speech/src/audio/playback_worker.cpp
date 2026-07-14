#include "savo_speech/audio/playback_worker.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::audio
{

PlaybackWorker::PlaybackWorker(
  PlaybackSink & sink,
  MicrophoneGate & microphone_gate,
  const PlaybackWorkerConfig config)
: sink_{sink},
  microphone_gate_{microphone_gate},
  config_{config},
  controller_{microphone_gate_, config.chunk_frames}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid playback-worker configuration"};
  }
}

PlaybackWorker::~PlaybackWorker()
{
  try {
    stop();
  } catch (...) {
  }
}

bool PlaybackWorker::start()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == PlaybackWorkerState::Starting ||
      state_ == PlaybackWorkerState::Running ||
      state_ == PlaybackWorkerState::Stopping)
    {
      return false;
    }

    state_ = PlaybackWorkerState::Starting;
    accepting_requests_ = false;
    last_error_.clear();
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  try {
    sink_.open();

    if (!sink_.format().is_valid()) {
      throw std::runtime_error{
              "playback sink negotiated an invalid audio format"};
    }

    {
      const std::scoped_lock lock{mutex_};

      state_ = PlaybackWorkerState::Running;
      accepting_requests_ = true;

      ++statistics_.starts;
    }

    thread_ = std::jthread{
      [this](const std::stop_token stop_token) {
        run(stop_token);
      }};
  } catch (const std::exception & exception) {
    sink_.close();

    const std::scoped_lock lock{mutex_};

    state_ = PlaybackWorkerState::Faulted;
    accepting_requests_ = false;
    last_error_ = exception.what();

    ++statistics_.faults;

    throw;
  } catch (...) {
    sink_.close();

    const std::scoped_lock lock{mutex_};

    state_ = PlaybackWorkerState::Faulted;
    accepting_requests_ = false;
    last_error_ =
      "unknown playback-worker start failure";

    ++statistics_.faults;

    throw;
  }

  return true;
}

void PlaybackWorker::stop()
{
  std::deque<PlaybackRequest> cancelled_requests;

  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == PlaybackWorkerState::Stopped &&
      !thread_.joinable())
    {
      return;
    }

    state_ = PlaybackWorkerState::Stopping;
    accepting_requests_ = false;

    controller_.request_cancel();

    requests_.swap(cancelled_requests);

    statistics_.pending_cancellations +=
      static_cast<std::uint64_t>(
      cancelled_requests.size());

    statistics_.cancelled_requests +=
      static_cast<std::uint64_t>(
      cancelled_requests.size());
  }

  for (const auto & request : cancelled_requests) {
    PlaybackCompletion completion;

    completion.request_id = request.request_id;
    completion.status =
      PlaybackCompletionStatus::Cancelled;

    completion.error =
      "cancelled_before_playback_worker_stop";

    append_completion(std::move(completion));
  }

  if (thread_.joinable()) {
    thread_.request_stop();
    request_condition_.notify_all();
    thread_.join();
  }

  sink_.close();

  {
    const std::scoped_lock lock{mutex_};

    current_request_id_.reset();

    state_ = PlaybackWorkerState::Stopped;
    accepting_requests_ = false;

    ++statistics_.stops;
  }
}

PlaybackEnqueueResult PlaybackWorker::enqueue(
  PlaybackRequest request)
{
  if (
    request.request_id == 0U ||
    !request.audio.is_consistent())
  {
    const std::scoped_lock lock{mutex_};
    ++statistics_.enqueue_rejected;

    return PlaybackEnqueueResult::
           RejectedInvalidRequest;
  }

  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ != PlaybackWorkerState::Running ||
      !accepting_requests_)
    {
      ++statistics_.enqueue_rejected;

      return PlaybackEnqueueResult::
             RejectedNotRunning;
    }

    if (request.audio.format != sink_.format()) {
      ++statistics_.enqueue_rejected;

      return PlaybackEnqueueResult::
             RejectedFormatMismatch;
    }

    if (request_id_exists_locked(request.request_id)) {
      ++statistics_.enqueue_rejected;

      return PlaybackEnqueueResult::
             RejectedDuplicateId;
    }

    if (requests_.size() >= config_.queue_capacity) {
      ++statistics_.enqueue_rejected;

      return PlaybackEnqueueResult::
             RejectedQueueFull;
    }

    requests_.push_back(std::move(request));

    ++statistics_.enqueue_accepted;
  }

  request_condition_.notify_one();

  return PlaybackEnqueueResult::Accepted;
}

bool PlaybackWorker::cancel_current() noexcept
{
  const std::scoped_lock lock{mutex_};

  if (!current_request_id_.has_value()) {
    return false;
  }

  controller_.request_cancel();

  ++statistics_.cancel_current_requests;

  return true;
}

std::size_t PlaybackWorker::cancel_pending()
{
  std::deque<PlaybackRequest> cancelled_requests;

  {
    const std::scoped_lock lock{mutex_};

    requests_.swap(cancelled_requests);

    statistics_.pending_cancellations +=
      static_cast<std::uint64_t>(
      cancelled_requests.size());

    statistics_.cancelled_requests +=
      static_cast<std::uint64_t>(
      cancelled_requests.size());
  }

  for (const auto & request : cancelled_requests) {
    PlaybackCompletion completion;

    completion.request_id = request.request_id;
    completion.status =
      PlaybackCompletionStatus::Cancelled;

    completion.error =
      "cancelled_before_playback";

    append_completion(std::move(completion));
  }

  return cancelled_requests.size();
}

void PlaybackWorker::cancel_all()
{
  static_cast<void>(cancel_current());
  static_cast<void>(cancel_pending());
}

std::optional<PlaybackCompletion>
PlaybackWorker::try_pop_completion()
{
  const std::scoped_lock lock{mutex_};

  if (completions_.empty()) {
    return std::nullopt;
  }

  PlaybackCompletion completion =
    std::move(completions_.front());

  completions_.pop_front();

  return completion;
}

std::optional<PlaybackCompletion>
PlaybackWorker::wait_completion_for(
  const std::chrono::milliseconds timeout)
{
  std::unique_lock lock{mutex_};

  const bool ready = completion_condition_.wait_for(
    lock,
    timeout,
    [this]() {
      return !completions_.empty();
    });

  if (!ready || completions_.empty()) {
    return std::nullopt;
  }

  PlaybackCompletion completion =
    std::move(completions_.front());

  completions_.pop_front();

  return completion;
}

PlaybackWorkerState PlaybackWorker::state() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return state_;
}

bool PlaybackWorker::running() const noexcept
{
  return state() == PlaybackWorkerState::Running;
}

std::size_t PlaybackWorker::pending_requests() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return requests_.size();
}

std::optional<std::uint64_t>
PlaybackWorker::current_request_id() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return current_request_id_;
}

PlaybackWorkerStatistics
PlaybackWorker::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return statistics_;
}

std::string PlaybackWorker::last_error() const
{
  const std::scoped_lock lock{mutex_};
  return last_error_;
}

void PlaybackWorker::run(
  const std::stop_token stop_token)
{
  while (true) {
    PlaybackRequest request;

    {
      std::unique_lock lock{mutex_};

      request_condition_.wait(
        lock,
        [this, &stop_token]() {
          return
            stop_token.stop_requested() ||
            !requests_.empty();
        });

      if (
        stop_token.stop_requested() &&
        requests_.empty())
      {
        return;
      }

      if (requests_.empty()) {
        continue;
      }

      request = std::move(requests_.front());
      requests_.pop_front();

      controller_.clear_cancel();

      current_request_id_ = request.request_id;
    }

    PlaybackCompletion completion;
    completion.request_id = request.request_id;

    try {
      const PlaybackExecutionResult execution =
        controller_.play(request.audio, sink_);

      completion.frames_submitted =
        execution.frames_submitted;

      completion.chunks_submitted =
        execution.chunks_submitted;

      if (
        execution.outcome ==
        PlaybackOutcome::Completed)
      {
        completion.status =
          PlaybackCompletionStatus::Completed;
      } else {
        completion.status =
          PlaybackCompletionStatus::Cancelled;
      }
    } catch (const std::exception & exception) {
      completion.status =
        PlaybackCompletionStatus::Failed;

      completion.error = exception.what();

      std::deque<PlaybackRequest> cancelled_requests;

      {
        const std::scoped_lock lock{mutex_};

        current_request_id_.reset();

        state_ = PlaybackWorkerState::Faulted;
        accepting_requests_ = false;

        last_error_ = exception.what();

        ++statistics_.failed_requests;
        ++statistics_.faults;

        requests_.swap(cancelled_requests);

        statistics_.pending_cancellations +=
          static_cast<std::uint64_t>(
          cancelled_requests.size());

        statistics_.cancelled_requests +=
          static_cast<std::uint64_t>(
          cancelled_requests.size());
      }

      sink_.close();

      append_completion(std::move(completion));

      for (const auto & cancelled_request :
        cancelled_requests)
      {
        PlaybackCompletion cancelled_completion;

        cancelled_completion.request_id =
          cancelled_request.request_id;

        cancelled_completion.status =
          PlaybackCompletionStatus::Cancelled;

        cancelled_completion.error =
          "cancelled_after_playback_worker_fault";

        append_completion(
          std::move(cancelled_completion));
      }

      return;
    } catch (...) {
      completion.status =
        PlaybackCompletionStatus::Failed;

      completion.error =
        "unknown playback-worker runtime failure";

      {
        const std::scoped_lock lock{mutex_};

        current_request_id_.reset();

        state_ = PlaybackWorkerState::Faulted;
        accepting_requests_ = false;

        last_error_ = completion.error;

        ++statistics_.failed_requests;
        ++statistics_.faults;
      }

      sink_.close();
      append_completion(std::move(completion));

      return;
    }

    {
      const std::scoped_lock lock{mutex_};

      current_request_id_.reset();

      if (
        completion.status ==
        PlaybackCompletionStatus::Completed)
      {
        ++statistics_.completed_requests;
      } else {
        ++statistics_.cancelled_requests;
      }
    }

    append_completion(std::move(completion));
  }
}

void PlaybackWorker::append_completion(
  PlaybackCompletion completion)
{
  {
    const std::scoped_lock lock{mutex_};
    completions_.push_back(std::move(completion));
  }

  completion_condition_.notify_all();
}

bool PlaybackWorker::request_id_exists_locked(
  const std::uint64_t request_id) const
{
  if (
    current_request_id_.has_value() &&
    current_request_id_.value() == request_id)
  {
    return true;
  }

  return std::any_of(
    requests_.begin(),
    requests_.end(),
    [request_id](const PlaybackRequest & request) {
      return request.request_id == request_id;
    });
}

}  // namespace savo_speech::audio
