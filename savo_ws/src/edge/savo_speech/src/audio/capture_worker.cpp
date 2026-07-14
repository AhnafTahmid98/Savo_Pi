#include "savo_speech/audio/capture_worker.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::audio
{

CaptureWorker::CaptureWorker(
  CaptureSource & source,
  CapturePipeline & pipeline)
: source_{source},
  pipeline_{pipeline}
{
}

CaptureWorker::~CaptureWorker()
{
  stop();
}

bool CaptureWorker::start()
{
  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ == CaptureWorkerState::Starting ||
      state_ == CaptureWorkerState::Running ||
      state_ == CaptureWorkerState::Stopping)
    {
      return false;
    }

    state_ = CaptureWorkerState::Starting;
    last_error_.clear();
  }

  // A completed faulted jthread remains joinable until joined.
  if (thread_.joinable()) {
    thread_.join();
  }

  try {
    pipeline_.clear();
    source_.open();

    {
      const std::scoped_lock lock{mutex_};

      state_ = CaptureWorkerState::Running;
      ++statistics_.starts;
    }

    thread_ = std::jthread{
      [this](const std::stop_token stop_token) {
        run(stop_token);
      }};
  } catch (const std::exception & exception) {
    source_.close();
    record_fault(exception.what());
    throw;
  } catch (...) {
    source_.close();
    record_fault("unknown capture-worker start failure");
    throw;
  }

  return true;
}

void CaptureWorker::stop() noexcept
{
  bool should_stop{false};

  {
    const std::scoped_lock lock{mutex_};

    if (
      state_ != CaptureWorkerState::Stopped ||
      thread_.joinable())
    {
      should_stop = true;

      if (state_ != CaptureWorkerState::Faulted) {
        state_ = CaptureWorkerState::Stopping;
      }
    }
  }

  if (!should_stop) {
    return;
  }

  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }

  source_.close();
  pipeline_.clear();

  {
    const std::scoped_lock lock{mutex_};

    state_ = CaptureWorkerState::Stopped;
    ++statistics_.stops;
  }
}

CaptureWorkerState CaptureWorker::state() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return state_;
}

bool CaptureWorker::running() const noexcept
{
  return state() == CaptureWorkerState::Running;
}

CaptureWorkerStatistics
CaptureWorker::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return statistics_;
}

std::string CaptureWorker::last_error() const
{
  const std::scoped_lock lock{mutex_};
  return last_error_;
}

void CaptureWorker::run(
  const std::stop_token stop_token)
{
  while (!stop_token.stop_requested()) {
    try {
      AudioFrame frame = source_.read_frame();

      {
        const std::scoped_lock lock{mutex_};
        ++statistics_.frames_read;
      }

      const CapturePipelineResult result =
        pipeline_.process(std::move(frame));

      record_pipeline_result(result);
    } catch (const std::exception & exception) {
      if (stop_token.stop_requested()) {
        return;
      }

      source_.close();
      record_fault(exception.what());
      return;
    } catch (...) {
      if (stop_token.stop_requested()) {
        return;
      }

      source_.close();
      record_fault("unknown capture-worker runtime failure");
      return;
    }
  }
}

void CaptureWorker::record_pipeline_result(
  const CapturePipelineResult result)
{
  const std::scoped_lock lock{mutex_};

  switch (result) {
    case CapturePipelineResult::Accepted:
      ++statistics_.accepted_frames;
      return;

    case CapturePipelineResult::AcceptedAfterDroppingOldest:
      ++statistics_.accepted_frames;
      ++statistics_.queue_drop_events;
      return;

    case CapturePipelineResult::DroppedWhileGated:
      ++statistics_.gated_frames;
      return;

    case CapturePipelineResult::RejectedInvalidFrame:
    case CapturePipelineResult::RejectedChannelUnavailable:
    case CapturePipelineResult::RejectedQueueFull:
    case CapturePipelineResult::RejectedQueueClosed:
      ++statistics_.rejected_frames;
      return;
  }

  ++statistics_.rejected_frames;
}

void CaptureWorker::record_fault(
  const std::string & message) noexcept
{
  const std::scoped_lock lock{mutex_};

  state_ = CaptureWorkerState::Faulted;
  last_error_ = message;

  ++statistics_.faults;
}

}  // namespace savo_speech::audio
