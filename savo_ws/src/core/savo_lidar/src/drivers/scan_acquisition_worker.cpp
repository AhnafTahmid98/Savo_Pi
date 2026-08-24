#include "savo_lidar/scan_acquisition_worker.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace savo_lidar
{

ScanAcquisitionWorker::ScanAcquisitionWorker(
  AcquireCallback acquire,
  PublishCallback publish,
  ErrorCallback on_error,
  std::chrono::duration<double> retry_delay,
  CancelCallback cancel_acquire)
: acquire_(std::move(acquire)),
  publish_(std::move(publish)),
  on_error_(std::move(on_error)),
  cancel_acquire_(std::move(cancel_acquire)),
  retry_delay_(retry_delay)
{
  if (!acquire_ || !publish_ || !on_error_) {
    throw std::invalid_argument("scan acquisition callbacks must be set");
  }

  if (retry_delay_.count() < 0.0) {
    throw std::invalid_argument("scan acquisition retry delay cannot be negative");
  }

  if (retry_delay_.count() == 0.0) {
    retry_delay_ = std::chrono::milliseconds(1);
  }
}

ScanAcquisitionWorker::~ScanAcquisitionWorker()
{
  request_stop();
  join();
}

void ScanAcquisitionWorker::start()
{
  if (thread_.joinable()) {
    throw std::runtime_error("scan acquisition worker is already started");
  }

  stop_requested_.store(false);
  thread_ = std::thread([this]() {run();});
}

void ScanAcquisitionWorker::request_stop() noexcept
{
  const bool already_requested = stop_requested_.exchange(true);

  if (!already_requested && cancel_acquire_) {
    try {
      cancel_acquire_();
    } catch (...) {
    }
  }

  wait_condition_.notify_all();
}

void ScanAcquisitionWorker::join() noexcept
{
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool ScanAcquisitionWorker::running() const noexcept
{
  return running_.load();
}

bool ScanAcquisitionWorker::stop_requested() const noexcept
{
  return stop_requested_.load();
}

void ScanAcquisitionWorker::run() noexcept
{
  running_.store(true);

  while (!stop_requested()) {
    try {
      auto scan = acquire_();

      if (stop_requested()) {
        break;
      }

      publish_(scan);
    } catch (const std::exception & exc) {
      if (stop_requested() || !handle_error(exc.what()) || wait_for_retry()) {
        break;
      }
    } catch (...) {
      if (stop_requested() ||
        !handle_error("unknown scan acquisition error") ||
        wait_for_retry())
      {
        break;
      }
    }
  }

  running_.store(false);
}

bool ScanAcquisitionWorker::handle_error(const std::string & message) noexcept
{
  try {
    return on_error_(message);
  } catch (...) {
    return false;
  }
}

bool ScanAcquisitionWorker::wait_for_retry()
{
  std::unique_lock<std::mutex> lock(wait_mutex_);
  return wait_condition_.wait_for(
    lock,
    retry_delay_,
    [this]() {return stop_requested();});
}

}  // namespace savo_lidar
