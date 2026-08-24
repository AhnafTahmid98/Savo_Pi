#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

// Runs one bounded blocking acquisition callback at a time and forwards every
// successful result exactly once. request_stop() interrupts retry backoff and
// invokes the cancellation callback to unblock an in-flight acquisition.
class SAVO_LIDAR_PUBLIC ScanAcquisitionWorker
{
public:
  using AcquireCallback = std::function<LidarScan()>;
  using PublishCallback = std::function<void(const LidarScan &)>;
  using ErrorCallback = std::function<bool(const std::string &)>;
  using CancelCallback = std::function<void()>;

  ScanAcquisitionWorker(
    AcquireCallback acquire,
    PublishCallback publish,
    ErrorCallback on_error,
    std::chrono::duration<double> retry_delay,
    CancelCallback cancel_acquire = {});
  ~ScanAcquisitionWorker();

  ScanAcquisitionWorker(const ScanAcquisitionWorker &) = delete;
  ScanAcquisitionWorker & operator=(const ScanAcquisitionWorker &) = delete;

  void start();
  void request_stop() noexcept;
  void join() noexcept;

  bool running() const noexcept;
  bool stop_requested() const noexcept;

private:
  void run() noexcept;
  bool handle_error(const std::string & message) noexcept;
  bool wait_for_retry();

  AcquireCallback acquire_;
  PublishCallback publish_;
  ErrorCallback on_error_;
  CancelCallback cancel_acquire_;
  std::chrono::duration<double> retry_delay_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread thread_;
};

}  // namespace savo_lidar
