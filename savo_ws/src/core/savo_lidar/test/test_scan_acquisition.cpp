#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_lidar/rplidar_protocol.hpp"
#include "savo_lidar/scan_acquisition_worker.hpp"
#include "savo_lidar/scan_frame_assembler.hpp"
#include "savo_lidar/scan_types.hpp"

namespace
{

using namespace std::chrono_literals;

savo_lidar::RplidarMeasurement measurement(
  bool start_flag,
  double angle_rad,
  float range_m)
{
  savo_lidar::RplidarMeasurement value;
  value.start_flag = start_flag;
  value.quality_valid = true;
  value.quality = 20U;
  value.angle_rad = angle_rad;
  value.distance_m = range_m;
  value.valid = true;
  return value;
}

TEST(ScanFrameAssembler, CompletesEveryPhysicalRevolutionWithoutDroppingBoundary)
{
  savo_lidar::ScanFrameAssembler assembler;
  const auto start = savo_lidar::ScanFrameAssembler::TimePoint{};

  EXPECT_FALSE(assembler.add_measurement(
    measurement(false, -1.0, 9.0F), start, 900));
  EXPECT_FALSE(assembler.add_measurement(
    measurement(true, 0.0, 1.0F), start, 1000));
  EXPECT_FALSE(assembler.add_measurement(
    measurement(false, 0.5, 1.5F), start + 50ms, 1050));

  const auto first = assembler.add_measurement(
    measurement(true, 1.0, 2.0F),
    start + 145ms,
    2000);
  ASSERT_TRUE(first);
  ASSERT_EQ(first->samples.size(), 2U);
  EXPECT_DOUBLE_EQ(first->samples[0].angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(first->samples[1].angle_rad, 0.5);
  EXPECT_NEAR(first->scan_time_s, 0.145, 1.0e-12);
  EXPECT_EQ(first->ros_start_time_ns, 1000);

  EXPECT_FALSE(assembler.add_measurement(
    measurement(false, 1.5, 2.5F),
    start + 200ms,
    2050));
  const auto second = assembler.add_measurement(
    measurement(true, 2.0, 3.0F),
    start + 291ms,
    3000);
  ASSERT_TRUE(second);
  ASSERT_EQ(second->samples.size(), 2U);
  EXPECT_DOUBLE_EQ(second->samples[0].angle_rad, 1.0);
  EXPECT_DOUBLE_EQ(second->samples[1].angle_rad, 1.5);
  EXPECT_NEAR(second->scan_time_s, 0.146, 1.0e-12);
  EXPECT_EQ(second->ros_start_time_ns, 2000);
  EXPECT_LT(first->ros_start_time_ns, second->ros_start_time_ns);
}

TEST(ScanFrameAssembler, ResetRequiresAFreshPhysicalBoundary)
{
  savo_lidar::ScanFrameAssembler assembler;
  const auto start = savo_lidar::ScanFrameAssembler::TimePoint{};

  EXPECT_FALSE(assembler.add_measurement(
    measurement(true, 0.0, 1.0F), start, 100));
  assembler.reset();
  EXPECT_FALSE(assembler.add_measurement(
    measurement(false, 0.5, 1.5F), start + 10ms, 150));
  EXPECT_FALSE(assembler.add_measurement(
    measurement(true, 1.0, 2.0F), start + 20ms, 200));
  const auto completed = assembler.add_measurement(
    measurement(true, 2.0, 3.0F),
    start + 165ms,
    300);
  ASSERT_TRUE(completed);
  ASSERT_EQ(completed->samples.size(), 1U);
  EXPECT_DOUBLE_EQ(completed->samples[0].angle_rad, 1.0);
  EXPECT_EQ(completed->ros_start_time_ns, 200);
}

TEST(ScanFrameAssembler, TimingMetadataUsesPhysicalRevolutionPeriod)
{
  savo_lidar::LidarScan scan;
  scan.ranges_m.resize(360U);
  scan.intensities.resize(360U);

  savo_lidar::apply_scan_timing(scan, 0.145634);

  EXPECT_DOUBLE_EQ(scan.scan_time_s, 0.145634);
  EXPECT_NEAR(scan.time_increment_s, 0.145634 / 360.0, 1.0e-12);
  EXPECT_THROW(savo_lidar::apply_scan_timing(scan, 0.0), std::invalid_argument);
}

TEST(ScanAcquisitionWorker, PublishesEachCompletedScanExactlyOnce)
{
  std::atomic<std::uint64_t> next_sequence{0U};
  std::vector<std::uint64_t> published;
  std::atomic<int> errors{0};

  savo_lidar::ScanAcquisitionWorker worker(
    [&next_sequence]() {
      savo_lidar::LidarScan scan;
      scan.sequence = next_sequence.fetch_add(1U) + 1U;
      if (scan.sequence > 3U) {
        throw std::runtime_error("fake acquisition complete");
      }
      return scan;
    },
    [&published](const savo_lidar::LidarScan & scan) {
      published.push_back(scan.sequence);
    },
    [&errors](const std::string &) {
      errors.fetch_add(1);
      return false;
    },
    10ms);

  worker.start();
  worker.join();

  EXPECT_EQ(published, (std::vector<std::uint64_t>{1U, 2U, 3U}));
  EXPECT_EQ(errors.load(), 1);
  EXPECT_FALSE(worker.running());
}

TEST(ScanAcquisitionWorker, DoesNotPublishAReadCompletedAfterStopRequest)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool acquire_entered = false;
  bool release_acquire = false;
  std::atomic<int> publishes{0};

  savo_lidar::ScanAcquisitionWorker worker(
    [&]() {
      std::unique_lock<std::mutex> lock(mutex);
      acquire_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_acquire;});
      savo_lidar::LidarScan scan;
      scan.sequence = 1U;
      return scan;
    },
    [&publishes](const savo_lidar::LidarScan &) {publishes.fetch_add(1);},
    [](const std::string &) {return false;},
    10ms);

  worker.start();
  bool entered = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    entered = condition.wait_for(lock, 1s, [&]() {return acquire_entered;});
  }

  if (!entered) {
    worker.request_stop();
    {
      std::lock_guard<std::mutex> lock(mutex);
      release_acquire = true;
    }
    condition.notify_all();
    worker.join();
  }
  ASSERT_TRUE(entered);

  worker.request_stop();
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_acquire = true;
  }
  condition.notify_all();
  worker.join();

  EXPECT_EQ(publishes.load(), 0);
}

TEST(ScanAcquisitionWorker, StopCancelsBlockedAcquisitionAndJoinsPromptly)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool acquire_entered = false;
  bool cancelled = false;
  std::atomic<int> publishes{0};
  std::atomic<int> cancellations{0};

  savo_lidar::ScanAcquisitionWorker worker(
    [&]() -> savo_lidar::LidarScan {
      std::unique_lock<std::mutex> lock(mutex);
      acquire_entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return cancelled;});
      throw std::runtime_error("fake acquisition cancelled");
    },
    [&publishes](const savo_lidar::LidarScan &) {publishes.fetch_add(1);},
    [](const std::string &) {return true;},
    5s,
    [&]() {
      cancellations.fetch_add(1);
      std::lock_guard<std::mutex> lock(mutex);
      cancelled = true;
      condition.notify_all();
    });

  worker.start();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() {return acquire_entered;}));
  }

  const auto stop_started = std::chrono::steady_clock::now();
  worker.request_stop();
  worker.request_stop();
  worker.join();
  const auto stop_elapsed = std::chrono::steady_clock::now() - stop_started;

  EXPECT_LT(stop_elapsed, 500ms);
  EXPECT_EQ(publishes.load(), 0);
  EXPECT_EQ(cancellations.load(), 1);
  EXPECT_FALSE(worker.running());
}

TEST(ScanAcquisitionWorker, SupportsRepeatedCompletedStartAndStopCycles)
{
  std::atomic<int> acquisitions{0};

  savo_lidar::ScanAcquisitionWorker worker(
    [&]() -> savo_lidar::LidarScan {
      acquisitions.fetch_add(1);
      throw std::runtime_error("fake completed cycle");
    },
    [](const savo_lidar::LidarScan &) {},
    [](const std::string &) {return false;},
    10ms);

  worker.start();
  worker.join();
  worker.start();
  worker.join();

  EXPECT_EQ(acquisitions.load(), 2);
  EXPECT_FALSE(worker.running());
}

TEST(ScanAcquisitionWorker, StopInterruptsReconnectBackoff)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool error_seen = false;

  savo_lidar::ScanAcquisitionWorker worker(
    []() -> savo_lidar::LidarScan {throw std::runtime_error("fake timeout");},
    [](const savo_lidar::LidarScan &) {},
    [&](const std::string &) {
      std::lock_guard<std::mutex> lock(mutex);
      error_seen = true;
      condition.notify_all();
      return true;
    },
    5s);

  worker.start();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() {return error_seen;}));
  }

  const auto stop_started = std::chrono::steady_clock::now();
  worker.request_stop();
  worker.join();
  const auto stop_elapsed = std::chrono::steady_clock::now() - stop_started;

  EXPECT_LT(stop_elapsed, 500ms);
  EXPECT_FALSE(worker.running());
}

}  // namespace
