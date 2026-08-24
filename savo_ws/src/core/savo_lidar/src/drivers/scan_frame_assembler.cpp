#include "savo_lidar/scan_frame_assembler.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_lidar
{

std::optional<CompletedScanFrame> ScanFrameAssembler::add_measurement(
  const RplidarMeasurement & measurement,
  TimePoint received_at,
  std::int64_t ros_received_time_ns)
{
  if (!measurement.valid) {
    return std::nullopt;
  }

  std::optional<CompletedScanFrame> completed;

  if (measurement.start_flag) {
    // A start sample closes the preceding revolution and is also the first
    // sample of the next one, so retain it after returning the completed frame.
    if (synchronized_ && !current_samples_.empty()) {
      if (!ros_scan_start_time_ns_) {
        throw std::logic_error("synchronized scan frame has no ROS start timestamp");
      }

      completed.emplace();
      completed->samples = std::move(current_samples_);
      completed->scan_time_s =
        std::chrono::duration<double>(received_at - scan_start_time_).count();
      completed->ros_start_time_ns = *ros_scan_start_time_ns_;
      current_samples_.clear();
    }

    synchronized_ = true;
    scan_start_time_ = received_at;
    ros_scan_start_time_ns_ = ros_received_time_ns;
  }

  if (synchronized_) {
    current_samples_.push_back(to_sample(measurement));
  }

  return completed;
}

void ScanFrameAssembler::reset() noexcept
{
  synchronized_ = false;
  scan_start_time_ = TimePoint{};
  ros_scan_start_time_ns_.reset();
  current_samples_.clear();
}

LidarSample ScanFrameAssembler::to_sample(const RplidarMeasurement & measurement)
{
  LidarSample sample;
  sample.angle_rad = measurement.angle_rad;
  sample.range_m = measurement.distance_m;
  sample.intensity = static_cast<float>(measurement.quality);
  sample.valid = measurement.valid;
  return sample;
}

void apply_scan_timing(LidarScan & scan, double scan_time_s)
{
  if (!std::isfinite(scan_time_s) || scan_time_s <= 0.0) {
    throw std::invalid_argument("scan_time_s must be finite and > 0");
  }

  scan.scan_time_s = scan_time_s;
  scan.time_increment_s = scan.size() > 1U ?
    scan_time_s / static_cast<double>(scan.size()) : 0.0;
}

}  // namespace savo_lidar
