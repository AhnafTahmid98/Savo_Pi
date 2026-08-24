#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "savo_lidar/rplidar_protocol.hpp"
#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

struct SAVO_LIDAR_PUBLIC CompletedScanFrame
{
  std::vector<LidarSample> samples;
  double scan_time_s{0.0};
  std::int64_t ros_start_time_ns{0};
};

class SAVO_LIDAR_PUBLIC ScanFrameAssembler
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  std::optional<CompletedScanFrame> add_measurement(
    const RplidarMeasurement & measurement,
    TimePoint received_at,
    std::int64_t ros_received_time_ns);

  void reset() noexcept;

private:
  static LidarSample to_sample(const RplidarMeasurement & measurement);

  bool synchronized_{false};
  TimePoint scan_start_time_{};
  std::optional<std::int64_t> ros_scan_start_time_ns_;
  std::vector<LidarSample> current_samples_;
};

SAVO_LIDAR_PUBLIC void apply_scan_timing(
  LidarScan & scan,
  double scan_time_s);

}  // namespace savo_lidar
