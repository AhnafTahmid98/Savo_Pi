#pragma once

#include <cstddef>
#include <vector>

#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

inline constexpr std::size_t RPLIDAR_STANDARD_ANGLE_BIN_COUNT = 360U;

SAVO_LIDAR_PUBLIC double normalize_scan_angle_rad(double angle_rad);

// Map the native sensor angle into ROS convention. Inversion negates the
// native angle; the mounting offset is then added before normalization.
SAVO_LIDAR_PUBLIC double transform_scan_angle_rad(
  double sensor_angle_rad,
  bool inverted,
  double angle_offset_rad);

SAVO_LIDAR_PUBLIC LidarScan bin_scan_samples_by_angle(
  const std::vector<LidarSample> & samples,
  LidarScan scan,
  bool inverted,
  double angle_offset_rad,
  std::size_t bin_count = RPLIDAR_STANDARD_ANGLE_BIN_COUNT);

}  // namespace savo_lidar
