#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

struct SAVO_LIDAR_PUBLIC LidarSample
{
  double angle_rad{0.0};
  float range_m{std::numeric_limits<float>::infinity()};
  float intensity{0.0F};
  bool valid{false};
};

struct SAVO_LIDAR_PUBLIC LidarScan
{
  std::string frame_id{"laser"};

  double angle_min_rad{-3.14159265358979323846};
  double angle_max_rad{3.14159265358979323846};
  double angle_increment_rad{0.0};

  double scan_time_s{0.0};
  double time_increment_s{0.0};

  float range_min_m{0.15F};
  float range_max_m{12.0F};

  std::uint64_t sequence{0};

  std::vector<float> ranges_m;
  std::vector<float> intensities;

  bool empty() const
  {
    return ranges_m.empty();
  }

  std::size_t size() const
  {
    return ranges_m.size();
  }

  bool has_intensities() const
  {
    return intensities.size() == ranges_m.size();
  }
};

struct SAVO_LIDAR_PUBLIC ScanStats
{
  std::size_t total_points{0};
  std::size_t valid_points{0};

  double valid_ratio{0.0};
  double scan_rate_hz{0.0};

  float min_range_m{std::numeric_limits<float>::quiet_NaN()};
  float max_range_m{std::numeric_limits<float>::quiet_NaN()};
  float mean_range_m{std::numeric_limits<float>::quiet_NaN()};
};

inline bool is_finite_range(float value)
{
  return value == value &&
         value != std::numeric_limits<float>::infinity() &&
         value != -std::numeric_limits<float>::infinity();
}

inline bool is_valid_range(float value, float min_range_m, float max_range_m)
{
  return is_finite_range(value) && value >= min_range_m && value <= max_range_m;
}

}  // namespace savo_lidar