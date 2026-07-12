#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace savo_mapping::simulation
{

inline constexpr double PI = 3.14159265358979323846;

struct SimulatedRoomScanConfig
{
  double half_width_m{2.50};
  double half_height_m{1.75};

  std::size_t sample_count{360};

  double range_min_m{0.15};
  double range_max_m{12.0};
};

std::string validate_simulated_room_scan_config(
  const SimulatedRoomScanConfig & config);

double scan_angle_min_rad();

double scan_angle_increment_rad(
  std::size_t sample_count);

double scan_angle_max_rad(
  std::size_t sample_count);

std::vector<float> generate_rectangular_room_scan(
  const SimulatedRoomScanConfig & config);

}  // namespace savo_mapping::simulation
