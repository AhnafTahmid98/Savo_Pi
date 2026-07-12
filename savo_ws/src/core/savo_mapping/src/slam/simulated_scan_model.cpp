#include "savo_mapping/simulated_scan_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace savo_mapping::simulation
{

std::string validate_simulated_room_scan_config(
  const SimulatedRoomScanConfig & config)
{
  if (!std::isfinite(config.half_width_m) ||
      config.half_width_m <= 0.0)
  {
    return "half_width_m_must_be_positive";
  }

  if (!std::isfinite(config.half_height_m) ||
      config.half_height_m <= 0.0)
  {
    return "half_height_m_must_be_positive";
  }

  if (config.sample_count < 36 ||
      config.sample_count > 1440)
  {
    return "sample_count_outside_supported_range";
  }

  if (!std::isfinite(config.range_min_m) ||
      config.range_min_m <= 0.0)
  {
    return "range_min_m_must_be_positive";
  }

  if (!std::isfinite(config.range_max_m) ||
      config.range_max_m <= config.range_min_m)
  {
    return "range_max_m_must_exceed_range_min_m";
  }

  if (config.range_min_m >=
      std::min(config.half_width_m, config.half_height_m))
  {
    return "room_walls_are_below_range_min";
  }

  const double farthest_corner_m = std::hypot(
    config.half_width_m,
    config.half_height_m);

  if (farthest_corner_m > config.range_max_m) {
    return "room_exceeds_range_max";
  }

  return {};
}

double scan_angle_min_rad()
{
  return -PI;
}

double scan_angle_increment_rad(
  std::size_t sample_count)
{
  if (sample_count == 0) {
    throw std::invalid_argument(
            "sample_count must be greater than zero");
  }

  return (2.0 * PI) /
         static_cast<double>(sample_count);
}

double scan_angle_max_rad(
  std::size_t sample_count)
{
  return scan_angle_min_rad() +
         scan_angle_increment_rad(sample_count) *
         static_cast<double>(sample_count - 1);
}

std::vector<float> generate_rectangular_room_scan(
  const SimulatedRoomScanConfig & config)
{
  const std::string error =
    validate_simulated_room_scan_config(config);

  if (!error.empty()) {
    throw std::invalid_argument(
            "invalid simulated scan configuration: " + error);
  }

  const double angle_increment =
    scan_angle_increment_rad(config.sample_count);

  std::vector<float> ranges;
  ranges.reserve(config.sample_count);

  constexpr double epsilon = 1.0e-12;
  const double infinity =
    std::numeric_limits<double>::infinity();

  for (std::size_t index = 0;
       index < config.sample_count;
       ++index)
  {
    const double angle =
      scan_angle_min_rad() +
      angle_increment * static_cast<double>(index);

    const double direction_x = std::cos(angle);
    const double direction_y = std::sin(angle);

    const double distance_to_vertical_wall =
      std::abs(direction_x) > epsilon ?
      config.half_width_m / std::abs(direction_x) :
      infinity;

    const double distance_to_horizontal_wall =
      std::abs(direction_y) > epsilon ?
      config.half_height_m / std::abs(direction_y) :
      infinity;

    const double distance = std::min(
      distance_to_vertical_wall,
      distance_to_horizontal_wall);

    if (!std::isfinite(distance) ||
        distance < config.range_min_m ||
        distance > config.range_max_m)
    {
      ranges.push_back(
        std::numeric_limits<float>::infinity());
      continue;
    }

    ranges.push_back(static_cast<float>(distance));
  }

  return ranges;
}

}  // namespace savo_mapping::simulation
