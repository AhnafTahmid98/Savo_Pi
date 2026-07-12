#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_mapping
{

struct MapQualityPlaceholder
{
  bool map_received{false};
  bool structurally_valid{false};
  bool evaluated{false};
  bool navigation_handoff_ready{false};

  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution_m{0.0};
  double map_area_m2{0.0};

  double free_ratio{0.0};
  double occupied_ratio{0.0};
  double unknown_ratio{0.0};
  double quality_score{0.0};

  std::uint64_t update_count{0};

  std::string reason{"waiting_for_map"};
};

MapQualityPlaceholder evaluate_map_quality_placeholder(
  double resolution_m,
  std::uint32_t width,
  std::uint32_t height,
  const std::vector<std::int8_t> & occupancy_data,
  std::uint64_t update_count,
  int free_threshold_percent,
  int occupied_threshold_percent);

std::string make_map_quality_json(
  const MapQualityPlaceholder & quality);

}  // namespace savo_mapping
