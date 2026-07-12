#include "savo_mapping/map_quality_placeholder.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace savo_mapping
{

namespace
{

MapQualityPlaceholder make_invalid_result(
  MapQualityPlaceholder result,
  const std::string & reason)
{
  result.structurally_valid = false;
  result.evaluated = false;
  result.navigation_handoff_ready = false;
  result.quality_score = 0.0;
  result.reason = reason;
  return result;
}

const char * bool_text(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

MapQualityPlaceholder evaluate_map_quality_placeholder(
  double resolution_m,
  std::uint32_t width,
  std::uint32_t height,
  const std::vector<std::int8_t> & occupancy_data,
  std::uint64_t update_count,
  int free_threshold_percent,
  int occupied_threshold_percent)
{
  MapQualityPlaceholder result;

  result.map_received = true;
  result.width = width;
  result.height = height;
  result.resolution_m = resolution_m;
  result.update_count = update_count;

  if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
    return make_invalid_result(
      result,
      "invalid_resolution");
  }

  if (width == 0 || height == 0) {
    return make_invalid_result(
      result,
      "empty_dimensions");
  }

  if (free_threshold_percent < 0 ||
      free_threshold_percent > 100 ||
      occupied_threshold_percent < 0 ||
      occupied_threshold_percent > 100 ||
      free_threshold_percent >= occupied_threshold_percent)
  {
    return make_invalid_result(
      result,
      "invalid_classification_thresholds");
  }

  const std::uint64_t expected_cell_count =
    static_cast<std::uint64_t>(width) *
    static_cast<std::uint64_t>(height);

  if (expected_cell_count >
      static_cast<std::uint64_t>(
        std::numeric_limits<std::size_t>::max()))
  {
    return make_invalid_result(
      result,
      "cell_count_overflow");
  }

  if (occupancy_data.size() !=
      static_cast<std::size_t>(expected_cell_count))
  {
    return make_invalid_result(
      result,
      "cell_count_mismatch");
  }

  std::uint64_t free_count = 0;
  std::uint64_t occupied_count = 0;
  std::uint64_t unknown_count = 0;

  for (const std::int8_t value : occupancy_data) {
    if (value == -1) {
      ++unknown_count;
      continue;
    }

    if (value < 0 || value > 100) {
      return make_invalid_result(
        result,
        "invalid_occupancy_value");
    }

    if (value <= free_threshold_percent) {
      ++free_count;
    } else if (value >= occupied_threshold_percent) {
      ++occupied_count;
    }
  }

  const double cell_count =
    static_cast<double>(expected_cell_count);

  result.map_area_m2 =
    cell_count * resolution_m * resolution_m;

  result.free_ratio =
    static_cast<double>(free_count) / cell_count;

  result.occupied_ratio =
    static_cast<double>(occupied_count) / cell_count;

  result.unknown_ratio =
    static_cast<double>(unknown_count) / cell_count;

  result.structurally_valid = true;

  // Real map-quality scoring is intentionally not implemented yet.
  result.evaluated = false;
  result.navigation_handoff_ready = false;
  result.quality_score = 0.0;
  result.reason = "quality_scoring_not_implemented";

  return result;
}

std::string make_map_quality_json(
  const MapQualityPlaceholder & quality)
{
  std::ostringstream out;

  out << std::fixed << std::setprecision(6);

  out
    << "{"
    << "\"map_received\":" << bool_text(quality.map_received)
    << ",\"structurally_valid\":"
    << bool_text(quality.structurally_valid)
    << ",\"evaluated\":" << bool_text(quality.evaluated)
    << ",\"navigation_handoff_ready\":"
    << bool_text(quality.navigation_handoff_ready)
    << ",\"width\":" << quality.width
    << ",\"height\":" << quality.height
    << ",\"resolution_m\":" << quality.resolution_m
    << ",\"map_area_m2\":" << quality.map_area_m2
    << ",\"free_ratio\":" << quality.free_ratio
    << ",\"occupied_ratio\":" << quality.occupied_ratio
    << ",\"unknown_ratio\":" << quality.unknown_ratio
    << ",\"quality_score\":" << quality.quality_score
    << ",\"update_count\":" << quality.update_count
    << ",\"reason\":\"" << quality.reason << "\""
    << "}";

  return out.str();
}

}  // namespace savo_mapping
