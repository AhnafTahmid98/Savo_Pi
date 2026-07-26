#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace savo_mapping::scan360
{

enum class QualityGrade
{
  InsufficientData,
  Rejected,
  Acceptable,
  Good
};

struct Scan360QualityThresholds
{
  std::size_t minimum_planned_targets{4};

  double minimum_completion_ratio{0.90};
  double minimum_settled_ratio{0.90};
  double minimum_map_update_ratio{0.75};

  std::size_t
    minimum_valid_scan_points_per_target{30};

  double maximum_mean_abs_yaw_error_rad{0.08};
  double maximum_peak_abs_yaw_error_rad{0.18};
};

struct Scan360Observation
{
  std::size_t target_index{0};

  double yaw_error_rad{0.0};

  std::size_t valid_scan_points{0};

  bool settled{false};
  bool map_updated{false};
};

struct Scan360QualityInput
{
  std::size_t planned_target_count{0};

  std::vector<Scan360Observation>
  observations;
};

struct Scan360QualityResult
{
  bool evaluated{false};
  bool accepted{false};

  QualityGrade grade{
    QualityGrade::InsufficientData};

  std::string reason{
    "not_evaluated"};

  std::size_t observed_target_count{0};
  std::size_t settled_target_count{0};
  std::size_t map_updated_target_count{0};

  std::size_t
    scan_sufficient_target_count{0};

  double completion_ratio{0.0};
  double settled_ratio{0.0};
  double map_update_ratio{0.0};
  double scan_sufficient_ratio{0.0};

  double mean_abs_yaw_error_rad{0.0};
  double peak_abs_yaw_error_rad{0.0};
};

std::string_view to_string(
  QualityGrade grade);

bool thresholds_are_valid(
  const Scan360QualityThresholds & thresholds);

Scan360QualityResult evaluate_quality(
  const Scan360QualityInput & input,
  const Scan360QualityThresholds & thresholds);

}  // namespace savo_mapping::scan360
