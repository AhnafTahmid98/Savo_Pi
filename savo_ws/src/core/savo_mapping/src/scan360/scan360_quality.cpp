#include "savo_mapping/scan360_quality.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace savo_mapping::scan360
{
namespace
{

bool ratio_is_valid(double ratio)
{
  return
    std::isfinite(ratio) &&
    ratio >= 0.0 &&
    ratio <= 1.0;
}

bool positive_finite(double value)
{
  return
    std::isfinite(value) &&
    value > 0.0;
}

Scan360QualityResult insufficient(
  std::string reason)
{
  Scan360QualityResult result;

  result.evaluated = false;
  result.accepted = false;

  result.grade =
    QualityGrade::InsufficientData;

  result.reason = std::move(reason);

  return result;
}

Scan360QualityResult rejected(
  Scan360QualityResult result,
  std::string reason)
{
  result.evaluated = true;
  result.accepted = false;

  result.grade =
    QualityGrade::Rejected;

  result.reason = std::move(reason);

  return result;
}

double ratio(
  std::size_t numerator,
  std::size_t denominator)
{
  if (denominator == 0U) {
    return 0.0;
  }

  return
    static_cast<double>(numerator) /
    static_cast<double>(denominator);
}

}  // namespace

std::string_view to_string(
  QualityGrade grade)
{
  switch (grade) {
    case QualityGrade::InsufficientData:
      return "insufficient_data";

    case QualityGrade::Rejected:
      return "rejected";

    case QualityGrade::Acceptable:
      return "acceptable";

    case QualityGrade::Good:
      return "good";
  }

  return "insufficient_data";
}

bool thresholds_are_valid(
  const Scan360QualityThresholds & thresholds)
{
  return
    thresholds.minimum_planned_targets > 0U &&
    ratio_is_valid(
    thresholds.minimum_completion_ratio) &&
    ratio_is_valid(
    thresholds.minimum_settled_ratio) &&
    ratio_is_valid(
    thresholds.minimum_map_update_ratio) &&
    thresholds.
    minimum_valid_scan_points_per_target >
    0U &&
    positive_finite(
    thresholds.
    maximum_mean_abs_yaw_error_rad) &&
    positive_finite(
    thresholds.
    maximum_peak_abs_yaw_error_rad) &&
    thresholds.
    maximum_mean_abs_yaw_error_rad <=
    thresholds.
    maximum_peak_abs_yaw_error_rad;
}

Scan360QualityResult evaluate_quality(
  const Scan360QualityInput & input,
  const Scan360QualityThresholds & thresholds)
{
  if (!thresholds_are_valid(thresholds)) {
    return insufficient(
      "invalid_thresholds");
  }

  if (
    input.planned_target_count <
    thresholds.minimum_planned_targets)
  {
    return insufficient(
      "insufficient_planned_targets");
  }

  if (input.observations.empty()) {
    return insufficient(
      "no_observations");
  }

  std::vector<bool> target_observed(
    input.planned_target_count,
    false);

  Scan360QualityResult result;

  double yaw_error_sum = 0.0;

  for (const auto & observation :
    input.observations)
  {
    if (
      observation.target_index >=
      input.planned_target_count)
    {
      return insufficient(
        "target_index_out_of_range");
    }

    if (
      target_observed[
        observation.target_index])
    {
      return insufficient(
        "duplicate_target_observation");
    }

    if (
      !std::isfinite(
        observation.yaw_error_rad))
    {
      return insufficient(
        "yaw_error_not_finite");
    }

    target_observed[
      observation.target_index] = true;

    ++result.observed_target_count;

    if (observation.settled) {
      ++result.settled_target_count;
    }

    if (observation.map_updated) {
      ++result.map_updated_target_count;
    }

    if (
      observation.valid_scan_points >=
      thresholds.
      minimum_valid_scan_points_per_target)
    {
      ++result.
      scan_sufficient_target_count;
    }

    const double absolute_yaw_error =
      std::abs(
      observation.yaw_error_rad);

    yaw_error_sum +=
      absolute_yaw_error;

    result.peak_abs_yaw_error_rad =
      std::max(
      result.peak_abs_yaw_error_rad,
      absolute_yaw_error);
  }

  result.completion_ratio =
    ratio(
    result.observed_target_count,
    input.planned_target_count);

  result.settled_ratio =
    ratio(
    result.settled_target_count,
    input.planned_target_count);

  result.map_update_ratio =
    ratio(
    result.map_updated_target_count,
    input.planned_target_count);

  result.scan_sufficient_ratio =
    ratio(
    result.scan_sufficient_target_count,
    input.planned_target_count);

  result.mean_abs_yaw_error_rad =
    yaw_error_sum /
    static_cast<double>(
    result.observed_target_count);

  if (
    result.completion_ratio <
    thresholds.minimum_completion_ratio)
  {
    return rejected(
      std::move(result),
      "completion_ratio_below_threshold");
  }

  if (
    result.settled_ratio <
    thresholds.minimum_settled_ratio)
  {
    return rejected(
      std::move(result),
      "settled_ratio_below_threshold");
  }

  if (
    result.map_update_ratio <
    thresholds.minimum_map_update_ratio)
  {
    return rejected(
      std::move(result),
      "map_update_ratio_below_threshold");
  }

  if (
    result.scan_sufficient_ratio <
    thresholds.minimum_completion_ratio)
  {
    return rejected(
      std::move(result),
      "scan_point_ratio_below_threshold");
  }

  if (
    result.mean_abs_yaw_error_rad >
    thresholds.
    maximum_mean_abs_yaw_error_rad)
  {
    return rejected(
      std::move(result),
      "mean_yaw_error_above_threshold");
  }

  if (
    result.peak_abs_yaw_error_rad >
    thresholds.
    maximum_peak_abs_yaw_error_rad)
  {
    return rejected(
      std::move(result),
      "peak_yaw_error_above_threshold");
  }

  const bool perfect_coverage =
    result.completion_ratio == 1.0 &&
    result.settled_ratio == 1.0 &&
    result.map_update_ratio == 1.0 &&
    result.scan_sufficient_ratio == 1.0;

  const bool strong_yaw_accuracy =
    result.mean_abs_yaw_error_rad <=
    thresholds.
    maximum_mean_abs_yaw_error_rad *
    0.5 &&
    result.peak_abs_yaw_error_rad <=
    thresholds.
    maximum_peak_abs_yaw_error_rad *
    0.5;

  result.evaluated = true;
  result.accepted = true;

  if (
    perfect_coverage &&
    strong_yaw_accuracy)
  {
    result.grade = QualityGrade::Good;
    result.reason = "quality_good";
  } else {
    result.grade =
      QualityGrade::Acceptable;

    result.reason =
      "quality_acceptable";
  }

  return result;
}

}  // namespace savo_mapping::scan360
