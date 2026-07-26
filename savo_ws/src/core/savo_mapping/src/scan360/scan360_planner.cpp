#include "savo_mapping/scan360_planner.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace savo_mapping::scan360
{
namespace
{

constexpr double kComparisonTolerance =
  1.0e-9;

constexpr std::size_t kMaximumTargets =
  720;

double direction_sign(
  RotationDirection direction)
{
  return
    direction ==
    RotationDirection::CounterClockwise ?
    1.0 :
    -1.0;
}

Scan360PlanResult reject(
  std::string reason)
{
  Scan360PlanResult result;

  result.accepted = false;
  result.reason = std::move(reason);

  return result;
}

void append_target(
  Scan360Plan & plan,
  double signed_progress_rad,
  bool final_target)
{
  Scan360Target target;

  target.index = plan.targets.size();

  target.relative_yaw_rad =
    signed_progress_rad;

  target.normalized_yaw_rad =
    normalize_yaw(
    plan.start_yaw_rad +
    signed_progress_rad);

  target.final_target = final_target;

  plan.targets.push_back(target);
}

}  // namespace

std::string_view to_string(
  RotationDirection direction)
{
  switch (direction) {
    case RotationDirection::
      CounterClockwise:
      return "counter_clockwise";

    case RotationDirection::Clockwise:
      return "clockwise";
  }

  return "counter_clockwise";
}

double normalize_yaw(double yaw_rad)
{
  if (!std::isfinite(yaw_rad)) {
    return std::numeric_limits<
      double>::quiet_NaN();
  }

  double normalized =
    std::fmod(
    yaw_rad + kPi,
    kTwoPi);

  if (normalized < 0.0) {
    normalized += kTwoPi;
  }

  return normalized - kPi;
}

Scan360PlanResult make_plan(
  double start_yaw_rad,
  const Scan360PlanOptions & options)
{
  if (!std::isfinite(start_yaw_rad)) {
    return reject(
      "start_yaw_not_finite");
  }

  if (
    !std::isfinite(
      options.sweep_angle_rad) ||
    options.sweep_angle_rad <= 0.0 ||
    options.sweep_angle_rad >
    kTwoPi + kComparisonTolerance)
  {
    return reject(
      "sweep_angle_out_of_range");
  }

  if (
    !std::isfinite(
      options.step_angle_rad) ||
    options.step_angle_rad <= 0.0 ||
    options.step_angle_rad > kTwoPi)
  {
    return reject(
      "step_angle_out_of_range");
  }

  Scan360Plan plan;

  plan.start_yaw_rad =
    normalize_yaw(start_yaw_rad);

  plan.requested_sweep_rad =
    options.sweep_angle_rad;

  plan.direction =
    options.direction;

  const double sign =
    direction_sign(
    options.direction);

  double progress_rad =
    options.step_angle_rad;

  while (
    progress_rad <
    options.sweep_angle_rad -
    kComparisonTolerance)
  {
    if (
      plan.targets.size() >=
      kMaximumTargets - 1)
    {
      return reject(
        "target_count_exceeds_limit");
    }

    append_target(
      plan,
      sign * progress_rad,
      false);

    progress_rad +=
      options.step_angle_rad;
  }

  append_target(
    plan,
    sign * options.sweep_angle_rad,
    true);

  plan.planned_sweep_rad =
    options.sweep_angle_rad;

  Scan360PlanResult result;

  result.accepted = true;
  result.reason = "accepted";
  result.plan = std::move(plan);

  return result;
}

}  // namespace savo_mapping::scan360
