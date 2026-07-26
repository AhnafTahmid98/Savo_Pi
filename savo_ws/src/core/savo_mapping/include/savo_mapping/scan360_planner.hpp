#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace savo_mapping::scan360
{

inline constexpr double kPi =
  3.14159265358979323846;

inline constexpr double kTwoPi =
  2.0 * kPi;

enum class RotationDirection
{
  CounterClockwise,
  Clockwise
};

struct Scan360PlanOptions
{
  double sweep_angle_rad{kTwoPi};
  double step_angle_rad{kPi / 6.0};

  RotationDirection direction{
    RotationDirection::CounterClockwise};
};

struct Scan360Target
{
  std::size_t index{0};

  double relative_yaw_rad{0.0};
  double normalized_yaw_rad{0.0};

  bool final_target{false};
};

struct Scan360Plan
{
  std::vector<Scan360Target> targets;

  double start_yaw_rad{0.0};
  double requested_sweep_rad{0.0};
  double planned_sweep_rad{0.0};

  RotationDirection direction{
    RotationDirection::CounterClockwise};
};

struct Scan360PlanResult
{
  bool accepted{false};
  std::string reason{"not_evaluated"};
  Scan360Plan plan;
};

std::string_view to_string(
  RotationDirection direction);

double normalize_yaw(double yaw_rad);

Scan360PlanResult make_plan(
  double start_yaw_rad,
  const Scan360PlanOptions & options);

}  // namespace savo_mapping::scan360
