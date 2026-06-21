#pragma once

#include "savo_control/heading_pid.hpp"

namespace savo_control
{

// Compatibility header.
//
// The production heading controller implementation lives in heading_pid.hpp.
// Keep this file thin so C++ code can include either:
//
//   #include "savo_control/heading_pid.hpp"
//   #include "savo_control/heading_controller.hpp"
//
// without creating a second heading-control implementation.

using HeadingController = HeadingPid;
using HeadingControllerConfig = HeadingControllerConfigCpp;
using HeadingControllerResult = HeadingControllerResultCpp;

inline double shortest_angular_distance_rad(
  const double from_rad,
  const double to_rad)
{
  return heading_shortest_angular_distance_rad(from_rad, to_rad);
}

}  // namespace savo_control
