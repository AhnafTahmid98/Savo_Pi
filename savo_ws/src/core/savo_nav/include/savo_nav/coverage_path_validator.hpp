// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "savo_nav/map_context.hpp"
#include "savo_nav/types.hpp"

namespace savo_nav
{

struct CoveragePathPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};

  std::string frame_id{};
};

struct CoveragePathValidationPolicy
{
  std::uint32_t expected_contract_version{1};

  std::size_t maximum_waypoints{10000};
  std::size_t maximum_mission_id_length{128};

  double maximum_absolute_coordinate_m{1000.0};
  double maximum_absolute_z_m{0.10};
  double maximum_total_distance_m{10000.0};
  double maximum_execution_timeout_seconds{3600.0};

  double quaternion_norm_tolerance{1.0e-3};
  double planar_quaternion_tolerance{1.0e-6};

  bool require_readiness{true};
  bool allow_degraded_readiness{false};
};

struct CoveragePathValidationRequest
{
  std::uint32_t contract_version{0};

  std::string mission_id{};
  std::string path_frame{"map"};

  std::vector<CoveragePathPoint> points{};

  double execution_timeout_seconds{0.0};

  MapContext map_context{};
  NavigationReadinessResult readiness{};
};

struct CoveragePathValidationResult
{
  ValidationResult validation{};
  double total_distance_m{0.0};

  [[nodiscard]] bool IsValid() const noexcept
  {
    return validation.IsValid();
  }
};

class CoveragePathValidator
{
public:
  [[nodiscard]] static CoveragePathValidationResult Validate(
    const CoveragePathValidationRequest & request,
    const CoveragePathValidationPolicy & policy = {});
};

}  // namespace savo_nav
