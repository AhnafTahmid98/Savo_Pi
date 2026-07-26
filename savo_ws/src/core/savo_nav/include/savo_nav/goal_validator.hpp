// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include "savo_nav/goal_context.hpp"
#include "savo_nav/map_context.hpp"
#include "savo_nav/types.hpp"

namespace savo_nav
{

struct GoalPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GoalValidationPolicy
{
  double max_abs_coordinate_m{1000.0};

  bool require_readiness{true};

  bool allow_degraded_readiness{false};

  bool require_map_id_match{true};
};

struct GoalValidationRequest
{
  GoalContext context{};

  GoalPose2D pose{};

  MapContext map_context{};

  NavigationReadinessResult readiness{};
};

class GoalValidator
{
public:
  [[nodiscard]] static ValidationResult Validate(
    const GoalValidationRequest & request,
    const GoalValidationPolicy & policy = {});
};

}  // namespace savo_nav
