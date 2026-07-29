// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>

#include "savo_msgs/action/execute_coverage_path.hpp"

#include "savo_nav/coverage_execution_model.hpp"
#include "savo_nav/coverage_path_validator.hpp"
#include "savo_nav/coverage_progress_tracker.hpp"
#include "savo_nav/goal_context.hpp"

namespace savo_nav
{

struct CoverageActionAdapterPolicy
{
  CoveragePathValidationPolicy validation_policy{};

  double default_execution_timeout_seconds{300.0};
  double maximum_execution_timeout_seconds{3600.0};
};

struct CoverageAdaptedGoal
{
  GoalContext context{};

  CoveragePathValidationRequest validation_request{};
  CoveragePathValidationResult validation{};

  double requested_execution_timeout_seconds{0.0};
  double resolved_execution_timeout_seconds{0.0};
};

class CoverageActionAdapter
{
public:
  using ExecuteCoveragePath =
    savo_msgs::action::ExecuteCoveragePath;

  [[nodiscard]] static CoverageAdaptedGoal AdaptGoal(
    const ExecuteCoveragePath::Goal & goal,
    const MapContext & map_context,
    const NavigationReadinessResult & readiness,
    std::uint64_t sequence,
    const CoverageActionAdapterPolicy & policy = {});

  [[nodiscard]] static ExecuteCoveragePath::Feedback
  MakeFeedback(
    const CoverageExecutionSnapshot & execution,
    const CoverageProgressSnapshot & progress);

  [[nodiscard]] static ExecuteCoveragePath::Result
  MakeResult(
    const CoverageExecutionSnapshot & execution,
    const CoverageProgressSnapshot & progress);
};

}  // namespace savo_nav
