// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>
#include <vector>

namespace savo_nav
{

enum class StartupQuality
{
  kBelowMinimum,
  kMinimum,
};

struct Nav2StartupEvidence
{
  std::vector<std::string> inactive_lifecycle_nodes;
  bool navigate_to_pose_action_ready{false};
  bool follow_waypoints_action_ready{false};
  bool odom_to_base_footprint_tf_ready{false};
};

struct Nav2StartupDecision
{
  bool process_started{false};
  bool startup_ready{false};
  StartupQuality quality{StartupQuality::kBelowMinimum};
  std::string reason{"nav2_starting"};
};

[[nodiscard]] Nav2StartupDecision EvaluateNav2Startup(
  const Nav2StartupEvidence & evidence);
[[nodiscard]] const char * ToString(StartupQuality quality) noexcept;

}  // namespace savo_nav
