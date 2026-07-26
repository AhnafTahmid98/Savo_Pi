// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "savo_nav/types.hpp"

namespace savo_nav
{

struct NavigationReadinessPolicy
{
  bool require_pointcloud{true};
  bool require_control_mode{true};
  bool require_safety_state{true};
  bool require_global_costmap{true};
  bool require_local_costmap{true};
};

struct NavigationDependencySnapshot
{
  bool map_available{false};

  bool map_to_odom_fresh{false};
  bool odom_to_base_fresh{false};
  bool base_footprint_to_base_link_available{false};

  bool localization_fresh{false};
  bool lidar_fresh{false};
  bool pointcloud_fresh{false};

  bool nav2_action_server_available{false};
  bool global_costmap_fresh{false};
  bool local_costmap_fresh{false};

  bool control_state_fresh{false};
  bool control_allows_navigation{false};

  bool safety_state_fresh{false};
  bool safety_stop_active{false};
  bool slowdown_valid{false};
  double slowdown_factor{0.0};
};

class NavigationReadiness
{
public:
  NavigationReadiness();

  [[nodiscard]] const NavigationReadinessResult &
  GetResult() const noexcept;

  [[nodiscard]] bool Update(
    NavigationReadinessState state,
    bool goal_acceptance_allowed,
    std::string reason,
    std::vector<std::string> failed_dependencies);

  [[nodiscard]] static NavigationReadinessResult Evaluate(
    const NavigationDependencySnapshot & snapshot,
    const NavigationReadinessPolicy & policy);

  [[nodiscard]] static std::string_view ToString(
    NavigationReadinessState state) noexcept;

private:
  [[nodiscard]] static bool IsUpdateValid(
    NavigationReadinessState state,
    bool goal_acceptance_allowed,
    const std::string & reason,
    const std::vector<std::string> & failed_dependencies) noexcept;

  NavigationReadinessResult result_{};
};

}  // namespace savo_nav
