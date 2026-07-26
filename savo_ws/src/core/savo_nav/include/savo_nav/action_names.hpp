// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string_view>

namespace savo_nav::actions
{

inline constexpr std::string_view kNavigationNavigateToPose =
  "/savo_nav/navigation/navigate_to_pose";

inline constexpr std::string_view kExplorationNavigateToPose =
  "/savo_nav/exploration/navigate_to_pose";

inline constexpr std::string_view kNav2NavigateToPose =
  "/navigate_to_pose";

}  // namespace savo_nav::actions
