// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string_view>

namespace savo_nav::constants
{

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;
inline constexpr int kContractVersion = 1;

inline constexpr std::string_view kPackageName = "savo_nav";
inline constexpr std::string_view kRobotName = "Robot Savo";
inline constexpr std::string_view kProjectName = "Robot SAVO Project";
inline constexpr std::string_view kRosDistro = "jazzy";
inline constexpr std::string_view kVersion = "0.1.0";

}  // namespace savo_nav::constants
