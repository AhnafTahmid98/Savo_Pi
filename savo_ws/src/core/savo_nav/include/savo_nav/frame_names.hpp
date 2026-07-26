// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string_view>

namespace savo_nav::frames
{

inline constexpr std::string_view kMap = "map";
inline constexpr std::string_view kOdom = "odom";
inline constexpr std::string_view kBaseFootprint =
  "base_footprint";
inline constexpr std::string_view kBaseLink = "base_link";
inline constexpr std::string_view kLaser = "laser_frame";
inline constexpr std::string_view kCameraDepthOptical =
  "camera_depth_optical_frame";

}  // namespace savo_nav::frames
