// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "savo_nav/types.hpp"

namespace savo_nav
{

enum class NavigationMapMode : std::uint8_t
{
  kUnknown = 0,
  kSavedMap,
  kLiveMapping
};

enum class MapToOdomAuthority : std::uint8_t
{
  kNone = 0,
  kAmcl,
  kSlamToolbox
};

struct MapContext
{
  NavigationMapMode mode{NavigationMapMode::kUnknown};
  MapToOdomAuthority authority{MapToOdomAuthority::kNone};
  std::string map_id{};
  std::string map_release_id{};
  std::string frame_id{"map"};
  std::uint64_t revision{0};
  bool available{false};
  bool localization_ready{false};
  bool mapping_active{false};
};

class MapContextContract
{
public:
  [[nodiscard]] static ValidationResult Validate(
    const MapContext & context);

  [[nodiscard]] static std::string_view ToString(
    NavigationMapMode mode) noexcept;

  [[nodiscard]] static std::string_view ToString(
    MapToOdomAuthority authority) noexcept;
};

}  // namespace savo_nav
