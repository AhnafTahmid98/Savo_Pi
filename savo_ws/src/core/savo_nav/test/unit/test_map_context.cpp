// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/map_context.hpp"

namespace
{

using Authority = savo_nav::MapToOdomAuthority;
using Mode = savo_nav::NavigationMapMode;

savo_nav::MapContext MakeSavedMap()
{
  savo_nav::MapContext context;

  context.mode = Mode::kSavedMap;
  context.authority = Authority::kAmcl;
  context.map_id = "campus-main";
  context.map_release_id = "campus-main-r3";
  context.frame_id = "map";
  context.revision = 3;
  context.available = true;
  context.localization_ready = true;
  context.mapping_active = false;

  return context;
}

savo_nav::MapContext MakeLiveMap()
{
  savo_nav::MapContext context;

  context.mode = Mode::kLiveMapping;
  context.authority = Authority::kSlamToolbox;
  context.frame_id = "map";
  context.revision = 1;
  context.available = true;
  context.localization_ready = true;
  context.mapping_active = true;

  return context;
}

TEST(MapContextTest, AcceptsCleanUnavailableContext)
{
  const savo_nav::MapContext context;

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_TRUE(validation.IsValid());
}

TEST(MapContextTest, AcceptsSavedMapContext)
{
  const auto validation =
    savo_nav::MapContextContract::Validate(
    MakeSavedMap());

  EXPECT_TRUE(validation.IsValid());
}

TEST(MapContextTest, AcceptsLiveMappingContext)
{
  const auto validation =
    savo_nav::MapContextContract::Validate(
    MakeLiveMap());

  EXPECT_TRUE(validation.IsValid());
}

TEST(MapContextTest, RejectsSavedMapWithoutMapId)
{
  auto context = MakeSavedMap();
  context.map_id.clear();

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kMissingMapId);
}

TEST(MapContextTest, RejectsSimultaneousSavedMapMapping)
{
  auto context = MakeSavedMap();
  context.mapping_active = true;

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidCombination);
}

TEST(MapContextTest, RejectsSavedMapWithSlamAuthority)
{
  auto context = MakeSavedMap();
  context.authority = Authority::kSlamToolbox;

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidAuthority);
}

TEST(MapContextTest, RejectsLiveMapWithAmclAuthority)
{
  auto context = MakeLiveMap();
  context.authority = Authority::kAmcl;

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidAuthority);
}

TEST(MapContextTest, RejectsAvailableUnknownMapMode)
{
  auto context = MakeSavedMap();
  context.mode = Mode::kUnknown;
  context.authority = Authority::kNone;
  context.map_id.clear();

  const auto validation =
    savo_nav::MapContextContract::Validate(context);

  EXPECT_EQ(
    validation.code,
    savo_nav::ValidationCode::kInvalidState);
}

TEST(MapContextTest, ConvertsMapModes)
{
  const std::vector<std::pair<Mode, std::string_view>> expected{
    {Mode::kUnknown, "unknown"},
    {Mode::kSavedMap, "saved_map"},
    {Mode::kLiveMapping, "live_mapping"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::MapContextContract::ToString(item.first),
      item.second);
  }
}

TEST(MapContextTest, ConvertsAuthorities)
{
  const std::vector<
    std::pair<Authority, std::string_view>> expected{
    {Authority::kNone, "none"},
    {Authority::kAmcl, "amcl"},
    {Authority::kSlamToolbox, "slam_toolbox"}
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::MapContextContract::ToString(item.first),
      item.second);
  }
}

}  // namespace
