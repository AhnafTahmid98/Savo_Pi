#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "savo_locations/read_only_catalog_view.hpp"


namespace
{

savo_locations::PoseData make_pose(
  const double x,
  const double y)
{
  savo_locations::PoseData pose;

  pose.frame_id = "map";
  pose.x = x;
  pose.y = y;
  pose.qw = 1.0;

  return pose;
}


savo_locations::LocationRecordData make_record(
  std::string id,
  std::string name,
  std::string map_id,
  const std::uint32_t revision,
  const bool enabled = true)
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::
      kApproved;

  record.enabled = enabled;
  record.record_revision = 1U;

  record.location.location_id =
    std::move(id);

  record.location.display_name =
    std::move(name);

  record.location.semantic_type =
    "classroom";

  record.location.map.map_id =
    std::move(map_id);

  record.location.map.map_revision =
    revision;

  record.location.approach_pose =
    make_pose(1.0, 2.0);

  record.location.tag.family =
    "tag36h11";

  record.location.tag.id =
    static_cast<std::int32_t>(
      revision);

  return record;
}


savo_locations::ReadOnlyCatalogView make_view()
{
  savo_locations::CatalogSnapshot snapshot;

  auto a201 = make_record(
    "A201",
    "Room A201",
    "campus_main",
    7U);

  a201.location.aliases = {
    "East classroom",
    "A 201",
  };

  auto b101 = make_record(
    "B101",
    "Room B101",
    "campus_main",
    7U,
    false);

  b101.location.aliases = {
    "Shared room",
  };

  auto c101 = make_record(
    "C101",
    "Room C101",
    "campus_annex",
    2U);

  c101.location.aliases = {
    "Shared room",
  };

  snapshot.locations = {
    c101,
    b101,
    a201,
  };

  return savo_locations::
    ReadOnlyCatalogView{
      std::move(snapshot)};
}

}  // namespace


TEST(ReadOnlyCatalogView, ResolvesExactId)
{
  const auto view = make_view();

  savo_locations::ReadResolveRequest request;
  request.query = "A201";

  const auto result =
    view.resolve(request);

  EXPECT_EQ(
    result.code,
    savo_locations::ReadResolveCode::
      kResolved);

  EXPECT_EQ(
    result.match_type,
    savo_locations::ResolveMatchType::
      kLocationId);

  EXPECT_EQ(
    result.location.location.location_id,
    "A201");
}


TEST(ReadOnlyCatalogView, ResolvesAlias)
{
  const auto view = make_view();

  savo_locations::ReadResolveRequest request;
  request.query = "east classroom";

  const auto result =
    view.resolve(request);

  EXPECT_EQ(
    result.code,
    savo_locations::ReadResolveCode::
      kResolved);

  EXPECT_EQ(
    result.match_type,
    savo_locations::ResolveMatchType::
      kAlias);
}


TEST(ReadOnlyCatalogView, ReportsAmbiguityAcrossMaps)
{
  const auto view = make_view();

  savo_locations::ReadResolveRequest request;
  request.query = "Shared room";

  const auto result =
    view.resolve(request);

  EXPECT_EQ(
    result.code,
    savo_locations::ReadResolveCode::
      kAmbiguous);

  ASSERT_EQ(
    result.ambiguous_location_ids.size(),
    2U);

  EXPECT_EQ(
    result.ambiguous_location_ids[0],
    "B101");

  EXPECT_EQ(
    result.ambiguous_location_ids[1],
    "C101");
}


TEST(ReadOnlyCatalogView, MapContextDisambiguates)
{
  const auto view = make_view();

  savo_locations::ReadResolveRequest request;

  request.query = "Shared room";
  request.enforce_map_context = true;
  request.map_id = "campus_annex";
  request.map_revision = 2U;

  const auto result =
    view.resolve(request);

  EXPECT_EQ(
    result.code,
    savo_locations::ReadResolveCode::
      kResolved);

  EXPECT_EQ(
    result.location.location.location_id,
    "C101");
}


TEST(ReadOnlyCatalogView, DisabledResolveIsFailClosed)
{
  const auto view = make_view();

  savo_locations::ReadResolveRequest request;
  request.query = "B101";

  const auto result =
    view.resolve(request);

  EXPECT_EQ(
    result.code,
    savo_locations::ReadResolveCode::
      kDisabled);
}


TEST(ReadOnlyCatalogView, GetHonoursDisabledPolicy)
{
  const auto view = make_view();

  const auto blocked =
    view.get(
      "B101",
      false,
      false);

  EXPECT_EQ(
    blocked.code,
    savo_locations::ReadGetCode::
      kDisabled);

  const auto included =
    view.get(
      "B101",
      true,
      false);

  EXPECT_EQ(
    included.code,
    savo_locations::ReadGetCode::
      kFound);
}


TEST(ReadOnlyCatalogView, ListFiltersMapAndEnabled)
{
  const auto view = make_view();

  savo_locations::ReadListRequest request;

  request.map_id = "campus_main";
  request.map_revision = 7U;
  request.enabled_only = true;

  const auto result =
    view.list(request);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.locations.size(), 1U);

  EXPECT_EQ(
    result.locations.front()
      .location
      .location_id,
    "A201");
}


TEST(ReadOnlyCatalogView, RejectsInvalidListContext)
{
  const auto view = make_view();

  savo_locations::ReadListRequest request;

  request.enforce_map_context = true;
  request.map_id = "campus_main";

  const auto result =
    view.list(request);

  EXPECT_FALSE(result.valid);
}
