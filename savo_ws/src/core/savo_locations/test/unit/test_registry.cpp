#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "savo_locations/registry.hpp"


namespace
{

savo_locations::LocationRecordData make_record(
  const std::string & location_id,
  const std::string & display_name,
  const std::string & map_id,
  const std::uint32_t map_revision,
  const std::int32_t tag_id,
  const std::vector<std::string> & aliases = {})
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::kApproved;

  record.enabled = true;
  record.record_revision = 1U;

  record.location.location_id =
    location_id;

  record.location.display_name =
    display_name;

  record.location.aliases = aliases;

  record.location.semantic_type = "room";

  record.location.map.map_id = map_id;

  record.location.map.map_revision =
    map_revision;

  record.location.approach_pose.frame_id =
    "map";

  record.location.approach_pose.qw = 1.0;

  record.location.tag.family = "tag36h11";
  record.location.tag.id = tag_id;

  return record;
}

}  // namespace


TEST(LocationRegistry, InsertsAndGetsRecord)
{
  savo_locations::InMemoryRegistry registry;

  const auto inserted =
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27,
        {"A 201"}));

  ASSERT_TRUE(inserted.success);

  EXPECT_EQ(
    inserted.code,
    savo_locations::MutationCode::kInserted);

  EXPECT_EQ(registry.size(), 1U);

  const auto stored = registry.get("a201");

  ASSERT_TRUE(stored.has_value());

  EXPECT_EQ(
    stored->location.location_id,
    "A201");
}


TEST(LocationRegistry, ResolvesExactId)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27,
        {"A 201"})).success);

  const auto result =
    registry.resolve("a201");

  ASSERT_TRUE(result.resolved);

  EXPECT_EQ(
    result.code,
    savo_locations::ResolveCode::kResolved);

  EXPECT_EQ(
    result.match_type,
    savo_locations::MatchType::kLocationId);

  ASSERT_TRUE(result.record.has_value());

  EXPECT_EQ(
    result.record->location.location_id,
    "A201");
}


TEST(LocationRegistry, ResolvesDisplayNameAndAlias)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27,
        {
          "Classroom A201",
          "East classroom",
        })).success);

  const auto display =
    registry.resolve(" room-a201 ");

  ASSERT_TRUE(display.resolved);

  EXPECT_EQ(
    display.match_type,
    savo_locations::MatchType::kDisplayName);

  const auto alias =
    registry.resolve("east_classroom");

  ASSERT_TRUE(alias.resolved);

  EXPECT_EQ(
    alias.match_type,
    savo_locations::MatchType::kAlias);
}


TEST(LocationRegistry, RejectsSameMapIdentityConflict)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27,
        {"Student service"})).success);

  const auto conflict =
    registry.insert(
      make_record(
        "B101",
        "Room B101",
        "campus_main",
        7U,
        28,
        {"student-service"}));

  EXPECT_FALSE(conflict.success);

  EXPECT_EQ(
    conflict.code,
    savo_locations::MutationCode::
      kIdentityConflict);
}


TEST(LocationRegistry, RejectsSameMapTagConflict)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27)).success);

  const auto conflict =
    registry.insert(
      make_record(
        "B101",
        "Room B101",
        "campus_main",
        7U,
        27));

  EXPECT_FALSE(conflict.success);

  EXPECT_EQ(
    conflict.code,
    savo_locations::MutationCode::
      kTagConflict);
}


TEST(LocationRegistry, AllowsIdentityAcrossMaps)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Information Desk North",
        "campus_north",
        3U,
        10,
        {"Information desk"})).success);

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "B101",
        "Information Desk South",
        "campus_south",
        4U,
        10,
        {"Information desk"})).success);

  const auto ambiguous =
    registry.resolve("information desk");

  EXPECT_FALSE(ambiguous.resolved);

  EXPECT_EQ(
    ambiguous.code,
    savo_locations::ResolveCode::kAmbiguous);

  EXPECT_EQ(
    ambiguous.ambiguous_location_ids.size(),
    2U);

  savo_locations::ResolveOptions options;
  options.enforce_map_context = true;
  options.map.map_id = "campus_north";
  options.map.map_revision = 3U;

  const auto resolved =
    registry.resolve(
      "information desk",
      options);

  ASSERT_TRUE(resolved.resolved);
  ASSERT_TRUE(resolved.record.has_value());

  EXPECT_EQ(
    resolved.record->location.location_id,
    "A201");
}


TEST(LocationRegistry, ReportsMapMismatch)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27)).success);

  savo_locations::ResolveOptions options;
  options.enforce_map_context = true;
  options.map.map_id = "campus_other";
  options.map.map_revision = 1U;

  const auto result =
    registry.resolve("A201", options);

  EXPECT_FALSE(result.resolved);

  EXPECT_EQ(
    result.code,
    savo_locations::ResolveCode::kMapMismatch);
}


TEST(LocationRegistry, ReportsDisabledLocation)
{
  savo_locations::InMemoryRegistry registry;

  auto record =
    make_record(
      "A201",
      "Room A201",
      "campus_main",
      7U,
      27);

  record.enabled = false;

  ASSERT_TRUE(
    registry.insert(record).success);

  const auto result =
    registry.resolve("A201");

  EXPECT_FALSE(result.resolved);

  EXPECT_EQ(
    result.code,
    savo_locations::ResolveCode::kDisabled);
}


TEST(LocationRegistry, ReportsRetiredLocation)
{
  savo_locations::InMemoryRegistry registry;

  auto record =
    make_record(
      "A201",
      "Room A201",
      "campus_main",
      7U,
      27,
      {"Old classroom"});

  record.state =
    savo_locations::LocationState::kRetired;

  record.enabled = false;

  ASSERT_TRUE(
    registry.insert(record).success);

  const auto result =
    registry.resolve("old classroom");

  EXPECT_FALSE(result.resolved);

  EXPECT_EQ(
    result.code,
    savo_locations::ResolveCode::kRetired);
}


TEST(LocationRegistry, RejectsStaleReplacement)
{
  savo_locations::InMemoryRegistry registry;

  auto original =
    make_record(
      "A201",
      "Room A201",
      "campus_main",
      7U,
      27);

  ASSERT_TRUE(
    registry.insert(original).success);

  auto replacement = original;
  replacement.record_revision = 2U;

  replacement.location.display_name =
    "Classroom A201";

  const auto stale =
    registry.replace(
      replacement,
      99U);

  EXPECT_FALSE(stale.success);

  EXPECT_EQ(
    stale.code,
    savo_locations::MutationCode::
      kStaleRevision);
}


TEST(LocationRegistry, EnforcesRevisionSequence)
{
  savo_locations::InMemoryRegistry registry;

  auto original =
    make_record(
      "A201",
      "Room A201",
      "campus_main",
      7U,
      27);

  ASSERT_TRUE(
    registry.insert(original).success);

  auto replacement = original;
  replacement.record_revision = 3U;

  const auto invalid =
    registry.replace(
      replacement,
      1U);

  EXPECT_FALSE(invalid.success);

  EXPECT_EQ(
    invalid.code,
    savo_locations::MutationCode::
      kRevisionSequenceError);
}


TEST(LocationRegistry, ReplacesRecordSafely)
{
  savo_locations::InMemoryRegistry registry;

  auto original =
    make_record(
      "A201",
      "Room A201",
      "campus_main",
      7U,
      27);

  ASSERT_TRUE(
    registry.insert(original).success);

  auto replacement = original;

  replacement.record_revision = 2U;

  replacement.location.display_name =
    "Classroom A201";

  replacement.location.aliases = {
    "A 201",
  };

  const auto updated =
    registry.replace(
      replacement,
      1U);

  ASSERT_TRUE(updated.success);

  EXPECT_EQ(
    updated.code,
    savo_locations::MutationCode::kUpdated);

  const auto resolved =
    registry.resolve("classroom a201");

  ASSERT_TRUE(resolved.resolved);
  ASSERT_TRUE(resolved.record.has_value());

  EXPECT_EQ(
    resolved.record->record_revision,
    2U);
}


TEST(LocationRegistry, SetEnabledIncrementsRevision)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27)).success);

  const auto disabled =
    registry.set_enabled(
      "a201",
      1U,
      false);

  ASSERT_TRUE(disabled.success);
  ASSERT_TRUE(disabled.record.has_value());

  EXPECT_FALSE(disabled.record->enabled);

  EXPECT_EQ(
    disabled.record->record_revision,
    2U);

  const auto resolved =
    registry.resolve("A201");

  EXPECT_EQ(
    resolved.code,
    savo_locations::ResolveCode::kDisabled);
}


TEST(LocationRegistry, ListIsDeterministic)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "C301",
        "Room C301",
        "campus_main",
        7U,
        31)).success);

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27)).success);

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "B101",
        "Room B101",
        "campus_main",
        7U,
        28)).success);

  const auto records = registry.list();

  ASSERT_EQ(records.size(), 3U);

  EXPECT_EQ(
    records[0].location.location_id,
    "A201");

  EXPECT_EQ(
    records[1].location.location_id,
    "B101");

  EXPECT_EQ(
    records[2].location.location_id,
    "C301");
}


TEST(LocationRegistry, ClearRemovesAllRecords)
{
  savo_locations::InMemoryRegistry registry;

  ASSERT_TRUE(
    registry.insert(
      make_record(
        "A201",
        "Room A201",
        "campus_main",
        7U,
        27)).success);

  registry.clear();

  EXPECT_EQ(registry.size(), 0U);

  EXPECT_EQ(
    registry.resolve("A201").code,
    savo_locations::ResolveCode::kNotFound);
}


TEST(LocationRegistry, ResultStringsAreStable)
{
  using savo_locations::MatchType;
  using savo_locations::MutationCode;
  using savo_locations::ResolveCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(MutationCode::kTagConflict),
    "tag_conflict");

  EXPECT_EQ(
    to_string(ResolveCode::kAmbiguous),
    "ambiguous");

  EXPECT_EQ(
    to_string(MatchType::kAlias),
    "alias");
}
