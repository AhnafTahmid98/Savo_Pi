#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>
#include <vector>

#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/sqlite_store.hpp"


namespace
{

std::filesystem::path clean_database_path(
  const std::string & name)
{
  const std::filesystem::path root{
    SAVO_LOCATIONS_TEST_DB_DIR};

  std::filesystem::create_directories(root);

  const auto database = root / name;

  std::filesystem::remove(database);
  std::filesystem::remove(
    database.string() + "-wal");

  std::filesystem::remove(
    database.string() + "-shm");

  return database;
}


savo_locations::PoseData make_pose(
  const double x,
  const double y,
  const double yaw_quaternion_z = 0.0,
  const double yaw_quaternion_w = 1.0)
{
  savo_locations::PoseData pose;

  pose.frame_id = "map";
  pose.x = x;
  pose.y = y;
  pose.z = 0.0;

  pose.qz = yaw_quaternion_z;
  pose.qw = yaw_quaternion_w;

  return pose;
}


savo_locations::LocationRecordData
make_location()
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::kApproved;

  record.enabled = true;
  record.record_revision = 3U;

  record.location.location_id = "A201";
  record.location.display_name = "Room A201";

  record.location.aliases = {
    "A 201",
    "East classroom",
  };

  record.location.semantic_type =
    "classroom";

  record.location.map.map_id =
    "campus_main";

  record.location.map.map_revision = 7U;

  record.location.map.map_release_id =
    "release-2026-07";

  record.location.approach_pose =
    make_pose(12.0, 7.5);

  record.location.confirmation_pose =
    make_pose(12.2, 7.7);

  record.location.tag.family = "tag36h11";
  record.location.tag.id = 27;

  record.location.tag_pose_map =
    make_pose(12.8, 8.1);

  record.location
    .arrival_confirmation_required =
    true;

  record.location.building = "Main";
  record.location.floor = "2";
  record.location.area = "East";
  record.location.notes = "Near laboratory";

  record.source_candidate_id =
    "candidate-27";

  return record;
}


savo_locations::CandidateRecordData
make_pending_candidate()
{
  savo_locations::CandidateRecordData record;

  record.state =
    savo_locations::CandidateState::
      kPendingReview;

  record.candidate_revision = 4U;

  record.candidate.candidate_id =
    "candidate-28";

  record.candidate.map.map_id =
    "campus_main";

  record.candidate.map.map_revision = 7U;

  record.candidate.map.map_release_id =
    "release-2026-07";

  record.candidate.tag.family = "tag36h11";
  record.candidate.tag.id = 28;

  record.candidate.tag_pose_map =
    make_pose(15.0, 9.0);

  record.candidate.detection_quality = 0.97;

  record.candidate.accepted_observations =
    10U;

  record.candidate.position_stddev_m =
    0.02;

  record.candidate.yaw_stddev_rad =
    0.03;

  record.candidate.approach_pose =
    make_pose(14.5, 8.5);

  record.candidate.confirmation_pose =
    make_pose(14.7, 8.7);

  record.candidate.suggested_location_id =
    "A202";

  record.candidate
    .suggested_display_name =
    "Room A202";

  record.candidate.suggested_aliases = {
    "A 202",
    "West classroom",
  };

  record.candidate
    .suggested_semantic_type =
    "classroom";

  record.candidate.building = "Main";
  record.candidate.floor = "2";
  record.candidate.area = "West";
  record.candidate.notes = "Candidate note";

  record.candidate.source_session_id =
    "mapping-session-1";

  record.candidate.source_component =
    "savo_mapping";

  return record;
}


savo_locations::CandidateRecordData
make_approved_candidate()
{
  auto record = make_pending_candidate();

  record.candidate.candidate_id =
    "candidate-27";

  record.candidate.tag.id = 27;

  record.candidate.suggested_location_id =
    "A201";

  record.candidate
    .suggested_display_name =
    "Room A201";

  record.candidate.suggested_aliases = {
    "A 201",
  };

  record.state =
    savo_locations::CandidateState::kApproved;

  record.candidate_revision = 5U;
  record.review_reason = "approved";
  record.approved_location_id = "A201";

  return record;
}


void open_and_migrate(
  savo_locations::SqliteStore * store)
{
  ASSERT_TRUE(store->open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store->migrate(&status).success);
}

}  // namespace


TEST(SqliteRepository, EmptySnapshotRoundTrips)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot input;

  ASSERT_TRUE(
    repository.save_snapshot(input).success);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository.load_snapshot(
      &output).success);

  EXPECT_TRUE(output.locations.empty());
  EXPECT_TRUE(output.candidates.empty());
}


TEST(SqliteRepository, LocationRoundTrips)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot input;
  input.locations.push_back(
    make_location());

  ASSERT_TRUE(
    repository.save_snapshot(input).success);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository.load_snapshot(
      &output).success);

  ASSERT_EQ(output.locations.size(), 1U);

  const auto & location =
    output.locations.front();

  EXPECT_EQ(
    location.location.location_id,
    "A201");

  EXPECT_EQ(location.record_revision, 3U);

  EXPECT_EQ(
    location.location.aliases.size(),
    2U);

  EXPECT_EQ(
    location.location.aliases[1],
    "East classroom");

  EXPECT_EQ(
    location.location.map.map_revision,
    7U);

  ASSERT_TRUE(
    location.location
      .confirmation_pose
      .has_value());

  ASSERT_TRUE(
    location.location
      .tag_pose_map
      .has_value());

  EXPECT_EQ(
    location.location.tag.id,
    27);
}


TEST(SqliteRepository, PendingCandidateRoundTrips)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot input;

  input.candidates.push_back(
    make_pending_candidate());

  ASSERT_TRUE(
    repository.save_snapshot(input).success);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository.load_snapshot(
      &output).success);

  ASSERT_EQ(output.candidates.size(), 1U);

  const auto & candidate =
    output.candidates.front();

  EXPECT_EQ(
    candidate.state,
    savo_locations::CandidateState::
      kPendingReview);

  EXPECT_EQ(
    candidate.candidate_revision,
    4U);

  EXPECT_EQ(
    candidate.candidate
      .suggested_aliases
      .size(),
    2U);

  EXPECT_EQ(
    candidate.candidate
      .source_component,
    "savo_mapping");
}


TEST(SqliteRepository, ApprovedCandidateRoundTrips)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot input;

  input.locations.push_back(
    make_location());

  input.candidates.push_back(
    make_approved_candidate());

  ASSERT_TRUE(
    repository.save_snapshot(input).success);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository.load_snapshot(
      &output).success);

  ASSERT_EQ(output.candidates.size(), 1U);

  EXPECT_EQ(
    output.candidates.front().state,
    savo_locations::CandidateState::kApproved);

  EXPECT_EQ(
    output.candidates
      .front()
      .approved_location_id,
    "A201");
}


TEST(SqliteRepository, FileSnapshotSurvivesReopen)
{
  const auto path =
    clean_database_path(
      "repository_reopen.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    savo_locations::CatalogSnapshot input;

    input.locations.push_back(
      make_location());

    input.candidates.push_back(
      make_approved_candidate());

    ASSERT_TRUE(
      repository
        .save_snapshot(input)
        .success);
  }

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    savo_locations::CatalogSnapshot output;

    ASSERT_TRUE(
      repository
        .load_snapshot(&output)
        .success);

    EXPECT_EQ(output.locations.size(), 1U);
    EXPECT_EQ(output.candidates.size(), 1U);
  }
}


TEST(SqliteRepository, InvalidSnapshotDoesNotReplaceStoredData)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot initial;

  initial.locations.push_back(
    make_location());

  ASSERT_TRUE(
    repository
      .save_snapshot(initial)
      .success);

  savo_locations::CatalogSnapshot invalid;

  auto broken = make_location();

  broken.location.approach_pose.frame_id =
    "base_link";

  invalid.locations.push_back(broken);

  const auto result =
    repository.save_snapshot(invalid);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
      kValidationFailed);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository
      .load_snapshot(&output)
      .success);

  ASSERT_EQ(output.locations.size(), 1U);

  EXPECT_EQ(
    output.locations
      .front()
      .location
      .location_id,
    "A201");
}


TEST(SqliteRepository, RejectsIdentityConflict)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot snapshot;

  auto first = make_location();

  auto second = make_location();

  second.location.location_id = "B201";
  second.location.display_name = "Room B201";
  second.location.tag.id = 29;

  second.location.aliases = {
    "east-classroom",
  };

  snapshot.locations = {
    first,
    second,
  };

  const auto result =
    repository.save_snapshot(snapshot);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
      kIdentityConflict);
}


TEST(SqliteRepository, RejectsPendingCandidateTagConflict)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot snapshot;

  snapshot.locations.push_back(
    make_location());

  auto candidate =
    make_pending_candidate();

  candidate.candidate.tag.id = 27;

  snapshot.candidates.push_back(candidate);

  const auto result =
    repository.save_snapshot(snapshot);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
      kTagConflict);
}


TEST(SqliteRepository, RejectsOperationDuringStoreTransaction)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  ASSERT_TRUE(
    store.begin_immediate().success);

  savo_locations::CatalogSnapshot snapshot;

  const auto save_result =
    repository.save_snapshot(snapshot);

  EXPECT_FALSE(save_result.success);

  EXPECT_EQ(
    save_result.code,
    savo_locations::SnapshotCode::
      kTransactionActive);

  const auto rollback_result =
    store.rollback();

  EXPECT_TRUE(rollback_result.success);
}


TEST(SqliteRepository, DetectsCorruptPersistedLocation)
{
  const auto path =
    clean_database_path(
      "repository_corrupt.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    savo_locations::CatalogSnapshot snapshot;

    snapshot.locations.push_back(
      make_location());

    ASSERT_TRUE(
      repository
        .save_snapshot(snapshot)
        .success);
  }

  sqlite3 * raw = nullptr;

  ASSERT_EQ(
    sqlite3_open_v2(
      path.string().c_str(),
      &raw,
      SQLITE_OPEN_READWRITE,
      nullptr),
    SQLITE_OK);

  ASSERT_EQ(
    sqlite3_exec(
      raw,
      "UPDATE locations SET "
      "approach_qx=0.0,"
      "approach_qy=0.0,"
      "approach_qz=0.0,"
      "approach_qw=0.0;",
      nullptr,
      nullptr,
      nullptr),
    SQLITE_OK);

  ASSERT_EQ(
    sqlite3_close(raw),
    SQLITE_OK);

  savo_locations::SqliteStore store{
    path.string()};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot output;

  const auto result =
    repository.load_snapshot(&output);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
      kCorruptData);
}


TEST(SqliteRepository, NewSnapshotAtomicallyReplacesOldSnapshot)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot first;

  first.locations.push_back(
    make_location());

  ASSERT_TRUE(
    repository.save_snapshot(first).success);

  savo_locations::CatalogSnapshot second;

  second.candidates.push_back(
    make_pending_candidate());

  ASSERT_TRUE(
    repository.save_snapshot(second).success);

  savo_locations::CatalogSnapshot output;

  ASSERT_TRUE(
    repository.load_snapshot(
      &output).success);

  EXPECT_TRUE(output.locations.empty());

  ASSERT_EQ(output.candidates.size(), 1U);

  EXPECT_EQ(
    output.candidates
      .front()
      .candidate
      .candidate_id,
    "candidate-28");
}


TEST(SqliteRepository, ReasonStringsAreStable)
{
  using savo_locations::SnapshotCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(
      SnapshotCode::kValidationFailed),
    "validation_failed");

  EXPECT_EQ(
    to_string(
      SnapshotCode::kTransactionActive),
    "transaction_active");

  EXPECT_EQ(
    to_string(
      SnapshotCode::kCorruptData),
    "corrupt_data");
}


TEST(SqliteRepository, CandidateRejectionCommitsSnapshotAndEventAtomically)
{
  savo_locations::SqliteStore store{":memory:"};
  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{store};

  savo_locations::CatalogSnapshot current;
  current.candidates.push_back(make_pending_candidate());
  ASSERT_TRUE(repository.save_snapshot(current).success);

  auto rejected = current.candidates.front();
  rejected.state = savo_locations::CandidateState::kRejected;
  rejected.candidate_revision = 5U;
  rejected.review_reason = "duplicate doorway marker";

  savo_locations::CatalogSnapshot post;
  post.candidates.push_back(rejected);

  savo_locations::CandidateRejectionCommit commit;
  commit.candidate_id = rejected.candidate.candidate_id;
  commit.expected_candidate_revision = 4U;
  commit.actor_id = "location_operator";
  commit.reason = rejected.review_reason;
  commit.post_rejection_snapshot = post;

  std::uint64_t event_sequence = 0U;
  const auto result = repository.commit_candidate_rejection(
    commit, &event_sequence);

  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_GT(event_sequence, 0U);

  savo_locations::CatalogSnapshot loaded;
  ASSERT_TRUE(repository.load_snapshot(&loaded).success);
  ASSERT_EQ(loaded.candidates.size(), 1U);
  EXPECT_EQ(
    loaded.candidates.front().state,
    savo_locations::CandidateState::kRejected);
  EXPECT_EQ(loaded.candidates.front().candidate_revision, 5U);
  EXPECT_EQ(
    loaded.candidates.front().review_reason,
    "duplicate doorway marker");

  std::vector<savo_locations::PersistenceEvent> events;
  ASSERT_TRUE(repository.list_events(0U, 10U, &events).success);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(
    events.front().event_type,
    savo_locations::PersistenceEventType::kCandidateRejected);
  EXPECT_EQ(events.front().entity_revision, 5U);
}
