#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/sqlite_schema.hpp"
#include "savo_locations/sqlite_store.hpp"


namespace
{

std::filesystem::path clean_database(
  const std::string & name)
{
  const std::filesystem::path root{
    SAVO_LOCATIONS_TEST_DB_DIR};

  std::filesystem::create_directories(root);

  const auto path = root / name;

  std::filesystem::remove(path);
  std::filesystem::remove(
    path.string() + "-wal");

  std::filesystem::remove(
    path.string() + "-shm");

  return path;
}


savo_locations::PoseData pose(
  const double x,
  const double y)
{
  savo_locations::PoseData value;

  value.frame_id = "map";
  value.x = x;
  value.y = y;
  value.qw = 1.0;

  return value;
}


savo_locations::CandidateRecordData
pending_candidate()
{
  savo_locations::CandidateRecordData record;

  record.state =
    savo_locations::CandidateState::
    kPendingReview;

  record.candidate_revision = 1U;

  auto & candidate = record.candidate;

  candidate.candidate_id = "candidate-31";
  candidate.map.map_id = "campus_main";
  candidate.map.map_revision = 7U;
  candidate.map.map_release_id = "release-7";
  candidate.tag.family = "tag36h11";
  candidate.tag.id = 31;
  candidate.tag_pose_map = pose(4.0, 5.0);
  candidate.detection_quality = 0.95;
  candidate.accepted_observations = 8U;
  candidate.position_stddev_m = 0.02;
  candidate.yaw_stddev_rad = 0.03;
  candidate.approach_pose = pose(3.5, 4.5);
  candidate.suggested_location_id = "B301";

  candidate.suggested_display_name =
    "Room B301";

  candidate.suggested_semantic_type =
    "classroom";

  candidate.source_session_id = "session-1";
  candidate.source_component = "savo_mapping";

  return record;
}


savo_locations::LocationRecordData
location(
  const bool enabled = true,
  const std::uint64_t revision = 1U)
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::
    kApproved;

  record.enabled = enabled;
  record.record_revision = revision;

  auto & value = record.location;

  value.location_id = "B301";
  value.display_name = "Room B301";
  value.semantic_type = "classroom";
  value.map.map_id = "campus_main";
  value.map.map_revision = 7U;
  value.map.map_release_id = "release-7";
  value.approach_pose = pose(3.5, 4.5);
  value.tag.family = "tag36h11";
  value.tag.id = 31;
  value.tag_pose_map = pose(4.0, 5.0);

  record.source_candidate_id =
    "candidate-31";

  return record;
}


void open_and_migrate(
  savo_locations::SqliteStore * store)
{
  ASSERT_TRUE(store->open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store->migrate(&status).success);

  EXPECT_EQ(status.current_version, 3U);
}


void reject_event_type(
  const std::filesystem::path & path,
  const int event_type)
{
  sqlite3 * database = nullptr;

  ASSERT_EQ(
    sqlite3_open_v2(
      path.string().c_str(),
      &database,
      SQLITE_OPEN_READWRITE,
      nullptr),
    SQLITE_OK);

  const std::string sql =
    "CREATE TRIGGER reject_event "
    "BEFORE INSERT ON location_events "
    "WHEN NEW.event_type=" +
    std::to_string(event_type) +
    " BEGIN "
    "SELECT RAISE(ABORT,'event rejected'); "
    "END;";

  ASSERT_EQ(
    sqlite3_exec(
      database,
      sql.c_str(),
      nullptr,
      nullptr,
      nullptr),
    SQLITE_OK);

  ASSERT_EQ(
    sqlite3_close(database),
    SQLITE_OK);
}


savo_locations::CandidateRegistrationCommit
registration_request()
{
  savo_locations::CandidateRegistrationCommit
    request;

  request.candidate_id = "candidate-31";
  request.actor_id = "savo_mapping";
  request.reason = "candidate confirmed";

  request
  .post_registration_snapshot
  .candidates
  .push_back(
      pending_candidate());

  return request;
}


savo_locations::LocationEnabledCommit
disable_request(
  const savo_locations::CatalogSnapshot &
  current)
{
  savo_locations::LocationEnabledCommit request;

  request.location_id = "B301";

  request.expected_record_revision = 1U;

  request.enabled = false;
  request.actor_id = "operator-ahnaf";
  request.reason = "maintenance";
  request.post_update_snapshot = current;

  auto & updated =
    request
    .post_update_snapshot
    .locations
    .front();

  updated.enabled = false;
  updated.record_revision = 2U;

  return request;
}

}  // namespace


TEST(
  PersistentMutationCommits,
  RegistrationCommitsSnapshotAndEvent)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  std::uint64_t sequence = 0U;

  ASSERT_TRUE(
    repository
    .commit_candidate_registration(
        registration_request(),
        &sequence)
    .success);

  EXPECT_EQ(sequence, 1U);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository
    .bootstrap(
        &snapshot,
        &report)
    .success);

  ASSERT_EQ(snapshot.candidates.size(), 1U);
  EXPECT_EQ(report.event_count, 1U);

  const auto duplicate =
    repository.commit_candidate_registration(
      registration_request(),
      nullptr);

  EXPECT_FALSE(duplicate.success);

  EXPECT_EQ(
    duplicate.code,
    savo_locations::SnapshotCode::
    kCandidateRegistrationDeltaInvalid);
}


TEST(
  PersistentMutationCommits,
  RegistrationEventFailureRollsBack)
{
  const auto path =
    clean_database(
      "registration_event_rollback.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);
  }

  reject_event_type(path, 3);

  savo_locations::SqliteStore store{
    path.string()};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  const auto result =
    repository.commit_candidate_registration(
      registration_request(),
      nullptr);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
    kEventJournalError);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository
    .bootstrap(
        &snapshot,
        &report)
    .success);

  EXPECT_TRUE(snapshot.candidates.empty());
  EXPECT_EQ(report.event_count, 0U);
}


TEST(
  PersistentMutationCommits,
  EnablementCommitsSnapshotAndEvent)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot current;

  current.locations.push_back(location());

  ASSERT_TRUE(
    repository
    .save_snapshot(current)
    .success);

  std::uint64_t sequence = 0U;

  ASSERT_TRUE(
    repository
    .commit_location_enabled(
        disable_request(current),
        &sequence)
    .success);

  EXPECT_EQ(sequence, 1U);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository
    .bootstrap(
        &snapshot,
        &report)
    .success);

  ASSERT_EQ(snapshot.locations.size(), 1U);
  EXPECT_FALSE(snapshot.locations.front().enabled);

  EXPECT_EQ(
    snapshot
    .locations
    .front()
    .record_revision,
    2U);

  EXPECT_EQ(report.event_count, 1U);
}


TEST(
  PersistentMutationCommits,
  EnablementRejectsStaleAndNoOp)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot current;

  current.locations.push_back(location());

  ASSERT_TRUE(
    repository
    .save_snapshot(current)
    .success);

  auto stale = disable_request(current);

  stale.expected_record_revision = 2U;

  const auto stale_result =
    repository.commit_location_enabled(
      stale,
      nullptr);

  EXPECT_FALSE(stale_result.success);

  EXPECT_EQ(
    stale_result.code,
    savo_locations::SnapshotCode::
    kStaleRevision);

  savo_locations::LocationEnabledCommit no_op;

  no_op.location_id = "B301";

  no_op.expected_record_revision = 1U;

  no_op.enabled = true;
  no_op.actor_id = "operator-ahnaf";
  no_op.reason = "no-op";
  no_op.post_update_snapshot = current;

  const auto no_op_result =
    repository.commit_location_enabled(
      no_op,
      nullptr);

  EXPECT_FALSE(no_op_result.success);

  EXPECT_EQ(
    no_op_result.code,
    savo_locations::SnapshotCode::
    kLocationEnabledDeltaInvalid);
}


TEST(
  PersistentMutationCommits,
  EnablementEventFailureRollsBack)
{
  const auto path =
    clean_database(
      "enablement_event_rollback.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    savo_locations::CatalogSnapshot current;

    current.locations.push_back(location());

    ASSERT_TRUE(
      repository
      .save_snapshot(current)
      .success);
  }

  reject_event_type(path, 5);

  savo_locations::SqliteStore store{
    path.string()};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  savo_locations::CatalogSnapshot current;

  ASSERT_TRUE(
    repository
    .load_snapshot(&current)
    .success);

  const auto result =
    repository.commit_location_enabled(
      disable_request(current),
      nullptr);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
    kEventJournalError);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository
    .bootstrap(
        &snapshot,
        &report)
    .success);

  EXPECT_TRUE(snapshot.locations.front().enabled);

  EXPECT_EQ(
    snapshot
    .locations
    .front()
    .record_revision,
    1U);

  EXPECT_EQ(report.event_count, 0U);
}


TEST(
  PersistentMutationCommits,
  ResultStringsAreStable)
{
  EXPECT_EQ(
    savo_locations::to_string(
      savo_locations::SnapshotCode::
      kCandidateRegistrationDeltaInvalid),
    "candidate_registration_delta_invalid");

  EXPECT_EQ(
    savo_locations::to_string(
      savo_locations::SnapshotCode::
      kLocationEnabledDeltaInvalid),
    "location_enabled_delta_invalid");
}
