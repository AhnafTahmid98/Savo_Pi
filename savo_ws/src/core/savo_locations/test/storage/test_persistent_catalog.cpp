#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>
#include <vector>

#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/sqlite_schema.hpp"
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
  const double y)
{
  savo_locations::PoseData pose;

  pose.frame_id = "map";
  pose.x = x;
  pose.y = y;
  pose.qw = 1.0;

  return pose;
}


savo_locations::CandidateRecordData
make_pending_candidate()
{
  savo_locations::CandidateRecordData record;

  record.state =
    savo_locations::CandidateState::
    kPendingReview;

  record.candidate_revision = 1U;

  record.candidate.candidate_id =
    "candidate-27";

  record.candidate.map.map_id =
    "campus_main";

  record.candidate.map.map_revision = 7U;

  record.candidate.map.map_release_id =
    "release-2026-07";

  record.candidate.tag.family =
    "tag36h11";

  record.candidate.tag.id = 27;

  record.candidate.tag_pose_map =
    make_pose(12.8, 8.1);

  record.candidate.detection_quality =
    0.98;

  record.candidate.accepted_observations =
    12U;

  record.candidate.position_stddev_m =
    0.02;

  record.candidate.yaw_stddev_rad =
    0.03;

  record.candidate.approach_pose =
    make_pose(12.0, 7.5);

  record.candidate.confirmation_pose =
    make_pose(12.2, 7.7);

  record.candidate.suggested_location_id =
    "A201";

  record.candidate.suggested_display_name =
    "Room A201";

  record.candidate.suggested_aliases = {
    "A 201",
  };

  record.candidate.suggested_semantic_type =
    "classroom";

  record.candidate.building = "Main";
  record.candidate.floor = "2";
  record.candidate.area = "East";

  record.candidate.source_session_id =
    "mapping-session-1";

  record.candidate.source_component =
    "savo_mapping";

  return record;
}


savo_locations::LocationRecordData
make_approved_location()
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::
    kApproved;

  record.enabled = true;
  record.record_revision = 1U;

  record.location.location_id = "A201";
  record.location.display_name = "Room A201";

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

  record.location.tag.family =
    "tag36h11";

  record.location.tag.id = 27;

  record.location.tag_pose_map =
    make_pose(12.8, 8.1);

  record.location
  .arrival_confirmation_required =
    true;

  record.location.building = "Main";
  record.location.floor = "2";
  record.location.area = "East";

  record.source_candidate_id =
    "candidate-27";

  return record;
}


savo_locations::CatalogSnapshot
make_pending_snapshot()
{
  savo_locations::CatalogSnapshot snapshot;

  snapshot.candidates.push_back(
    make_pending_candidate());

  return snapshot;
}


savo_locations::CatalogSnapshot
make_approved_snapshot()
{
  savo_locations::CatalogSnapshot snapshot;

  auto candidate =
    make_pending_candidate();

  candidate.state =
    savo_locations::CandidateState::
    kApproved;

  candidate.candidate_revision = 2U;
  candidate.review_reason = "approved";

  candidate.approved_location_id =
    "A201";

  snapshot.locations.push_back(
    make_approved_location());

  snapshot.candidates.push_back(
    candidate);

  return snapshot;
}


void open_and_migrate(
  savo_locations::SqliteStore * store)
{
  ASSERT_TRUE(store->open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store->migrate(&status).success);

  EXPECT_EQ(
    status.current_version,
    savo_locations::
    kSupportedSqliteSchemaVersion);
}


savo_locations::CandidateApprovalCommit
make_commit()
{
  savo_locations::CandidateApprovalCommit request;

  request.candidate_id = "candidate-27";

  request.expected_candidate_revision =
    1U;

  request.approved_location_id = "A201";
  request.actor_id = "operator-ahnaf";

  request.reason =
    "mapping evidence approved";

  request.payload_json =
    R"({"source":"operator_app"})";

  request.post_approval_snapshot =
    make_approved_snapshot();

  return request;
}

}  // namespace


TEST(PersistentCatalog, MigratesSchemaOneToTwo)
{
  const auto path =
    clean_database_path(
      "migration_v1_to_v2.sqlite3");

  sqlite3 * raw = nullptr;

  ASSERT_EQ(
    sqlite3_open_v2(
      path.string().c_str(),
      &raw,
      SQLITE_OPEN_READWRITE |
      SQLITE_OPEN_CREATE,
      nullptr),
    SQLITE_OK);

  const std::string migration_001{
    savo_locations::kMigration001Sql};

  ASSERT_EQ(
    sqlite3_exec(
      raw,
      migration_001.c_str(),
      nullptr,
      nullptr,
      nullptr),
    SQLITE_OK);

  ASSERT_EQ(
    sqlite3_exec(
      raw,
      "INSERT INTO schema_migrations("
      "schema_version,"
      "migration_name,"
      "applied_at_unix_ns"
      ") VALUES(1,'001_initial_schema',1);"
      "PRAGMA user_version=1;",
      nullptr,
      nullptr,
      nullptr),
    SQLITE_OK);

  ASSERT_EQ(
    sqlite3_close(raw),
    SQLITE_OK);

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  EXPECT_EQ(status.previous_version, 1U);
  EXPECT_EQ(status.current_version, 3U);
  EXPECT_TRUE(status.migration_applied);
}


TEST(PersistentCatalog, BootstrapReportsCatalogState)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  ASSERT_TRUE(
    repository
    .save_snapshot(
        make_pending_snapshot())
    .success);

  savo_locations::PersistenceEvent event;

  event.event_type =
    savo_locations::PersistenceEventType::
    kCandidateRegistered;

  event.candidate_id = "candidate-27";
  event.entity_revision = 1U;
  event.actor_id = "savo_mapping";
  event.reason = "candidate registered";

  ASSERT_TRUE(
    repository
    .append_event(event, nullptr)
    .success);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository
    .bootstrap(
        &snapshot,
        &report)
    .success);

  EXPECT_TRUE(report.integrity_healthy);

  EXPECT_EQ(
    report.schema_version,
    3U);

  EXPECT_EQ(report.location_count, 0U);
  EXPECT_EQ(report.candidate_count, 1U);
  EXPECT_EQ(report.event_count, 1U);
  EXPECT_EQ(report.last_event_sequence, 1U);

  ASSERT_EQ(snapshot.candidates.size(), 1U);
}


TEST(PersistentCatalog, EventJournalIsOrdered)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  for (int index = 0; index < 3; ++index) {
    savo_locations::PersistenceEvent event;

    event.event_type =
      savo_locations::PersistenceEventType::
      kCandidateRegistered;

    event.candidate_id =
      "candidate-" +
      std::to_string(index);

    event.entity_revision = 1U;
    event.actor_id = "savo_mapping";
    event.reason = "candidate registered";

    ASSERT_TRUE(
      repository
      .append_event(event, nullptr)
      .success);
  }

  std::vector<
    savo_locations::PersistenceEvent> events;

  ASSERT_TRUE(
    repository
    .list_events(
        1U,
        2U,
        &events)
    .success);

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].sequence, 2U);
  EXPECT_EQ(events[1].sequence, 3U);
}


TEST(PersistentCatalog, EventJournalRejectsMutation)
{
  const auto path =
    clean_database_path(
      "append_only_events.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    savo_locations::PersistenceEvent event;

    event.event_type =
      savo_locations::PersistenceEventType::
      kCandidateRegistered;

    event.candidate_id = "candidate-27";
    event.entity_revision = 1U;
    event.actor_id = "savo_mapping";
    event.reason = "candidate registered";

    ASSERT_TRUE(
      repository
      .append_event(event, nullptr)
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

  EXPECT_EQ(
    sqlite3_exec(
      raw,
      "UPDATE location_events "
      "SET reason='changed';",
      nullptr,
      nullptr,
      nullptr),
    SQLITE_CONSTRAINT);

  EXPECT_EQ(
    sqlite3_exec(
      raw,
      "DELETE FROM location_events;",
      nullptr,
      nullptr,
      nullptr),
    SQLITE_CONSTRAINT);

  ASSERT_EQ(
    sqlite3_close(raw),
    SQLITE_OK);
}


TEST(PersistentCatalog, ApprovalCommitsSnapshotAndEvent)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  ASSERT_TRUE(
    repository
    .save_snapshot(
        make_pending_snapshot())
    .success);

  std::uint64_t sequence = 0U;

  const auto result =
    repository.commit_candidate_approval(
      make_commit(),
      &sequence);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(sequence, 1U);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(
      &snapshot,
      &report).success);

  ASSERT_EQ(snapshot.locations.size(), 1U);
  ASSERT_EQ(snapshot.candidates.size(), 1U);

  EXPECT_EQ(
    snapshot.candidates.front().state,
    savo_locations::CandidateState::
    kApproved);

  EXPECT_EQ(report.event_count, 1U);

  std::vector<
    savo_locations::PersistenceEvent> events;

  ASSERT_TRUE(
    repository.list_events(
      0U,
      10U,
      &events).success);

  ASSERT_EQ(events.size(), 1U);

  EXPECT_EQ(
    events.front().event_type,
    savo_locations::PersistenceEventType::
    kCandidateApproved);

  EXPECT_EQ(
    events.front().location_id,
    "A201");
}


TEST(PersistentCatalog, StaleApprovalDoesNotMutateDatabase)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  ASSERT_TRUE(
    repository
    .save_snapshot(
        make_pending_snapshot())
    .success);

  auto request = make_commit();

  request.expected_candidate_revision =
    99U;

  const auto result =
    repository.commit_candidate_approval(
      request,
      nullptr);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
    kStaleRevision);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(
      &snapshot,
      &report).success);

  EXPECT_TRUE(snapshot.locations.empty());

  ASSERT_EQ(snapshot.candidates.size(), 1U);

  EXPECT_EQ(
    snapshot.candidates.front().state,
    savo_locations::CandidateState::
    kPendingReview);

  EXPECT_EQ(report.event_count, 0U);
}


TEST(PersistentCatalog, InvalidApprovalDeltaRollsBack)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  open_and_migrate(&store);

  savo_locations::SqliteRepository repository{
    store};

  ASSERT_TRUE(
    repository
    .save_snapshot(
        make_pending_snapshot())
    .success);

  auto request = make_commit();

  request.post_approval_snapshot
  .locations
  .front()
  .record_revision = 4U;

  const auto result =
    repository.commit_candidate_approval(
      request,
      nullptr);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
    kApprovalDeltaInvalid);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(
      &snapshot,
      &report).success);

  EXPECT_TRUE(snapshot.locations.empty());
  EXPECT_EQ(report.event_count, 0U);
}


TEST(PersistentCatalog, EventFailureRollsBackApproval)
{
  const auto path =
    clean_database_path(
      "approval_event_rollback.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    open_and_migrate(&store);

    savo_locations::SqliteRepository repository{
      store};

    ASSERT_TRUE(
      repository
      .save_snapshot(
          make_pending_snapshot())
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
      "CREATE TRIGGER reject_approval_event "
      "BEFORE INSERT ON location_events "
      "WHEN NEW.event_type=2 "
      "BEGIN "
      "SELECT RAISE(ABORT,'event rejected'); "
      "END;",
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

  const auto result =
    repository.commit_candidate_approval(
      make_commit(),
      nullptr);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::SnapshotCode::
    kEventJournalError);

  savo_locations::CatalogSnapshot snapshot;
  savo_locations::BootstrapReport report;

  ASSERT_TRUE(
    repository.bootstrap(
      &snapshot,
      &report).success);

  EXPECT_TRUE(snapshot.locations.empty());

  ASSERT_EQ(snapshot.candidates.size(), 1U);

  EXPECT_EQ(
    snapshot.candidates.front().state,
    savo_locations::CandidateState::
    kPendingReview);

  EXPECT_EQ(report.event_count, 0U);
}


TEST(PersistentCatalog, ResultStringsAreStable)
{
  using savo_locations::SnapshotCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(
      SnapshotCode::kStaleRevision),
    "stale_revision");

  EXPECT_EQ(
    to_string(
      SnapshotCode::kApprovalDeltaInvalid),
    "approval_delta_invalid");

  EXPECT_EQ(
    to_string(
      SnapshotCode::kEventJournalError),
    "event_journal_error");
}
