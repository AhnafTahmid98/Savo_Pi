#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
import tarfile
import textwrap
import xml.etree.ElementTree as ET


ROOT = (
    Path.home()
    / "Savo_Pi"
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


def clean(text: str) -> str:
    return textwrap.dedent(text).lstrip()


def write(relative: str, content: str) -> None:
    path = ROOT / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(clean(content), encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def replace_function(
    text: str,
    signature: str,
    replacement: str,
) -> str:
    signature_position = text.find(signature)

    if signature_position < 0:
        raise RuntimeError(
            f"Function signature not found: {signature}"
        )

    line_start = (
        text.rfind("\n", 0, signature_position) + 1
    )

    opening_brace = text.find(
        "{",
        signature_position,
    )

    if opening_brace < 0:
        raise RuntimeError(
            f"Opening brace not found: {signature}"
        )

    depth = 0
    index = opening_brace
    state = "normal"
    quote = ""

    while index < len(text):
        character = text[index]

        next_character = (
            text[index + 1]
            if index + 1 < len(text)
            else ""
        )

        if state == "line_comment":
            if character == "\n":
                state = "normal"

            index += 1
            continue

        if state == "block_comment":
            if (
                character == "*"
                and next_character == "/"
            ):
                state = "normal"
                index += 2
                continue

            index += 1
            continue

        if state == "string":
            if character == "\\":
                index += 2
                continue

            if character == quote:
                state = "normal"
                quote = ""

            index += 1
            continue

        if (
            character == "/"
            and next_character == "/"
        ):
            state = "line_comment"
            index += 2
            continue

        if (
            character == "/"
            and next_character == "*"
        ):
            state = "block_comment"
            index += 2
            continue

        if character in ('"', "'"):
            state = "string"
            quote = character
            index += 1
            continue

        if character == "{":
            depth += 1

        elif character == "}":
            depth -= 1

            if depth == 0:
                return (
                    text[:line_start]
                    + replacement.rstrip()
                    + "\n"
                    + text[index + 1:]
                )

        index += 1

    raise RuntimeError(
        f"Closing brace not found: {signature}"
    )


def update_package_version() -> None:
    package_xml = ROOT / "package.xml"

    tree = ET.parse(package_xml)
    package = tree.getroot()

    version = package.find("version")

    if version is None:
        raise RuntimeError(
            "package.xml has no version element"
        )

    version.text = "0.7.0"

    ET.indent(tree, space="  ")

    tree.write(
        package_xml,
        encoding="utf-8",
        xml_declaration=True,
    )


MIGRATE_FUNCTION = r'''
StorageResult SqliteStore::migrate(
  SchemaStatus * status)
{
  if (status == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "schema status output is required");
  }

  std::lock_guard<std::mutex> lock{mutex_};

  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  const auto owner_result =
    check_transaction_owner_locked();

  if (!owner_result.success) {
    return owner_result;
  }

  int current_version = 0;

  const auto version_result =
    query_single_int_locked(
      "PRAGMA user_version;",
      &current_version);

  if (!version_result.success) {
    return version_result;
  }

  if (current_version < 0) {
    return failure(
      StorageCode::kMigrationFailed,
      SQLITE_CORRUPT,
      "schema version is negative");
  }

  status->previous_version =
    static_cast<std::uint32_t>(
      current_version);

  status->current_version =
    status->previous_version;

  status->supported_version =
    kSupportedSqliteSchemaVersion;

  status->migration_applied = false;

  if (
    status->previous_version >
    kSupportedSqliteSchemaVersion)
  {
    return failure(
      StorageCode::kSchemaTooNew,
      SQLITE_ERROR,
      "database schema is newer than this package");
  }

  if (
    status->previous_version ==
    kSupportedSqliteSchemaVersion)
  {
    return success("schema already current");
  }

  auto begin_result =
    execute_locked("BEGIN IMMEDIATE;");

  if (!begin_result.success) {
    return failure(
      StorageCode::kMigrationFailed,
      begin_result.sqlite_code,
      begin_result.reason);
  }

  transaction_active_ = true;

  transaction_owner_ =
    std::this_thread::get_id();

  auto rollback_migration =
    [this]()
    {
      static_cast<void>(rollback_locked());
    };

  std::uint32_t working_version =
    status->previous_version;

  if (working_version == 0U) {
    const auto schema_result =
      execute_locked(kMigration001Sql);

    if (!schema_result.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        schema_result.sqlite_code,
        schema_result.reason);
    }

    const std::string migration_insert =
      "INSERT INTO schema_migrations("
      "schema_version,"
      "migration_name,"
      "applied_at_unix_ns"
      ") VALUES("
      "1,"
      "'001_initial_schema',"
      + std::to_string(unix_time_ns()) +
      ");";

    const auto insert_result =
      execute_locked(migration_insert);

    if (!insert_result.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        insert_result.sqlite_code,
        insert_result.reason);
    }

    const auto version_update =
      execute_locked("PRAGMA user_version=1;");

    if (!version_update.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        version_update.sqlite_code,
        version_update.reason);
    }

    working_version = 1U;
  }

  if (working_version == 1U) {
    const auto schema_result =
      execute_locked(kMigration002Sql);

    if (!schema_result.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        schema_result.sqlite_code,
        schema_result.reason);
    }

    const std::string migration_insert =
      "INSERT INTO schema_migrations("
      "schema_version,"
      "migration_name,"
      "applied_at_unix_ns"
      ") VALUES("
      "2,"
      "'002_append_only_event_journal',"
      + std::to_string(unix_time_ns()) +
      ");";

    const auto insert_result =
      execute_locked(migration_insert);

    if (!insert_result.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        insert_result.sqlite_code,
        insert_result.reason);
    }

    const auto version_update =
      execute_locked("PRAGMA user_version=2;");

    if (!version_update.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        version_update.sqlite_code,
        version_update.reason);
    }

    working_version = 2U;
  }

  if (
    working_version !=
    kSupportedSqliteSchemaVersion)
  {
    rollback_migration();

    return failure(
      StorageCode::kMigrationFailed,
      SQLITE_ERROR,
      "no migration path exists for schema version");
  }

  const auto commit_result =
    execute_locked("COMMIT;");

  if (!commit_result.success) {
    rollback_migration();

    return failure(
      StorageCode::kMigrationFailed,
      commit_result.sqlite_code,
      commit_result.reason);
  }

  transaction_active_ = false;
  transaction_owner_ = std::thread::id{};

  status->current_version =
    kSupportedSqliteSchemaVersion;

  status->migration_applied = true;

  return success("schema migrations applied");
}
'''


REPOSITORY_HELPERS = r'''

bool same_pose(
  const PoseData & lhs,
  const PoseData & rhs)
{
  return
    lhs.frame_id == rhs.frame_id &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.qx == rhs.qx &&
    lhs.qy == rhs.qy &&
    lhs.qz == rhs.qz &&
    lhs.qw == rhs.qw;
}


bool same_optional_pose(
  const std::optional<PoseData> & lhs,
  const std::optional<PoseData> & rhs)
{
  if (
    lhs.has_value() !=
    rhs.has_value())
  {
    return false;
  }

  return
    !lhs.has_value() ||
    same_pose(
      lhs.value(),
      rhs.value());
}


bool same_map(
  const MapContext & lhs,
  const MapContext & rhs)
{
  return
    lhs.map_id == rhs.map_id &&
    lhs.map_revision == rhs.map_revision &&
    lhs.map_release_id == rhs.map_release_id;
}


bool same_tag(
  const TagBinding & lhs,
  const TagBinding & rhs)
{
  return
    lhs.family == rhs.family &&
    lhs.id == rhs.id;
}


bool same_location_draft(
  const LocationDraft & lhs,
  const LocationDraft & rhs)
{
  return
    lhs.location_id == rhs.location_id &&
    lhs.display_name == rhs.display_name &&
    lhs.aliases == rhs.aliases &&
    lhs.semantic_type == rhs.semantic_type &&
    same_map(lhs.map, rhs.map) &&
    same_pose(
      lhs.approach_pose,
      rhs.approach_pose) &&
    same_optional_pose(
      lhs.confirmation_pose,
      rhs.confirmation_pose) &&
    same_tag(lhs.tag, rhs.tag) &&
    same_optional_pose(
      lhs.tag_pose_map,
      rhs.tag_pose_map) &&
    lhs.arrival_confirmation_required ==
      rhs.arrival_confirmation_required &&
    lhs.building == rhs.building &&
    lhs.floor == rhs.floor &&
    lhs.area == rhs.area &&
    lhs.notes == rhs.notes;
}


bool same_location_record(
  const LocationRecordData & lhs,
  const LocationRecordData & rhs)
{
  return
    lhs.state == rhs.state &&
    lhs.enabled == rhs.enabled &&
    lhs.record_revision ==
      rhs.record_revision &&
    lhs.source_candidate_id ==
      rhs.source_candidate_id &&
    same_location_draft(
      lhs.location,
      rhs.location);
}


bool same_candidate_draft(
  const CandidateDraft & lhs,
  const CandidateDraft & rhs)
{
  return
    lhs.candidate_id == rhs.candidate_id &&
    same_map(lhs.map, rhs.map) &&
    same_tag(lhs.tag, rhs.tag) &&
    same_pose(
      lhs.tag_pose_map,
      rhs.tag_pose_map) &&
    lhs.detection_quality ==
      rhs.detection_quality &&
    lhs.accepted_observations ==
      rhs.accepted_observations &&
    lhs.position_stddev_m ==
      rhs.position_stddev_m &&
    lhs.yaw_stddev_rad ==
      rhs.yaw_stddev_rad &&
    same_optional_pose(
      lhs.approach_pose,
      rhs.approach_pose) &&
    same_optional_pose(
      lhs.confirmation_pose,
      rhs.confirmation_pose) &&
    lhs.suggested_location_id ==
      rhs.suggested_location_id &&
    lhs.suggested_display_name ==
      rhs.suggested_display_name &&
    lhs.suggested_aliases ==
      rhs.suggested_aliases &&
    lhs.suggested_semantic_type ==
      rhs.suggested_semantic_type &&
    lhs.building == rhs.building &&
    lhs.floor == rhs.floor &&
    lhs.area == rhs.area &&
    lhs.notes == rhs.notes &&
    lhs.source_session_id ==
      rhs.source_session_id &&
    lhs.source_component ==
      rhs.source_component;
}


bool same_candidate_record(
  const CandidateRecordData & lhs,
  const CandidateRecordData & rhs)
{
  return
    lhs.state == rhs.state &&
    lhs.candidate_revision ==
      rhs.candidate_revision &&
    lhs.review_reason ==
      rhs.review_reason &&
    lhs.approved_location_id ==
      rhs.approved_location_id &&
    same_candidate_draft(
      lhs.candidate,
      rhs.candidate);
}


const CandidateRecordData * find_candidate(
  const CatalogSnapshot & snapshot,
  const std::string_view candidate_id)
{
  for (
    const auto & candidate :
    snapshot.candidates)
  {
    if (
      candidate.candidate.candidate_id ==
      candidate_id)
    {
      return &candidate;
    }
  }

  return nullptr;
}


const LocationRecordData * find_location(
  const CatalogSnapshot & snapshot,
  const std::string_view location_id)
{
  for (
    const auto & location :
    snapshot.locations)
  {
    if (
      location.location.location_id ==
      location_id)
    {
      return &location;
    }
  }

  return nullptr;
}


int replace_snapshot_rows(
  sqlite3 * database,
  const CatalogSnapshot & snapshot,
  const std::int64_t timestamp)
{
  int code = execute_sql(
    database,
    "DELETE FROM candidate_aliases;"
    "DELETE FROM location_candidates;"
    "DELETE FROM location_aliases;"
    "DELETE FROM locations;");

  if (code != SQLITE_OK) {
    return code;
  }

  for (
    const auto & location :
    snapshot.locations)
  {
    code = insert_location(
      database,
      location,
      timestamp);

    if (code != SQLITE_OK) {
      return code;
    }

    code = insert_location_aliases(
      database,
      location);

    if (code != SQLITE_OK) {
      return code;
    }
  }

  for (
    const auto & candidate :
    snapshot.candidates)
  {
    code = insert_candidate(
      database,
      candidate,
      timestamp);

    if (code != SQLITE_OK) {
      return code;
    }

    code = insert_candidate_aliases(
      database,
      candidate);

    if (code != SQLITE_OK) {
      return code;
    }
  }

  return SQLITE_OK;
}


int insert_event_row(
  sqlite3 * database,
  const PersistenceEvent & event,
  std::uint64_t * sequence)
{
  static constexpr const char * sql =
    "INSERT INTO location_events("
    "event_time_unix_ns,"
    "event_type,"
    "candidate_id,"
    "location_id,"
    "entity_revision,"
    "actor_id,"
    "reason,"
    "event_payload_json"
    ") VALUES(?1,?2,?3,?4,?5,?6,?7,?8);";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return statement.prepare_code();
  }

  auto * raw = statement.get();

  const std::int64_t event_time =
    event.event_time_unix_ns > 0 ?
      event.event_time_unix_ns :
      unix_time_ns();

  int code = sqlite3_bind_int64(
    raw,
    1,
    event_time);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      2,
      static_cast<int>(event.event_type));
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      3,
      event.candidate_id);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      4,
      event.location_id);
  }

  if (code == SQLITE_OK) {
    code = bind_int64(
      raw,
      5,
      event.entity_revision);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      6,
      event.actor_id);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      7,
      event.reason);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      8,
      event.payload_json.empty() ?
        std::string_view{"{}"} :
        std::string_view{event.payload_json});
  }

  if (code != SQLITE_OK) {
    return code;
  }

  code = sqlite3_step(raw);

  if (code != SQLITE_DONE) {
    return sqlite3_extended_errcode(database);
  }

  const sqlite3_int64 inserted =
    sqlite3_last_insert_rowid(database);

  if (inserted <= 0) {
    return SQLITE_CORRUPT;
  }

  if (sequence != nullptr) {
    *sequence =
      static_cast<std::uint64_t>(inserted);
  }

  return SQLITE_OK;
}


SnapshotResult validate_approval_delta(
  const CatalogSnapshot & current,
  const CandidateApprovalCommit & request)
{
  const auto & post =
    request.post_approval_snapshot;

  const auto * current_candidate =
    find_candidate(
      current,
      request.candidate_id);

  if (current_candidate == nullptr) {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_NOTFOUND,
      "approval candidate does not exist");
  }

  if (
    current_candidate->state !=
    CandidateState::kPendingReview)
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approval candidate is not pending");
  }

  if (
    current_candidate->candidate_revision !=
    request.expected_candidate_revision)
  {
    return snapshot_failure(
      SnapshotCode::kStaleRevision,
      SQLITE_BUSY,
      "candidate revision is stale");
  }

  const auto * approved_candidate =
    find_candidate(
      post,
      request.candidate_id);

  if (approved_candidate == nullptr) {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "post-approval candidate is missing");
  }

  if (
    approved_candidate->state !=
      CandidateState::kApproved ||
    approved_candidate->candidate_revision !=
      request.expected_candidate_revision + 1U ||
    approved_candidate->approved_location_id !=
      request.approved_location_id ||
    !same_candidate_draft(
      current_candidate->candidate,
      approved_candidate->candidate))
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "candidate approval transition is invalid");
  }

  if (
    post.candidates.size() !=
    current.candidates.size())
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approval changed the candidate set");
  }

  for (
    const auto & previous :
    current.candidates)
  {
    if (
      previous.candidate.candidate_id ==
      request.candidate_id)
    {
      continue;
    }

    const auto * next =
      find_candidate(
        post,
        previous.candidate.candidate_id);

    if (
      next == nullptr ||
      !same_candidate_record(
        previous,
        *next))
    {
      return snapshot_failure(
        SnapshotCode::kApprovalDeltaInvalid,
        SQLITE_CONSTRAINT,
        "approval changed an unrelated candidate");
    }
  }

  if (
    find_location(
      current,
      request.approved_location_id) != nullptr)
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approval location already exists");
  }

  if (
    post.locations.size() !=
    current.locations.size() + 1U)
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approval must create exactly one location");
  }

  for (
    const auto & previous :
    current.locations)
  {
    const auto * next =
      find_location(
        post,
        previous.location.location_id);

    if (
      next == nullptr ||
      !same_location_record(
        previous,
        *next))
    {
      return snapshot_failure(
        SnapshotCode::kApprovalDeltaInvalid,
        SQLITE_CONSTRAINT,
        "approval changed an existing location");
    }
  }

  const auto * approved_location =
    find_location(
      post,
      request.approved_location_id);

  if (approved_location == nullptr) {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approved location is missing");
  }

  if (
    approved_location->state !=
      LocationState::kApproved ||
    !approved_location->enabled ||
    approved_location->record_revision != 1U ||
    approved_location->source_candidate_id !=
      request.candidate_id ||
    !same_map(
      approved_location->location.map,
      approved_candidate->candidate.map) ||
    !same_tag(
      approved_location->location.tag,
      approved_candidate->candidate.tag) ||
    !approved_location
      ->location
      .tag_pose_map
      .has_value() ||
    !same_pose(
      approved_location
        ->location
        .tag_pose_map
        .value(),
      approved_candidate
        ->candidate
        .tag_pose_map))
  {
    return snapshot_failure(
      SnapshotCode::kApprovalDeltaInvalid,
      SQLITE_CONSTRAINT,
      "approved location does not match candidate evidence");
  }

  return snapshot_success(
    "approval delta is valid");
}
'''


REPOSITORY_METHODS = r'''

SnapshotResult SqliteRepository::bootstrap(
  CatalogSnapshot * snapshot,
  BootstrapReport * report) const
{
  if (
    snapshot == nullptr ||
    report == nullptr)
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "bootstrap outputs are required");
  }

  std::uint32_t schema_version = 0U;

  const auto version_result =
    store_.schema_version(
      &schema_version);

  if (!version_result.success) {
    return snapshot_failure(
      SnapshotCode::kStoreNotOpen,
      version_result.sqlite_code,
      version_result.reason);
  }

  IntegrityReport integrity;

  const auto integrity_result =
    store_.integrity_check(
      &integrity);

  if (!integrity_result.success) {
    return snapshot_failure(
      SnapshotCode::kCorruptData,
      integrity_result.sqlite_code,
      integrity_result.reason);
  }

  CatalogSnapshot loaded;

  const auto load_result =
    load_snapshot(&loaded);

  if (!load_result.success) {
    return load_result;
  }

  std::uint64_t event_count = 0U;
  std::uint64_t last_sequence = 0U;

  {
    std::lock_guard<std::mutex> lock{
      store_.mutex_};

    if (store_.database_ == nullptr) {
      return snapshot_failure(
        SnapshotCode::kStoreNotOpen,
        SQLITE_MISUSE,
        "SQLite store is not open");
    }

    Statement statement{
      store_.database_,
      "SELECT "
      "COUNT(*),"
      "COALESCE(MAX(event_sequence),0) "
      "FROM location_events;"};

    if (
      statement.prepare_code() !=
      SQLITE_OK)
    {
      return sqlite_failure(
        store_.database_,
        statement.prepare_code(),
        "could not prepare bootstrap event summary");
    }

    const int code =
      sqlite3_step(statement.get());

    if (code != SQLITE_ROW) {
      return sqlite_failure(
        store_.database_,
        sqlite3_extended_errcode(
          store_.database_),
        "could not read bootstrap event summary");
    }

    if (
      !column_uint64(
        statement.get(),
        0,
        &event_count) ||
      !column_uint64(
        statement.get(),
        1,
        &last_sequence))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid event journal counters");
    }
  }

  report->schema_version = schema_version;
  report->integrity_healthy = integrity.healthy;

  report->location_count =
    loaded.locations.size();

  report->candidate_count =
    loaded.candidates.size();

  report->event_count = event_count;
  report->last_event_sequence = last_sequence;

  *snapshot = std::move(loaded);

  return snapshot_success(
    "persistent catalog bootstrapped");
}


SnapshotResult SqliteRepository::append_event(
  const PersistenceEvent & event,
  std::uint64_t * sequence)
{
  if (
    event.event_type ==
      PersistenceEventType::kUnknown ||
    trim_ascii(event.actor_id).empty() ||
    trim_ascii(event.reason).empty())
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "event type, actor and reason are required");
  }

  std::lock_guard<std::mutex> lock{
    store_.mutex_};

  if (store_.database_ == nullptr) {
    return snapshot_failure(
      SnapshotCode::kStoreNotOpen,
      SQLITE_MISUSE,
      "SQLite store is not open");
  }

  if (store_.transaction_active_) {
    return snapshot_failure(
      SnapshotCode::kTransactionActive,
      SQLITE_BUSY,
      "SQLite store already has an active transaction");
  }

  int code = execute_sql(
    store_.database_,
    "BEGIN IMMEDIATE;");

  if (code != SQLITE_OK) {
    return sqlite_failure(
      store_.database_,
      code,
      "could not begin event transaction");
  }

  store_.transaction_active_ = true;

  store_.transaction_owner_ =
    std::this_thread::get_id();

  auto rollback =
    [&]()
    {
      static_cast<void>(
        execute_sql(
          store_.database_,
          "ROLLBACK;"));

      store_.transaction_active_ = false;
      store_.transaction_owner_ =
        std::thread::id{};
    };

  std::uint64_t inserted_sequence = 0U;

  code = insert_event_row(
    store_.database_,
    event,
    &inserted_sequence);

  if (code != SQLITE_OK) {
    rollback();

    return snapshot_failure(
      SnapshotCode::kEventJournalError,
      code,
      "could not append audit event");
  }

  code = execute_sql(
    store_.database_,
    "COMMIT;");

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not commit event transaction");
  }

  store_.transaction_active_ = false;
  store_.transaction_owner_ =
    std::thread::id{};

  if (sequence != nullptr) {
    *sequence = inserted_sequence;
  }

  return snapshot_success(
    "audit event appended");
}


SnapshotResult SqliteRepository::list_events(
  const std::uint64_t after_sequence,
  const std::size_t limit,
  std::vector<PersistenceEvent> * events) const
{
  if (
    events == nullptr ||
    limit == 0U ||
    limit > 1000U)
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "event output and limit 1..1000 are required");
  }

  std::lock_guard<std::mutex> lock{
    store_.mutex_};

  if (store_.database_ == nullptr) {
    return snapshot_failure(
      SnapshotCode::kStoreNotOpen,
      SQLITE_MISUSE,
      "SQLite store is not open");
  }

  if (store_.transaction_active_) {
    return snapshot_failure(
      SnapshotCode::kTransactionActive,
      SQLITE_BUSY,
      "cannot list events during an active transaction");
  }

  Statement statement{
    store_.database_,
    "SELECT "
    "event_sequence,"
    "event_time_unix_ns,"
    "event_type,"
    "candidate_id,"
    "location_id,"
    "entity_revision,"
    "actor_id,"
    "reason,"
    "event_payload_json "
    "FROM location_events "
    "WHERE event_sequence > ?1 "
    "ORDER BY event_sequence ASC "
    "LIMIT ?2;"};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return sqlite_failure(
      store_.database_,
      statement.prepare_code(),
      "could not prepare event list");
  }

  int code = bind_int64(
    statement.get(),
    1,
    after_sequence);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      statement.get(),
      2,
      static_cast<sqlite3_int64>(limit));
  }

  if (code != SQLITE_OK) {
    return sqlite_failure(
      store_.database_,
      code,
      "could not bind event list query");
  }

  std::vector<PersistenceEvent> loaded;

  while (
    (code = sqlite3_step(
      statement.get())) ==
    SQLITE_ROW)
  {
    PersistenceEvent event;

    if (
      !column_uint64(
        statement.get(),
        0,
        &event.sequence))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid event sequence");
    }

    event.event_time_unix_ns =
      sqlite3_column_int64(
        statement.get(),
        1);

    const int raw_type =
      sqlite3_column_int(
        statement.get(),
        2);

    if (
      raw_type <
        static_cast<int>(
          PersistenceEventType::
            kSnapshotReplaced) ||
      raw_type >
        static_cast<int>(
          PersistenceEventType::
            kLocationEnabledChanged))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid persisted event type");
    }

    event.event_type =
      static_cast<PersistenceEventType>(
        raw_type);

    event.candidate_id =
      column_text(
        statement.get(),
        3);

    event.location_id =
      column_text(
        statement.get(),
        4);

    if (
      !column_uint64(
        statement.get(),
        5,
        &event.entity_revision))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid event entity revision");
    }

    event.actor_id =
      column_text(
        statement.get(),
        6);

    event.reason =
      column_text(
        statement.get(),
        7);

    event.payload_json =
      column_text(
        statement.get(),
        8);

    loaded.push_back(
      std::move(event));
  }

  if (code != SQLITE_DONE) {
    return sqlite_failure(
      store_.database_,
      sqlite3_extended_errcode(
        store_.database_),
      "could not list audit events");
  }

  *events = std::move(loaded);

  return snapshot_success(
    "audit events listed");
}


SnapshotResult
SqliteRepository::commit_candidate_approval(
  const CandidateApprovalCommit & request,
  std::uint64_t * event_sequence)
{
  if (
    trim_ascii(request.candidate_id).empty() ||
    request.expected_candidate_revision == 0U ||
    trim_ascii(
      request.approved_location_id).empty() ||
    trim_ascii(request.actor_id).empty() ||
    trim_ascii(request.reason).empty())
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "approval identity, revision, actor and reason "
      "are required");
  }

  const auto post_validation =
    validate_snapshot(
      request.post_approval_snapshot);

  if (!post_validation.success) {
    return post_validation;
  }

  std::lock_guard<std::mutex> lock{
    store_.mutex_};

  if (store_.database_ == nullptr) {
    return snapshot_failure(
      SnapshotCode::kStoreNotOpen,
      SQLITE_MISUSE,
      "SQLite store is not open");
  }

  if (store_.transaction_active_) {
    return snapshot_failure(
      SnapshotCode::kTransactionActive,
      SQLITE_BUSY,
      "SQLite store already has an active transaction");
  }

  int code = execute_sql(
    store_.database_,
    "BEGIN IMMEDIATE;");

  if (code != SQLITE_OK) {
    return sqlite_failure(
      store_.database_,
      code,
      "could not begin approval transaction");
  }

  store_.transaction_active_ = true;

  store_.transaction_owner_ =
    std::this_thread::get_id();

  auto rollback =
    [&]()
    {
      static_cast<void>(
        execute_sql(
          store_.database_,
          "ROLLBACK;"));

      store_.transaction_active_ = false;
      store_.transaction_owner_ =
        std::thread::id{};
    };

  CatalogSnapshot current;

  auto result = read_locations(
    store_.database_,
    &current);

  if (!result.success) {
    rollback();
    return result;
  }

  result = read_candidates(
    store_.database_,
    &current);

  if (!result.success) {
    rollback();
    return result;
  }

  result = validate_snapshot(current);

  if (!result.success) {
    rollback();

    result.code =
      SnapshotCode::kCorruptData;

    result.sqlite_code =
      SQLITE_CORRUPT;

    result.reason =
      "persisted pre-approval catalog is invalid: " +
      result.reason;

    return result;
  }

  result = validate_approval_delta(
    current,
    request);

  if (!result.success) {
    rollback();
    return result;
  }

  const std::int64_t timestamp =
    unix_time_ns();

  code = replace_snapshot_rows(
    store_.database_,
    request.post_approval_snapshot,
    timestamp);

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not persist approval snapshot");
  }

  PersistenceEvent event;

  event.event_time_unix_ns = timestamp;

  event.event_type =
    PersistenceEventType::
      kCandidateApproved;

  event.candidate_id =
    request.candidate_id;

  event.location_id =
    request.approved_location_id;

  event.entity_revision =
    request.expected_candidate_revision + 1U;

  event.actor_id = request.actor_id;
  event.reason = request.reason;

  event.payload_json =
    request.payload_json.empty() ?
      "{}" :
      request.payload_json;

  std::uint64_t inserted_sequence = 0U;

  code = insert_event_row(
    store_.database_,
    event,
    &inserted_sequence);

  if (code != SQLITE_OK) {
    rollback();

    return snapshot_failure(
      SnapshotCode::kEventJournalError,
      code,
      "approval event append failed; "
      "snapshot was rolled back");
  }

  code = execute_sql(
    store_.database_,
    "COMMIT;");

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not commit approval transaction");
  }

  store_.transaction_active_ = false;
  store_.transaction_owner_ =
    std::thread::id{};

  if (event_sequence != nullptr) {
    *event_sequence = inserted_sequence;
  }

  return snapshot_success(
    "candidate approval persisted atomically");
}
'''


def main() -> None:
    required = (
        ROOT / "package.xml",
        ROOT / "CMakeLists.txt",
        ROOT / "include/savo_locations/constants.hpp",
        ROOT / "include/savo_locations/sqlite_schema.hpp",
        ROOT / "include/savo_locations/sqlite_repository.hpp",
        ROOT / "src/sqlite_store.cpp",
        ROOT / "src/sqlite_repository.cpp",
        ROOT / "test/storage/test_sqlite_store.cpp",
    )

    for path in required:
        if not path.is_file():
            raise SystemExit(
                f"Required LOC-2B file missing: {path}"
            )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC2C_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    update_package_version()

    # -------------------------------------------------------------------------
    # Package/CMake version
    # -------------------------------------------------------------------------

    cmake_path = ROOT / "CMakeLists.txt"
    cmake = cmake_path.read_text(encoding="utf-8")

    cmake = re.sub(
        r"project\(savo_locations VERSION [0-9.]+",
        "project(savo_locations VERSION 0.7.0",
        cmake,
        count=1,
    )

    constants_path = (
        ROOT
        / "include"
        / "savo_locations"
        / "constants.hpp"
    )

    constants = constants_path.read_text(
        encoding="utf-8"
    )

    constants = re.sub(
        r'"0\.[0-9]+\.[0-9]+"',
        '"0.7.0"',
        constants,
        count=1,
    )

    constants_path.write_text(
        constants,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Schema migration 002
    # -------------------------------------------------------------------------

    schema_path = (
        ROOT
        / "include"
        / "savo_locations"
        / "sqlite_schema.hpp"
    )

    schema = schema_path.read_text(
        encoding="utf-8"
    )

    schema = schema.replace(
        "kSupportedSqliteSchemaVersion{1U}",
        "kSupportedSqliteSchemaVersion{2U}",
    )

    if "kMigration002Sql" not in schema:
        marker = "}  // namespace savo_locations"

        if marker not in schema:
            raise RuntimeError(
                "Could not locate sqlite_schema namespace end"
            )

        migration_002 = clean(
            r'''

            inline constexpr std::string_view
              kMigration002Sql{R"SQL(
            CREATE TRIGGER IF NOT EXISTS
              location_events_reject_update
            BEFORE UPDATE ON location_events
            BEGIN
              SELECT RAISE(
                ABORT,
                'location_events is append-only'
              );
            END;

            CREATE TRIGGER IF NOT EXISTS
              location_events_reject_delete
            BEFORE DELETE ON location_events
            BEGIN
              SELECT RAISE(
                ABORT,
                'location_events is append-only'
              );
            END;
            )SQL"};


            '''
        )

        schema = schema.replace(
            marker,
            migration_002 + marker,
            1,
        )

    schema_path.write_text(
        schema,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Migration implementation
    # -------------------------------------------------------------------------

    store_cpp_path = ROOT / "src/sqlite_store.cpp"
    store_cpp = store_cpp_path.read_text(
        encoding="utf-8"
    )

    store_cpp = replace_function(
        store_cpp,
        "StorageResult SqliteStore::migrate(",
        clean(MIGRATE_FUNCTION),
    )

    store_cpp_path.write_text(
        store_cpp,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Repository header
    # -------------------------------------------------------------------------

    repository_header_path = (
        ROOT
        / "include"
        / "savo_locations"
        / "sqlite_repository.hpp"
    )

    header = repository_header_path.read_text(
        encoding="utf-8"
    )

    if "#include <cstddef>" not in header:
        header = header.replace(
            "#include <cstdint>",
            "#include <cstddef>\n#include <cstdint>",
            1,
        )

    if "kStaleRevision" not in header:
        header = header.replace(
            "kCorruptData,\n",
            (
                "kCorruptData,\n"
                "  kStaleRevision,\n"
                "  kApprovalDeltaInvalid,\n"
                "  kEventJournalError,\n"
            ),
            1,
        )

    if "enum class PersistenceEventType" not in header:
        marker = "class SqliteRepository"

        if marker not in header:
            raise RuntimeError(
                "Could not locate SqliteRepository class"
            )

        declarations = clean(
            r'''

            enum class PersistenceEventType : std::uint8_t
            {
              kUnknown = 0U,
              kSnapshotReplaced = 1U,
              kCandidateApproved = 2U,
              kCandidateRegistered = 3U,
              kCandidateRejected = 4U,
              kLocationEnabledChanged = 5U,
            };


            struct PersistenceEvent
            {
              std::uint64_t sequence{0U};
              std::int64_t event_time_unix_ns{0};

              PersistenceEventType event_type{
                PersistenceEventType::kUnknown};

              std::string candidate_id;
              std::string location_id;

              std::uint64_t entity_revision{0U};

              std::string actor_id;
              std::string reason;
              std::string payload_json{"{}"};
            };


            struct BootstrapReport
            {
              std::uint32_t schema_version{0U};
              bool integrity_healthy{false};

              std::size_t location_count{0U};
              std::size_t candidate_count{0U};

              std::uint64_t event_count{0U};
              std::uint64_t last_event_sequence{0U};
            };


            struct CandidateApprovalCommit
            {
              std::string candidate_id;

              std::uint64_t
                expected_candidate_revision{0U};

              std::string approved_location_id;

              std::string actor_id;
              std::string reason;
              std::string payload_json{"{}"};

              CatalogSnapshot post_approval_snapshot;
            };


            '''
        )

        header = header.replace(
            marker,
            declarations + marker,
            1,
        )

    if "commit_candidate_approval" not in header:
        marker = "private:\n"

        if marker not in header:
            raise RuntimeError(
                "Could not locate repository private section"
            )

        methods = clean(
            r'''

              [[nodiscard]]
              SnapshotResult bootstrap(
                CatalogSnapshot * snapshot,
                BootstrapReport * report) const;

              [[nodiscard]]
              SnapshotResult append_event(
                const PersistenceEvent & event,
                std::uint64_t * sequence);

              [[nodiscard]]
              SnapshotResult list_events(
                std::uint64_t after_sequence,
                std::size_t limit,
                std::vector<PersistenceEvent> * events) const;

              [[nodiscard]]
              SnapshotResult commit_candidate_approval(
                const CandidateApprovalCommit & request,
                std::uint64_t * event_sequence);

            '''
        )

        header = header.replace(
            marker,
            methods + marker,
            1,
        )

    repository_header_path.write_text(
        header,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Repository implementation
    # -------------------------------------------------------------------------

    repository_cpp_path = (
        ROOT
        / "src"
        / "sqlite_repository.cpp"
    )

    repository_cpp = repository_cpp_path.read_text(
        encoding="utf-8"
    )

    if "same_candidate_record(" not in repository_cpp:
        anonymous_end = repository_cpp.find(
            "\n}  // namespace\n\n\nstd::string_view to_string("
        )

        if anonymous_end < 0:
            raise RuntimeError(
                "Could not locate repository anonymous namespace end"
            )

        repository_cpp = (
            repository_cpp[:anonymous_end]
            + "\n"
            + clean(REPOSITORY_HELPERS).rstrip()
            + "\n"
            + repository_cpp[anonymous_end:]
        )

    if "case SnapshotCode::kStaleRevision:" not in repository_cpp:
        repository_cpp = repository_cpp.replace(
            (
                '    case SnapshotCode::kCorruptData:\n'
                '      return "corrupt_data";\n'
            ),
            (
                '    case SnapshotCode::kCorruptData:\n'
                '      return "corrupt_data";\n\n'
                '    case SnapshotCode::kStaleRevision:\n'
                '      return "stale_revision";\n\n'
                '    case SnapshotCode::kApprovalDeltaInvalid:\n'
                '      return "approval_delta_invalid";\n\n'
                '    case SnapshotCode::kEventJournalError:\n'
                '      return "event_journal_error";\n'
            ),
            1,
        )

    if "SqliteRepository::bootstrap(" not in repository_cpp:
        namespace_end = repository_cpp.rfind(
            "}  // namespace savo_locations"
        )

        if namespace_end < 0:
            raise RuntimeError(
                "Could not locate repository namespace end"
            )

        repository_cpp = (
            repository_cpp[:namespace_end]
            + clean(REPOSITORY_METHODS).rstrip()
            + "\n\n"
            + repository_cpp[namespace_end:]
        )

    repository_cpp_path.write_text(
        repository_cpp,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Update existing migration tests for schema v2
    # -------------------------------------------------------------------------

    store_test_path = (
        ROOT
        / "test"
        / "storage"
        / "test_sqlite_store.cpp"
    )

    store_test = store_test_path.read_text(
        encoding="utf-8"
    )

    store_test = store_test.replace(
        "EXPECT_EQ(version, 1U);",
        (
            "EXPECT_EQ(\n"
            "    version,\n"
            "    savo_locations::\n"
            "      kSupportedSqliteSchemaVersion);"
        ),
    )

    store_test = store_test.replace(
        "EXPECT_EQ(status.previous_version, 1U);",
        "EXPECT_EQ(status.previous_version, 2U);",
    )

    store_test = store_test.replace(
        "EXPECT_EQ(status.current_version, 1U);",
        "EXPECT_EQ(status.current_version, 2U);",
    )

    store_test_path.write_text(
        store_test,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # LOC-2C tests
    # -------------------------------------------------------------------------

    write(
        "test/storage/test_persistent_catalog.cpp",
        r'''
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
          EXPECT_EQ(status.current_version, 2U);
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
            2U);

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
        ''',
    )

    # -------------------------------------------------------------------------
    # LOC-2C contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase2c_contracts.py",
        r'''
        from pathlib import Path
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[2]


        def read(relative: str) -> str:
            return (ROOT / relative).read_text(
                encoding="utf-8"
            )


        def parse_version(
            value: str,
        ) -> tuple[int, int, int]:
            major, minor, patch = value.split(".")

            return (
                int(major),
                int(minor),
                int(patch),
            )


        def test_package_contains_loc2c_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 7, 0)

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_schema_version_two_is_locked() -> None:
            schema = read(
                "include/savo_locations/sqlite_schema.hpp"
            )

            assert (
                "kSupportedSqliteSchemaVersion{2U}"
                in schema
            )

            assert "kMigration002Sql" in schema

            assert (
                "location_events_reject_update"
                in schema
            )

            assert (
                "location_events_reject_delete"
                in schema
            )

            assert (
                "location_events is append-only"
                in schema
            )


        def test_repository_has_bootstrap_api() -> None:
            header = read(
                "include/savo_locations/sqlite_repository.hpp"
            )

            assert "struct BootstrapReport" in header
            assert "SnapshotResult bootstrap(" in header
            assert "integrity_healthy" in header
            assert "last_event_sequence" in header


        def test_repository_has_event_journal_api() -> None:
            header = read(
                "include/savo_locations/sqlite_repository.hpp"
            )

            assert "enum class PersistenceEventType" in header
            assert "struct PersistenceEvent" in header
            assert "append_event(" in header
            assert "list_events(" in header


        def test_approval_commit_is_atomic() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert "commit_candidate_approval(" in implementation
            assert '"BEGIN IMMEDIATE;"' in implementation
            assert '"COMMIT;"' in implementation
            assert '"ROLLBACK;"' in implementation

            assert (
                "approval event append failed; "
                "snapshot was rolled back"
                in implementation
            )


        def test_approval_delta_is_strict() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert "validate_approval_delta(" in implementation
            assert "candidate revision is stale" in implementation

            assert (
                "approval changed an unrelated candidate"
                in implementation
            )

            assert (
                "approval changed an existing location"
                in implementation
            )

            assert (
                "approval must create exactly one location"
                in implementation
            )


        def test_loc2c_tests_exist() -> None:
            assert (
                ROOT
                / "test"
                / "storage"
                / "test_persistent_catalog.cpp"
            ).is_file()

            cmake = read("CMakeLists.txt")

            assert "test_persistent_catalog" in cmake
            assert "test_phase2c_contracts" in cmake


        def test_loc2c_remains_without_ros_runtime() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(rclcpp" not in cmake
            assert "add_executable(" not in cmake

            assert not (
                ROOT
                / "src"
                / "location_registry_node.cpp"
            ).exists()

            assert not (ROOT / "launch").exists()
        ''',
    )

    # -------------------------------------------------------------------------
    # CMake test wiring
    # -------------------------------------------------------------------------

    if "test_persistent_catalog" not in cmake:
        marker = (
            "          ament_add_pytest_test(\n"
            "            test_phase0_contracts"
        )

        if marker not in cmake:
            raise RuntimeError(
                "Could not locate CMake test insertion point"
            )

        target = clean(
            r'''
              ament_add_gtest(
                test_persistent_catalog
                test/storage/test_persistent_catalog.cpp
              )

              if(TARGET test_persistent_catalog)
                target_link_libraries(
                  test_persistent_catalog
                  savo_locations_storage
                  SQLite::SQLite3
                )

                target_compile_definitions(
                  test_persistent_catalog
                  PRIVATE
                    "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
                )
              endif()

            '''
        )

        cmake = cmake.replace(
            marker,
            target + marker,
            1,
        )

    if "test_phase2c_contracts" not in cmake:
        marker = (
            "          ament_add_pytest_test(\n"
            "            test_phase2b_contracts\n"
            "            test/contracts/test_phase2b_contracts.py\n"
            "            TIMEOUT 60\n"
            "          )"
        )

        if marker not in cmake:
            raise RuntimeError(
                "Could not locate phase2b CMake test"
            )

        addition = marker + clean(
            r'''

              ament_add_pytest_test(
                test_phase2c_contracts
                test/contracts/test_phase2c_contracts.py
                TIMEOUT 60
              )
            '''
        )

        cmake = cmake.replace(
            marker,
            addition,
            1,
        )

    cmake_path.write_text(
        cmake,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Configuration/documentation
    # -------------------------------------------------------------------------

    storage_path = ROOT / "config" / "storage.yaml"
    storage = storage_path.read_text(encoding="utf-8")

    storage = storage.replace(
        "supported_schema_version: 1",
        "supported_schema_version: 2",
    )

    if "\npersistent_catalog:\n" not in storage:
        storage += clean(
            r'''

            persistent_catalog:
              bootstrap_integrity_check: true
              bootstrap_domain_validation: true
              fail_closed_on_bootstrap_error: true

              approval_commit:
                transaction_begin_mode: IMMEDIATE
                expected_revision_required: true
                create_exactly_one_location: true
                preserve_existing_locations: true
                preserve_unrelated_candidates: true
                append_event_in_same_transaction: true
                rollback_on_event_failure: true

              event_journal:
                append_only: true
                database_update_trigger: reject
                database_delete_trigger: reject
                deterministic_sequence: true
                maximum_query_limit: 1000
            '''
        )

        storage_path.write_text(
            storage,
            encoding="utf-8",
        )

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(encoding="utf-8")

    if "## LOC-2C persistent catalog" not in readme:
        readme += clean(
            r'''

            ## LOC-2C persistent catalog

            LOC-2C adds persistent catalog bootstrap, atomic candidate
            approval persistence and an append-only audit journal.

            Bootstrap performs:

            - schema-version validation;
            - SQLite integrity validation;
            - foreign-key validation;
            - complete typed snapshot loading;
            - post-load domain validation;
            - location, candidate and event counters.

            Candidate approval persistence requires the expected pending
            candidate revision. The committed post-approval snapshot must:

            - preserve all existing locations;
            - preserve every unrelated candidate;
            - change only the selected candidate from pending to approved;
            - increment its candidate revision by exactly one;
            - create exactly one approved and enabled location;
            - bind that location to the candidate's map and AprilTag evidence;
            - set the location's source candidate ID.

            The snapshot replacement and `candidate_approved` event append
            occur in one `BEGIN IMMEDIATE` transaction. Failure to append the
            audit event rolls back the location and candidate changes.

            Schema migration 002 installs SQLite triggers that reject UPDATE
            and DELETE operations on `location_events`. The journal is
            therefore append-only at both the repository API and database
            levels.

            LOC-2C does not add a ROS node, services, launch files, hardware
            access or automatic startup integration.
            '''
        )

        readme_path.write_text(
            readme,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # Verification and manifest
    # -------------------------------------------------------------------------

    verification_files = (
        ROOT / "include/savo_locations/sqlite_schema.hpp",
        ROOT / "include/savo_locations/sqlite_repository.hpp",
        ROOT / "src/sqlite_store.cpp",
        ROOT / "src/sqlite_repository.cpp",
        ROOT / "test/storage/test_persistent_catalog.cpp",
        ROOT / "test/contracts/test_phase2c_contracts.py",
    )

    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in verification_files
    )

    required_fragments = (
        "kSupportedSqliteSchemaVersion{2U}",
        "kMigration002Sql",
        "location_events_reject_update",
        "struct BootstrapReport",
        "struct CandidateApprovalCommit",
        "commit_candidate_approval",
        "validate_approval_delta",
        "approval event append failed",
        "test_phase2c_contracts",
    )

    for fragment in required_fragments:
        if fragment not in combined and fragment not in cmake:
            raise RuntimeError(
                f"LOC-2C verification failed: {fragment}"
            )

    changed_files = sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
    )

    manifest = (
        LOGS
        / f"LOC2C_savo_locations_{stamp}.sha256"
    )

    manifest.write_text(
        "\n".join(
            f"{sha256(path)}  "
            f"{path.relative_to(ROOT)}"
            for path in changed_files
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")

    print(
        "LOC-2C persistent bootstrap, atomic approval "
        "and append-only event journal applied."
    )

    print(
        "No ROS node, ROS services, launch files "
        "or hardware runtime was added."
    )


if __name__ == "__main__":
    main()
