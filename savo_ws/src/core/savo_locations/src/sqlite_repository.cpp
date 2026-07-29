#include "savo_locations/sqlite_repository.hpp"

#include <sqlite3.h>

#include <chrono>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <utility>

#include "savo_locations/normalization.hpp"
#include "savo_locations/registry.hpp"

namespace savo_locations
{
namespace
{

class Statement
{
public:
  Statement(
    sqlite3 * database,
    const char * sql)
  : database_(database)
  {
    prepare_code_ = sqlite3_prepare_v2(
      database_,
      sql,
      -1,
      &statement_,
      nullptr);
  }

  ~Statement()
  {
    if (statement_ != nullptr) {
      sqlite3_finalize(statement_);
    }
  }

  Statement(const Statement &) = delete;
  Statement & operator=(const Statement &) = delete;

  [[nodiscard]]
  int prepare_code() const noexcept
  {
    return prepare_code_;
  }

  [[nodiscard]]
  sqlite3_stmt * get() const noexcept
  {
    return statement_;
  }

private:
  sqlite3 * database_{nullptr};
  sqlite3_stmt * statement_{nullptr};
  int prepare_code_{SQLITE_ERROR};
};


SnapshotResult snapshot_success(
  std::string reason)
{
  SnapshotResult result;
  result.success = true;
  result.code = SnapshotCode::kOk;
  result.sqlite_code = SQLITE_OK;
  result.reason = std::move(reason);
  return result;
}


SnapshotResult snapshot_failure(
  const SnapshotCode code,
  const int sqlite_code,
  std::string reason)
{
  SnapshotResult result;
  result.success = false;
  result.code = code;
  result.sqlite_code = sqlite_code;
  result.reason = std::move(reason);
  return result;
}


SnapshotResult validation_failure(
  std::string reason,
  const ValidationResult & validation)
{
  SnapshotResult result;

  result.success = false;
  result.code = SnapshotCode::kValidationFailed;
  result.sqlite_code = SQLITE_CONSTRAINT;
  result.reason = std::move(reason);

  result.validation_issues =
    validation.issues();

  return result;
}


std::int64_t unix_time_ns()
{
  const auto now =
    std::chrono::system_clock::now()
    .time_since_epoch();

  return std::chrono::duration_cast<
    std::chrono::nanoseconds>(now).count();
}


bool sqlite_constraint(
  const int code) noexcept
{
  return
    (code & 0xFF) ==
    SQLITE_CONSTRAINT;
}


SnapshotResult sqlite_failure(
  sqlite3 * database,
  const int code,
  const std::string_view operation)
{
  const SnapshotCode snapshot_code =
    sqlite_constraint(code) ?
      SnapshotCode::kIdentityConflict :
      SnapshotCode::kSqlError;

  std::string reason{operation};

  if (database != nullptr) {
    reason += ": ";
    reason += sqlite3_errmsg(database);
  }

  return snapshot_failure(
    snapshot_code,
    code,
    std::move(reason));
}


int execute_sql(
  sqlite3 * database,
  const char * sql)
{
  return sqlite3_exec(
    database,
    sql,
    nullptr,
    nullptr,
    nullptr);
}


int bind_text(
  sqlite3_stmt * statement,
  const int index,
  const std::string_view value)
{
  if (
    value.size() >
    static_cast<std::size_t>(
      std::numeric_limits<int>::max()))
  {
    return SQLITE_TOOBIG;
  }

  return sqlite3_bind_text(
    statement,
    index,
    value.data(),
    static_cast<int>(value.size()),
    SQLITE_TRANSIENT);
}


int bind_int64(
  sqlite3_stmt * statement,
  const int index,
  const std::uint64_t value)
{
  if (
    value >
    static_cast<std::uint64_t>(
      std::numeric_limits<
        sqlite3_int64>::max()))
  {
    return SQLITE_RANGE;
  }

  return sqlite3_bind_int64(
    statement,
    index,
    static_cast<sqlite3_int64>(value));
}


int bind_pose(
  sqlite3_stmt * statement,
  const int first_index,
  const PoseData & pose)
{
  int code = bind_text(
    statement,
    first_index,
    pose.frame_id);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 1,
      pose.x);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 2,
      pose.y);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 3,
      pose.z);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 4,
      pose.qx);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 5,
      pose.qy);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 6,
      pose.qz);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      statement,
      first_index + 7,
      pose.qw);
  }

  return code;
}


int bind_optional_pose(
  sqlite3_stmt * statement,
  const int valid_index,
  const int pose_index,
  const std::optional<PoseData> & pose)
{
  int code = sqlite3_bind_int(
    statement,
    valid_index,
    pose.has_value() ? 1 : 0);

  if (code != SQLITE_OK) {
    return code;
  }

  if (pose.has_value()) {
    return bind_pose(
      statement,
      pose_index,
      pose.value());
  }

  for (
    int index = pose_index;
    index < pose_index + 8;
    ++index)
  {
    code = sqlite3_bind_null(
      statement,
      index);

    if (code != SQLITE_OK) {
      return code;
    }
  }

  return SQLITE_OK;
}


std::string column_text(
  sqlite3_stmt * statement,
  const int index)
{
  const auto * text =
    sqlite3_column_text(
      statement,
      index);

  if (text == nullptr) {
    return {};
  }

  return std::string{
    reinterpret_cast<const char *>(text)};
}


bool column_uint64(
  sqlite3_stmt * statement,
  const int index,
  std::uint64_t * value)
{
  const sqlite3_int64 raw =
    sqlite3_column_int64(
      statement,
      index);

  if (raw < 0) {
    return false;
  }

  *value =
    static_cast<std::uint64_t>(raw);

  return true;
}


bool column_uint32(
  sqlite3_stmt * statement,
  const int index,
  std::uint32_t * value)
{
  std::uint64_t raw = 0U;

  if (
    !column_uint64(
      statement,
      index,
      &raw) ||
    raw >
      std::numeric_limits<
        std::uint32_t>::max())
  {
    return false;
  }

  *value =
    static_cast<std::uint32_t>(raw);

  return true;
}


PoseData read_pose(
  sqlite3_stmt * statement,
  const int first_index)
{
  PoseData pose;

  pose.frame_id =
    column_text(
      statement,
      first_index);

  pose.x = sqlite3_column_double(
    statement,
    first_index + 1);

  pose.y = sqlite3_column_double(
    statement,
    first_index + 2);

  pose.z = sqlite3_column_double(
    statement,
    first_index + 3);

  pose.qx = sqlite3_column_double(
    statement,
    first_index + 4);

  pose.qy = sqlite3_column_double(
    statement,
    first_index + 5);

  pose.qz = sqlite3_column_double(
    statement,
    first_index + 6);

  pose.qw = sqlite3_column_double(
    statement,
    first_index + 7);

  return pose;
}


std::optional<PoseData>
read_optional_pose(
  sqlite3_stmt * statement,
  const int valid_index,
  const int pose_index)
{
  if (
    sqlite3_column_int(
      statement,
      valid_index) == 0)
  {
    return std::nullopt;
  }

  return read_pose(
    statement,
    pose_index);
}


bool location_record_envelope_valid(
  const LocationRecordData & record)
{
  if (record.record_revision == 0U) {
    return false;
  }

  if (
    record.state ==
    LocationState::kUnknown)
  {
    return false;
  }

  if (
    record.state ==
      LocationState::kRetired &&
    record.enabled)
  {
    return false;
  }

  return true;
}


bool candidate_record_envelope_valid(
  const CandidateRecordData & record)
{
  if (record.candidate_revision == 0U) {
    return false;
  }

  switch (record.state) {
    case CandidateState::kPendingReview:
      return
        record.review_reason.empty() &&
        record.approved_location_id.empty();

    case CandidateState::kApproved:
      return
        !record.approved_location_id.empty();

    case CandidateState::kRejected:
      return
        !trim_ascii(
          record.review_reason).empty() &&
        record.approved_location_id.empty();

    case CandidateState::kUnknown:
    default:
      return false;
  }
}


std::string tag_key(
  const MapContext & map,
  const TagBinding & tag)
{
  return
    map.map_id +
    "\n" +
    std::to_string(map.map_revision) +
    "\n" +
    normalize_lookup_key(tag.family) +
    "\n" +
    std::to_string(tag.id);
}


SnapshotResult validate_snapshot(
  const CatalogSnapshot & snapshot)
{
  InMemoryRegistry registry;

  std::set<std::string>
    active_location_tags;

  for (
    const auto & location :
    snapshot.locations)
  {
    const auto validation =
      validate_location_draft(
        location.location);

    if (!validation.valid()) {
      return validation_failure(
        "location validation failed",
        validation);
    }

    if (
      !location_record_envelope_valid(
        location))
    {
      return snapshot_failure(
        SnapshotCode::kValidationFailed,
        SQLITE_CONSTRAINT,
        "location record envelope is invalid");
    }

    const auto inserted =
      registry.insert(location);

    if (!inserted.success) {
      const SnapshotCode code =
        inserted.code ==
          MutationCode::kTagConflict ?
          SnapshotCode::kTagConflict :
          SnapshotCode::kIdentityConflict;

      return snapshot_failure(
        code,
        SQLITE_CONSTRAINT,
        inserted.reason);
    }

    if (
      location.state !=
      LocationState::kRetired)
    {
      active_location_tags.insert(
        tag_key(
          location.location.map,
          location.location.tag));
    }
  }

  std::set<std::string> candidate_ids;
  std::set<std::string> pending_tags;

  for (
    const auto & candidate :
    snapshot.candidates)
  {
    const auto validation =
      validate_candidate_draft(
        candidate.candidate);

    if (!validation.valid()) {
      return validation_failure(
        "candidate validation failed",
        validation);
    }

    if (
      !candidate_record_envelope_valid(
        candidate))
    {
      return snapshot_failure(
        SnapshotCode::kValidationFailed,
        SQLITE_CONSTRAINT,
        "candidate record envelope is invalid");
    }

    const auto inserted_id =
      candidate_ids.insert(
        candidate.candidate.candidate_id);

    if (!inserted_id.second) {
      return snapshot_failure(
        SnapshotCode::kIdentityConflict,
        SQLITE_CONSTRAINT,
        "duplicate candidate ID");
    }

    if (
      candidate.state ==
      CandidateState::kPendingReview)
    {
      const auto key =
        tag_key(
          candidate.candidate.map,
          candidate.candidate.tag);

      if (
        active_location_tags.count(key) != 0U)
      {
        return snapshot_failure(
          SnapshotCode::kTagConflict,
          SQLITE_CONSTRAINT,
          "pending candidate tag conflicts "
          "with an active location");
      }

      if (!pending_tags.insert(key).second) {
        return snapshot_failure(
          SnapshotCode::kTagConflict,
          SQLITE_CONSTRAINT,
          "pending candidate tag conflict");
      }
    }

    if (
      candidate.state ==
      CandidateState::kApproved &&
      !registry.get(
        candidate.approved_location_id)
        .has_value())
    {
      return snapshot_failure(
        SnapshotCode::kValidationFailed,
        SQLITE_CONSTRAINT,
        "approved candidate references "
        "a missing location");
    }
  }

  return snapshot_success(
    "snapshot is valid");
}


int insert_location(
  sqlite3 * database,
  const LocationRecordData & record,
  const std::int64_t timestamp)
{
  static constexpr const char * sql =
    "INSERT INTO locations("
    "location_id,state,enabled,record_revision,"
    "display_name,semantic_type,"
    "map_id,map_revision,map_release_id,"
    "approach_frame_id,"
    "approach_x,approach_y,approach_z,"
    "approach_qx,approach_qy,approach_qz,approach_qw,"
    "confirmation_pose_valid,"
    "confirmation_frame_id,"
    "confirmation_x,confirmation_y,confirmation_z,"
    "confirmation_qx,confirmation_qy,"
    "confirmation_qz,confirmation_qw,"
    "tag_family,tag_id,"
    "tag_pose_map_valid,"
    "tag_frame_id,"
    "tag_x,tag_y,tag_z,"
    "tag_qx,tag_qy,tag_qz,tag_qw,"
    "arrival_confirmation_required,"
    "building,floor,area,notes,"
    "source_candidate_id,"
    "created_at_unix_ns,updated_at_unix_ns"
    ") VALUES("
    "?1,?2,?3,?4,?5,?6,?7,?8,?9,"
    "?10,?11,?12,?13,?14,?15,?16,?17,"
    "?18,?19,?20,?21,?22,?23,?24,?25,?26,"
    "?27,?28,?29,?30,?31,?32,?33,?34,?35,"
    "?36,?37,?38,?39,?40,?41,?42,?43,?44,?45"
    ");";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return statement.prepare_code();
  }

  auto * raw = statement.get();
  int code = SQLITE_OK;

  const auto & location =
    record.location;

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      1,
      location.location_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      2,
      static_cast<int>(record.state));
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      3,
      record.enabled ? 1 : 0);
  }

  if (code == SQLITE_OK) {
    code = bind_int64(
      raw,
      4,
      record.record_revision);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      5,
      location.display_name);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      6,
      location.semantic_type);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      7,
      location.map.map_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      8,
      static_cast<sqlite3_int64>(
        location.map.map_revision));
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      9,
      location.map.map_release_id);
  }

  if (code == SQLITE_OK) {
    code = bind_pose(
      raw,
      10,
      location.approach_pose);
  }

  if (code == SQLITE_OK) {
    code = bind_optional_pose(
      raw,
      18,
      19,
      location.confirmation_pose);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      27,
      location.tag.family);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      28,
      location.tag.id);
  }

  if (code == SQLITE_OK) {
    code = bind_optional_pose(
      raw,
      29,
      30,
      location.tag_pose_map);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      38,
      location
        .arrival_confirmation_required ?
        1 :
        0);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      39,
      location.building);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      40,
      location.floor);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      41,
      location.area);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      42,
      location.notes);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      43,
      record.source_candidate_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      44,
      timestamp);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      45,
      timestamp);
  }

  if (code != SQLITE_OK) {
    return code;
  }

  code = sqlite3_step(raw);

  return
    code == SQLITE_DONE ?
    SQLITE_OK :
    sqlite3_extended_errcode(database);
}


int insert_location_identity(
  sqlite3 * database,
  const LocationRecordData & record,
  const int alias_order,
  const int alias_kind,
  const std::string_view alias_text)
{
  static constexpr const char * sql =
    "INSERT INTO location_aliases("
    "location_id,alias_order,alias_kind,"
    "alias_text,normalized_key,"
    "map_id,map_revision,reserves_identity"
    ") VALUES(?1,?2,?3,?4,?5,?6,?7,?8);";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return statement.prepare_code();
  }

  auto * raw = statement.get();

  int code = bind_text(
    raw,
    1,
    record.location.location_id);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      2,
      alias_order);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      3,
      alias_kind);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      4,
      alias_text);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      5,
      normalize_lookup_key(alias_text));
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      6,
      record.location.map.map_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      7,
      static_cast<sqlite3_int64>(
        record.location
          .map
          .map_revision));
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      8,
      record.state ==
        LocationState::kRetired ?
        0 :
        1);
  }

  if (code != SQLITE_OK) {
    return code;
  }

  code = sqlite3_step(raw);

  return
    code == SQLITE_DONE ?
    SQLITE_OK :
    sqlite3_extended_errcode(database);
}


int insert_location_aliases(
  sqlite3 * database,
  const LocationRecordData & record)
{
  int order = 0;

  int code = insert_location_identity(
    database,
    record,
    order++,
    1,
    record.location.location_id);

  if (code != SQLITE_OK) {
    return code;
  }

  code = insert_location_identity(
    database,
    record,
    order++,
    2,
    record.location.display_name);

  if (code != SQLITE_OK) {
    return code;
  }

  for (
    const auto & alias :
    record.location.aliases)
  {
    code = insert_location_identity(
      database,
      record,
      order++,
      3,
      alias);

    if (code != SQLITE_OK) {
      return code;
    }
  }

  return SQLITE_OK;
}


int insert_candidate(
  sqlite3 * database,
  const CandidateRecordData & record,
  const std::int64_t timestamp)
{
  static constexpr const char * sql =
    "INSERT INTO location_candidates("
    "candidate_id,state,candidate_revision,"
    "map_id,map_revision,map_release_id,"
    "tag_family,tag_id,"
    "tag_frame_id,"
    "tag_x,tag_y,tag_z,"
    "tag_qx,tag_qy,tag_qz,tag_qw,"
    "detection_quality,accepted_observations,"
    "position_stddev_m,yaw_stddev_rad,"
    "approach_pose_valid,"
    "approach_frame_id,"
    "approach_x,approach_y,approach_z,"
    "approach_qx,approach_qy,approach_qz,approach_qw,"
    "confirmation_pose_valid,"
    "confirmation_frame_id,"
    "confirmation_x,confirmation_y,confirmation_z,"
    "confirmation_qx,confirmation_qy,"
    "confirmation_qz,confirmation_qw,"
    "suggested_location_id,"
    "suggested_display_name,"
    "suggested_semantic_type,"
    "building,floor,area,notes,"
    "source_session_id,source_component,"
    "review_reason,approved_location_id,"
    "created_at_unix_ns,updated_at_unix_ns"
    ") VALUES("
    "?1,?2,?3,?4,?5,?6,?7,?8,"
    "?9,?10,?11,?12,?13,?14,?15,?16,"
    "?17,?18,?19,?20,"
    "?21,?22,?23,?24,?25,?26,?27,?28,?29,"
    "?30,?31,?32,?33,?34,?35,?36,?37,?38,"
    "?39,?40,?41,?42,?43,?44,?45,?46,?47,"
    "?48,?49,?50,?51"
    ");";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return statement.prepare_code();
  }

  auto * raw = statement.get();

  const auto & candidate =
    record.candidate;

  int code = bind_text(
    raw,
    1,
    candidate.candidate_id);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      2,
      static_cast<int>(record.state));
  }

  if (code == SQLITE_OK) {
    code = bind_int64(
      raw,
      3,
      record.candidate_revision);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      4,
      candidate.map.map_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      5,
      static_cast<sqlite3_int64>(
        candidate.map.map_revision));
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      6,
      candidate.map.map_release_id);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      7,
      candidate.tag.family);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      raw,
      8,
      candidate.tag.id);
  }

  if (code == SQLITE_OK) {
    code = bind_pose(
      raw,
      9,
      candidate.tag_pose_map);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      raw,
      17,
      candidate.detection_quality);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      18,
      static_cast<sqlite3_int64>(
        candidate.accepted_observations));
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      raw,
      19,
      candidate.position_stddev_m);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_double(
      raw,
      20,
      candidate.yaw_stddev_rad);
  }

  if (code == SQLITE_OK) {
    code = bind_optional_pose(
      raw,
      21,
      22,
      candidate.approach_pose);
  }

  if (code == SQLITE_OK) {
    code = bind_optional_pose(
      raw,
      30,
      31,
      candidate.confirmation_pose);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      39,
      candidate.suggested_location_id);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      40,
      candidate.suggested_display_name);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      41,
      candidate.suggested_semantic_type);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      42,
      candidate.building);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      43,
      candidate.floor);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      44,
      candidate.area);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      45,
      candidate.notes);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      46,
      candidate.source_session_id);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      47,
      candidate.source_component);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      48,
      record.review_reason);
  }

  if (code == SQLITE_OK) {
    code = bind_text(
      raw,
      49,
      record.approved_location_id);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      50,
      timestamp);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      raw,
      51,
      timestamp);
  }

  if (code != SQLITE_OK) {
    return code;
  }

  code = sqlite3_step(raw);

  return
    code == SQLITE_DONE ?
    SQLITE_OK :
    sqlite3_extended_errcode(database);
}


int insert_candidate_aliases(
  sqlite3 * database,
  const CandidateRecordData & record)
{
  static constexpr const char * sql =
    "INSERT INTO candidate_aliases("
    "candidate_id,alias_order,"
    "alias_text,normalized_key"
    ") VALUES(?1,?2,?3,?4);";

  int order = 0;

  for (
    const auto & alias :
    record.candidate.suggested_aliases)
  {
    Statement statement{database, sql};

    if (
      statement.prepare_code() !=
      SQLITE_OK)
    {
      return statement.prepare_code();
    }

    auto * raw = statement.get();

    int code = bind_text(
      raw,
      1,
      record.candidate.candidate_id);

    if (code == SQLITE_OK) {
      code = sqlite3_bind_int(
        raw,
        2,
        order++);
    }

    if (code == SQLITE_OK) {
      code = bind_text(
        raw,
        3,
        alias);
    }

    if (code == SQLITE_OK) {
      code = bind_text(
        raw,
        4,
        normalize_lookup_key(alias));
    }

    if (code != SQLITE_OK) {
      return code;
    }

    code = sqlite3_step(raw);

    if (code != SQLITE_DONE) {
      return sqlite3_extended_errcode(
        database);
    }
  }

  return SQLITE_OK;
}


SnapshotResult load_location_aliases(
  sqlite3 * database,
  LocationRecordData * record)
{
  static constexpr const char * sql =
    "SELECT alias_text "
    "FROM location_aliases "
    "WHERE location_id=?1 AND alias_kind=3 "
    "ORDER BY alias_order;";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return sqlite_failure(
      database,
      statement.prepare_code(),
      "could not prepare location alias load");
  }

  int code = bind_text(
    statement.get(),
    1,
    record->location.location_id);

  if (code != SQLITE_OK) {
    return sqlite_failure(
      database,
      code,
      "could not bind location alias ID");
  }

  record->location.aliases.clear();

  while (
    (code = sqlite3_step(
      statement.get())) ==
    SQLITE_ROW)
  {
    record->location.aliases.push_back(
      column_text(
        statement.get(),
        0));
  }

  if (code != SQLITE_DONE) {
    return sqlite_failure(
      database,
      sqlite3_extended_errcode(database),
      "could not load location aliases");
  }

  return snapshot_success(
    "location aliases loaded");
}


SnapshotResult load_candidate_aliases(
  sqlite3 * database,
  CandidateRecordData * record)
{
  static constexpr const char * sql =
    "SELECT alias_text "
    "FROM candidate_aliases "
    "WHERE candidate_id=?1 "
    "ORDER BY alias_order;";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return sqlite_failure(
      database,
      statement.prepare_code(),
      "could not prepare candidate alias load");
  }

  int code = bind_text(
    statement.get(),
    1,
    record->candidate.candidate_id);

  if (code != SQLITE_OK) {
    return sqlite_failure(
      database,
      code,
      "could not bind candidate alias ID");
  }

  record->candidate
    .suggested_aliases
    .clear();

  while (
    (code = sqlite3_step(
      statement.get())) ==
    SQLITE_ROW)
  {
    record->candidate
      .suggested_aliases
      .push_back(
        column_text(
          statement.get(),
          0));
  }

  if (code != SQLITE_DONE) {
    return sqlite_failure(
      database,
      sqlite3_extended_errcode(database),
      "could not load candidate aliases");
  }

  return snapshot_success(
    "candidate aliases loaded");
}


SnapshotResult read_locations(
  sqlite3 * database,
  CatalogSnapshot * snapshot)
{
  static constexpr const char * sql =
    "SELECT "
    "location_id,state,enabled,record_revision,"
    "display_name,semantic_type,"
    "map_id,map_revision,map_release_id,"
    "approach_frame_id,"
    "approach_x,approach_y,approach_z,"
    "approach_qx,approach_qy,approach_qz,approach_qw,"
    "confirmation_pose_valid,"
    "confirmation_frame_id,"
    "confirmation_x,confirmation_y,confirmation_z,"
    "confirmation_qx,confirmation_qy,"
    "confirmation_qz,confirmation_qw,"
    "tag_family,tag_id,"
    "tag_pose_map_valid,"
    "tag_frame_id,"
    "tag_x,tag_y,tag_z,"
    "tag_qx,tag_qy,tag_qz,tag_qw,"
    "arrival_confirmation_required,"
    "building,floor,area,notes,"
    "source_candidate_id,"
    "created_at_unix_ns,updated_at_unix_ns "
    "FROM locations "
    "ORDER BY location_id;";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return sqlite_failure(
      database,
      statement.prepare_code(),
      "could not prepare location snapshot load");
  }

  int code = SQLITE_OK;

  while (
    (code = sqlite3_step(
      statement.get())) ==
    SQLITE_ROW)
  {
    auto * raw = statement.get();

    LocationRecordData record;

    record.location.location_id =
      column_text(raw, 0);

    const int state =
      sqlite3_column_int(raw, 1);

    if (
      state <
        static_cast<int>(
          LocationState::kApproved) ||
      state >
        static_cast<int>(
          LocationState::kRetired))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid persisted location state");
    }

    record.state =
      static_cast<LocationState>(state);

    record.enabled =
      sqlite3_column_int(raw, 2) != 0;

    if (
      !column_uint64(
        raw,
        3,
        &record.record_revision))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid location revision");
    }

    record.location.display_name =
      column_text(raw, 4);

    record.location.semantic_type =
      column_text(raw, 5);

    record.location.map.map_id =
      column_text(raw, 6);

    if (
      !column_uint32(
        raw,
        7,
        &record.location
          .map
          .map_revision))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid location map revision");
    }

    record.location.map.map_release_id =
      column_text(raw, 8);

    record.location.approach_pose =
      read_pose(raw, 9);

    record.location.confirmation_pose =
      read_optional_pose(
        raw,
        17,
        18);

    record.location.tag.family =
      column_text(raw, 26);

    record.location.tag.id =
      sqlite3_column_int(raw, 27);

    record.location.tag_pose_map =
      read_optional_pose(
        raw,
        28,
        29);

    record.location
      .arrival_confirmation_required =
      sqlite3_column_int(raw, 37) != 0;

    record.location.building =
      column_text(raw, 38);

    record.location.floor =
      column_text(raw, 39);

    record.location.area =
      column_text(raw, 40);

    record.location.notes =
      column_text(raw, 41);

    record.source_candidate_id =
      column_text(raw, 42);

    const auto alias_result =
      load_location_aliases(
        database,
        &record);

    if (!alias_result.success) {
      return alias_result;
    }

    snapshot->locations.push_back(
      std::move(record));
  }

  if (code != SQLITE_DONE) {
    return sqlite_failure(
      database,
      sqlite3_extended_errcode(database),
      "could not load location snapshot");
  }

  return snapshot_success(
    "locations loaded");
}


SnapshotResult read_candidates(
  sqlite3 * database,
  CatalogSnapshot * snapshot)
{
  static constexpr const char * sql =
    "SELECT "
    "candidate_id,state,candidate_revision,"
    "map_id,map_revision,map_release_id,"
    "tag_family,tag_id,"
    "tag_frame_id,"
    "tag_x,tag_y,tag_z,"
    "tag_qx,tag_qy,tag_qz,tag_qw,"
    "detection_quality,accepted_observations,"
    "position_stddev_m,yaw_stddev_rad,"
    "approach_pose_valid,"
    "approach_frame_id,"
    "approach_x,approach_y,approach_z,"
    "approach_qx,approach_qy,approach_qz,approach_qw,"
    "confirmation_pose_valid,"
    "confirmation_frame_id,"
    "confirmation_x,confirmation_y,confirmation_z,"
    "confirmation_qx,confirmation_qy,"
    "confirmation_qz,confirmation_qw,"
    "suggested_location_id,"
    "suggested_display_name,"
    "suggested_semantic_type,"
    "building,floor,area,notes,"
    "source_session_id,source_component,"
    "review_reason,approved_location_id,"
    "created_at_unix_ns,updated_at_unix_ns "
    "FROM location_candidates "
    "ORDER BY candidate_id;";

  Statement statement{database, sql};

  if (
    statement.prepare_code() !=
    SQLITE_OK)
  {
    return sqlite_failure(
      database,
      statement.prepare_code(),
      "could not prepare candidate snapshot load");
  }

  int code = SQLITE_OK;

  while (
    (code = sqlite3_step(
      statement.get())) ==
    SQLITE_ROW)
  {
    auto * raw = statement.get();

    CandidateRecordData record;

    record.candidate.candidate_id =
      column_text(raw, 0);

    const int state =
      sqlite3_column_int(raw, 1);

    if (
      state <
        static_cast<int>(
          CandidateState::kPendingReview) ||
      state >
        static_cast<int>(
          CandidateState::kRejected))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid persisted candidate state");
    }

    record.state =
      static_cast<CandidateState>(state);

    if (
      !column_uint64(
        raw,
        2,
        &record.candidate_revision))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid candidate revision");
    }

    record.candidate.map.map_id =
      column_text(raw, 3);

    if (
      !column_uint32(
        raw,
        4,
        &record.candidate
          .map
          .map_revision))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid candidate map revision");
    }

    record.candidate
      .map
      .map_release_id =
      column_text(raw, 5);

    record.candidate.tag.family =
      column_text(raw, 6);

    record.candidate.tag.id =
      sqlite3_column_int(raw, 7);

    record.candidate.tag_pose_map =
      read_pose(raw, 8);

    record.candidate.detection_quality =
      sqlite3_column_double(raw, 16);

    std::uint32_t observations = 0U;

    if (
      !column_uint32(
        raw,
        17,
        &observations))
    {
      return snapshot_failure(
        SnapshotCode::kCorruptData,
        SQLITE_CORRUPT,
        "invalid accepted observation count");
    }

    record.candidate
      .accepted_observations =
      observations;

    record.candidate.position_stddev_m =
      sqlite3_column_double(raw, 18);

    record.candidate.yaw_stddev_rad =
      sqlite3_column_double(raw, 19);

    record.candidate.approach_pose =
      read_optional_pose(
        raw,
        20,
        21);

    record.candidate.confirmation_pose =
      read_optional_pose(
        raw,
        29,
        30);

    record.candidate
      .suggested_location_id =
      column_text(raw, 38);

    record.candidate
      .suggested_display_name =
      column_text(raw, 39);

    record.candidate
      .suggested_semantic_type =
      column_text(raw, 40);

    record.candidate.building =
      column_text(raw, 41);

    record.candidate.floor =
      column_text(raw, 42);

    record.candidate.area =
      column_text(raw, 43);

    record.candidate.notes =
      column_text(raw, 44);

    record.candidate.source_session_id =
      column_text(raw, 45);

    record.candidate.source_component =
      column_text(raw, 46);

    record.review_reason =
      column_text(raw, 47);

    record.approved_location_id =
      column_text(raw, 48);

    const auto alias_result =
      load_candidate_aliases(
        database,
        &record);

    if (!alias_result.success) {
      return alias_result;
    }

    snapshot->candidates.push_back(
      std::move(record));
  }

  if (code != SQLITE_DONE) {
    return sqlite_failure(
      database,
      sqlite3_extended_errcode(database),
      "could not load candidate snapshot");
  }

  return snapshot_success(
    "candidates loaded");
}

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


SnapshotResult validate_registration_delta(
  const CatalogSnapshot & current,
  const CandidateRegistrationCommit & request)
{
  const auto & post =
    request.post_registration_snapshot;

  if (
    find_candidate(
      current,
      request.candidate_id) != nullptr)
  {
    return snapshot_failure(
      SnapshotCode::
        kCandidateRegistrationDeltaInvalid,
      SQLITE_CONSTRAINT,
      "candidate registration ID already exists");
  }

  if (
    post.locations.size() !=
    current.locations.size())
  {
    return snapshot_failure(
      SnapshotCode::
        kCandidateRegistrationDeltaInvalid,
      SQLITE_CONSTRAINT,
      "candidate registration changed locations");
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
        SnapshotCode::
          kCandidateRegistrationDeltaInvalid,
        SQLITE_CONSTRAINT,
        "candidate registration changed "
        "an existing location");
    }
  }

  if (
    post.candidates.size() !=
    current.candidates.size() + 1U)
  {
    return snapshot_failure(
      SnapshotCode::
        kCandidateRegistrationDeltaInvalid,
      SQLITE_CONSTRAINT,
      "candidate registration must add "
      "exactly one candidate");
  }

  for (
    const auto & previous :
    current.candidates)
  {
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
        SnapshotCode::
          kCandidateRegistrationDeltaInvalid,
        SQLITE_CONSTRAINT,
        "candidate registration changed "
        "an existing candidate");
    }
  }

  const auto * registered =
    find_candidate(
      post,
      request.candidate_id);

  if (
    registered == nullptr ||
    registered->state !=
      CandidateState::kPendingReview ||
    registered->candidate_revision != 1U ||
    registered->candidate.candidate_id !=
      request.candidate_id ||
    !registered->review_reason.empty() ||
    !registered->approved_location_id.empty())
  {
    return snapshot_failure(
      SnapshotCode::
        kCandidateRegistrationDeltaInvalid,
      SQLITE_CONSTRAINT,
      "registered candidate transition is invalid");
  }

  return snapshot_success(
    "candidate registration delta is valid");
}


SnapshotResult validate_location_enabled_delta(
  const CatalogSnapshot & current,
  const LocationEnabledCommit & request)
{
  const auto & post =
    request.post_update_snapshot;

  const auto * current_location =
    find_location(
      current,
      request.location_id);

  if (current_location == nullptr) {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_NOTFOUND,
      "location does not exist");
  }

  if (
    current_location->record_revision !=
    request.expected_record_revision)
  {
    return snapshot_failure(
      SnapshotCode::kStaleRevision,
      SQLITE_BUSY,
      "location revision is stale");
  }

  if (
    current_location->state ==
      LocationState::kRetired)
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "retired location cannot change enablement");
  }

  if (
    current_location->enabled ==
    request.enabled)
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "location already has the requested "
      "enablement state");
  }

  if (
    request.expected_record_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "location revision cannot be incremented");
  }

  if (
    post.candidates.size() !=
    current.candidates.size())
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "enablement change modified candidates");
  }

  for (
    const auto & previous :
    current.candidates)
  {
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
        SnapshotCode::
          kLocationEnabledDeltaInvalid,
        SQLITE_CONSTRAINT,
        "enablement change modified a candidate");
    }
  }

  if (
    post.locations.size() !=
    current.locations.size())
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "enablement change modified location count");
  }

  for (
    const auto & previous :
    current.locations)
  {
    const auto * next =
      find_location(
        post,
        previous.location.location_id);

    if (next == nullptr) {
      return snapshot_failure(
        SnapshotCode::
          kLocationEnabledDeltaInvalid,
        SQLITE_CONSTRAINT,
        "enablement change removed a location");
    }

    if (
      previous.location.location_id ==
      request.location_id)
    {
      continue;
    }

    if (
      !same_location_record(
        previous,
        *next))
    {
      return snapshot_failure(
        SnapshotCode::
          kLocationEnabledDeltaInvalid,
        SQLITE_CONSTRAINT,
        "enablement change modified "
        "an unrelated location");
    }
  }

  const auto * updated =
    find_location(
      post,
      request.location_id);

  if (
    updated == nullptr ||
    updated->state !=
      current_location->state ||
    updated->enabled != request.enabled ||
    updated->record_revision !=
      request.expected_record_revision + 1U ||
    updated->source_candidate_id !=
      current_location->source_candidate_id ||
    !same_location_draft(
      updated->location,
      current_location->location))
  {
    return snapshot_failure(
      SnapshotCode::
        kLocationEnabledDeltaInvalid,
      SQLITE_CONSTRAINT,
      "location enablement transition is invalid");
  }

  return snapshot_success(
    "location enablement delta is valid");
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

}  // namespace


std::string_view to_string(
  const SnapshotCode code) noexcept
{
  switch (code) {
    case SnapshotCode::kOk:
      return "ok";

    case SnapshotCode::kInvalidArgument:
      return "invalid_argument";

    case SnapshotCode::kStoreNotOpen:
      return "store_not_open";

    case SnapshotCode::kValidationFailed:
      return "validation_failed";

    case SnapshotCode::kIdentityConflict:
      return "identity_conflict";

    case SnapshotCode::kTagConflict:
      return "tag_conflict";

    case SnapshotCode::kTransactionActive:
      return "transaction_active";

    case SnapshotCode::kSqlError:
      return "sql_error";

    case SnapshotCode::kCorruptData:
      return "corrupt_data";

    case SnapshotCode::kStaleRevision:
      return "stale_revision";

    case SnapshotCode::
        kCandidateRegistrationDeltaInvalid:
      return "candidate_registration_delta_invalid";

    case SnapshotCode::kApprovalDeltaInvalid:
      return "approval_delta_invalid";

    case SnapshotCode::
        kLocationEnabledDeltaInvalid:
      return "location_enabled_delta_invalid";

    case SnapshotCode::kEventJournalError:
      return "event_journal_error";

    default:
      return "unknown";
  }
}


SqliteRepository::SqliteRepository(
  SqliteStore & store)
: store_(store)
{
}


SnapshotResult SqliteRepository::save_snapshot(
  const CatalogSnapshot & snapshot)
{
  const auto validation =
    validate_snapshot(snapshot);

  if (!validation.success) {
    return validation;
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
      "could not begin snapshot transaction");
  }

  store_.transaction_active_ = true;

  store_.transaction_owner_ =
    std::this_thread::get_id();

  const auto rollback =
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

  code = execute_sql(
    store_.database_,
    "DELETE FROM candidate_aliases;"
    "DELETE FROM location_candidates;"
    "DELETE FROM location_aliases;"
    "DELETE FROM locations;");

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not clear persisted snapshot");
  }

  const std::int64_t timestamp =
    unix_time_ns();

  for (
    const auto & location :
    snapshot.locations)
  {
    code = insert_location(
      store_.database_,
      location,
      timestamp);

    if (code != SQLITE_OK) {
      rollback();

      return sqlite_failure(
        store_.database_,
        code,
        "could not persist location");
    }

    code = insert_location_aliases(
      store_.database_,
      location);

    if (code != SQLITE_OK) {
      rollback();

      return sqlite_failure(
        store_.database_,
        code,
        "could not persist location identities");
    }
  }

  for (
    const auto & candidate :
    snapshot.candidates)
  {
    code = insert_candidate(
      store_.database_,
      candidate,
      timestamp);

    if (code != SQLITE_OK) {
      rollback();

      return sqlite_failure(
        store_.database_,
        code,
        "could not persist candidate");
    }

    code = insert_candidate_aliases(
      store_.database_,
      candidate);

    if (code != SQLITE_OK) {
      rollback();

      return sqlite_failure(
        store_.database_,
        code,
        "could not persist candidate aliases");
    }
  }

  code = execute_sql(
    store_.database_,
    "COMMIT;");

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not commit snapshot");
  }

  store_.transaction_active_ = false;

  store_.transaction_owner_ =
    std::thread::id{};

  return snapshot_success(
    "catalog snapshot persisted");
}


SnapshotResult SqliteRepository::load_snapshot(
  CatalogSnapshot * snapshot) const
{
  if (snapshot == nullptr) {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "snapshot output is required");
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
      "cannot load during an active transaction");
  }

  CatalogSnapshot loaded;

  auto result = read_locations(
    store_.database_,
    &loaded);

  if (!result.success) {
    return result;
  }

  result = read_candidates(
    store_.database_,
    &loaded);

  if (!result.success) {
    return result;
  }

  result = validate_snapshot(loaded);

  if (!result.success) {
    result.code = SnapshotCode::kCorruptData;
    result.sqlite_code = SQLITE_CORRUPT;
    result.reason =
      "persisted catalog failed domain validation: " +
      result.reason;

    return result;
  }

  *snapshot = std::move(loaded);

  return snapshot_success(
    "catalog snapshot loaded");
}

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
SqliteRepository::commit_candidate_registration(
  const CandidateRegistrationCommit & request,
  std::uint64_t * event_sequence)
{
  if (
    trim_ascii(request.candidate_id).empty() ||
    trim_ascii(request.actor_id).empty() ||
    trim_ascii(request.reason).empty())
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "candidate ID, actor and reason are required");
  }

  const auto post_validation =
    validate_snapshot(
      request.post_registration_snapshot);

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
      "could not begin candidate registration "
      "transaction");
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
      "persisted pre-registration catalog "
      "is invalid: " +
      result.reason;

    return result;
  }

  result = validate_registration_delta(
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
    request.post_registration_snapshot,
    timestamp);

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not persist candidate registration");
  }

  PersistenceEvent event;

  event.event_time_unix_ns = timestamp;

  event.event_type =
    PersistenceEventType::
      kCandidateRegistered;

  event.candidate_id =
    request.candidate_id;

  event.entity_revision = 1U;
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
      "candidate registration event append failed; "
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
      "could not commit candidate registration "
      "transaction");
  }

  store_.transaction_active_ = false;
  store_.transaction_owner_ =
    std::thread::id{};

  if (event_sequence != nullptr) {
    *event_sequence = inserted_sequence;
  }

  return snapshot_success(
    "candidate registration persisted atomically");
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


SnapshotResult
SqliteRepository::commit_location_enabled(
  const LocationEnabledCommit & request,
  std::uint64_t * event_sequence)
{
  if (
    trim_ascii(request.location_id).empty() ||
    request.expected_record_revision == 0U ||
    trim_ascii(request.actor_id).empty() ||
    trim_ascii(request.reason).empty())
  {
    return snapshot_failure(
      SnapshotCode::kInvalidArgument,
      SQLITE_MISUSE,
      "location ID, revision, actor and reason "
      "are required");
  }

  const auto post_validation =
    validate_snapshot(
      request.post_update_snapshot);

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
      "could not begin location enablement "
      "transaction");
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
      "persisted pre-enablement catalog "
      "is invalid: " +
      result.reason;

    return result;
  }

  result = validate_location_enabled_delta(
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
    request.post_update_snapshot,
    timestamp);

  if (code != SQLITE_OK) {
    rollback();

    return sqlite_failure(
      store_.database_,
      code,
      "could not persist location enablement");
  }

  PersistenceEvent event;

  event.event_time_unix_ns = timestamp;

  event.event_type =
    PersistenceEventType::
      kLocationEnabledChanged;

  event.location_id =
    request.location_id;

  event.entity_revision =
    request.expected_record_revision + 1U;

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
      "location enablement event append failed; "
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
      "could not commit location enablement "
      "transaction");
  }

  store_.transaction_active_ = false;
  store_.transaction_owner_ =
    std::thread::id{};

  if (event_sequence != nullptr) {
    *event_sequence = inserted_sequence;
  }

  return snapshot_success(
    "location enablement persisted atomically");
}

}  // namespace savo_locations
