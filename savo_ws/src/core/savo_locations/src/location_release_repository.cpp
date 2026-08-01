#include "savo_locations/sqlite_repository.hpp"

#include <openssl/evp.h>
#include <sqlite3.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace savo_locations
{
namespace
{

class Statement
{
public:
  Statement(sqlite3 * database, const char * sql)
  {
    code_ = sqlite3_prepare_v2(
      database, sql, -1, &statement_, nullptr);
  }

  ~Statement()
  {
    if (statement_ != nullptr) {
      sqlite3_finalize(statement_);
    }
  }

  Statement(const Statement &) = delete;
  Statement & operator=(const Statement &) = delete;

  [[nodiscard]] int code() const noexcept
  {
    return code_;
  }

  [[nodiscard]] sqlite3_stmt * get() const noexcept
  {
    return statement_;
  }

private:
  sqlite3_stmt * statement_{nullptr};
  int code_{SQLITE_ERROR};
};


SnapshotResult ok(std::string reason)
{
  SnapshotResult result;
  result.success = true;
  result.code = SnapshotCode::kOk;
  result.sqlite_code = SQLITE_OK;
  result.reason = std::move(reason);
  return result;
}


SnapshotResult fail(
  const SnapshotCode code,
  const int sqlite_code,
  std::string reason)
{
  SnapshotResult result;
  result.code = code;
  result.sqlite_code = sqlite_code;
  result.reason = std::move(reason);
  return result;
}


std::int64_t unix_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}


int bind_text(
  sqlite3_stmt * statement,
  const int index,
  const std::string_view value)
{
  if (
    value.size() >
    static_cast<std::size_t>(std::numeric_limits<int>::max()))
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


std::string column_text(sqlite3_stmt * statement, const int index)
{
  const auto * value = sqlite3_column_text(statement, index);
  return value == nullptr ? "" :
         reinterpret_cast<const char *>(value);
}


std::string json_escape(const std::string_view input)
{
  std::string output;
  for (const char value : input) {
    switch (value) {
      case '\\': output += "\\\\"; break;
      case '"': output += "\\\""; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default: output += value; break;
    }
  }
  return output;
}


bool valid_identifier(const std::string_view value)
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }

  return std::all_of(
    value.begin(), value.end(),
    [](const char character) {
      const auto raw = static_cast<unsigned char>(character);
      return std::isalnum(raw) != 0 ||
             character == '-' || character == '_' || character == '.';
    });
}


std::string sha256(const std::string_view contents)
{
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr) {
    return {};
  }

  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int digest_size = 0U;

  const bool succeeded =
    EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1 &&
    EVP_DigestUpdate(context, contents.data(), contents.size()) == 1 &&
    EVP_DigestFinal_ex(context, digest, &digest_size) == 1;

  EVP_MD_CTX_free(context);

  if (!succeeded) {
    return {};
  }

  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}


std::optional<std::string> read_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input.good()) {
    return std::nullopt;
  }

  std::ostringstream contents;
  contents << input.rdbuf();
  if (!input.good() && !input.eof()) {
    return std::nullopt;
  }
  return contents.str();
}


std::string serialize_snapshot(
  const PrepareLocationReleaseRequest & request,
  const std::vector<const LocationRecordData *> & locations,
  const std::size_t candidate_count)
{
  std::ostringstream output;
  output << std::setprecision(17);
  output << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"release_id\": \"" << json_escape(request.release_id) << "\",\n"
         << "  \"mission_id\": \"" << json_escape(request.mission_id) << "\",\n"
         << "  \"map_id\": \"" << json_escape(request.map_id) << "\",\n"
         << "  \"map_revision\": " << request.map_revision << ",\n"
         << "  \"candidate_count\": " << candidate_count << ",\n"
         << "  \"locations\": [";

  for (std::size_t index = 0U; index < locations.size(); ++index) {
    const auto & record = *locations[index];
    const auto & location = record.location;
    const auto & pose = location.approach_pose;
    output << (index == 0U ? "\n" : ",\n")
           << "    {\"location_id\":\"" << json_escape(location.location_id)
           << "\",\"record_revision\":" << record.record_revision
           << ",\"enabled\":" << (record.enabled ? "true" : "false")
           << ",\"display_name\":\"" << json_escape(location.display_name)
           << "\",\"semantic_type\":\"" << json_escape(location.semantic_type)
           << "\",\"approach_pose\":{\"frame_id\":\""
           << json_escape(pose.frame_id) << "\",\"position\":["
           << pose.x << ',' << pose.y << ',' << pose.z
           << "],\"orientation\":[" << pose.qx << ',' << pose.qy << ','
           << pose.qz << ',' << pose.qw << "]},\"tag\":{\"family\":\""
           << json_escape(location.tag.family) << "\",\"id\":"
           << location.tag.id << "}}";
  }

  output << (locations.empty() ? "" : "\n") << "  ]\n}\n";
  return output.str();
}


SnapshotResult load_release_locked(
  sqlite3 * database,
  const std::string & release_id,
  LocationReleaseRecord * release)
{
  if (release == nullptr) {
    return fail(
      SnapshotCode::kInvalidArgument, SQLITE_MISUSE,
      "release output is required");
  }

  Statement query(
    database,
    "SELECT release_id,transaction_token,mission_id,map_id,map_revision,"
    "state,snapshot_path,snapshot_sha256,location_count,candidate_count,"
    "approving_actor,approval_reason,prepared_at_unix_ns,"
    "COALESCE(committed_at_unix_ns,0),previous_active_location_release,"
    "failure_rollback_reason FROM location_releases WHERE release_id=?1;");

  if (query.code() != SQLITE_OK ||
    bind_text(query.get(), 1, release_id) != SQLITE_OK)
  {
    return fail(
      SnapshotCode::kSqlError, sqlite3_errcode(database),
      "could not query location release");
  }

  const int code = sqlite3_step(query.get());
  if (code == SQLITE_DONE) {
    return fail(
      SnapshotCode::kReleaseNotFound, SQLITE_NOTFOUND,
      "location release was not found");
  }
  if (code != SQLITE_ROW) {
    return fail(
      SnapshotCode::kSqlError, code,
      "location release query failed");
  }

  release->release_id = column_text(query.get(), 0);
  release->transaction_token = column_text(query.get(), 1);
  release->mission_id = column_text(query.get(), 2);
  release->map_id = column_text(query.get(), 3);
  release->map_revision = static_cast<std::uint32_t>(
    sqlite3_column_int64(query.get(), 4));
  release->state = column_text(query.get(), 5);
  release->snapshot_path = column_text(query.get(), 6);
  release->snapshot_sha256 = column_text(query.get(), 7);
  release->location_count = static_cast<std::size_t>(
    sqlite3_column_int64(query.get(), 8));
  release->candidate_count = static_cast<std::size_t>(
    sqlite3_column_int64(query.get(), 9));
  release->approving_actor = column_text(query.get(), 10);
  release->approval_reason = column_text(query.get(), 11);
  release->prepared_at_unix_ns = sqlite3_column_int64(query.get(), 12);
  release->committed_at_unix_ns = sqlite3_column_int64(query.get(), 13);
  release->previous_active_location_release = column_text(query.get(), 14);
  release->failure_rollback_reason = column_text(query.get(), 15);
  return ok("location release loaded");
}


SnapshotResult verify_correlation(
  const LocationReleaseRecord & release,
  const std::string & mission_id,
  const std::string & token,
  const std::string & expected_digest)
{
  if (release.mission_id != mission_id) {
    return fail(
      SnapshotCode::kReleaseConflict, SQLITE_CONSTRAINT,
      "mission ID does not match prepared release");
  }
  if (release.transaction_token != token) {
    return fail(
      SnapshotCode::kStaleTransaction, SQLITE_CONSTRAINT,
      "transaction token is stale");
  }
  if (release.snapshot_sha256 != expected_digest) {
    return fail(
      SnapshotCode::kDigestMismatch, SQLITE_CONSTRAINT,
      "expected location snapshot digest does not match");
  }
  return ok("release correlation verified");
}


int append_release_event(
  sqlite3 * database,
  const PersistenceEventType event_type,
  const std::string & release_id,
  const std::string & actor_id,
  const std::string & reason,
  const std::int64_t timestamp,
  std::uint64_t * sequence)
{
  Statement insert(
    database,
    "INSERT INTO location_events(event_time_unix_ns,event_type,candidate_id,"
    "location_id,entity_revision,actor_id,reason,event_payload_json) "
    "VALUES(?1,?2,'','',0,?3,?4,?5);");
  if (insert.code() != SQLITE_OK) {
    return insert.code();
  }

  const std::string payload =
    "{\"release_id\":\"" + json_escape(release_id) + "\"}";
  int code = sqlite3_bind_int64(insert.get(), 1, timestamp);
  if (code == SQLITE_OK) {
    code = sqlite3_bind_int(
      insert.get(), 2, static_cast<int>(event_type));
  }
  if (code == SQLITE_OK) {
    code = bind_text(insert.get(), 3, actor_id);
  }
  if (code == SQLITE_OK) {
    code = bind_text(insert.get(), 4, reason);
  }
  if (code == SQLITE_OK) {
    code = bind_text(insert.get(), 5, payload);
  }
  if (code == SQLITE_OK) {
    code = sqlite3_step(insert.get());
  }
  if (code == SQLITE_DONE && sequence != nullptr) {
    *sequence = static_cast<std::uint64_t>(
      sqlite3_last_insert_rowid(database));
  }
  return code == SQLITE_DONE ? SQLITE_OK : code;
}


int set_active_release(
  sqlite3 * database,
  const std::string & release_id,
  const std::int64_t timestamp)
{
  Statement statement(
    database,
    "INSERT INTO registry_metadata(metadata_key,metadata_value,updated_at_unix_ns) "
    "VALUES('active_location_release_id',?1,?2) ON CONFLICT(metadata_key) "
    "DO UPDATE SET metadata_value=excluded.metadata_value,"
    "updated_at_unix_ns=excluded.updated_at_unix_ns;");
  if (statement.code() != SQLITE_OK) {
    return statement.code();
  }
  int code = bind_text(statement.get(), 1, release_id);
  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(statement.get(), 2, timestamp);
  }
  if (code == SQLITE_OK) {
    code = sqlite3_step(statement.get());
  }
  return code == SQLITE_DONE ? SQLITE_OK : code;
}

}  // namespace


SnapshotResult SqliteRepository::prepare_location_release(
  const PrepareLocationReleaseRequest & request,
  const CatalogSnapshot & snapshot,
  LocationReleaseRecord * release,
  std::uint64_t * event_sequence)
{
  if (
    release == nullptr || event_sequence == nullptr ||
    !valid_identifier(request.release_id) ||
    !valid_identifier(request.mission_id) ||
    request.map_id.empty() || request.map_revision == 0U ||
    request.actor_id.empty() || request.releases_root.empty())
  {
    return fail(
      SnapshotCode::kInvalidArgument, SQLITE_MISUSE,
      "valid release, mission, map, actor, root and outputs are required");
  }

  std::vector<const LocationRecordData *> locations;
  std::size_t candidate_count = 0U;
  for (const auto & candidate : snapshot.candidates) {
    if (candidate.candidate.map.map_id == request.map_id &&
      candidate.candidate.map.map_revision == request.map_revision)
    {
      ++candidate_count;
      if (candidate.state == CandidateState::kPendingReview) {
        return fail(
          SnapshotCode::kPendingCandidates, SQLITE_CONSTRAINT,
          "pending location candidates block release");
      }
    }
  }

  for (const auto & record : snapshot.locations) {
    if (record.location.map.map_id != request.map_id ||
      record.location.map.map_revision != request.map_revision ||
      record.state != LocationState::kApproved)
    {
      continue;
    }
    const auto validation = validate_location_draft(record.location);
    if (!validation.valid() || record.location.approach_pose.frame_id != "map") {
      return fail(
        SnapshotCode::kValidationFailed, SQLITE_CONSTRAINT,
        "approved location record is malformed or not in map frame");
    }
    locations.push_back(&record);
  }

  std::sort(
    locations.begin(), locations.end(),
    [](const auto * left, const auto * right) {
      return left->location.location_id < right->location.location_id;
    });

  if (request.require_approved_location && locations.empty()) {
    return fail(
      SnapshotCode::kNoApprovedLocations, SQLITE_CONSTRAINT,
      "at least one approved location is required");
  }

  const std::string contents =
    serialize_snapshot(request, locations, candidate_count);
  const std::string digest = sha256(contents);
  if (digest.size() != 64U) {
    return fail(
      SnapshotCode::kFilesystemError, SQLITE_IOERR,
      "could not compute location snapshot SHA-256");
  }

  const std::int64_t timestamp = unix_time_ns();
  const std::string token = sha256(
    request.release_id + "\n" + request.mission_id + "\n" +
    request.map_id + "\n" + std::to_string(request.map_revision) + "\n" +
    std::to_string(timestamp));

  const std::filesystem::path root(request.releases_root);
  const std::filesystem::path final_directory = root / request.release_id;
  const std::filesystem::path staging_directory =
    root / ("." + request.release_id + ".staging-" + token.substr(0U, 12U));
  const std::filesystem::path final_snapshot = final_directory / "locations.json";

  std::error_code filesystem_error;
  std::filesystem::create_directories(root, filesystem_error);
  if (filesystem_error || std::filesystem::exists(final_directory)) {
    return fail(
      SnapshotCode::kReleaseConflict, SQLITE_CONSTRAINT,
      filesystem_error ? filesystem_error.message() :
      "location release directory already exists");
  }
  std::filesystem::create_directory(staging_directory, filesystem_error);
  if (filesystem_error) {
    return fail(
      SnapshotCode::kFilesystemError, SQLITE_IOERR,
      "could not create location release staging directory: " +
      filesystem_error.message());
  }

  const std::filesystem::path staged_snapshot =
    staging_directory / "locations.json";
  {
    std::ofstream output(staged_snapshot, std::ios::binary | std::ios::trunc);
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.flush();
    if (!output.good()) {
      std::filesystem::remove_all(staging_directory, filesystem_error);
      return fail(
        SnapshotCode::kFilesystemError, SQLITE_IOERR,
        "could not write location release snapshot");
    }
  }

  std::filesystem::rename(staging_directory, final_directory, filesystem_error);
  if (filesystem_error) {
    std::filesystem::remove_all(staging_directory, filesystem_error);
    return fail(
      SnapshotCode::kFilesystemError, SQLITE_IOERR,
      "could not atomically publish location snapshot");
  }

  std::lock_guard<std::mutex> lock(store_.mutex_);
  if (store_.database_ == nullptr || store_.transaction_active_) {
    std::filesystem::remove_all(final_directory, filesystem_error);
    return fail(
      store_.database_ == nullptr ? SnapshotCode::kStoreNotOpen :
      SnapshotCode::kTransactionActive,
      SQLITE_BUSY, "location store is not available for release preparation");
  }

  std::string previous_active;
  {
    Statement query(
      store_.database_,
      "SELECT metadata_value FROM registry_metadata "
      "WHERE metadata_key='active_location_release_id';");
    if (query.code() == SQLITE_OK && sqlite3_step(query.get()) == SQLITE_ROW) {
      previous_active = column_text(query.get(), 0);
    }
  }

  int code = sqlite3_exec(store_.database_, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr);
  if (code == SQLITE_OK) {
    store_.transaction_active_ = true;
    store_.transaction_owner_ = std::this_thread::get_id();
  }

  Statement insert(
    store_.database_,
    "INSERT INTO location_releases(release_id,transaction_token,mission_id,"
    "map_id,map_revision,state,snapshot_path,snapshot_sha256,location_count,"
    "candidate_count,approving_actor,approval_reason,prepared_at_unix_ns,"
    "previous_active_location_release) VALUES(?1,?2,?3,?4,?5,'prepared',?6,"
    "?7,?8,?9,?10,?11,?12,?13);");
  if (code == SQLITE_OK) {
    code = insert.code();
  }
  const std::vector<std::string> text_values{
    request.release_id, token, request.mission_id, request.map_id,
    final_snapshot.string(), digest, request.actor_id,
    request.approval_reason, previous_active};
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 1, text_values[0]);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 2, text_values[1]);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 3, text_values[2]);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 4, text_values[3]);}
  if (code == SQLITE_OK) {code = sqlite3_bind_int64(insert.get(), 5, request.map_revision);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 6, text_values[4]);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 7, text_values[5]);}
  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      insert.get(), 8, static_cast<sqlite3_int64>(locations.size()));
  }
  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      insert.get(), 9, static_cast<sqlite3_int64>(candidate_count));
  }
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 10, text_values[6]);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 11, text_values[7]);}
  if (code == SQLITE_OK) {code = sqlite3_bind_int64(insert.get(), 12, timestamp);}
  if (code == SQLITE_OK) {code = bind_text(insert.get(), 13, text_values[8]);}
  if (code == SQLITE_OK) {code = sqlite3_step(insert.get());}
  if (code == SQLITE_DONE) {
    code = append_release_event(
      store_.database_, PersistenceEventType::kLocationReleasePrepared,
      request.release_id, request.actor_id, "location release prepared",
      timestamp, event_sequence);
  }
  if (code == SQLITE_OK) {
    code = sqlite3_exec(store_.database_, "COMMIT;", nullptr, nullptr, nullptr);
  }
  if (code != SQLITE_OK) {
    sqlite3_exec(store_.database_, "ROLLBACK;", nullptr, nullptr, nullptr);
    store_.transaction_active_ = false;
    store_.transaction_owner_ = std::thread::id{};
    std::filesystem::remove_all(final_directory, filesystem_error);
    return fail(
      code == SQLITE_CONSTRAINT ? SnapshotCode::kReleaseConflict :
      SnapshotCode::kSqlError,
      code, "could not persist prepared location release");
  }
  store_.transaction_active_ = false;
  store_.transaction_owner_ = std::thread::id{};

  release->release_id = request.release_id;
  release->transaction_token = token;
  release->mission_id = request.mission_id;
  release->map_id = request.map_id;
  release->map_revision = request.map_revision;
  release->state = "prepared";
  release->snapshot_path = final_snapshot.string();
  release->snapshot_sha256 = digest;
  release->location_count = locations.size();
  release->candidate_count = candidate_count;
  release->approving_actor = request.actor_id;
  release->approval_reason = request.approval_reason;
  release->prepared_at_unix_ns = timestamp;
  release->previous_active_location_release = previous_active;

  std::filesystem::permissions(
    final_snapshot,
    std::filesystem::perms::owner_read |
    std::filesystem::perms::group_read |
    std::filesystem::perms::others_read,
    std::filesystem::perm_options::replace,
    filesystem_error);
  return ok("location release prepared and audited");
}


SnapshotResult SqliteRepository::verify_location_release(
  const std::string & release_id,
  const std::string & mission_id,
  const std::string & transaction_token,
  const std::string & expected_digest,
  LocationReleaseRecord * release) const
{
  std::lock_guard<std::mutex> lock(store_.mutex_);
  if (store_.database_ == nullptr) {
    return fail(SnapshotCode::kStoreNotOpen, SQLITE_MISUSE, "location store is closed");
  }
  auto result = load_release_locked(store_.database_, release_id, release);
  if (!result.success) {return result;}
  result = verify_correlation(*release, mission_id, transaction_token, expected_digest);
  if (!result.success) {return result;}
  const auto contents = read_file(release->snapshot_path);
  if (!contents.has_value() || sha256(*contents) != release->snapshot_sha256) {
    return fail(
      SnapshotCode::kDigestMismatch, SQLITE_CORRUPT,
      "location snapshot is missing or its SHA-256 changed");
  }
  return ok("prepared location release verified");
}


SnapshotResult SqliteRepository::commit_location_release(
  const std::string & release_id,
  const std::string & mission_id,
  const std::string & transaction_token,
  const std::string & expected_digest,
  const std::string & actor_id,
  LocationReleaseRecord * release,
  std::uint64_t * event_sequence)
{
  auto verified = verify_location_release(
    release_id, mission_id, transaction_token, expected_digest, release);
  if (!verified.success) {return verified;}
  if (actor_id.empty() || event_sequence == nullptr || release->state != "prepared") {
    return fail(
      SnapshotCode::kInvalidArgument, SQLITE_CONSTRAINT,
      "prepared release, actor and event output are required");
  }

  std::lock_guard<std::mutex> lock(store_.mutex_);
  const std::int64_t timestamp = unix_time_ns();
  int code = sqlite3_exec(store_.database_, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr);
  if (code == SQLITE_OK) {
    store_.transaction_active_ = true;
    store_.transaction_owner_ = std::this_thread::get_id();
  }

  Statement generation(
    store_.database_,
    "SELECT (SELECT COUNT(*) FROM location_candidates WHERE map_id=?1 AND "
    "map_revision=?2 AND state=1), (SELECT COUNT(*) FROM locations WHERE "
    "map_id=?1 AND map_revision=?2 AND state=1);");
  if (code == SQLITE_OK) {code = generation.code();}
  if (code == SQLITE_OK) {code = bind_text(generation.get(), 1, release->map_id);}
  if (code == SQLITE_OK) {code = sqlite3_bind_int64(generation.get(), 2, release->map_revision);}
  if (code == SQLITE_OK) {code = sqlite3_step(generation.get());}
  if (code == SQLITE_ROW &&
    (sqlite3_column_int64(generation.get(), 0) != 0 ||
    static_cast<std::size_t>(sqlite3_column_int64(generation.get(), 1)) !=
    release->location_count))
  {
    code = SQLITE_CONSTRAINT;
  } else if (code == SQLITE_ROW) {
    code = SQLITE_OK;
  }

  Statement bind_locations(
    store_.database_,
    "UPDATE locations SET map_release_id=?1,updated_at_unix_ns=?2 "
    "WHERE map_id=?3 AND map_revision=?4 AND state=1;");
  if (code == SQLITE_OK) {code = bind_locations.code();}
  if (code == SQLITE_OK) {code = bind_text(bind_locations.get(), 1, release_id);}
  if (code == SQLITE_OK) {code = sqlite3_bind_int64(bind_locations.get(), 2, timestamp);}
  if (code == SQLITE_OK) {code = bind_text(bind_locations.get(), 3, release->map_id);}
  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(bind_locations.get(), 4, release->map_revision);
  }
  if (code == SQLITE_OK) {code = sqlite3_step(bind_locations.get());}
  if (code == SQLITE_DONE) {code = SQLITE_OK;}

  Statement update(
    store_.database_,
    "UPDATE location_releases SET state='committed',committed_at_unix_ns=?1 "
    "WHERE release_id=?2 AND state='prepared' AND transaction_token=?3;");
  if (code == SQLITE_OK) {code = update.code();}
  if (code == SQLITE_OK) {code = sqlite3_bind_int64(update.get(), 1, timestamp);}
  if (code == SQLITE_OK) {code = bind_text(update.get(), 2, release_id);}
  if (code == SQLITE_OK) {code = bind_text(update.get(), 3, transaction_token);}
  if (code == SQLITE_OK) {code = sqlite3_step(update.get());}
  if (code == SQLITE_DONE && sqlite3_changes(store_.database_) == 1) {
    code = SQLITE_OK;
  } else if (code == SQLITE_DONE) {code = SQLITE_CONSTRAINT;}
  if (code == SQLITE_OK) {code = set_active_release(store_.database_, release_id, timestamp);}
  if (code == SQLITE_OK) {
    code = append_release_event(
      store_.database_, PersistenceEventType::kLocationReleaseCommitted,
      release_id, actor_id, "location release committed", timestamp, event_sequence);
  }
  if (code == SQLITE_OK) {
    code = sqlite3_exec(store_.database_, "COMMIT;", nullptr, nullptr, nullptr);
  }
  if (code != SQLITE_OK) {
    sqlite3_exec(store_.database_, "ROLLBACK;", nullptr, nullptr, nullptr);
    store_.transaction_active_ = false;
    store_.transaction_owner_ = std::thread::id{};
    return fail(
      SnapshotCode::kSqlError, code,
      "location release commit failed; database transaction rolled back");
  }
  store_.transaction_active_ = false;
  store_.transaction_owner_ = std::thread::id{};
  release->state = "committed";
  release->committed_at_unix_ns = timestamp;
  return ok("location release committed and active metadata updated atomically");
}


SnapshotResult SqliteRepository::rollback_location_release(
  const std::string & release_id,
  const std::string & mission_id,
  const std::string & transaction_token,
  const std::string & expected_digest,
  const std::string & actor_id,
  const std::string & reason,
  LocationReleaseRecord * release,
  std::uint64_t * event_sequence)
{
  auto verified = verify_location_release(
    release_id, mission_id, transaction_token, expected_digest, release);
  if (!verified.success) {return verified;}
  if (actor_id.empty() || reason.empty() || event_sequence == nullptr) {
    return fail(
      SnapshotCode::kInvalidArgument, SQLITE_CONSTRAINT,
      "rollback requires actor, reason and event output");
  }

  std::lock_guard<std::mutex> lock(store_.mutex_);
  const std::int64_t timestamp = unix_time_ns();
  if (release->state == "rolled_back") {
    int duplicate_code = sqlite3_exec(
      store_.database_, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr);
    if (duplicate_code == SQLITE_OK) {
      store_.transaction_active_ = true;
      store_.transaction_owner_ = std::this_thread::get_id();
      duplicate_code = append_release_event(
        store_.database_, PersistenceEventType::kLocationReleaseRolledBack,
        release_id, actor_id, "idempotent recovery: " + reason,
        timestamp, event_sequence);
    }
    if (duplicate_code == SQLITE_OK) {
      duplicate_code = sqlite3_exec(
        store_.database_, "COMMIT;", nullptr, nullptr, nullptr);
    }
    if (duplicate_code != SQLITE_OK) {
      sqlite3_exec(store_.database_, "ROLLBACK;", nullptr, nullptr, nullptr);
      store_.transaction_active_ = false;
      store_.transaction_owner_ = std::thread::id{};
      return fail(
        SnapshotCode::kSqlError, duplicate_code,
        "idempotent rollback audit failed");
    }
    store_.transaction_active_ = false;
    store_.transaction_owner_ = std::thread::id{};
    return ok("location release was already rolled back; recovery audited");
  }

  int code = sqlite3_exec(store_.database_, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr);
  if (code == SQLITE_OK) {
    store_.transaction_active_ = true;
    store_.transaction_owner_ = std::this_thread::get_id();
  }
  if (code == SQLITE_OK && release->state == "committed") {
    Statement restore(
      store_.database_,
      "UPDATE locations SET map_release_id=?1,updated_at_unix_ns=?2 "
      "WHERE map_release_id=?3;");
    code = restore.code();
    if (code == SQLITE_OK) {
      code = bind_text(
        restore.get(), 1, release->previous_active_location_release);
    }
    if (code == SQLITE_OK) {code = sqlite3_bind_int64(restore.get(), 2, timestamp);}
    if (code == SQLITE_OK) {code = bind_text(restore.get(), 3, release_id);}
    if (code == SQLITE_OK) {code = sqlite3_step(restore.get());}
    if (code == SQLITE_DONE) {code = SQLITE_OK;}
    if (code == SQLITE_OK) {
      code = set_active_release(
        store_.database_, release->previous_active_location_release, timestamp);
    }
  }

  Statement update(
    store_.database_,
    "UPDATE location_releases SET state='rolled_back',failure_rollback_reason=?1 "
    "WHERE release_id=?2 AND transaction_token=?3 AND state!='rolled_back';");
  if (code == SQLITE_OK) {code = update.code();}
  if (code == SQLITE_OK) {code = bind_text(update.get(), 1, reason);}
  if (code == SQLITE_OK) {code = bind_text(update.get(), 2, release_id);}
  if (code == SQLITE_OK) {code = bind_text(update.get(), 3, transaction_token);}
  if (code == SQLITE_OK) {code = sqlite3_step(update.get());}
  if (code == SQLITE_DONE && sqlite3_changes(store_.database_) == 1) {
    code = SQLITE_OK;
  } else if (code == SQLITE_DONE) {code = SQLITE_CONSTRAINT;}
  if (code == SQLITE_OK) {
    code = append_release_event(
      store_.database_, PersistenceEventType::kLocationReleaseRolledBack,
      release_id, actor_id, reason, timestamp, event_sequence);
  }
  if (code == SQLITE_OK) {
    code = sqlite3_exec(store_.database_, "COMMIT;", nullptr, nullptr, nullptr);
  }
  if (code != SQLITE_OK) {
    sqlite3_exec(store_.database_, "ROLLBACK;", nullptr, nullptr, nullptr);
    store_.transaction_active_ = false;
    store_.transaction_owner_ = std::thread::id{};
    return fail(
      SnapshotCode::kSqlError, code,
      "location release rollback failed; authoritative state is unchanged");
  }
  store_.transaction_active_ = false;
  store_.transaction_owner_ = std::thread::id{};
  release->state = "rolled_back";
  release->failure_rollback_reason = reason;
  return ok("location release rolled back and previous active release restored");
}


SnapshotResult SqliteRepository::active_location_release_id(
  std::string * release_id) const
{
  if (release_id == nullptr) {
    return fail(
      SnapshotCode::kInvalidArgument, SQLITE_MISUSE,
      "active release output is required");
  }
  std::lock_guard<std::mutex> lock(store_.mutex_);
  if (store_.database_ == nullptr) {
    return fail(SnapshotCode::kStoreNotOpen, SQLITE_MISUSE, "location store is closed");
  }
  Statement query(
    store_.database_,
    "SELECT metadata_value FROM registry_metadata "
    "WHERE metadata_key='active_location_release_id';");
  if (query.code() != SQLITE_OK) {
    return fail(SnapshotCode::kSqlError, query.code(), "active release query failed");
  }
  const int code = sqlite3_step(query.get());
  if (code == SQLITE_DONE) {
    release_id->clear();
    return ok("no active location release");
  }
  if (code != SQLITE_ROW) {
    return fail(SnapshotCode::kSqlError, code, "active release query failed");
  }
  *release_id = column_text(query.get(), 0);
  return ok("active location release loaded");
}

}  // namespace savo_locations
