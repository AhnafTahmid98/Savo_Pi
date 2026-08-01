#include "savo_locations/sqlite_store.hpp"

#include <sqlite3.h>

#include <chrono>
#include <filesystem>
#include <limits>
#include <utility>

#include "savo_locations/sqlite_schema.hpp"

namespace savo_locations
{
namespace
{

constexpr int kBusyTimeoutMs{5000};


StorageResult success(
  std::string reason = "ok")
{
  StorageResult result;
  result.success = true;
  result.code = StorageCode::kOk;
  result.sqlite_code = SQLITE_OK;
  result.reason = std::move(reason);
  return result;
}


StorageResult failure(
  const StorageCode code,
  const int sqlite_code,
  std::string reason)
{
  StorageResult result;
  result.success = false;
  result.code = code;
  result.sqlite_code = sqlite_code;
  result.reason = std::move(reason);
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


bool is_memory_database(
  const std::string_view path)
{
  return
    path == ":memory:" ||
    path.find("mode=memory") !=
    std::string_view::npos;
}


std::string sqlite_error(
  sqlite3 * database,
  const std::string_view prefix)
{
  std::string output{prefix};

  if (database != nullptr) {
    output += ": ";
    output += sqlite3_errmsg(database);
  }

  return output;
}

}  // namespace


std::string_view to_string(
  const StorageCode code) noexcept
{
  switch (code) {
    case StorageCode::kOk:
      return "ok";

    case StorageCode::kInvalidArgument:
      return "invalid_argument";

    case StorageCode::kNotOpen:
      return "not_open";

    case StorageCode::kOpenFailed:
      return "open_failed";

    case StorageCode::kConfigurationFailed:
      return "configuration_failed";

    case StorageCode::kSqlError:
      return "sql_error";

    case StorageCode::kMigrationFailed:
      return "migration_failed";

    case StorageCode::kSchemaTooNew:
      return "schema_too_new";

    case StorageCode::kIntegrityFailed:
      return "integrity_failed";

    case StorageCode::kTransactionAlreadyActive:
      return "transaction_already_active";

    case StorageCode::kTransactionNotActive:
      return "transaction_not_active";

    case StorageCode::kTransactionOwnerMismatch:
      return "transaction_owner_mismatch";

    default:
      return "unknown";
  }
}


SqliteStore::SqliteStore(
  std::string database_path)
: database_path_(std::move(database_path))
{
}


SqliteStore::~SqliteStore()
{
  const auto result = close();
  static_cast<void>(result);
}


StorageResult SqliteStore::open()
{
  std::lock_guard<std::mutex> lock{mutex_};

  if (database_ != nullptr) {
    return success("database already open");
  }

  if (database_path_.empty()) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "database path is empty");
  }

  if (!is_memory_database(database_path_)) {
    std::error_code error;

    const std::filesystem::path path{
      database_path_};

    const auto parent = path.parent_path();

    if (!parent.empty()) {
      std::filesystem::create_directories(
        parent,
        error);

      if (error) {
        return failure(
          StorageCode::kOpenFailed,
          SQLITE_CANTOPEN,
          "could not create database directory: " +
          error.message());
      }
    }
  }

  sqlite3 * opened = nullptr;

  const int flags =
    SQLITE_OPEN_READWRITE |
    SQLITE_OPEN_CREATE |
    SQLITE_OPEN_FULLMUTEX |
    SQLITE_OPEN_URI;

  const int code = sqlite3_open_v2(
    database_path_.c_str(),
    &opened,
    flags,
    nullptr);

  if (code != SQLITE_OK) {
    const auto reason =
      sqlite_error(
        opened,
        "could not open database");

    if (opened != nullptr) {
      sqlite3_close_v2(opened);
    }

    return failure(
      StorageCode::kOpenFailed,
      code,
      reason);
  }

  database_ = opened;

  sqlite3_extended_result_codes(
    database_,
    1);

  int timeout_code = sqlite3_busy_timeout(
    database_,
    kBusyTimeoutMs);

  if (timeout_code != SQLITE_OK) {
    const auto reason =
      sqlite_error(
        database_,
        "could not set busy timeout");

    sqlite3_close_v2(database_);
    database_ = nullptr;

    return failure(
      StorageCode::kConfigurationFailed,
      timeout_code,
      reason);
  }

  configuration_.busy_timeout_ms =
    kBusyTimeoutMs;

  for (const auto sql : {
      std::string_view{"PRAGMA foreign_keys=ON;"},
      std::string_view{"PRAGMA synchronous=NORMAL;"},
      std::string_view{"PRAGMA temp_store=MEMORY;"}
    })
  {
    const auto result =
      execute_locked(sql);

    if (!result.success) {
      sqlite3_close_v2(database_);
      database_ = nullptr;

      return failure(
        StorageCode::kConfigurationFailed,
        result.sqlite_code,
        result.reason);
    }
  }

  std::string journal_mode;

  const auto journal_result =
    query_single_text_locked(
      is_memory_database(database_path_) ?
        "PRAGMA journal_mode;" :
        "PRAGMA journal_mode=WAL;",
      &journal_mode);

  if (!journal_result.success) {
    sqlite3_close_v2(database_);
    database_ = nullptr;

    return failure(
      StorageCode::kConfigurationFailed,
      journal_result.sqlite_code,
      journal_result.reason);
  }

  if (
    !is_memory_database(database_path_) &&
    journal_mode != "wal")
  {
    sqlite3_close_v2(database_);
    database_ = nullptr;

    return failure(
      StorageCode::kConfigurationFailed,
      SQLITE_ERROR,
      "file database did not enter WAL mode");
  }

  configuration_.journal_mode =
    journal_mode;

  int foreign_keys = 0;

  const auto foreign_key_result =
    query_single_int_locked(
      "PRAGMA foreign_keys;",
      &foreign_keys);

  if (
    !foreign_key_result.success ||
    foreign_keys != 1)
  {
    const int failure_code =
      foreign_key_result.success ?
      SQLITE_ERROR :
      foreign_key_result.sqlite_code;

    sqlite3_close_v2(database_);
    database_ = nullptr;

    return failure(
      StorageCode::kConfigurationFailed,
      failure_code,
      "foreign-key enforcement is not enabled");
  }

  configuration_.foreign_keys_enabled = true;

  return success("database opened");
}


StorageResult SqliteStore::close()
{
  std::lock_guard<std::mutex> lock{mutex_};

  if (database_ == nullptr) {
    transaction_active_ = false;
    transaction_owner_ = std::thread::id{};
    return success("database already closed");
  }

  if (transaction_active_) {
    const auto rollback_result =
      rollback_locked();

    if (!rollback_result.success) {
      return rollback_result;
    }
  }

  const int code =
    sqlite3_close_v2(database_);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not close database"));
  }

  database_ = nullptr;
  configuration_ = StorageConfiguration{};

  return success("database closed");
}


bool SqliteStore::is_open() const noexcept
{
  std::lock_guard<std::mutex> lock{mutex_};
  return database_ != nullptr;
}


const std::string &
SqliteStore::database_path() const noexcept
{
  return database_path_;
}


StorageConfiguration
SqliteStore::configuration() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return configuration_;
}


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
      "'001_initial_schema'," +
      std::to_string(unix_time_ns()) +
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
      "'002_append_only_event_journal'," +
      std::to_string(unix_time_ns()) +
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

  if (working_version == 2U) {
    const auto schema_result =
      execute_locked(kMigration003Sql);

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
      "3,"
      "'003_location_release_authority'," +
      std::to_string(unix_time_ns()) +
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
      execute_locked("PRAGMA user_version=3;");

    if (!version_update.success) {
      rollback_migration();

      return failure(
        StorageCode::kMigrationFailed,
        version_update.sqlite_code,
        version_update.reason);
    }

    working_version = 3U;
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


StorageResult SqliteStore::schema_version(
  std::uint32_t * version) const
{
  if (version == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "schema version output is required");
  }

  std::lock_guard<std::mutex> lock{mutex_};

  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  int raw_version = 0;

  const auto result =
    query_single_int_locked(
      "PRAGMA user_version;",
      &raw_version);

  if (!result.success) {
    return result;
  }

  if (raw_version < 0) {
    return failure(
      StorageCode::kSqlError,
      SQLITE_CORRUPT,
      "schema version is negative");
  }

  *version =
    static_cast<std::uint32_t>(
    raw_version);

  return success("schema version read");
}


StorageResult SqliteStore::integrity_check(
  IntegrityReport * report) const
{
  if (report == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "integrity report output is required");
  }

  std::lock_guard<std::mutex> lock{mutex_};

  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  report->healthy = false;
  report->integrity_messages.clear();
  report->foreign_key_violations.clear();

  sqlite3_stmt * statement = nullptr;

  int code = sqlite3_prepare_v2(
    database_,
    "PRAGMA integrity_check;",
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kIntegrityFailed,
      code,
      sqlite_error(
        database_,
        "could not prepare integrity check"));
  }

  while (
    (code = sqlite3_step(statement)) ==
    SQLITE_ROW)
  {
    const auto * text =
      sqlite3_column_text(statement, 0);

    report->integrity_messages.emplace_back(
      text == nullptr ?
        "" :
        reinterpret_cast<const char *>(text));
  }

  const int finalize_code =
    sqlite3_finalize(statement);

  if (
    code != SQLITE_DONE ||
    finalize_code != SQLITE_OK)
  {
    return failure(
      StorageCode::kIntegrityFailed,
      code != SQLITE_DONE ?
        code :
        finalize_code,
      sqlite_error(
        database_,
        "integrity check failed"));
  }

  statement = nullptr;

  code = sqlite3_prepare_v2(
    database_,
    "PRAGMA foreign_key_check;",
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kIntegrityFailed,
      code,
      sqlite_error(
        database_,
        "could not prepare foreign-key check"));
  }

  while (
    (code = sqlite3_step(statement)) ==
    SQLITE_ROW)
  {
    const auto * table =
      sqlite3_column_text(statement, 0);

    const sqlite3_int64 row_id =
      sqlite3_column_int64(statement, 1);

    const auto * parent =
      sqlite3_column_text(statement, 2);

    std::string message{
      table == nullptr ?
      "unknown_table" :
      reinterpret_cast<const char *>(table)};

    message += ":rowid=";
    message += std::to_string(row_id);
    message += ":parent=";

    message +=
      parent == nullptr ?
      "unknown_parent" :
      reinterpret_cast<const char *>(parent);

    report->foreign_key_violations.push_back(
      std::move(message));
  }

  const int foreign_finalize_code =
    sqlite3_finalize(statement);

  if (
    code != SQLITE_DONE ||
    foreign_finalize_code != SQLITE_OK)
  {
    return failure(
      StorageCode::kIntegrityFailed,
      code != SQLITE_DONE ?
        code :
        foreign_finalize_code,
      sqlite_error(
        database_,
        "foreign-key check failed"));
  }

  report->healthy =
    report->integrity_messages.size() == 1U &&
    report->integrity_messages.front() == "ok" &&
    report->foreign_key_violations.empty();

  if (!report->healthy) {
    return failure(
      StorageCode::kIntegrityFailed,
      SQLITE_CORRUPT,
      "database integrity validation failed");
  }

  return success("database integrity is healthy");
}


StorageResult SqliteStore::begin_immediate()
{
  std::lock_guard<std::mutex> lock{mutex_};

  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  if (transaction_active_) {
    return failure(
      StorageCode::kTransactionAlreadyActive,
      SQLITE_BUSY,
      "a transaction is already active");
  }

  const auto result =
    execute_locked("BEGIN IMMEDIATE;");

  if (!result.success) {
    return result;
  }

  transaction_active_ = true;

  transaction_owner_ =
    std::this_thread::get_id();

  return success("transaction started");
}


StorageResult SqliteStore::commit()
{
  std::lock_guard<std::mutex> lock{mutex_};

  const auto owner_result =
    check_transaction_owner_locked();

  if (!owner_result.success) {
    return owner_result;
  }

  const auto result =
    execute_locked("COMMIT;");

  if (!result.success) {
    return result;
  }

  transaction_active_ = false;
  transaction_owner_ = std::thread::id{};

  return success("transaction committed");
}


StorageResult SqliteStore::rollback()
{
  std::lock_guard<std::mutex> lock{mutex_};

  const auto owner_result =
    check_transaction_owner_locked();

  if (!owner_result.success) {
    return owner_result;
  }

  return rollback_locked();
}


bool SqliteStore::in_transaction() const noexcept
{
  std::lock_guard<std::mutex> lock{mutex_};
  return transaction_active_;
}


StorageResult SqliteStore::set_metadata(
  const std::string_view key,
  const std::string_view value)
{
  if (key.empty()) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "metadata key is empty");
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

  sqlite3_stmt * statement = nullptr;

  int code = sqlite3_prepare_v2(
    database_,
    "INSERT INTO registry_metadata("
    "metadata_key,"
    "metadata_value,"
    "updated_at_unix_ns"
    ") VALUES(?1,?2,?3) "
    "ON CONFLICT(metadata_key) DO UPDATE SET "
    "metadata_value=excluded.metadata_value,"
    "updated_at_unix_ns=excluded.updated_at_unix_ns;",
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not prepare metadata write"));
  }

  code = sqlite3_bind_text(
    statement,
    1,
    key.data(),
    static_cast<int>(key.size()),
    SQLITE_TRANSIENT);

  if (code == SQLITE_OK) {
    code = sqlite3_bind_text(
      statement,
      2,
      value.data(),
      static_cast<int>(value.size()),
      SQLITE_TRANSIENT);
  }

  if (code == SQLITE_OK) {
    code = sqlite3_bind_int64(
      statement,
      3,
      unix_time_ns());
  }

  if (code == SQLITE_OK) {
    code = sqlite3_step(statement);
  }

  const int finalize_code =
    sqlite3_finalize(statement);

  if (
    code != SQLITE_DONE ||
    finalize_code != SQLITE_OK)
  {
    return failure(
      StorageCode::kSqlError,
      code != SQLITE_DONE ?
        code :
        finalize_code,
      sqlite_error(
        database_,
        "metadata write failed"));
  }

  return success("metadata written");
}


StorageResult SqliteStore::get_metadata(
  const std::string_view key,
  std::optional<std::string> * value) const
{
  if (key.empty() || value == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "metadata key and output are required");
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

  sqlite3_stmt * statement = nullptr;

  int code = sqlite3_prepare_v2(
    database_,
    "SELECT metadata_value "
    "FROM registry_metadata "
    "WHERE metadata_key=?1;",
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not prepare metadata read"));
  }

  code = sqlite3_bind_text(
    statement,
    1,
    key.data(),
    static_cast<int>(key.size()),
    SQLITE_TRANSIENT);

  if (code != SQLITE_OK) {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not bind metadata key"));
  }

  code = sqlite3_step(statement);

  if (code == SQLITE_ROW) {
    const auto * text =
      sqlite3_column_text(statement, 0);

    *value =
      text == nullptr ?
      std::string{} :
    std::string{
      reinterpret_cast<
        const char *>(text)};

    const int finalize_code =
      sqlite3_finalize(statement);

    if (finalize_code != SQLITE_OK) {
      return failure(
        StorageCode::kSqlError,
        finalize_code,
        sqlite_error(
          database_,
          "metadata read finalization failed"));
    }

    return success("metadata found");
  }

  if (code == SQLITE_DONE) {
    value->reset();

    const int finalize_code =
      sqlite3_finalize(statement);

    if (finalize_code != SQLITE_OK) {
      return failure(
        StorageCode::kSqlError,
        finalize_code,
        sqlite_error(
          database_,
          "metadata read finalization failed"));
    }

    return success("metadata not found");
  }

  sqlite3_finalize(statement);

  return failure(
    StorageCode::kSqlError,
    code,
    sqlite_error(
      database_,
      "metadata read failed"));
}


StorageResult SqliteStore::table_exists(
  const std::string_view table_name,
  bool * exists) const
{
  if (
    table_name.empty() ||
    exists == nullptr)
  {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "table name and output are required");
  }

  std::lock_guard<std::mutex> lock{mutex_};

  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  sqlite3_stmt * statement = nullptr;

  int code = sqlite3_prepare_v2(
    database_,
    "SELECT 1 "
    "FROM sqlite_master "
    "WHERE type='table' AND name=?1 "
    "LIMIT 1;",
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not prepare table query"));
  }

  code = sqlite3_bind_text(
    statement,
    1,
    table_name.data(),
    static_cast<int>(table_name.size()),
    SQLITE_TRANSIENT);

  if (code != SQLITE_OK) {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not bind table name"));
  }

  code = sqlite3_step(statement);

  if (code == SQLITE_ROW) {
    *exists = true;
  } else if (code == SQLITE_DONE) {
    *exists = false;
  } else {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "table query failed"));
  }

  const int finalize_code =
    sqlite3_finalize(statement);

  if (finalize_code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      finalize_code,
      sqlite_error(
        database_,
        "table query finalization failed"));
  }

  return success("table query completed");
}


StorageResult SqliteStore::execute_locked(
  const std::string_view sql) const
{
  const auto open_result =
    check_open_locked();

  if (!open_result.success) {
    return open_result;
  }

  char * error_message = nullptr;

  const std::string sql_string{sql};

  const int code = sqlite3_exec(
    database_,
    sql_string.c_str(),
    nullptr,
    nullptr,
    &error_message);

  if (code != SQLITE_OK) {
    std::string reason{
      error_message == nullptr ?
      sqlite3_errmsg(database_) :
      error_message};

    sqlite3_free(error_message);

    return failure(
      StorageCode::kSqlError,
      code,
      std::move(reason));
  }

  return success();
}


StorageResult
SqliteStore::query_single_int_locked(
  const std::string_view sql,
  int * value) const
{
  if (value == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "integer query output is required");
  }

  sqlite3_stmt * statement = nullptr;
  const std::string sql_string{sql};

  int code = sqlite3_prepare_v2(
    database_,
    sql_string.c_str(),
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not prepare integer query"));
  }

  code = sqlite3_step(statement);

  if (code != SQLITE_ROW) {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "integer query returned no row"));
  }

  const sqlite3_int64 raw =
    sqlite3_column_int64(statement, 0);

  if (
    raw < std::numeric_limits<int>::min() ||
    raw > std::numeric_limits<int>::max())
  {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      SQLITE_RANGE,
      "integer query result is out of range");
  }

  *value = static_cast<int>(raw);

  code = sqlite3_step(statement);

  const int finalize_code =
    sqlite3_finalize(statement);

  if (
    code != SQLITE_DONE ||
    finalize_code != SQLITE_OK)
  {
    return failure(
      StorageCode::kSqlError,
      code != SQLITE_DONE ?
        code :
        finalize_code,
      sqlite_error(
        database_,
        "integer query did not finish cleanly"));
  }

  return success();
}


StorageResult
SqliteStore::query_single_text_locked(
  const std::string_view sql,
  std::string * value) const
{
  if (value == nullptr) {
    return failure(
      StorageCode::kInvalidArgument,
      SQLITE_MISUSE,
      "text query output is required");
  }

  sqlite3_stmt * statement = nullptr;
  const std::string sql_string{sql};

  int code = sqlite3_prepare_v2(
    database_,
    sql_string.c_str(),
    -1,
    &statement,
    nullptr);

  if (code != SQLITE_OK) {
    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "could not prepare text query"));
  }

  code = sqlite3_step(statement);

  if (code != SQLITE_ROW) {
    sqlite3_finalize(statement);

    return failure(
      StorageCode::kSqlError,
      code,
      sqlite_error(
        database_,
        "text query returned no row"));
  }

  const auto * text =
    sqlite3_column_text(statement, 0);

  *value =
    text == nullptr ?
    std::string{} :
  std::string{
    reinterpret_cast<
      const char *>(text)};

  code = sqlite3_step(statement);

  const int finalize_code =
    sqlite3_finalize(statement);

  if (
    code != SQLITE_DONE ||
    finalize_code != SQLITE_OK)
  {
    return failure(
      StorageCode::kSqlError,
      code != SQLITE_DONE ?
        code :
        finalize_code,
      sqlite_error(
        database_,
        "text query did not finish cleanly"));
  }

  return success();
}


StorageResult
SqliteStore::check_open_locked() const
{
  if (database_ == nullptr) {
    return failure(
      StorageCode::kNotOpen,
      SQLITE_MISUSE,
      "database is not open");
  }

  return success();
}


StorageResult
SqliteStore::check_transaction_owner_locked()
const
{
  if (!transaction_active_) {
    return success();
  }

  if (
    transaction_owner_ !=
    std::this_thread::get_id())
  {
    return failure(
      StorageCode::
      kTransactionOwnerMismatch,
      SQLITE_BUSY,
      "active transaction belongs to another thread");
  }

  return success();
}


StorageResult SqliteStore::rollback_locked()
{
  if (!transaction_active_) {
    return failure(
      StorageCode::kTransactionNotActive,
      SQLITE_MISUSE,
      "no transaction is active");
  }

  const auto result =
    execute_locked("ROLLBACK;");

  if (!result.success) {
    return result;
  }

  transaction_active_ = false;
  transaction_owner_ = std::thread::id{};

  return success("transaction rolled back");
}

}  // namespace savo_locations
