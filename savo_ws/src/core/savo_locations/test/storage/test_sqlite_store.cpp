#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "savo_locations/sqlite_schema.hpp"
#include "savo_locations/sqlite_store.hpp"


namespace
{

std::filesystem::path database_root()
{
  return std::filesystem::path{
    SAVO_LOCATIONS_TEST_DB_DIR};
}


std::filesystem::path clean_database_path(
  const std::string & name)
{
  const auto root = database_root();

  std::filesystem::create_directories(root);

  const auto database =
    root / name;

  std::filesystem::remove(database);
  std::filesystem::remove(
    database.string() + "-wal");

  std::filesystem::remove(
    database.string() + "-shm");

  return database;
}


void set_raw_user_version(
  const std::filesystem::path & path,
  const int version)
{
  sqlite3 * database = nullptr;

  ASSERT_EQ(
    sqlite3_open_v2(
      path.string().c_str(),
      &database,
      SQLITE_OPEN_READWRITE |
      SQLITE_OPEN_CREATE,
      nullptr),
    SQLITE_OK);

  const std::string sql =
    "PRAGMA user_version=" +
    std::to_string(version) +
    ";";

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

}  // namespace


TEST(SqliteStore, RejectsEmptyPath)
{
  savo_locations::SqliteStore store{""};

  const auto result = store.open();

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::StorageCode::
      kInvalidArgument);
}


TEST(SqliteStore, OpensFileDatabaseWithSafePolicy)
{
  const auto path =
    clean_database_path(
      "safe_policy.sqlite3");

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  const auto configuration =
    store.configuration();

  EXPECT_TRUE(
    configuration.foreign_keys_enabled);

  EXPECT_EQ(
    configuration.journal_mode,
    "wal");

  EXPECT_EQ(
    configuration.busy_timeout_ms,
    5000);

  EXPECT_TRUE(store.close().success);
}


TEST(SqliteStore, AppliesMigration001)
{
  const auto path =
    clean_database_path(
      "migration.sqlite3");

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  const auto result =
    store.migrate(&status);

  ASSERT_TRUE(result.success);

  EXPECT_EQ(status.previous_version, 0U);

  EXPECT_EQ(
    status.current_version,
    savo_locations::
      kSupportedSqliteSchemaVersion);

  EXPECT_TRUE(status.migration_applied);

  std::uint32_t version = 0U;

  ASSERT_TRUE(
    store.schema_version(&version).success);

  EXPECT_EQ(version, 1U);
}


TEST(SqliteStore, MigrationCreatesRequiredTables)
{
  const auto path =
    clean_database_path(
      "tables.sqlite3");

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  const std::vector<std::string> tables{
    "schema_migrations",
    "registry_metadata",
    "locations",
    "location_aliases",
    "location_candidates",
    "candidate_aliases",
    "location_events",
  };

  for (const auto & table : tables) {
    bool exists = false;

    ASSERT_TRUE(
      store.table_exists(
        table,
        &exists).success);

    EXPECT_TRUE(exists) << table;
  }
}


TEST(SqliteStore, MigrationIsIdempotent)
{
  const auto path =
    clean_database_path(
      "idempotent.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    ASSERT_TRUE(store.open().success);

    savo_locations::SchemaStatus status;

    ASSERT_TRUE(
      store.migrate(&status).success);

    EXPECT_TRUE(status.migration_applied);
  }

  {
    savo_locations::SqliteStore store{
      path.string()};

    ASSERT_TRUE(store.open().success);

    savo_locations::SchemaStatus status;

    ASSERT_TRUE(
      store.migrate(&status).success);

    EXPECT_FALSE(status.migration_applied);
    EXPECT_EQ(status.previous_version, 1U);
    EXPECT_EQ(status.current_version, 1U);
  }
}


TEST(SqliteStore, RejectsNewerSchema)
{
  const auto path =
    clean_database_path(
      "newer_schema.sqlite3");

  set_raw_user_version(path, 99);

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  const auto result =
    store.migrate(&status);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::StorageCode::
      kSchemaTooNew);

  EXPECT_EQ(status.previous_version, 99U);
}


TEST(SqliteStore, IntegrityCheckPasses)
{
  const auto path =
    clean_database_path(
      "integrity.sqlite3");

  savo_locations::SqliteStore store{
    path.string()};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  savo_locations::IntegrityReport report;

  const auto result =
    store.integrity_check(&report);

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(report.healthy);

  ASSERT_EQ(
    report.integrity_messages.size(),
    1U);

  EXPECT_EQ(
    report.integrity_messages.front(),
    "ok");

  EXPECT_TRUE(
    report.foreign_key_violations.empty());
}


TEST(SqliteStore, TransactionRollbackIsAtomic)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  ASSERT_TRUE(
    store.begin_immediate().success);

  EXPECT_TRUE(store.in_transaction());

  ASSERT_TRUE(
    store.set_metadata(
      "active_map_id",
      "campus_main").success);

  ASSERT_TRUE(store.rollback().success);

  EXPECT_FALSE(store.in_transaction());

  std::optional<std::string> value;

  ASSERT_TRUE(
    store.get_metadata(
      "active_map_id",
      &value).success);

  EXPECT_FALSE(value.has_value());
}


TEST(SqliteStore, TransactionCommitPersists)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  ASSERT_TRUE(
    store.begin_immediate().success);

  ASSERT_TRUE(
    store.set_metadata(
      "active_map_id",
      "campus_main").success);

  ASSERT_TRUE(store.commit().success);

  std::optional<std::string> value;

  ASSERT_TRUE(
    store.get_metadata(
      "active_map_id",
      &value).success);

  ASSERT_TRUE(value.has_value());

  EXPECT_EQ(
    value.value(),
    "campus_main");
}


TEST(SqliteStore, RejectsNestedTransaction)
{
  savo_locations::SqliteStore store{
    ":memory:"};

  ASSERT_TRUE(store.open().success);

  savo_locations::SchemaStatus status;

  ASSERT_TRUE(
    store.migrate(&status).success);

  ASSERT_TRUE(
    store.begin_immediate().success);

  const auto nested =
    store.begin_immediate();

  EXPECT_FALSE(nested.success);

  EXPECT_EQ(
    nested.code,
    savo_locations::StorageCode::
      kTransactionAlreadyActive);

  EXPECT_TRUE(store.rollback().success);
}


TEST(SqliteStore, CloseRollsBackActiveTransaction)
{
  const auto path =
    clean_database_path(
      "close_rollback.sqlite3");

  {
    savo_locations::SqliteStore store{
      path.string()};

    ASSERT_TRUE(store.open().success);

    savo_locations::SchemaStatus status;

    ASSERT_TRUE(
      store.migrate(&status).success);

    ASSERT_TRUE(
      store.begin_immediate().success);

    ASSERT_TRUE(
      store.set_metadata(
        "uncommitted",
        "value").success);

    ASSERT_TRUE(store.close().success);
  }

  {
    savo_locations::SqliteStore store{
      path.string()};

    ASSERT_TRUE(store.open().success);

    savo_locations::SchemaStatus status;

    ASSERT_TRUE(
      store.migrate(&status).success);

    std::optional<std::string> value;

    ASSERT_TRUE(
      store.get_metadata(
        "uncommitted",
        &value).success);

    EXPECT_FALSE(value.has_value());
  }
}


TEST(SqliteStore, ReasonStringsAreStable)
{
  using savo_locations::StorageCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(StorageCode::kSchemaTooNew),
    "schema_too_new");

  EXPECT_EQ(
    to_string(
      StorageCode::
        kTransactionAlreadyActive),
    "transaction_already_active");

  EXPECT_EQ(
    to_string(
      StorageCode::
        kIntegrityFailed),
    "integrity_failed");
}
