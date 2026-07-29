#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
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
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def update_package_xml() -> None:
    package_xml = ROOT / "package.xml"

    tree = ET.parse(package_xml)
    package = tree.getroot()

    version = package.find("version")

    if version is None:
        raise RuntimeError(
            "package.xml has no version element"
        )

    version.text = "0.5.0"

    dependencies = {
        (element.text or "").strip()
        for element in package.findall("depend")
    }

    if "sqlite3" not in dependencies:
        insertion_index = len(package)

        for index, element in enumerate(package):
            if element.tag == "test_depend":
                insertion_index = index
                break

        dependency = ET.Element("depend")
        dependency.text = "sqlite3"

        package.insert(
            insertion_index,
            dependency,
        )

    ET.indent(tree, space="  ")

    tree.write(
        package_xml,
        encoding="utf-8",
        xml_declaration=True,
    )


def main() -> None:
    if not (ROOT / "package.xml").is_file():
        raise SystemExit(
            f"savo_locations package not found: {ROOT}"
        )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC2A_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    update_package_xml()

    # -------------------------------------------------------------------------
    # CMake
    # -------------------------------------------------------------------------

    write(
        "CMakeLists.txt",
        r'''
        cmake_minimum_required(VERSION 3.16)
        project(savo_locations VERSION 0.5.0 LANGUAGES CXX)

        # ---------------------------------------------------------------------------
        # Compiler policy
        # ---------------------------------------------------------------------------

        if(NOT CMAKE_CXX_STANDARD)
          set(CMAKE_CXX_STANDARD 17)
        endif()

        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)

        # ---------------------------------------------------------------------------
        # Dependencies
        # ---------------------------------------------------------------------------

        find_package(ament_cmake REQUIRED)
        find_package(savo_msgs REQUIRED)
        find_package(SQLite3 REQUIRED)

        # ---------------------------------------------------------------------------
        # Deterministic location-domain library
        # ---------------------------------------------------------------------------

        add_library(
          savo_locations_contracts
          src/types.cpp
          src/normalization.cpp
          src/validation.cpp
          src/registry.cpp
          src/location_catalog.cpp
        )

        add_library(
          savo_locations::contracts
          ALIAS savo_locations_contracts
        )

        target_include_directories(
          savo_locations_contracts
          PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )

        target_compile_features(
          savo_locations_contracts
          PUBLIC
            cxx_std_17
        )

        # ---------------------------------------------------------------------------
        # SQLite storage foundation
        # ---------------------------------------------------------------------------

        add_library(
          savo_locations_storage
          src/sqlite_store.cpp
        )

        add_library(
          savo_locations::storage
          ALIAS savo_locations_storage
        )

        target_include_directories(
          savo_locations_storage
          PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )

        target_compile_features(
          savo_locations_storage
          PUBLIC
            cxx_std_17
        )

        target_link_libraries(
          savo_locations_storage
          PUBLIC
            savo_locations_contracts
            SQLite::SQLite3
        )

        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
          foreach(target
              savo_locations_contracts
              savo_locations_storage)
            target_compile_options(
              ${target}
              PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
            )
          endforeach()
        endif()

        # ---------------------------------------------------------------------------
        # Installation
        # ---------------------------------------------------------------------------

        install(
          TARGETS
            savo_locations_contracts
            savo_locations_storage
          EXPORT
            export_savo_locations
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin
        )

        install(
          DIRECTORY
            include/
          DESTINATION
            include
        )

        install(
          DIRECTORY
            config/
          DESTINATION
            share/${PROJECT_NAME}/config
        )

        install(
          FILES
            README.md
            LICENSE
          DESTINATION
            share/${PROJECT_NAME}
        )

        # ---------------------------------------------------------------------------
        # Tests
        # ---------------------------------------------------------------------------

        if(BUILD_TESTING)
          find_package(ament_cmake_gtest REQUIRED)
          find_package(ament_cmake_pytest REQUIRED)

          ament_add_gtest(
            test_location_types
            test/unit/test_types.cpp
          )

          if(TARGET test_location_types)
            target_link_libraries(
              test_location_types
              savo_locations_contracts
            )
          endif()

          ament_add_gtest(
            test_location_normalization
            test/unit/test_normalization.cpp
          )

          if(TARGET test_location_normalization)
            target_link_libraries(
              test_location_normalization
              savo_locations_contracts
            )
          endif()

          ament_add_gtest(
            test_location_validation
            test/unit/test_validation.cpp
          )

          if(TARGET test_location_validation)
            target_link_libraries(
              test_location_validation
              savo_locations_contracts
            )
          endif()

          ament_add_gtest(
            test_location_registry
            test/unit/test_registry.cpp
          )

          if(TARGET test_location_registry)
            target_link_libraries(
              test_location_registry
              savo_locations_contracts
            )
          endif()

          ament_add_gtest(
            test_location_catalog
            test/unit/test_location_catalog.cpp
          )

          if(TARGET test_location_catalog)
            target_link_libraries(
              test_location_catalog
              savo_locations_contracts
            )
          endif()

          ament_add_gtest(
            test_sqlite_store
            test/storage/test_sqlite_store.cpp
          )

          if(TARGET test_sqlite_store)
            target_link_libraries(
              test_sqlite_store
              savo_locations_storage
              SQLite::SQLite3
            )

            target_compile_definitions(
              test_sqlite_store
              PRIVATE
                SAVO_LOCATIONS_TEST_DB_DIR=
                "${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime"
            )
          endif()

          ament_add_pytest_test(
            test_phase0_contracts
            test/contracts/test_phase0_contracts.py
            TIMEOUT 60
          )

          ament_add_pytest_test(
            test_phase1a_contracts
            test/contracts/test_phase1a_contracts.py
            TIMEOUT 60
          )

          ament_add_pytest_test(
            test_phase1b_contracts
            test/contracts/test_phase1b_contracts.py
            TIMEOUT 60
          )

          ament_add_pytest_test(
            test_phase1c_contracts
            test/contracts/test_phase1c_contracts.py
            TIMEOUT 60
          )

          ament_add_pytest_test(
            test_phase2a_contracts
            test/contracts/test_phase2a_contracts.py
            TIMEOUT 60
          )
        endif()

        # ---------------------------------------------------------------------------
        # Package exports
        # ---------------------------------------------------------------------------

        ament_export_targets(
          export_savo_locations
          HAS_LIBRARY_TARGET
        )

        ament_export_include_directories(include)

        ament_export_dependencies(
          savo_msgs
          SQLite3
        )

        ament_package()
        ''',
    )

    # -------------------------------------------------------------------------
    # Package constants
    # -------------------------------------------------------------------------

    constants_path = (
        ROOT
        / "include"
        / "savo_locations"
        / "constants.hpp"
    )

    constants = constants_path.read_text(
        encoding="utf-8"
    )

    constants = constants.replace(
        '"0.4.0"',
        '"0.5.0"',
    )

    constants_path.write_text(
        constants,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Compiled migration schema
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/sqlite_schema.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_
        #define SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_

        #include <cstdint>
        #include <string_view>

        namespace savo_locations
        {

        inline constexpr std::uint32_t
          kSupportedSqliteSchemaVersion{1U};

        inline constexpr std::string_view
          kMigration001Sql{R"SQL(
        CREATE TABLE IF NOT EXISTS schema_migrations (
          schema_version INTEGER PRIMARY KEY
            CHECK(schema_version > 0),
          migration_name TEXT NOT NULL,
          applied_at_unix_ns INTEGER NOT NULL
            CHECK(applied_at_unix_ns > 0)
        );

        CREATE TABLE IF NOT EXISTS registry_metadata (
          metadata_key TEXT PRIMARY KEY,
          metadata_value TEXT NOT NULL,
          updated_at_unix_ns INTEGER NOT NULL
            CHECK(updated_at_unix_ns > 0)
        );

        CREATE TABLE IF NOT EXISTS locations (
          location_id TEXT PRIMARY KEY,

          state INTEGER NOT NULL
            CHECK(state IN (1, 2)),

          enabled INTEGER NOT NULL
            CHECK(enabled IN (0, 1)),

          record_revision INTEGER NOT NULL
            CHECK(record_revision > 0),

          display_name TEXT NOT NULL,
          semantic_type TEXT NOT NULL,

          map_id TEXT NOT NULL,

          map_revision INTEGER NOT NULL
            CHECK(map_revision > 0),

          map_release_id TEXT NOT NULL DEFAULT '',

          approach_frame_id TEXT NOT NULL
            CHECK(approach_frame_id = 'map'),

          approach_x REAL NOT NULL,
          approach_y REAL NOT NULL,
          approach_z REAL NOT NULL,

          approach_qx REAL NOT NULL,
          approach_qy REAL NOT NULL,
          approach_qz REAL NOT NULL,
          approach_qw REAL NOT NULL,

          confirmation_pose_valid INTEGER NOT NULL
            CHECK(confirmation_pose_valid IN (0, 1)),

          confirmation_frame_id TEXT,
          confirmation_x REAL,
          confirmation_y REAL,
          confirmation_z REAL,
          confirmation_qx REAL,
          confirmation_qy REAL,
          confirmation_qz REAL,
          confirmation_qw REAL,

          tag_family TEXT NOT NULL,

          tag_id INTEGER NOT NULL
            CHECK(tag_id >= 0),

          tag_pose_map_valid INTEGER NOT NULL
            CHECK(tag_pose_map_valid IN (0, 1)),

          tag_frame_id TEXT,
          tag_x REAL,
          tag_y REAL,
          tag_z REAL,
          tag_qx REAL,
          tag_qy REAL,
          tag_qz REAL,
          tag_qw REAL,

          arrival_confirmation_required INTEGER NOT NULL
            CHECK(arrival_confirmation_required IN (0, 1)),

          building TEXT NOT NULL DEFAULT '',
          floor TEXT NOT NULL DEFAULT '',
          area TEXT NOT NULL DEFAULT '',
          notes TEXT NOT NULL DEFAULT '',

          source_candidate_id TEXT NOT NULL DEFAULT '',

          created_at_unix_ns INTEGER NOT NULL
            CHECK(created_at_unix_ns > 0),

          updated_at_unix_ns INTEGER NOT NULL
            CHECK(updated_at_unix_ns > 0)
        );

        CREATE TABLE IF NOT EXISTS location_aliases (
          location_id TEXT NOT NULL,

          alias_order INTEGER NOT NULL
            CHECK(alias_order >= 0),

          alias_kind INTEGER NOT NULL
            CHECK(alias_kind IN (1, 2, 3)),

          alias_text TEXT NOT NULL,
          normalized_key TEXT NOT NULL,

          map_id TEXT NOT NULL,

          map_revision INTEGER NOT NULL
            CHECK(map_revision > 0),

          reserves_identity INTEGER NOT NULL
            CHECK(reserves_identity IN (0, 1)),

          PRIMARY KEY(location_id, alias_order),

          UNIQUE(location_id, normalized_key),

          FOREIGN KEY(location_id)
            REFERENCES locations(location_id)
            ON UPDATE CASCADE
            ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS location_candidates (
          candidate_id TEXT PRIMARY KEY,

          state INTEGER NOT NULL
            CHECK(state IN (1, 2, 3)),

          candidate_revision INTEGER NOT NULL
            CHECK(candidate_revision > 0),

          map_id TEXT NOT NULL,

          map_revision INTEGER NOT NULL
            CHECK(map_revision > 0),

          map_release_id TEXT NOT NULL DEFAULT '',

          tag_family TEXT NOT NULL,

          tag_id INTEGER NOT NULL
            CHECK(tag_id >= 0),

          tag_frame_id TEXT NOT NULL
            CHECK(tag_frame_id = 'map'),

          tag_x REAL NOT NULL,
          tag_y REAL NOT NULL,
          tag_z REAL NOT NULL,

          tag_qx REAL NOT NULL,
          tag_qy REAL NOT NULL,
          tag_qz REAL NOT NULL,
          tag_qw REAL NOT NULL,

          detection_quality REAL NOT NULL
            CHECK(
              detection_quality >= 0.0 AND
              detection_quality <= 1.0
            ),

          accepted_observations INTEGER NOT NULL
            CHECK(accepted_observations > 0),

          position_stddev_m REAL NOT NULL
            CHECK(position_stddev_m >= 0.0),

          yaw_stddev_rad REAL NOT NULL
            CHECK(yaw_stddev_rad >= 0.0),

          approach_pose_valid INTEGER NOT NULL
            CHECK(approach_pose_valid IN (0, 1)),

          approach_frame_id TEXT,
          approach_x REAL,
          approach_y REAL,
          approach_z REAL,
          approach_qx REAL,
          approach_qy REAL,
          approach_qz REAL,
          approach_qw REAL,

          confirmation_pose_valid INTEGER NOT NULL
            CHECK(confirmation_pose_valid IN (0, 1)),

          confirmation_frame_id TEXT,
          confirmation_x REAL,
          confirmation_y REAL,
          confirmation_z REAL,
          confirmation_qx REAL,
          confirmation_qy REAL,
          confirmation_qz REAL,
          confirmation_qw REAL,

          suggested_location_id TEXT NOT NULL DEFAULT '',
          suggested_display_name TEXT NOT NULL DEFAULT '',
          suggested_semantic_type TEXT NOT NULL DEFAULT '',

          building TEXT NOT NULL DEFAULT '',
          floor TEXT NOT NULL DEFAULT '',
          area TEXT NOT NULL DEFAULT '',
          notes TEXT NOT NULL DEFAULT '',

          source_session_id TEXT NOT NULL DEFAULT '',
          source_component TEXT NOT NULL DEFAULT '',

          review_reason TEXT NOT NULL DEFAULT '',
          approved_location_id TEXT NOT NULL DEFAULT '',

          created_at_unix_ns INTEGER NOT NULL
            CHECK(created_at_unix_ns > 0),

          updated_at_unix_ns INTEGER NOT NULL
            CHECK(updated_at_unix_ns > 0)
        );

        CREATE TABLE IF NOT EXISTS candidate_aliases (
          candidate_id TEXT NOT NULL,

          alias_order INTEGER NOT NULL
            CHECK(alias_order >= 0),

          alias_text TEXT NOT NULL,
          normalized_key TEXT NOT NULL,

          PRIMARY KEY(candidate_id, alias_order),

          UNIQUE(candidate_id, normalized_key),

          FOREIGN KEY(candidate_id)
            REFERENCES location_candidates(candidate_id)
            ON UPDATE CASCADE
            ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS location_events (
          event_sequence INTEGER PRIMARY KEY AUTOINCREMENT,

          event_time_unix_ns INTEGER NOT NULL
            CHECK(event_time_unix_ns > 0),

          event_type INTEGER NOT NULL
            CHECK(event_type >= 0),

          candidate_id TEXT NOT NULL DEFAULT '',
          location_id TEXT NOT NULL DEFAULT '',

          entity_revision INTEGER NOT NULL
            CHECK(entity_revision >= 0),

          actor_id TEXT NOT NULL DEFAULT '',
          reason TEXT NOT NULL DEFAULT '',

          event_payload_json TEXT NOT NULL DEFAULT '{}'
        );

        CREATE UNIQUE INDEX IF NOT EXISTS
          idx_locations_active_tag_unique
        ON locations(
          map_id,
          map_revision,
          tag_family,
          tag_id
        )
        WHERE state != 2;

        CREATE UNIQUE INDEX IF NOT EXISTS
          idx_location_aliases_active_identity_unique
        ON location_aliases(
          map_id,
          map_revision,
          normalized_key
        )
        WHERE reserves_identity = 1;

        CREATE UNIQUE INDEX IF NOT EXISTS
          idx_candidates_pending_tag_unique
        ON location_candidates(
          map_id,
          map_revision,
          tag_family,
          tag_id
        )
        WHERE state = 1;

        CREATE INDEX IF NOT EXISTS
          idx_locations_map_context
        ON locations(
          map_id,
          map_revision,
          state,
          enabled
        );

        CREATE INDEX IF NOT EXISTS
          idx_candidates_state
        ON location_candidates(
          state,
          map_id,
          map_revision
        );

        CREATE INDEX IF NOT EXISTS
          idx_location_events_location
        ON location_events(
          location_id,
          event_sequence
        );

        CREATE INDEX IF NOT EXISTS
          idx_location_events_candidate
        ON location_events(
          candidate_id,
          event_sequence
        );
        )SQL"};

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # SQLite store API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/sqlite_store.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__SQLITE_STORE_HPP_
        #define SAVO_LOCATIONS__SQLITE_STORE_HPP_

        #include <cstdint>
        #include <mutex>
        #include <optional>
        #include <string>
        #include <string_view>
        #include <thread>
        #include <vector>

        struct sqlite3;

        namespace savo_locations
        {

        enum class StorageCode : std::uint8_t
        {
          kOk = 0U,
          kInvalidArgument,
          kNotOpen,
          kOpenFailed,
          kConfigurationFailed,
          kSqlError,
          kMigrationFailed,
          kSchemaTooNew,
          kIntegrityFailed,
          kTransactionAlreadyActive,
          kTransactionNotActive,
          kTransactionOwnerMismatch,
        };


        struct StorageResult
        {
          bool success{false};
          StorageCode code{StorageCode::kSqlError};
          int sqlite_code{0};
          std::string reason;
        };


        struct StorageConfiguration
        {
          bool foreign_keys_enabled{false};
          std::string journal_mode;
          int busy_timeout_ms{0};
        };


        struct SchemaStatus
        {
          std::uint32_t previous_version{0U};
          std::uint32_t current_version{0U};
          std::uint32_t supported_version{0U};
          bool migration_applied{false};
        };


        struct IntegrityReport
        {
          bool healthy{false};
          std::vector<std::string> integrity_messages;
          std::vector<std::string> foreign_key_violations;
        };


        [[nodiscard]]
        std::string_view to_string(
          StorageCode code) noexcept;


        class SqliteStore
        {
        public:
          explicit SqliteStore(
            std::string database_path);

          ~SqliteStore();

          SqliteStore(
            const SqliteStore &) = delete;

          SqliteStore & operator=(
            const SqliteStore &) = delete;

          SqliteStore(
            SqliteStore &&) = delete;

          SqliteStore & operator=(
            SqliteStore &&) = delete;

          [[nodiscard]]
          StorageResult open();

          [[nodiscard]]
          StorageResult close();

          [[nodiscard]]
          bool is_open() const noexcept;

          [[nodiscard]]
          const std::string &
          database_path() const noexcept;

          [[nodiscard]]
          StorageConfiguration
          configuration() const;

          [[nodiscard]]
          StorageResult migrate(
            SchemaStatus * status);

          [[nodiscard]]
          StorageResult schema_version(
            std::uint32_t * version) const;

          [[nodiscard]]
          StorageResult integrity_check(
            IntegrityReport * report) const;

          [[nodiscard]]
          StorageResult begin_immediate();

          [[nodiscard]]
          StorageResult commit();

          [[nodiscard]]
          StorageResult rollback();

          [[nodiscard]]
          bool in_transaction() const noexcept;

          [[nodiscard]]
          StorageResult set_metadata(
            std::string_view key,
            std::string_view value);

          [[nodiscard]]
          StorageResult get_metadata(
            std::string_view key,
            std::optional<std::string> * value) const;

          [[nodiscard]]
          StorageResult table_exists(
            std::string_view table_name,
            bool * exists) const;

        private:
          [[nodiscard]]
          StorageResult execute_locked(
            std::string_view sql) const;

          [[nodiscard]]
          StorageResult query_single_int_locked(
            std::string_view sql,
            int * value) const;

          [[nodiscard]]
          StorageResult query_single_text_locked(
            std::string_view sql,
            std::string * value) const;

          [[nodiscard]]
          StorageResult check_open_locked() const;

          [[nodiscard]]
          StorageResult check_transaction_owner_locked()
            const;

          [[nodiscard]]
          StorageResult rollback_locked();

          std::string database_path_;

          mutable std::mutex mutex_;
          sqlite3 * database_{nullptr};

          StorageConfiguration configuration_;

          bool transaction_active_{false};
          std::thread::id transaction_owner_;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__SQLITE_STORE_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # SQLite implementation
    # -------------------------------------------------------------------------

    write(
        "src/sqlite_store.cpp",
        r'''
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
              std::string_view{"PRAGMA temp_store=MEMORY;"}})
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

          if (status->previous_version != 0U) {
            return failure(
              StorageCode::kMigrationFailed,
              SQLITE_ERROR,
              "no migration path exists for schema version");
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

          const auto schema_result =
            execute_locked(kMigration001Sql);

          if (!schema_result.success) {
            static_cast<void>(rollback_locked());

            return failure(
              StorageCode::kMigrationFailed,
              schema_result.sqlite_code,
              schema_result.reason);
          }

          const auto timestamp =
            std::to_string(unix_time_ns());

          const std::string migration_insert =
            "INSERT INTO schema_migrations("
            "schema_version,"
            "migration_name,"
            "applied_at_unix_ns"
            ") VALUES("
            "1,"
            "'001_initial_schema',"
            + timestamp +
            ");";

          const auto insert_result =
            execute_locked(migration_insert);

          if (!insert_result.success) {
            static_cast<void>(rollback_locked());

            return failure(
              StorageCode::kMigrationFailed,
              insert_result.sqlite_code,
              insert_result.reason);
          }

          const auto version_update =
            execute_locked("PRAGMA user_version=1;");

          if (!version_update.success) {
            static_cast<void>(rollback_locked());

            return failure(
              StorageCode::kMigrationFailed,
              version_update.sqlite_code,
              version_update.reason);
          }

          const auto commit_result =
            execute_locked("COMMIT;");

          if (!commit_result.success) {
            static_cast<void>(rollback_locked());

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

          return success("migration 001 applied");
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
        ''',
    )

    # -------------------------------------------------------------------------
    # Storage configuration
    # -------------------------------------------------------------------------

    write(
        "config/storage.yaml",
        r'''
        schema_version: 1

        storage:
          database_path: /var/lib/robot_savo/locations/locations.db
          backup_directory: /var/lib/robot_savo/locations/backups

          authoritative_runtime_store: sqlite
          create_parent_directories: true

          journal_mode: WAL
          synchronous: NORMAL
          foreign_keys: true
          busy_timeout_ms: 5000
          temporary_store: MEMORY

          migrate_on_startup: true
          fail_on_newer_schema: true
          integrity_check_on_startup: true
          fail_closed_on_integrity_error: true

        runtime_permissions:
          directory_mode: "0750"
          database_mode: "0640"

        testing:
          use_tmp_directory: false
          file_database_root: build/savo_locations/storage_test_runtime
          in_memory_database_allowed: true
        ''',
    )

    # -------------------------------------------------------------------------
    # SQLite tests
    # -------------------------------------------------------------------------

    write(
        "test/storage/test_sqlite_store.cpp",
        r'''
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
        ''',
    )

    # -------------------------------------------------------------------------
    # LOC-2A contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase2a_contracts.py",
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


        def test_package_contains_loc2a_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 5, 0)

            dependencies = {
                (element.text or "").strip()
                for element in package.findall("depend")
            }

            assert "sqlite3" in dependencies

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_sqlite_foundation_files_exist() -> None:
            for relative in (
                "include/savo_locations/sqlite_schema.hpp",
                "include/savo_locations/sqlite_store.hpp",
                "src/sqlite_store.cpp",
                "config/storage.yaml",
                "test/storage/test_sqlite_store.cpp",
            ):
                assert (ROOT / relative).is_file()


        def test_cmake_builds_storage_library() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(SQLite3 REQUIRED)" in cmake
            assert "savo_locations_storage" in cmake
            assert "src/sqlite_store.cpp" in cmake
            assert "SQLite::SQLite3" in cmake
            assert "test_sqlite_store" in cmake
            assert "test_phase2a_contracts" in cmake


        def test_schema_contains_required_tables() -> None:
            schema = read(
                "include/savo_locations/sqlite_schema.hpp"
            )

            for table in (
                "schema_migrations",
                "registry_metadata",
                "locations",
                "location_aliases",
                "location_candidates",
                "candidate_aliases",
                "location_events",
            ):
                assert f"CREATE TABLE IF NOT EXISTS {table}" in schema


        def test_schema_has_partial_uniqueness() -> None:
            schema = read(
                "include/savo_locations/sqlite_schema.hpp"
            )

            assert (
                "idx_locations_active_tag_unique"
                in schema
            )

            assert "WHERE state != 2" in schema

            assert (
                "idx_location_aliases_active_identity_unique"
                in schema
            )

            assert "WHERE reserves_identity = 1" in schema

            assert (
                "idx_candidates_pending_tag_unique"
                in schema
            )

            assert "WHERE state = 1" in schema


        def test_store_enforces_safe_sqlite_policy() -> None:
            implementation = read(
                "src/sqlite_store.cpp"
            )

            for required in (
                "PRAGMA foreign_keys=ON",
                "PRAGMA synchronous=NORMAL",
                "PRAGMA journal_mode=WAL",
                "sqlite3_busy_timeout",
                "SQLITE_OPEN_FULLMUTEX",
                "BEGIN IMMEDIATE",
                "PRAGMA integrity_check",
                "PRAGMA foreign_key_check",
            ):
                assert required in implementation


        def test_store_has_fail_closed_schema_handling() -> None:
            header = read(
                "include/savo_locations/sqlite_store.hpp"
            )

            implementation = read(
                "src/sqlite_store.cpp"
            )

            assert "kSchemaTooNew" in header

            assert (
                "database schema is newer than this package"
                in implementation
            )

            assert (
                "kSupportedSqliteSchemaVersion"
                in implementation
            )


        def test_loc2a_has_no_ros_runtime_node() -> None:
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
    # README and policy
    # -------------------------------------------------------------------------

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(
        encoding="utf-8"
    )

    if "## LOC-2A SQLite foundation" not in readme:
        readme += clean(
            r'''

            ## LOC-2A SQLite foundation

            LOC-2A introduces the persistent-store foundation without adding
            a ROS node or record serialization.

            Runtime database:

            ```text
            /var/lib/robot_savo/locations/locations.db
            ```

            Backup directory:

            ```text
            /var/lib/robot_savo/locations/backups/
            ```

            The store configures:

            - WAL journaling for file-backed databases;
            - foreign-key enforcement;
            - `NORMAL` synchronous policy;
            - a 5000 ms busy timeout;
            - memory-backed SQLite temporary storage;
            - full-mutex SQLite connection mode.

            Schema migration is explicit and transactional. Databases with a
            schema newer than the supported version fail closed. Migration 001
            creates the location, alias, candidate, event, metadata and schema
            history tables.

            Active location tags, active normalized identity keys and pending
            candidate tags use partial unique indexes. Retired locations and
            terminal candidates therefore do not permanently reserve those
            identities.

            Integrity validation runs both SQLite `integrity_check` and
            `foreign_key_check`.

            LOC-2A does not yet serialize domain records, load the in-memory
            catalog, create backups, expose ROS services or start a runtime
            node. Those capabilities follow in later LOC-2 phases.
            '''
        )

        readme_path.write_text(
            readme,
            encoding="utf-8",
        )

    policy_path = ROOT / "config" / "location_policy.yaml"
    policy = policy_path.read_text(encoding="utf-8")

    if "\nsqlite_foundation:\n" not in policy:
        policy += clean(
            r'''

            sqlite_foundation:
              supported_schema_version: 1
              migration_mode: transactional
              transaction_begin_mode: IMMEDIATE
              reject_newer_schema: true
              integrity_check_required: true
              foreign_key_check_required: true

              active_location_tag_unique: true
              active_identity_key_unique: true
              pending_candidate_tag_unique: true

              metadata_upsert_supported: true
              transaction_owner_thread_enforced: true
              rollback_active_transaction_on_close: true
            '''
        )

        policy_path.write_text(
            policy,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # Permanent manifest
    # -------------------------------------------------------------------------

    changed_files = sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
    )

    manifest = (
        LOGS
        / f"LOC2A_savo_locations_{stamp}.sha256"
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
        "LOC-2A SQLite schema and transaction foundation applied."
    )

    print(
        "No ROS node, mapping, navigation or hardware runtime was added."
    )


if __name__ == "__main__":
    main()
