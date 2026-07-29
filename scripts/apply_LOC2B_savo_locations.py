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


def update_package_version() -> None:
    package_xml = ROOT / "package.xml"

    tree = ET.parse(package_xml)
    package = tree.getroot()

    version = package.find("version")

    if version is None:
        raise RuntimeError(
            "package.xml has no version element"
        )

    version.text = "0.6.0"

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
        / f"pre_LOC2B_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    update_package_version()

    # -------------------------------------------------------------------------
    # CMake
    # -------------------------------------------------------------------------

    write(
        "CMakeLists.txt",
        r'''
        cmake_minimum_required(VERSION 3.16)
        project(savo_locations VERSION 0.6.0 LANGUAGES CXX)

        if(NOT CMAKE_CXX_STANDARD)
          set(CMAKE_CXX_STANDARD 17)
        endif()

        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)

        find_package(ament_cmake REQUIRED)
        find_package(savo_msgs REQUIRED)
        find_package(SQLite3 REQUIRED)

        # ---------------------------------------------------------------------------
        # Deterministic domain library
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
        # SQLite storage library
        # ---------------------------------------------------------------------------

        add_library(
          savo_locations_storage
          src/sqlite_store.cpp
          src/sqlite_repository.cpp
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
                "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
            )
          endif()

          ament_add_gtest(
            test_sqlite_repository
            test/storage/test_sqlite_repository.cpp
          )

          if(TARGET test_sqlite_repository)
            target_link_libraries(
              test_sqlite_repository
              savo_locations_storage
              SQLite::SQLite3
            )

            target_compile_definitions(
              test_sqlite_repository
              PRIVATE
                "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
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

          ament_add_pytest_test(
            test_phase2b_contracts
            test/contracts/test_phase2b_contracts.py
            TIMEOUT 60
          )
        endif()

        # ---------------------------------------------------------------------------
        # Exports
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
    # Version constant
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
        '"0.5.0"',
        '"0.6.0"',
    )

    constants_path.write_text(
        constants,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Permit the typed repository to use the protected SQLite connection
    # -------------------------------------------------------------------------

    store_header = (
        ROOT
        / "include"
        / "savo_locations"
        / "sqlite_store.hpp"
    )

    store_text = store_header.read_text(
        encoding="utf-8"
    )

    if "friend class SqliteRepository;" not in store_text:
        lines = store_text.splitlines(
            keepends=True
        )

        private_index = None

        for index, line in enumerate(lines):
            if line.strip() == "private:":
                private_index = index
                break

        if private_index is None:
            raise RuntimeError(
                "Could not locate SqliteStore private section"
            )

        private_line = lines[private_index]

        indentation = private_line[
            :len(private_line) -
            len(private_line.lstrip())
        ]

        lines.insert(
            private_index + 1,
            (
                f"{indentation}  "
                "friend class SqliteRepository;\n\n"
            ),
        )

        store_text = "".join(lines)

        if (
            "friend class SqliteRepository;"
            not in store_text
        ):
            raise RuntimeError(
                "SqliteRepository friend declaration "
                "failed verification"
            )

        store_header.write_text(
            store_text,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # Repository API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/sqlite_repository.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_
        #define SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_

        #include <cstdint>
        #include <string>
        #include <string_view>
        #include <vector>

        #include "savo_locations/model.hpp"
        #include "savo_locations/sqlite_store.hpp"
        #include "savo_locations/validation.hpp"

        namespace savo_locations
        {

        enum class SnapshotCode : std::uint8_t
        {
          kOk = 0U,
          kInvalidArgument,
          kStoreNotOpen,
          kValidationFailed,
          kIdentityConflict,
          kTagConflict,
          kTransactionActive,
          kSqlError,
          kCorruptData,
        };


        struct SnapshotResult
        {
          bool success{false};

          SnapshotCode code{
            SnapshotCode::kSqlError};

          int sqlite_code{0};
          std::string reason;

          std::vector<ValidationIssue>
            validation_issues;
        };


        struct CatalogSnapshot
        {
          std::vector<LocationRecordData> locations;
          std::vector<CandidateRecordData> candidates;
        };


        [[nodiscard]]
        std::string_view to_string(
          SnapshotCode code) noexcept;


        class SqliteRepository
        {
        public:
          explicit SqliteRepository(
            SqliteStore & store);

          [[nodiscard]]
          SnapshotResult save_snapshot(
            const CatalogSnapshot & snapshot);

          [[nodiscard]]
          SnapshotResult load_snapshot(
            CatalogSnapshot * snapshot) const;

        private:
          SqliteStore & store_;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Repository implementation
    # -------------------------------------------------------------------------

    write(
        "src/sqlite_repository.cpp",
        r'''
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

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # Repository tests
    # -------------------------------------------------------------------------

    write(
        "test/storage/test_sqlite_repository.cpp",
        r'''
        #include <gtest/gtest.h>
        #include <sqlite3.h>

        #include <filesystem>
        #include <string>

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
        ''',
    )

    # -------------------------------------------------------------------------
    # LOC-2B contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase2b_contracts.py",
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


        def test_package_contains_loc2b_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 6, 0)

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_repository_files_exist() -> None:
            for relative in (
                "include/savo_locations/sqlite_repository.hpp",
                "src/sqlite_repository.cpp",
                "test/storage/test_sqlite_repository.cpp",
            ):
                assert (ROOT / relative).is_file()


        def test_cmake_builds_repository() -> None:
            cmake = read("CMakeLists.txt")

            assert "src/sqlite_repository.cpp" in cmake
            assert "test_sqlite_repository" in cmake
            assert "test_phase2b_contracts" in cmake

            assert (
                'SAVO_LOCATIONS_TEST_DB_DIR=\\"'
                in cmake
            )


        def test_repository_has_typed_snapshot_api() -> None:
            header = read(
                "include/savo_locations/sqlite_repository.hpp"
            )

            assert "struct CatalogSnapshot" in header
            assert "std::vector<LocationRecordData>" in header
            assert "std::vector<CandidateRecordData>" in header

            assert "save_snapshot" in header
            assert "load_snapshot" in header


        def test_snapshot_write_is_transactional() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert '"BEGIN IMMEDIATE;"' in implementation
            assert '"COMMIT;"' in implementation
            assert '"ROLLBACK;"' in implementation

            assert (
                "DELETE FROM location_candidates"
                in implementation
            )

            assert (
                "DELETE FROM locations"
                in implementation
            )


        def test_repository_persists_location_identity_rows() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert "insert_location_identity" in implementation
            assert "alias_kind" in implementation
            assert "normalize_lookup_key(alias_text)" in implementation
            assert "reserves_identity" in implementation


        def test_repository_persists_candidates() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert "INSERT INTO location_candidates" in implementation
            assert "INSERT INTO candidate_aliases" in implementation
            assert "approved_location_id" in implementation
            assert "review_reason" in implementation


        def test_repository_validates_before_and_after_io() -> None:
            implementation = read(
                "src/sqlite_repository.cpp"
            )

            assert implementation.count(
                "validate_snapshot("
            ) >= 3

            assert "kCorruptData" in implementation
            assert "persisted catalog failed domain validation" in implementation


        def test_loc2b_still_has_no_ros_runtime_node() -> None:
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
    # Configuration and documentation
    # -------------------------------------------------------------------------

    storage_path = ROOT / "config" / "storage.yaml"
    storage = storage_path.read_text(encoding="utf-8")

    if "\nsnapshot_persistence:\n" not in storage:
        storage += clean(
            r'''

            snapshot_persistence:
              typed_domain_records: true
              validate_before_write: true
              validate_after_read: true

              write_mode: atomic_replace
              transaction_begin_mode: IMMEDIATE

              preserve_event_journal: true
              replace_locations: true
              replace_candidates: true

              reject_identity_conflicts: true
              reject_tag_conflicts: true
              reject_missing_approved_location: true
              reject_corrupt_loaded_records: true
            '''
        )

        storage_path.write_text(
            storage,
            encoding="utf-8",
        )

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(encoding="utf-8")

    if "## LOC-2B typed SQLite snapshots" not in readme:
        readme += clean(
            r'''

            ## LOC-2B typed SQLite snapshots

            LOC-2B persists complete typed location and candidate snapshots.

            Persisted locations include:

            - approval/retirement state;
            - enabled state and record revision;
            - canonical identity, display name and aliases;
            - semantic type;
            - map ID, map revision and map release ID;
            - approach, confirmation and tag poses;
            - AprilTag binding;
            - semantic area metadata;
            - originating candidate ID.

            Persisted candidates include:

            - lifecycle state and candidate revision;
            - mapping and AprilTag evidence;
            - detection quality and observation statistics;
            - optional approach and confirmation poses;
            - suggested identity and semantic metadata;
            - mapping session and source component;
            - review reason and approved location binding.

            Snapshot replacement uses one `BEGIN IMMEDIATE` transaction.
            Existing locations and candidates are restored automatically if
            any new row or identity index fails.

            Domain validation runs before persistence and after loading.
            Persisted data that no longer satisfies the domain contracts is
            reported as corrupt and is not returned to runtime consumers.

            LOC-2B still does not introduce a ROS node, runtime services,
            automatic catalog rehydration, backup rotation or event writes.
            '''
        )

        readme_path.write_text(
            readme,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # Manifest
    # -------------------------------------------------------------------------

    changed_files = sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
    )

    manifest = (
        LOGS
        / f"LOC2B_savo_locations_{stamp}.sha256"
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
        "LOC-2B typed SQLite snapshot persistence applied."
    )

    print(
        "No ROS node, ROS services or hardware runtime was added."
    )


if __name__ == "__main__":
    main()
