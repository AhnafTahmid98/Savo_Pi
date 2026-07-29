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


def main() -> None:
    package_xml = ROOT / "package.xml"

    if not package_xml.is_file():
        raise SystemExit(
            f"savo_locations package not found: {ROOT}"
        )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    backup = (
        BACKUPS
        / f"pre_LOC1B_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    # -------------------------------------------------------------------------
    # Package version
    # -------------------------------------------------------------------------

    tree = ET.parse(package_xml)
    package_root = tree.getroot()
    version_element = package_root.find("version")

    if version_element is None:
        raise RuntimeError(
            "package.xml does not contain a version element"
        )

    version_element.text = "0.3.0"

    ET.indent(tree, space="  ")
    tree.write(
        package_xml,
        encoding="utf-8",
        xml_declaration=True,
    )

    # -------------------------------------------------------------------------
    # CMake
    # -------------------------------------------------------------------------

    write(
        "CMakeLists.txt",
        r'''
        cmake_minimum_required(VERSION 3.16)
        project(savo_locations VERSION 0.3.0 LANGUAGES CXX)

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

        # ---------------------------------------------------------------------------
        # Deterministic location-domain library
        # ---------------------------------------------------------------------------

        add_library(
          savo_locations_contracts
          src/types.cpp
          src/normalization.cpp
          src/validation.cpp
          src/registry.cpp
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

        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
          target_compile_options(
            savo_locations_contracts
            PRIVATE
              -Wall
              -Wextra
              -Wpedantic
              -Wconversion
              -Wsign-conversion
          )
        endif()

        # ---------------------------------------------------------------------------
        # Installation
        # ---------------------------------------------------------------------------

        install(
          TARGETS
            savo_locations_contracts
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
        endif()

        # ---------------------------------------------------------------------------
        # Package exports
        # ---------------------------------------------------------------------------

        ament_export_targets(
          export_savo_locations
          HAS_LIBRARY_TARGET
        )

        ament_export_include_directories(include)
        ament_export_dependencies(savo_msgs)

        ament_package()
        ''',
    )

    # -------------------------------------------------------------------------
    # Constants
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/constants.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__CONSTANTS_HPP_
        #define SAVO_LOCATIONS__CONSTANTS_HPP_

        #include <cstddef>
        #include <cstdint>
        #include <string_view>

        namespace savo_locations
        {

        inline constexpr std::string_view kPackageName{
          "savo_locations"};

        inline constexpr std::string_view kPackageVersion{
          "0.3.0"};

        inline constexpr std::uint32_t kSchemaVersion{1U};

        inline constexpr std::size_t
          kMaximumLocationIdLength{64U};

        inline constexpr std::size_t
          kMaximumCandidateIdLength{128U};

        inline constexpr std::size_t
          kMaximumDisplayNameLength{128U};

        inline constexpr std::size_t
          kMaximumAliasCount{32U};

        inline constexpr std::size_t
          kMaximumAliasLength{128U};

        inline constexpr std::size_t
          kMaximumMapIdLength{128U};

        inline constexpr std::size_t
          kMaximumMapReleaseIdLength{128U};

        inline constexpr std::size_t
          kMaximumTagFamilyLength{64U};

        inline constexpr std::string_view
          kCanonicalMapFrame{"map"};

        inline constexpr double
          kQuaternionNormTolerance{0.01};

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__CONSTANTS_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Extended domain model
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/model.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__MODEL_HPP_
        #define SAVO_LOCATIONS__MODEL_HPP_

        #include <cstdint>
        #include <optional>
        #include <string>
        #include <vector>

        #include "savo_locations/types.hpp"

        namespace savo_locations
        {

        struct PoseData
        {
          std::string frame_id{"map"};

          double x{0.0};
          double y{0.0};
          double z{0.0};

          double qx{0.0};
          double qy{0.0};
          double qz{0.0};
          double qw{1.0};
        };


        struct MapContext
        {
          std::string map_id;
          std::uint32_t map_revision{0U};
          std::string map_release_id;
        };


        struct TagBinding
        {
          std::string family;
          std::int32_t id{-1};
        };


        struct LocationDraft
        {
          std::string location_id;
          std::string display_name;
          std::vector<std::string> aliases;
          std::string semantic_type;

          MapContext map;

          PoseData approach_pose;
          std::optional<PoseData> confirmation_pose;
          std::optional<PoseData> tag_pose_map;

          TagBinding tag;

          bool arrival_confirmation_required{true};

          std::string building;
          std::string floor;
          std::string area;
          std::string notes;
        };


        struct LocationRecordData
        {
          LocationState state{LocationState::kApproved};
          bool enabled{true};
          std::uint64_t record_revision{1U};

          LocationDraft location;

          std::string source_candidate_id;
        };


        struct CandidateDraft
        {
          std::string candidate_id;

          MapContext map;
          TagBinding tag;
          PoseData tag_pose_map;

          double detection_quality{0.0};
          std::uint32_t accepted_observations{0U};
          double position_stddev_m{0.0};
          double yaw_stddev_rad{0.0};

          std::optional<PoseData> approach_pose;
          std::optional<PoseData> confirmation_pose;

          std::string suggested_location_id;
          std::string suggested_display_name;
          std::vector<std::string> suggested_aliases;
          std::string suggested_semantic_type;

          std::string building;
          std::string floor;
          std::string area;
          std::string notes;

          std::string source_session_id;
          std::string source_component;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__MODEL_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Registry API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/registry.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__REGISTRY_HPP_
        #define SAVO_LOCATIONS__REGISTRY_HPP_

        #include <cstddef>
        #include <cstdint>
        #include <map>
        #include <optional>
        #include <shared_mutex>
        #include <string>
        #include <string_view>
        #include <vector>

        #include "savo_locations/model.hpp"
        #include "savo_locations/validation.hpp"

        namespace savo_locations
        {

        enum class MutationCode : std::uint8_t
        {
          kInserted = 0U,
          kUpdated,
          kNotFound,
          kInvalidRecord,
          kStaleRevision,
          kRevisionSequenceError,
          kLocationIdConflict,
          kIdentityConflict,
          kTagConflict,
          kRetired,
        };


        enum class ResolveCode : std::uint8_t
        {
          kResolved = 0U,
          kInvalidQuery,
          kNotFound,
          kAmbiguous,
          kDisabled,
          kRetired,
          kMapMismatch,
        };


        enum class MatchType : std::uint8_t
        {
          kNone = 0U,
          kLocationId,
          kDisplayName,
          kAlias,
        };


        struct RegistryMutationResult
        {
          bool success{false};
          MutationCode code{MutationCode::kInvalidRecord};
          std::string reason;

          std::vector<ValidationIssue> validation_issues;
          std::optional<LocationRecordData> record;
        };


        struct ResolveOptions
        {
          bool enforce_map_context{false};
          MapContext map;
        };


        struct ResolveResult
        {
          bool resolved{false};
          ResolveCode code{ResolveCode::kNotFound};
          std::string reason;
          std::string normalized_query;
          MatchType match_type{MatchType::kNone};

          std::optional<LocationRecordData> record;
          std::vector<std::string> ambiguous_location_ids;
        };


        [[nodiscard]]
        std::string_view to_string(
          MutationCode code) noexcept;

        [[nodiscard]]
        std::string_view to_string(
          ResolveCode code) noexcept;

        [[nodiscard]]
        std::string_view to_string(
          MatchType type) noexcept;


        class InMemoryRegistry
        {
        public:
          [[nodiscard]]
          RegistryMutationResult insert(
            LocationRecordData record);

          [[nodiscard]]
          RegistryMutationResult replace(
            LocationRecordData record,
            std::uint64_t expected_current_revision);

          [[nodiscard]]
          RegistryMutationResult set_enabled(
            std::string_view location_id,
            std::uint64_t expected_current_revision,
            bool enabled);

          [[nodiscard]]
          ResolveResult resolve(
            std::string_view query,
            const ResolveOptions & options =
              ResolveOptions{}) const;

          [[nodiscard]]
          std::optional<LocationRecordData> get(
            std::string_view location_id) const;

          [[nodiscard]]
          std::vector<LocationRecordData> list() const;

          [[nodiscard]]
          std::size_t size() const noexcept;

          void clear();

        private:
          [[nodiscard]]
          bool has_identity_collision_locked(
            const LocationRecordData & candidate,
            std::string_view excluded_location_id,
            std::string * conflicting_location_id) const;

          [[nodiscard]]
          bool has_tag_collision_locked(
            const LocationRecordData & candidate,
            std::string_view excluded_location_id,
            std::string * conflicting_location_id) const;

          mutable std::shared_mutex mutex_;

          std::map<
            std::string,
            LocationRecordData,
            std::less<>>
          records_;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__REGISTRY_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Registry implementation
    # -------------------------------------------------------------------------

    write(
        "src/registry.cpp",
        r'''
        #include "savo_locations/registry.hpp"

        #include <limits>
        #include <mutex>
        #include <set>
        #include <utility>

        #include "savo_locations/normalization.hpp"

        namespace savo_locations
        {
        namespace
        {

        struct Match
        {
          LocationRecordData record;
          MatchType type{MatchType::kNone};
        };


        bool same_map_context(
          const MapContext & left,
          const MapContext & right) noexcept
        {
          return
            left.map_id == right.map_id &&
            left.map_revision == right.map_revision;
        }


        bool reserves_identity(
          const LocationRecordData & record) noexcept
        {
          return record.state != LocationState::kRetired;
        }


        std::set<std::string> identity_keys(
          const LocationRecordData & record)
        {
          std::set<std::string> keys;

          const auto id_key =
            normalize_lookup_key(
              record.location.location_id);

          const auto display_key =
            normalize_lookup_key(
              record.location.display_name);

          if (!id_key.empty()) {
            keys.insert(id_key);
          }

          if (!display_key.empty()) {
            keys.insert(display_key);
          }

          for (
            const auto & alias :
            record.location.aliases)
          {
            const auto alias_key =
              normalize_lookup_key(alias);

            if (!alias_key.empty()) {
              keys.insert(alias_key);
            }
          }

          return keys;
        }


        bool records_have_identity_overlap(
          const LocationRecordData & left,
          const LocationRecordData & right)
        {
          const auto left_keys =
            identity_keys(left);

          const auto right_keys =
            identity_keys(right);

          for (const auto & key : left_keys) {
            if (right_keys.count(key) != 0U) {
              return true;
            }
          }

          return false;
        }


        bool valid_record_envelope(
          const LocationRecordData & record,
          std::string * reason)
        {
          if (record.record_revision == 0U) {
            if (reason != nullptr) {
              *reason =
                "record revision must be greater than zero";
            }

            return false;
          }

          if (record.state == LocationState::kUnknown) {
            if (reason != nullptr) {
              *reason =
                "location state must not be unknown";
            }

            return false;
          }

          if (
            record.state == LocationState::kRetired &&
            record.enabled)
          {
            if (reason != nullptr) {
              *reason =
                "retired locations must be disabled";
            }

            return false;
          }

          return true;
        }


        RegistryMutationResult mutation_failure(
          const MutationCode code,
          std::string reason)
        {
          RegistryMutationResult result;
          result.success = false;
          result.code = code;
          result.reason = std::move(reason);
          return result;
        }


        RegistryMutationResult validation_failure(
          const ValidationResult & validation,
          std::string reason)
        {
          RegistryMutationResult result;
          result.success = false;
          result.code = MutationCode::kInvalidRecord;
          result.reason = std::move(reason);
          result.validation_issues = validation.issues();
          return result;
        }


        RegistryMutationResult mutation_success(
          const MutationCode code,
          std::string reason,
          const LocationRecordData & record)
        {
          RegistryMutationResult result;
          result.success = true;
          result.code = code;
          result.reason = std::move(reason);
          result.record = record;
          return result;
        }


        ResolveResult resolve_status(
          const LocationRecordData & record,
          const MatchType match_type,
          const std::string & normalized_query)
        {
          ResolveResult result;
          result.normalized_query = normalized_query;
          result.match_type = match_type;
          result.record = record;

          if (record.state == LocationState::kRetired) {
            result.resolved = false;
            result.code = ResolveCode::kRetired;
            result.reason = "location is retired";
            return result;
          }

          if (!record.enabled) {
            result.resolved = false;
            result.code = ResolveCode::kDisabled;
            result.reason = "location is disabled";
            return result;
          }

          result.resolved = true;
          result.code = ResolveCode::kResolved;
          result.reason = "location resolved";
          return result;
        }


        std::optional<MatchType> generic_match_type(
          const LocationRecordData & record,
          const std::string & normalized_query)
        {
          if (
            normalize_lookup_key(
              record.location.location_id) ==
            normalized_query)
          {
            return MatchType::kLocationId;
          }

          if (
            normalize_lookup_key(
              record.location.display_name) ==
            normalized_query)
          {
            return MatchType::kDisplayName;
          }

          for (
            const auto & alias :
            record.location.aliases)
          {
            if (
              normalize_lookup_key(alias) ==
              normalized_query)
            {
              return MatchType::kAlias;
            }
          }

          return std::nullopt;
        }

        }  // namespace


        std::string_view to_string(
          const MutationCode code) noexcept
        {
          switch (code) {
            case MutationCode::kInserted:
              return "inserted";

            case MutationCode::kUpdated:
              return "updated";

            case MutationCode::kNotFound:
              return "not_found";

            case MutationCode::kInvalidRecord:
              return "invalid_record";

            case MutationCode::kStaleRevision:
              return "stale_revision";

            case MutationCode::kRevisionSequenceError:
              return "revision_sequence_error";

            case MutationCode::kLocationIdConflict:
              return "location_id_conflict";

            case MutationCode::kIdentityConflict:
              return "identity_conflict";

            case MutationCode::kTagConflict:
              return "tag_conflict";

            case MutationCode::kRetired:
              return "retired";

            default:
              return "unknown";
          }
        }


        std::string_view to_string(
          const ResolveCode code) noexcept
        {
          switch (code) {
            case ResolveCode::kResolved:
              return "resolved";

            case ResolveCode::kInvalidQuery:
              return "invalid_query";

            case ResolveCode::kNotFound:
              return "not_found";

            case ResolveCode::kAmbiguous:
              return "ambiguous";

            case ResolveCode::kDisabled:
              return "disabled";

            case ResolveCode::kRetired:
              return "retired";

            case ResolveCode::kMapMismatch:
              return "map_mismatch";

            default:
              return "unknown";
          }
        }


        std::string_view to_string(
          const MatchType type) noexcept
        {
          switch (type) {
            case MatchType::kLocationId:
              return "location_id";

            case MatchType::kDisplayName:
              return "display_name";

            case MatchType::kAlias:
              return "alias";

            case MatchType::kNone:
            default:
              return "none";
          }
        }


        RegistryMutationResult InMemoryRegistry::insert(
          LocationRecordData record)
        {
          const auto validation =
            validate_location_draft(record.location);

          if (!validation.valid()) {
            return validation_failure(
              validation,
              "location record validation failed");
          }

          std::string envelope_reason;

          if (
            !valid_record_envelope(
              record,
              &envelope_reason))
          {
            return mutation_failure(
              MutationCode::kInvalidRecord,
              envelope_reason);
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          if (
            records_.find(
              record.location.location_id) !=
            records_.end())
          {
            return mutation_failure(
              MutationCode::kLocationIdConflict,
              "location ID already exists");
          }

          std::string conflict;

          if (
            has_identity_collision_locked(
              record,
              "",
              &conflict))
          {
            return mutation_failure(
              MutationCode::kIdentityConflict,
              "identity key conflicts with " +
              conflict);
          }

          if (
            has_tag_collision_locked(
              record,
              "",
              &conflict))
          {
            return mutation_failure(
              MutationCode::kTagConflict,
              "AprilTag conflicts with " +
              conflict);
          }

          const auto location_id =
            record.location.location_id;

          records_.emplace(
            location_id,
            record);

          return mutation_success(
            MutationCode::kInserted,
            "location inserted",
            record);
        }


        RegistryMutationResult InMemoryRegistry::replace(
          LocationRecordData record,
          const std::uint64_t expected_current_revision)
        {
          const auto validation =
            validate_location_draft(record.location);

          if (!validation.valid()) {
            return validation_failure(
              validation,
              "location record validation failed");
          }

          std::string envelope_reason;

          if (
            !valid_record_envelope(
              record,
              &envelope_reason))
          {
            return mutation_failure(
              MutationCode::kInvalidRecord,
              envelope_reason);
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          const auto existing =
            records_.find(
              record.location.location_id);

          if (existing == records_.end()) {
            return mutation_failure(
              MutationCode::kNotFound,
              "location does not exist");
          }

          if (
            existing->second.record_revision !=
            expected_current_revision)
          {
            return mutation_failure(
              MutationCode::kStaleRevision,
              "expected revision does not match current revision");
          }

          if (
            expected_current_revision ==
            std::numeric_limits<
              std::uint64_t>::max())
          {
            return mutation_failure(
              MutationCode::kRevisionSequenceError,
              "record revision cannot be incremented");
          }

          const auto required_revision =
            expected_current_revision +
            std::uint64_t{1U};

          if (
            record.record_revision !=
            required_revision)
          {
            return mutation_failure(
              MutationCode::kRevisionSequenceError,
              "replacement revision must equal current revision plus one");
          }

          std::string conflict;

          if (
            has_identity_collision_locked(
              record,
              record.location.location_id,
              &conflict))
          {
            return mutation_failure(
              MutationCode::kIdentityConflict,
              "identity key conflicts with " +
              conflict);
          }

          if (
            has_tag_collision_locked(
              record,
              record.location.location_id,
              &conflict))
          {
            return mutation_failure(
              MutationCode::kTagConflict,
              "AprilTag conflicts with " +
              conflict);
          }

          existing->second = record;

          return mutation_success(
            MutationCode::kUpdated,
            "location replaced",
            record);
        }


        RegistryMutationResult
        InMemoryRegistry::set_enabled(
          const std::string_view location_id,
          const std::uint64_t expected_current_revision,
          const bool enabled)
        {
          const auto canonical_id =
            canonicalize_location_id(location_id);

          if (
            !is_canonical_location_id(
              canonical_id))
          {
            return mutation_failure(
              MutationCode::kInvalidRecord,
              "location ID is invalid");
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          const auto existing =
            records_.find(canonical_id);

          if (existing == records_.end()) {
            return mutation_failure(
              MutationCode::kNotFound,
              "location does not exist");
          }

          if (
            existing->second.record_revision !=
            expected_current_revision)
          {
            return mutation_failure(
              MutationCode::kStaleRevision,
              "expected revision does not match current revision");
          }

          if (
            existing->second.state ==
            LocationState::kRetired)
          {
            return mutation_failure(
              MutationCode::kRetired,
              "retired location cannot be enabled or disabled");
          }

          if (
            existing->second.record_revision ==
            std::numeric_limits<
              std::uint64_t>::max())
          {
            return mutation_failure(
              MutationCode::kRevisionSequenceError,
              "record revision cannot be incremented");
          }

          auto updated = existing->second;
          updated.enabled = enabled;

          updated.record_revision +=
            std::uint64_t{1U};

          existing->second = updated;

          return mutation_success(
            MutationCode::kUpdated,
            enabled ?
              "location enabled" :
              "location disabled",
            updated);
        }


        ResolveResult InMemoryRegistry::resolve(
          const std::string_view query,
          const ResolveOptions & options) const
        {
          ResolveResult invalid;

          invalid.normalized_query =
            normalize_lookup_key(query);

          if (invalid.normalized_query.empty()) {
            invalid.resolved = false;
            invalid.code = ResolveCode::kInvalidQuery;
            invalid.reason = "query is empty";
            return invalid;
          }

          if (options.enforce_map_context) {
            const auto map_validation =
              validate_map_context(options.map);

            if (!map_validation.valid()) {
              invalid.resolved = false;
              invalid.code =
                ResolveCode::kInvalidQuery;

              invalid.reason =
                "map context is invalid";

              return invalid;
            }
          }

          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          const auto canonical_query =
            canonicalize_location_id(query);

          if (
            is_canonical_location_id(
              canonical_query))
          {
            const auto exact =
              records_.find(canonical_query);

            if (exact != records_.end()) {
              if (
                options.enforce_map_context &&
                !same_map_context(
                  exact->second.location.map,
                  options.map))
              {
                ResolveResult mismatch;
                mismatch.resolved = false;
                mismatch.code =
                  ResolveCode::kMapMismatch;

                mismatch.reason =
                  "location belongs to a different map context";

                mismatch.normalized_query =
                  invalid.normalized_query;

                mismatch.match_type =
                  MatchType::kLocationId;

                mismatch.record = exact->second;
                return mismatch;
              }

              return resolve_status(
                exact->second,
                MatchType::kLocationId,
                invalid.normalized_query);
            }
          }

          std::vector<Match> all_matches;

          for (const auto & entry : records_) {
            const auto match_type =
              generic_match_type(
                entry.second,
                invalid.normalized_query);

            if (match_type.has_value()) {
              all_matches.push_back(
                Match{
                  entry.second,
                  match_type.value()});
            }
          }

          if (all_matches.empty()) {
            ResolveResult not_found;
            not_found.resolved = false;
            not_found.code = ResolveCode::kNotFound;
            not_found.reason = "location was not found";

            not_found.normalized_query =
              invalid.normalized_query;

            return not_found;
          }

          std::vector<Match> contextual_matches;

          for (const auto & match : all_matches) {
            if (
              !options.enforce_map_context ||
              same_map_context(
                match.record.location.map,
                options.map))
            {
              contextual_matches.push_back(match);
            }
          }

          if (contextual_matches.empty()) {
            ResolveResult mismatch;
            mismatch.resolved = false;
            mismatch.code = ResolveCode::kMapMismatch;

            mismatch.reason =
              "matching identity exists only in another map context";

            mismatch.normalized_query =
              invalid.normalized_query;

            return mismatch;
          }

          std::vector<Match> non_retired_matches;

          for (
            const auto & match :
            contextual_matches)
          {
            if (
              match.record.state !=
              LocationState::kRetired)
            {
              non_retired_matches.push_back(match);
            }
          }

          const auto & preferred_matches =
            non_retired_matches.empty() ?
            contextual_matches :
            non_retired_matches;

          if (preferred_matches.size() > 1U) {
            ResolveResult ambiguous;
            ambiguous.resolved = false;
            ambiguous.code =
              ResolveCode::kAmbiguous;

            ambiguous.reason =
              "query matches multiple locations";

            ambiguous.normalized_query =
              invalid.normalized_query;

            for (
              const auto & match :
              preferred_matches)
            {
              ambiguous.ambiguous_location_ids.push_back(
                match.record.location.location_id);
            }

            return ambiguous;
          }

          const auto & selected =
            preferred_matches.front();

          return resolve_status(
            selected.record,
            selected.type,
            invalid.normalized_query);
        }


        std::optional<LocationRecordData>
        InMemoryRegistry::get(
          const std::string_view location_id) const
        {
          const auto canonical_id =
            canonicalize_location_id(location_id);

          if (
            !is_canonical_location_id(
              canonical_id))
          {
            return std::nullopt;
          }

          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          const auto record =
            records_.find(canonical_id);

          if (record == records_.end()) {
            return std::nullopt;
          }

          return record->second;
        }


        std::vector<LocationRecordData>
        InMemoryRegistry::list() const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          std::vector<LocationRecordData> output;
          output.reserve(records_.size());

          for (const auto & entry : records_) {
            output.push_back(entry.second);
          }

          return output;
        }


        std::size_t
        InMemoryRegistry::size() const noexcept
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return records_.size();
        }


        void InMemoryRegistry::clear()
        {
          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          records_.clear();
        }


        bool
        InMemoryRegistry::has_identity_collision_locked(
          const LocationRecordData & candidate,
          const std::string_view excluded_location_id,
          std::string * conflicting_location_id) const
        {
          if (!reserves_identity(candidate)) {
            return false;
          }

          for (const auto & entry : records_) {
            if (
              entry.first ==
              excluded_location_id)
            {
              continue;
            }

            if (!reserves_identity(entry.second)) {
              continue;
            }

            if (
              !same_map_context(
                candidate.location.map,
                entry.second.location.map))
            {
              continue;
            }

            if (
              records_have_identity_overlap(
                candidate,
                entry.second))
            {
              if (
                conflicting_location_id !=
                nullptr)
              {
                *conflicting_location_id =
                  entry.first;
              }

              return true;
            }
          }

          return false;
        }


        bool
        InMemoryRegistry::has_tag_collision_locked(
          const LocationRecordData & candidate,
          const std::string_view excluded_location_id,
          std::string * conflicting_location_id) const
        {
          if (!reserves_identity(candidate)) {
            return false;
          }

          for (const auto & entry : records_) {
            if (
              entry.first ==
              excluded_location_id)
            {
              continue;
            }

            if (!reserves_identity(entry.second)) {
              continue;
            }

            if (
              !same_map_context(
                candidate.location.map,
                entry.second.location.map))
            {
              continue;
            }

            const bool same_tag =
              trim_ascii(
                candidate.location.tag.family) ==
              trim_ascii(
                entry.second.location.tag.family) &&
              candidate.location.tag.id ==
              entry.second.location.tag.id;

            if (same_tag) {
              if (
                conflicting_location_id !=
                nullptr)
              {
                *conflicting_location_id =
                  entry.first;
              }

              return true;
            }
          }

          return false;
        }

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # Registry tests
    # -------------------------------------------------------------------------

    write(
        "test/unit/test_registry.cpp",
        r'''
        #include <gtest/gtest.h>

        #include <cstdint>
        #include <string>
        #include <vector>

        #include "savo_locations/registry.hpp"


        namespace
        {

        savo_locations::LocationRecordData make_record(
          const std::string & location_id,
          const std::string & display_name,
          const std::string & map_id,
          const std::uint32_t map_revision,
          const std::int32_t tag_id,
          const std::vector<std::string> & aliases = {})
        {
          savo_locations::LocationRecordData record;

          record.state =
            savo_locations::LocationState::kApproved;

          record.enabled = true;
          record.record_revision = 1U;

          record.location.location_id =
            location_id;

          record.location.display_name =
            display_name;

          record.location.aliases = aliases;

          record.location.semantic_type = "room";

          record.location.map.map_id = map_id;

          record.location.map.map_revision =
            map_revision;

          record.location.approach_pose.frame_id =
            "map";

          record.location.approach_pose.qw = 1.0;

          record.location.tag.family = "tag36h11";
          record.location.tag.id = tag_id;

          return record;
        }

        }  // namespace


        TEST(LocationRegistry, InsertsAndGetsRecord)
        {
          savo_locations::InMemoryRegistry registry;

          const auto inserted =
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27,
                {"A 201"}));

          ASSERT_TRUE(inserted.success);

          EXPECT_EQ(
            inserted.code,
            savo_locations::MutationCode::kInserted);

          EXPECT_EQ(registry.size(), 1U);

          const auto stored = registry.get("a201");

          ASSERT_TRUE(stored.has_value());

          EXPECT_EQ(
            stored->location.location_id,
            "A201");
        }


        TEST(LocationRegistry, ResolvesExactId)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27,
                {"A 201"})).success);

          const auto result =
            registry.resolve("a201");

          ASSERT_TRUE(result.resolved);

          EXPECT_EQ(
            result.code,
            savo_locations::ResolveCode::kResolved);

          EXPECT_EQ(
            result.match_type,
            savo_locations::MatchType::kLocationId);

          ASSERT_TRUE(result.record.has_value());

          EXPECT_EQ(
            result.record->location.location_id,
            "A201");
        }


        TEST(LocationRegistry, ResolvesDisplayNameAndAlias)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27,
                {
                  "Classroom A201",
                  "East classroom",
                })).success);

          const auto display =
            registry.resolve(" room-a201 ");

          ASSERT_TRUE(display.resolved);

          EXPECT_EQ(
            display.match_type,
            savo_locations::MatchType::kDisplayName);

          const auto alias =
            registry.resolve("east_classroom");

          ASSERT_TRUE(alias.resolved);

          EXPECT_EQ(
            alias.match_type,
            savo_locations::MatchType::kAlias);
        }


        TEST(LocationRegistry, RejectsSameMapIdentityConflict)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27,
                {"Student service"})).success);

          const auto conflict =
            registry.insert(
              make_record(
                "B101",
                "Room B101",
                "campus_main",
                7U,
                28,
                {"student-service"}));

          EXPECT_FALSE(conflict.success);

          EXPECT_EQ(
            conflict.code,
            savo_locations::MutationCode::
              kIdentityConflict);
        }


        TEST(LocationRegistry, RejectsSameMapTagConflict)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27)).success);

          const auto conflict =
            registry.insert(
              make_record(
                "B101",
                "Room B101",
                "campus_main",
                7U,
                27));

          EXPECT_FALSE(conflict.success);

          EXPECT_EQ(
            conflict.code,
            savo_locations::MutationCode::
              kTagConflict);
        }


        TEST(LocationRegistry, AllowsIdentityAcrossMaps)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Information Desk North",
                "campus_north",
                3U,
                10,
                {"Information desk"})).success);

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "B101",
                "Information Desk South",
                "campus_south",
                4U,
                10,
                {"Information desk"})).success);

          const auto ambiguous =
            registry.resolve("information desk");

          EXPECT_FALSE(ambiguous.resolved);

          EXPECT_EQ(
            ambiguous.code,
            savo_locations::ResolveCode::kAmbiguous);

          EXPECT_EQ(
            ambiguous.ambiguous_location_ids.size(),
            2U);

          savo_locations::ResolveOptions options;
          options.enforce_map_context = true;
          options.map.map_id = "campus_north";
          options.map.map_revision = 3U;

          const auto resolved =
            registry.resolve(
              "information desk",
              options);

          ASSERT_TRUE(resolved.resolved);
          ASSERT_TRUE(resolved.record.has_value());

          EXPECT_EQ(
            resolved.record->location.location_id,
            "A201");
        }


        TEST(LocationRegistry, ReportsMapMismatch)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27)).success);

          savo_locations::ResolveOptions options;
          options.enforce_map_context = true;
          options.map.map_id = "campus_other";
          options.map.map_revision = 1U;

          const auto result =
            registry.resolve("A201", options);

          EXPECT_FALSE(result.resolved);

          EXPECT_EQ(
            result.code,
            savo_locations::ResolveCode::kMapMismatch);
        }


        TEST(LocationRegistry, ReportsDisabledLocation)
        {
          savo_locations::InMemoryRegistry registry;

          auto record =
            make_record(
              "A201",
              "Room A201",
              "campus_main",
              7U,
              27);

          record.enabled = false;

          ASSERT_TRUE(
            registry.insert(record).success);

          const auto result =
            registry.resolve("A201");

          EXPECT_FALSE(result.resolved);

          EXPECT_EQ(
            result.code,
            savo_locations::ResolveCode::kDisabled);
        }


        TEST(LocationRegistry, ReportsRetiredLocation)
        {
          savo_locations::InMemoryRegistry registry;

          auto record =
            make_record(
              "A201",
              "Room A201",
              "campus_main",
              7U,
              27,
              {"Old classroom"});

          record.state =
            savo_locations::LocationState::kRetired;

          record.enabled = false;

          ASSERT_TRUE(
            registry.insert(record).success);

          const auto result =
            registry.resolve("old classroom");

          EXPECT_FALSE(result.resolved);

          EXPECT_EQ(
            result.code,
            savo_locations::ResolveCode::kRetired);
        }


        TEST(LocationRegistry, RejectsStaleReplacement)
        {
          savo_locations::InMemoryRegistry registry;

          auto original =
            make_record(
              "A201",
              "Room A201",
              "campus_main",
              7U,
              27);

          ASSERT_TRUE(
            registry.insert(original).success);

          auto replacement = original;
          replacement.record_revision = 2U;

          replacement.location.display_name =
            "Classroom A201";

          const auto stale =
            registry.replace(
              replacement,
              99U);

          EXPECT_FALSE(stale.success);

          EXPECT_EQ(
            stale.code,
            savo_locations::MutationCode::
              kStaleRevision);
        }


        TEST(LocationRegistry, EnforcesRevisionSequence)
        {
          savo_locations::InMemoryRegistry registry;

          auto original =
            make_record(
              "A201",
              "Room A201",
              "campus_main",
              7U,
              27);

          ASSERT_TRUE(
            registry.insert(original).success);

          auto replacement = original;
          replacement.record_revision = 3U;

          const auto invalid =
            registry.replace(
              replacement,
              1U);

          EXPECT_FALSE(invalid.success);

          EXPECT_EQ(
            invalid.code,
            savo_locations::MutationCode::
              kRevisionSequenceError);
        }


        TEST(LocationRegistry, ReplacesRecordSafely)
        {
          savo_locations::InMemoryRegistry registry;

          auto original =
            make_record(
              "A201",
              "Room A201",
              "campus_main",
              7U,
              27);

          ASSERT_TRUE(
            registry.insert(original).success);

          auto replacement = original;

          replacement.record_revision = 2U;

          replacement.location.display_name =
            "Classroom A201";

          replacement.location.aliases = {
            "A 201",
          };

          const auto updated =
            registry.replace(
              replacement,
              1U);

          ASSERT_TRUE(updated.success);

          EXPECT_EQ(
            updated.code,
            savo_locations::MutationCode::kUpdated);

          const auto resolved =
            registry.resolve("classroom a201");

          ASSERT_TRUE(resolved.resolved);
          ASSERT_TRUE(resolved.record.has_value());

          EXPECT_EQ(
            resolved.record->record_revision,
            2U);
        }


        TEST(LocationRegistry, SetEnabledIncrementsRevision)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27)).success);

          const auto disabled =
            registry.set_enabled(
              "a201",
              1U,
              false);

          ASSERT_TRUE(disabled.success);
          ASSERT_TRUE(disabled.record.has_value());

          EXPECT_FALSE(disabled.record->enabled);

          EXPECT_EQ(
            disabled.record->record_revision,
            2U);

          const auto resolved =
            registry.resolve("A201");

          EXPECT_EQ(
            resolved.code,
            savo_locations::ResolveCode::kDisabled);
        }


        TEST(LocationRegistry, ListIsDeterministic)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "C301",
                "Room C301",
                "campus_main",
                7U,
                31)).success);

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27)).success);

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "B101",
                "Room B101",
                "campus_main",
                7U,
                28)).success);

          const auto records = registry.list();

          ASSERT_EQ(records.size(), 3U);

          EXPECT_EQ(
            records[0].location.location_id,
            "A201");

          EXPECT_EQ(
            records[1].location.location_id,
            "B101");

          EXPECT_EQ(
            records[2].location.location_id,
            "C301");
        }


        TEST(LocationRegistry, ClearRemovesAllRecords)
        {
          savo_locations::InMemoryRegistry registry;

          ASSERT_TRUE(
            registry.insert(
              make_record(
                "A201",
                "Room A201",
                "campus_main",
                7U,
                27)).success);

          registry.clear();

          EXPECT_EQ(registry.size(), 0U);

          EXPECT_EQ(
            registry.resolve("A201").code,
            savo_locations::ResolveCode::kNotFound);
        }


        TEST(LocationRegistry, ResultStringsAreStable)
        {
          using savo_locations::MatchType;
          using savo_locations::MutationCode;
          using savo_locations::ResolveCode;
          using savo_locations::to_string;

          EXPECT_EQ(
            to_string(MutationCode::kTagConflict),
            "tag_conflict");

          EXPECT_EQ(
            to_string(ResolveCode::kAmbiguous),
            "ambiguous");

          EXPECT_EQ(
            to_string(MatchType::kAlias),
            "alias");
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # Keep LOC-1A contract test future-safe
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase1a_contracts.py",
        r'''
        from pathlib import Path
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[2]


        def read(relative: str) -> str:
            return (ROOT / relative).read_text(
                encoding="utf-8"
            )


        def parse_version(value: str) -> tuple[int, int, int]:
            major, minor, patch = value.split(".")

            return (
                int(major),
                int(minor),
                int(patch),
            )


        def test_package_contains_loc1a_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 2, 0)

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_domain_files_exist() -> None:
            for relative in (
                "include/savo_locations/model.hpp",
                "include/savo_locations/normalization.hpp",
                "include/savo_locations/validation.hpp",
                "src/normalization.cpp",
                "src/validation.cpp",
                "test/unit/test_normalization.cpp",
                "test/unit/test_validation.cpp",
            ):
                assert (ROOT / relative).is_file()


        def test_cmake_builds_loc1a_sources() -> None:
            cmake = read("CMakeLists.txt")

            assert "src/normalization.cpp" in cmake
            assert "src/validation.cpp" in cmake

            assert "test_location_normalization" in cmake
            assert "test_location_validation" in cmake
            assert "test_phase1a_contracts" in cmake


        def test_loc1a_core_remains_ros_independent() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(rclcpp" not in cmake
            assert "find_package(SQLite3" not in cmake
            assert "add_executable(" not in cmake

            assert not (
                ROOT
                / "src"
                / "location_registry_node.cpp"
            ).exists()


        def test_validation_contract_covers_safety_fields() -> None:
            validation = read(
                "include/savo_locations/validation.hpp"
            )

            implementation = read(
                "src/validation.cpp"
            )

            for required in (
                "kDuplicateNormalizedKey",
                "kNonFiniteNumber",
                "kMapRevisionZero",
                "kWrongFrame",
                "kInvalidQuaternion",
            ):
                assert required in validation

            assert "validate_location_draft" in validation
            assert "validate_candidate_draft" in validation

            assert "kCanonicalMapFrame" in implementation
            assert "kQuaternionNormTolerance" in implementation


        def test_normalization_is_locale_independent() -> None:
            implementation = read(
                "src/normalization.cpp"
            )

            assert "std::locale" not in implementation
            assert "std::tolower" not in implementation
            assert "std::toupper" not in implementation

            assert "normalize_lookup_key" in implementation
            assert "canonicalize_location_id" in implementation
            assert "is_canonical_location_id" in implementation
        ''',
    )

    # -------------------------------------------------------------------------
    # LOC-1B contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase1b_contracts.py",
        r'''
        from pathlib import Path
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[2]


        def read(relative: str) -> str:
            return (ROOT / relative).read_text(
                encoding="utf-8"
            )


        def parse_version(value: str) -> tuple[int, int, int]:
            major, minor, patch = value.split(".")

            return (
                int(major),
                int(minor),
                int(patch),
            )


        def test_package_contains_loc1b_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 3, 0)

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_registry_files_exist() -> None:
            for relative in (
                "include/savo_locations/registry.hpp",
                "src/registry.cpp",
                "test/unit/test_registry.cpp",
            ):
                assert (ROOT / relative).is_file()


        def test_cmake_builds_registry() -> None:
            cmake = read("CMakeLists.txt")

            assert "src/registry.cpp" in cmake
            assert "test_location_registry" in cmake
            assert "test_phase1b_contracts" in cmake


        def test_registry_supports_safe_resolution() -> None:
            header = read(
                "include/savo_locations/registry.hpp"
            )

            for required in (
                "kInvalidQuery",
                "kNotFound",
                "kAmbiguous",
                "kDisabled",
                "kRetired",
                "kMapMismatch",
                "kLocationId",
                "kDisplayName",
                "kAlias",
            ):
                assert required in header

            assert "enforce_map_context" in header
            assert "ambiguous_location_ids" in header


        def test_registry_supports_revision_guards() -> None:
            header = read(
                "include/savo_locations/registry.hpp"
            )

            implementation = read(
                "src/registry.cpp"
            )

            assert "expected_current_revision" in header
            assert "kStaleRevision" in header
            assert "kRevisionSequenceError" in header

            assert (
                "current revision plus one"
                in implementation
            )


        def test_registry_checks_identity_and_tag_conflicts() -> None:
            header = read(
                "include/savo_locations/registry.hpp"
            )

            implementation = read(
                "src/registry.cpp"
            )

            assert "kIdentityConflict" in header
            assert "kTagConflict" in header

            assert (
                "has_identity_collision_locked"
                in implementation
            )

            assert (
                "has_tag_collision_locked"
                in implementation
            )


        def test_loc1b_has_no_ros_or_sqlite_runtime() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(rclcpp" not in cmake
            assert "find_package(SQLite3" not in cmake
            assert "add_executable(" not in cmake

            assert not (ROOT / "launch").exists()
            assert not (ROOT / "database").exists()
        ''',
    )

    # -------------------------------------------------------------------------
    # Policy
    # -------------------------------------------------------------------------

    policy_path = (
        ROOT
        / "config"
        / "location_policy.yaml"
    )

    policy = policy_path.read_text(encoding="utf-8")

    if "\nin_memory_registry:\n" not in policy:
        policy += clean(
            r'''

            in_memory_registry:
              deterministic_order: location_id
              location_id_unique_globally: true
              identity_unique_within_map_revision: true
              apriltag_unique_within_map_revision: true

              disabled_locations_reserve_identity: true
              retired_locations_release_identity: true

              exact_location_id_match_preferred: true
              map_context_required_when_requested: true
              ambiguous_resolution_is_failure: true

              replacement_requires_expected_revision: true
              replacement_revision_increment: 1
              enable_disable_increments_revision: true
            '''
        )

        policy_path.write_text(
            policy,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # README
    # -------------------------------------------------------------------------

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(encoding="utf-8")

    if "## LOC-1B in-memory registry" not in readme:
        readme += clean(
            r'''

            ## LOC-1B in-memory registry

            LOC-1B adds a deterministic, thread-safe, ROS-independent
            in-memory location registry.

            The registry supports:

            - canonical location-ID insertion;
            - globally unique location IDs;
            - identity collision protection within a map revision;
            - AprilTag collision protection within a map revision;
            - exact ID, display-name and alias resolution;
            - optional map-context enforcement;
            - explicit ambiguous, disabled, retired and map-mismatch results;
            - optimistic record revision checks;
            - strict revision increment sequencing;
            - revisioned enable/disable changes;
            - deterministic location-ID-sorted listing.

            A disabled location continues to reserve its identity and tag.
            A retired location releases its identity and tag for future use.

            A matching identity across different maps is allowed. Resolution
            without map context must return ambiguous rather than selecting
            one destination silently.

            LOC-1B still does not add SQLite, a ROS node, ROS services,
            launch files, mapping integration or navigation integration.
            '''
        )

        readme_path.write_text(
            readme,
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
        / f"LOC1B_savo_locations_{stamp}.sha256"
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
        "LOC-1B deterministic in-memory registry applied."
    )

    print(
        "No SQLite, ROS node or hardware runtime package was added."
    )


if __name__ == "__main__":
    main()
