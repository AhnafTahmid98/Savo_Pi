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
        / f"pre_LOC1C_savo_locations_{stamp}.tar.gz"
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

    version_element.text = "0.4.0"

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
        project(savo_locations VERSION 0.4.0 LANGUAGES CXX)

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
        '"0.3.0"',
        '"0.4.0"',
    )

    constants_path.write_text(
        constants,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # Domain models
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


        struct CandidateRecordData
        {
          CandidateState state{
            CandidateState::kPendingReview};

          std::uint64_t candidate_revision{1U};

          CandidateDraft candidate;

          std::string review_reason;
          std::string approved_location_id;
        };


        struct ApprovalRequest
        {
          std::string candidate_id;
          std::uint64_t expected_candidate_revision{0U};

          std::string location_id;
          std::string display_name;
          std::vector<std::string> aliases;
          std::string semantic_type;

          std::optional<PoseData> approach_pose;
          std::optional<PoseData> confirmation_pose;

          bool arrival_confirmation_required{true};

          std::string building;
          std::string floor;
          std::string area;
          std::string notes;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__MODEL_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Location catalog API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/location_catalog.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__LOCATION_CATALOG_HPP_
        #define SAVO_LOCATIONS__LOCATION_CATALOG_HPP_

        #include <cstddef>
        #include <cstdint>
        #include <map>
        #include <optional>
        #include <shared_mutex>
        #include <string>
        #include <string_view>
        #include <vector>

        #include "savo_locations/model.hpp"
        #include "savo_locations/registry.hpp"
        #include "savo_locations/validation.hpp"

        namespace savo_locations
        {

        enum class CandidateMutationCode : std::uint8_t
        {
          kRegistered = 0U,
          kUpdated,
          kRejected,
          kNotFound,
          kInvalidCandidate,
          kStaleRevision,
          kRevisionSequenceError,
          kCandidateIdConflict,
          kTagConflict,
          kNotPending,
          kEmptyReviewReason,
        };


        enum class ApprovalCode : std::uint8_t
        {
          kApproved = 0U,
          kInvalidRequest,
          kCandidateNotFound,
          kCandidateNotPending,
          kStaleRevision,
          kRevisionSequenceError,
          kMissingApproachPose,
          kInvalidLocation,
          kLocationConflict,
          kLocationMutationFailed,
        };


        struct CandidateMutationResult
        {
          bool success{false};

          CandidateMutationCode code{
            CandidateMutationCode::kInvalidCandidate};

          std::string reason;

          std::vector<ValidationIssue> validation_issues;

          std::optional<CandidateRecordData> candidate;
        };


        struct ApprovalResult
        {
          bool success{false};

          ApprovalCode code{
            ApprovalCode::kInvalidRequest};

          std::string reason;

          std::vector<ValidationIssue> validation_issues;

          MutationCode location_mutation_code{
            MutationCode::kInvalidRecord};

          std::optional<CandidateRecordData> candidate;
          std::optional<LocationRecordData> location;
        };


        [[nodiscard]]
        std::string_view to_string(
          CandidateMutationCode code) noexcept;

        [[nodiscard]]
        std::string_view to_string(
          ApprovalCode code) noexcept;


        class InMemoryLocationCatalog
        {
        public:
          [[nodiscard]]
          RegistryMutationResult insert_location(
            LocationRecordData record);

          [[nodiscard]]
          RegistryMutationResult replace_location(
            LocationRecordData record,
            std::uint64_t expected_current_revision);

          [[nodiscard]]
          RegistryMutationResult set_location_enabled(
            std::string_view location_id,
            std::uint64_t expected_current_revision,
            bool enabled);

          [[nodiscard]]
          ResolveResult resolve_location(
            std::string_view query,
            const ResolveOptions & options =
              ResolveOptions{}) const;

          [[nodiscard]]
          std::optional<LocationRecordData> get_location(
            std::string_view location_id) const;

          [[nodiscard]]
          std::vector<LocationRecordData>
          list_locations() const;

          [[nodiscard]]
          CandidateMutationResult register_candidate(
            CandidateDraft candidate);

          [[nodiscard]]
          CandidateMutationResult replace_candidate(
            std::string_view candidate_id,
            CandidateDraft replacement,
            std::uint64_t expected_current_revision);

          [[nodiscard]]
          CandidateMutationResult reject_candidate(
            std::string_view candidate_id,
            std::uint64_t expected_current_revision,
            std::string review_reason);

          [[nodiscard]]
          ApprovalResult approve_candidate(
            const ApprovalRequest & request);

          [[nodiscard]]
          std::optional<CandidateRecordData>
          get_candidate(
            std::string_view candidate_id) const;

          [[nodiscard]]
          std::vector<CandidateRecordData>
          list_candidates() const;

          [[nodiscard]]
          std::size_t location_size() const;

          [[nodiscard]]
          std::size_t candidate_size() const;

          void clear();

        private:
          [[nodiscard]]
          bool candidate_tag_conflict_locked(
            const CandidateDraft & candidate,
            std::string_view excluded_candidate_id,
            std::string * conflicting_entity) const;

          [[nodiscard]]
          bool candidate_tag_conflicts_with_location_locked(
            const CandidateDraft & candidate,
            std::string * conflicting_location_id) const;

          mutable std::shared_mutex mutex_;

          InMemoryRegistry locations_;

          std::map<
            std::string,
            CandidateRecordData,
            std::less<>>
          candidates_;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__LOCATION_CATALOG_HPP_
        ''',
    )

    # -------------------------------------------------------------------------
    # Location catalog implementation
    # -------------------------------------------------------------------------

    write(
        "src/location_catalog.cpp",
        r'''
        #include "savo_locations/location_catalog.hpp"

        #include <limits>
        #include <mutex>
        #include <utility>

        #include "savo_locations/normalization.hpp"

        namespace savo_locations
        {
        namespace
        {

        bool same_map_context(
          const MapContext & left,
          const MapContext & right) noexcept
        {
          return
            left.map_id == right.map_id &&
            left.map_revision == right.map_revision;
        }


        bool same_tag(
          const TagBinding & left,
          const TagBinding & right)
        {
          return
            normalize_lookup_key(left.family) ==
              normalize_lookup_key(right.family) &&
            left.id == right.id;
        }


        std::string candidate_key(
          const std::string_view candidate_id)
        {
          return trim_ascii(candidate_id);
        }


        CandidateMutationResult candidate_failure(
          const CandidateMutationCode code,
          std::string reason)
        {
          CandidateMutationResult result;
          result.success = false;
          result.code = code;
          result.reason = std::move(reason);
          return result;
        }


        CandidateMutationResult candidate_validation_failure(
          const ValidationResult & validation)
        {
          CandidateMutationResult result;
          result.success = false;

          result.code =
            CandidateMutationCode::kInvalidCandidate;

          result.reason =
            "candidate validation failed";

          result.validation_issues =
            validation.issues();

          return result;
        }


        CandidateMutationResult candidate_success(
          const CandidateMutationCode code,
          std::string reason,
          const CandidateRecordData & candidate)
        {
          CandidateMutationResult result;
          result.success = true;
          result.code = code;
          result.reason = std::move(reason);
          result.candidate = candidate;
          return result;
        }


        ApprovalResult approval_failure(
          const ApprovalCode code,
          std::string reason)
        {
          ApprovalResult result;
          result.success = false;
          result.code = code;
          result.reason = std::move(reason);
          return result;
        }


        ApprovalResult approval_validation_failure(
          const ValidationResult & validation)
        {
          ApprovalResult result;
          result.success = false;
          result.code = ApprovalCode::kInvalidLocation;

          result.reason =
            "approved location validation failed";

          result.validation_issues =
            validation.issues();

          return result;
        }


        std::string choose_text(
          const std::string & requested,
          const std::string & suggested)
        {
          const auto cleaned =
            collapse_ascii_whitespace(requested);

          if (!cleaned.empty()) {
            return cleaned;
          }

          return collapse_ascii_whitespace(
            suggested);
        }


        std::vector<std::string> choose_aliases(
          const std::vector<std::string> & requested,
          const std::vector<std::string> & suggested)
        {
          const auto & source =
            requested.empty() ?
            suggested :
            requested;

          std::vector<std::string> aliases;
          aliases.reserve(source.size());

          for (const auto & alias : source) {
            aliases.push_back(
              collapse_ascii_whitespace(alias));
          }

          return aliases;
        }

        }  // namespace


        std::string_view to_string(
          const CandidateMutationCode code) noexcept
        {
          switch (code) {
            case CandidateMutationCode::kRegistered:
              return "registered";

            case CandidateMutationCode::kUpdated:
              return "updated";

            case CandidateMutationCode::kRejected:
              return "rejected";

            case CandidateMutationCode::kNotFound:
              return "not_found";

            case CandidateMutationCode::kInvalidCandidate:
              return "invalid_candidate";

            case CandidateMutationCode::kStaleRevision:
              return "stale_revision";

            case CandidateMutationCode::
              kRevisionSequenceError:
              return "revision_sequence_error";

            case CandidateMutationCode::
              kCandidateIdConflict:
              return "candidate_id_conflict";

            case CandidateMutationCode::kTagConflict:
              return "tag_conflict";

            case CandidateMutationCode::kNotPending:
              return "not_pending";

            case CandidateMutationCode::
              kEmptyReviewReason:
              return "empty_review_reason";

            default:
              return "unknown";
          }
        }


        std::string_view to_string(
          const ApprovalCode code) noexcept
        {
          switch (code) {
            case ApprovalCode::kApproved:
              return "approved";

            case ApprovalCode::kInvalidRequest:
              return "invalid_request";

            case ApprovalCode::kCandidateNotFound:
              return "candidate_not_found";

            case ApprovalCode::kCandidateNotPending:
              return "candidate_not_pending";

            case ApprovalCode::kStaleRevision:
              return "stale_revision";

            case ApprovalCode::kRevisionSequenceError:
              return "revision_sequence_error";

            case ApprovalCode::kMissingApproachPose:
              return "missing_approach_pose";

            case ApprovalCode::kInvalidLocation:
              return "invalid_location";

            case ApprovalCode::kLocationConflict:
              return "location_conflict";

            case ApprovalCode::kLocationMutationFailed:
              return "location_mutation_failed";

            default:
              return "unknown";
          }
        }


        RegistryMutationResult
        InMemoryLocationCatalog::insert_location(
          LocationRecordData record)
        {
          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.insert(std::move(record));
        }


        RegistryMutationResult
        InMemoryLocationCatalog::replace_location(
          LocationRecordData record,
          const std::uint64_t expected_current_revision)
        {
          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.replace(
            std::move(record),
            expected_current_revision);
        }


        RegistryMutationResult
        InMemoryLocationCatalog::set_location_enabled(
          const std::string_view location_id,
          const std::uint64_t expected_current_revision,
          const bool enabled)
        {
          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.set_enabled(
            location_id,
            expected_current_revision,
            enabled);
        }


        ResolveResult
        InMemoryLocationCatalog::resolve_location(
          const std::string_view query,
          const ResolveOptions & options) const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.resolve(query, options);
        }


        std::optional<LocationRecordData>
        InMemoryLocationCatalog::get_location(
          const std::string_view location_id) const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.get(location_id);
        }


        std::vector<LocationRecordData>
        InMemoryLocationCatalog::list_locations() const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.list();
        }


        CandidateMutationResult
        InMemoryLocationCatalog::register_candidate(
          CandidateDraft candidate)
        {
          candidate.candidate_id =
            candidate_key(candidate.candidate_id);

          const auto validation =
            validate_candidate_draft(candidate);

          if (!validation.valid()) {
            return candidate_validation_failure(
              validation);
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          if (
            candidates_.find(candidate.candidate_id) !=
            candidates_.end())
          {
            return candidate_failure(
              CandidateMutationCode::
                kCandidateIdConflict,
              "candidate ID already exists");
          }

          std::string conflict;

          if (
            candidate_tag_conflict_locked(
              candidate,
              "",
              &conflict))
          {
            return candidate_failure(
              CandidateMutationCode::kTagConflict,
              "candidate AprilTag conflicts with " +
              conflict);
          }

          if (
            candidate_tag_conflicts_with_location_locked(
              candidate,
              &conflict))
          {
            return candidate_failure(
              CandidateMutationCode::kTagConflict,
              "candidate AprilTag conflicts with location " +
              conflict);
          }

          CandidateRecordData record;
          record.state =
            CandidateState::kPendingReview;

          record.candidate_revision = 1U;
          record.candidate = std::move(candidate);

          const auto key =
            record.candidate.candidate_id;

          candidates_.emplace(key, record);

          return candidate_success(
            CandidateMutationCode::kRegistered,
            "candidate registered",
            record);
        }


        CandidateMutationResult
        InMemoryLocationCatalog::replace_candidate(
          const std::string_view candidate_id,
          CandidateDraft replacement,
          const std::uint64_t expected_current_revision)
        {
          const auto key = candidate_key(candidate_id);

          replacement.candidate_id = key;

          const auto validation =
            validate_candidate_draft(replacement);

          if (!validation.valid()) {
            return candidate_validation_failure(
              validation);
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          const auto existing =
            candidates_.find(key);

          if (existing == candidates_.end()) {
            return candidate_failure(
              CandidateMutationCode::kNotFound,
              "candidate does not exist");
          }

          if (
            existing->second.state !=
            CandidateState::kPendingReview)
          {
            return candidate_failure(
              CandidateMutationCode::kNotPending,
              "candidate is no longer pending");
          }

          if (
            existing->second.candidate_revision !=
            expected_current_revision)
          {
            return candidate_failure(
              CandidateMutationCode::kStaleRevision,
              "expected candidate revision does not match");
          }

          if (
            existing->second.candidate_revision ==
            std::numeric_limits<
              std::uint64_t>::max())
          {
            return candidate_failure(
              CandidateMutationCode::
                kRevisionSequenceError,
              "candidate revision cannot be incremented");
          }

          std::string conflict;

          if (
            candidate_tag_conflict_locked(
              replacement,
              key,
              &conflict))
          {
            return candidate_failure(
              CandidateMutationCode::kTagConflict,
              "candidate AprilTag conflicts with " +
              conflict);
          }

          if (
            candidate_tag_conflicts_with_location_locked(
              replacement,
              &conflict))
          {
            return candidate_failure(
              CandidateMutationCode::kTagConflict,
              "candidate AprilTag conflicts with location " +
              conflict);
          }

          auto updated = existing->second;
          updated.candidate = std::move(replacement);

          updated.candidate_revision +=
            std::uint64_t{1U};

          existing->second = updated;

          return candidate_success(
            CandidateMutationCode::kUpdated,
            "candidate updated",
            updated);
        }


        CandidateMutationResult
        InMemoryLocationCatalog::reject_candidate(
          const std::string_view candidate_id,
          const std::uint64_t expected_current_revision,
          std::string review_reason)
        {
          const auto key = candidate_key(candidate_id);

          review_reason =
            collapse_ascii_whitespace(review_reason);

          if (review_reason.empty()) {
            return candidate_failure(
              CandidateMutationCode::
                kEmptyReviewReason,
              "rejection reason is required");
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          const auto existing =
            candidates_.find(key);

          if (existing == candidates_.end()) {
            return candidate_failure(
              CandidateMutationCode::kNotFound,
              "candidate does not exist");
          }

          if (
            existing->second.state !=
            CandidateState::kPendingReview)
          {
            return candidate_failure(
              CandidateMutationCode::kNotPending,
              "candidate is no longer pending");
          }

          if (
            existing->second.candidate_revision !=
            expected_current_revision)
          {
            return candidate_failure(
              CandidateMutationCode::kStaleRevision,
              "expected candidate revision does not match");
          }

          if (
            existing->second.candidate_revision ==
            std::numeric_limits<
              std::uint64_t>::max())
          {
            return candidate_failure(
              CandidateMutationCode::
                kRevisionSequenceError,
              "candidate revision cannot be incremented");
          }

          auto rejected = existing->second;

          rejected.state =
            CandidateState::kRejected;

          rejected.candidate_revision +=
            std::uint64_t{1U};

          rejected.review_reason =
            std::move(review_reason);

          existing->second = rejected;

          return candidate_success(
            CandidateMutationCode::kRejected,
            "candidate rejected",
            rejected);
        }


        ApprovalResult
        InMemoryLocationCatalog::approve_candidate(
          const ApprovalRequest & request)
        {
          const auto key =
            candidate_key(request.candidate_id);

          if (key.empty()) {
            return approval_failure(
              ApprovalCode::kInvalidRequest,
              "candidate ID is required");
          }

          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          const auto existing =
            candidates_.find(key);

          if (existing == candidates_.end()) {
            return approval_failure(
              ApprovalCode::kCandidateNotFound,
              "candidate does not exist");
          }

          if (
            existing->second.state !=
            CandidateState::kPendingReview)
          {
            return approval_failure(
              ApprovalCode::kCandidateNotPending,
              "candidate is no longer pending");
          }

          if (
            existing->second.candidate_revision !=
            request.expected_candidate_revision)
          {
            return approval_failure(
              ApprovalCode::kStaleRevision,
              "expected candidate revision does not match");
          }

          if (
            existing->second.candidate_revision ==
            std::numeric_limits<
              std::uint64_t>::max())
          {
            return approval_failure(
              ApprovalCode::kRevisionSequenceError,
              "candidate revision cannot be incremented");
          }

          const auto & source =
            existing->second.candidate;

          const auto approach_pose =
            request.approach_pose.has_value() ?
            request.approach_pose :
            source.approach_pose;

          if (!approach_pose.has_value()) {
            return approval_failure(
              ApprovalCode::kMissingApproachPose,
              "approval requires a safe approach pose");
          }

          LocationDraft location;

          location.location_id =
            canonicalize_location_id(
              choose_text(
                request.location_id,
                source.suggested_location_id));

          location.display_name =
            choose_text(
              request.display_name,
              source.suggested_display_name);

          location.aliases =
            choose_aliases(
              request.aliases,
              source.suggested_aliases);

          location.semantic_type =
            choose_text(
              request.semantic_type,
              source.suggested_semantic_type);

          #pragma GCC diagnostic push
          #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
          location.approach_pose =
            approach_pose.value();
          #pragma GCC diagnostic pop

          location.confirmation_pose =
            request.confirmation_pose.has_value() ?
            request.confirmation_pose :
            source.confirmation_pose;

          location.tag_pose_map =
            source.tag_pose_map;

          location.map = source.map;
          location.tag = source.tag;

          location.arrival_confirmation_required =
            request.arrival_confirmation_required;

          location.building =
            choose_text(
              request.building,
              source.building);

          location.floor =
            choose_text(
              request.floor,
              source.floor);

          location.area =
            choose_text(
              request.area,
              source.area);

          location.notes =
            choose_text(
              request.notes,
              source.notes);

          const auto validation =
            validate_location_draft(location);

          if (!validation.valid()) {
            return approval_validation_failure(
              validation);
          }

          LocationRecordData location_record;

          location_record.state =
            LocationState::kApproved;

          location_record.enabled = true;
          location_record.record_revision = 1U;

          location_record.location =
            std::move(location);

          location_record.source_candidate_id =
            key;

          auto approved_candidate =
            existing->second;

          approved_candidate.state =
            CandidateState::kApproved;

          approved_candidate.candidate_revision +=
            std::uint64_t{1U};

          approved_candidate.review_reason =
            "approved";

          approved_candidate.approved_location_id =
            location_record.location.location_id;

          const auto location_result =
            locations_.insert(location_record);

          if (!location_result.success) {
            ApprovalResult result;
            result.success = false;
            result.location_mutation_code =
              location_result.code;

            result.reason =
              location_result.reason;

            result.validation_issues =
              location_result.validation_issues;

            switch (location_result.code) {
              case MutationCode::kInvalidRecord:
                result.code =
                  ApprovalCode::kInvalidLocation;
                break;

              case MutationCode::
                  kLocationIdConflict:
              case MutationCode::kIdentityConflict:
              case MutationCode::kTagConflict:
                result.code =
                  ApprovalCode::kLocationConflict;
                break;

              default:
                result.code =
                  ApprovalCode::
                    kLocationMutationFailed;
                break;
            }

            return result;
          }

          existing->second =
            approved_candidate;

          ApprovalResult result;
          result.success = true;
          result.code = ApprovalCode::kApproved;
          result.reason = "candidate approved";

          result.location_mutation_code =
            MutationCode::kInserted;

          result.candidate =
            approved_candidate;

          result.location =
            location_result.record;

          return result;
        }


        std::optional<CandidateRecordData>
        InMemoryLocationCatalog::get_candidate(
          const std::string_view candidate_id) const
        {
          const auto key = candidate_key(candidate_id);

          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          const auto candidate =
            candidates_.find(key);

          if (candidate == candidates_.end()) {
            return std::nullopt;
          }

          return candidate->second;
        }


        std::vector<CandidateRecordData>
        InMemoryLocationCatalog::list_candidates() const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          std::vector<CandidateRecordData> output;
          output.reserve(candidates_.size());

          for (const auto & entry : candidates_) {
            output.push_back(entry.second);
          }

          return output;
        }


        std::size_t
        InMemoryLocationCatalog::location_size() const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return locations_.size();
        }


        std::size_t
        InMemoryLocationCatalog::candidate_size() const
        {
          std::shared_lock<std::shared_mutex> lock{
            mutex_};

          return candidates_.size();
        }


        void InMemoryLocationCatalog::clear()
        {
          std::unique_lock<std::shared_mutex> lock{
            mutex_};

          locations_.clear();
          candidates_.clear();
        }


        bool
        InMemoryLocationCatalog::
        candidate_tag_conflict_locked(
          const CandidateDraft & candidate,
          const std::string_view excluded_candidate_id,
          std::string * conflicting_entity) const
        {
          for (const auto & entry : candidates_) {
            if (entry.first == excluded_candidate_id) {
              continue;
            }

            if (
              entry.second.state !=
              CandidateState::kPendingReview)
            {
              continue;
            }

            if (
              !same_map_context(
                candidate.map,
                entry.second.candidate.map))
            {
              continue;
            }

            if (
              same_tag(
                candidate.tag,
                entry.second.candidate.tag))
            {
              if (conflicting_entity != nullptr) {
                *conflicting_entity = entry.first;
              }

              return true;
            }
          }

          return false;
        }


        bool
        InMemoryLocationCatalog::
        candidate_tag_conflicts_with_location_locked(
          const CandidateDraft & candidate,
          std::string * conflicting_location_id) const
        {
          const auto locations = locations_.list();

          for (const auto & location : locations) {
            if (
              location.state ==
              LocationState::kRetired)
            {
              continue;
            }

            if (
              !same_map_context(
                candidate.map,
                location.location.map))
            {
              continue;
            }

            if (
              same_tag(
                candidate.tag,
                location.location.tag))
            {
              if (
                conflicting_location_id !=
                nullptr)
              {
                *conflicting_location_id =
                  location.location.location_id;
              }

              return true;
            }
          }

          return false;
        }

        }  // namespace savo_locations
        ''',
    )

    # Remove compiler-specific pragma from the implementation. The optional
    # value is checked immediately before value(), so no suppression is needed.
    implementation_path = (
        ROOT
        / "src"
        / "location_catalog.cpp"
    )

    implementation = implementation_path.read_text(
        encoding="utf-8"
    )

    implementation = implementation.replace(
        '''          #pragma GCC diagnostic push
          #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
          location.approach_pose =
            approach_pose.value();
          #pragma GCC diagnostic pop
''',
        '''          location.approach_pose =
            approach_pose.value();
''',
    )

    implementation_path.write_text(
        implementation,
        encoding="utf-8",
    )

    # -------------------------------------------------------------------------
    # LOC-1C unit tests
    # -------------------------------------------------------------------------

    write(
        "test/unit/test_location_catalog.cpp",
        r'''
        #include <gtest/gtest.h>

        #include <cstdint>
        #include <string>
        #include <vector>

        #include "savo_locations/location_catalog.hpp"


        namespace
        {

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


        savo_locations::CandidateDraft make_candidate(
          const std::string & candidate_id,
          const std::string & map_id,
          const std::uint32_t map_revision,
          const std::int32_t tag_id,
          const std::string & suggested_location_id)
        {
          savo_locations::CandidateDraft candidate;

          candidate.candidate_id = candidate_id;

          candidate.map.map_id = map_id;
          candidate.map.map_revision = map_revision;

          candidate.tag.family = "tag36h11";
          candidate.tag.id = tag_id;

          candidate.tag_pose_map =
            make_pose(12.8, 8.1);

          candidate.detection_quality = 0.95;
          candidate.accepted_observations = 8U;
          candidate.position_stddev_m = 0.02;
          candidate.yaw_stddev_rad = 0.03;

          candidate.suggested_location_id =
            suggested_location_id;

          candidate.suggested_display_name =
            "Room " + suggested_location_id;

          candidate.suggested_aliases = {
            suggested_location_id.substr(0U, 1U) +
            " " +
            suggested_location_id.substr(1U),
          };

          candidate.suggested_semantic_type =
            "classroom";

          return candidate;
        }


        savo_locations::LocationRecordData make_location(
          const std::string & location_id,
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
            "Room " + location_id;

          record.location.aliases = aliases;
          record.location.semantic_type = "classroom";

          record.location.map.map_id = map_id;

          record.location.map.map_revision =
            map_revision;

          record.location.approach_pose =
            make_pose(10.0, 5.0);

          record.location.tag.family = "tag36h11";
          record.location.tag.id = tag_id;

          return record;
        }


        savo_locations::ApprovalRequest
        make_approval(
          const std::string & candidate_id,
          const std::uint64_t revision)
        {
          savo_locations::ApprovalRequest request;

          request.candidate_id = candidate_id;

          request.expected_candidate_revision =
            revision;

          request.approach_pose =
            make_pose(12.0, 7.5);

          return request;
        }

        }  // namespace


        TEST(LocationCatalog, RegistersCandidate)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          const auto result =
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201"));

          ASSERT_TRUE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::CandidateMutationCode::
              kRegistered);

          ASSERT_TRUE(result.candidate.has_value());

          EXPECT_EQ(
            result.candidate->state,
            savo_locations::CandidateState::
              kPendingReview);

          EXPECT_EQ(
            result.candidate->candidate_revision,
            1U);

          EXPECT_EQ(catalog.candidate_size(), 1U);
        }


        TEST(LocationCatalog, RejectsDuplicateCandidateId)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto duplicate =
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                28,
                "A202"));

          EXPECT_FALSE(duplicate.success);

          EXPECT_EQ(
            duplicate.code,
            savo_locations::CandidateMutationCode::
              kCandidateIdConflict);
        }


        TEST(LocationCatalog, RejectsPendingTagConflict)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27-a",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto conflict =
            catalog.register_candidate(
              make_candidate(
                "candidate-27-b",
                "campus_main",
                7U,
                27,
                "A202"));

          EXPECT_FALSE(conflict.success);

          EXPECT_EQ(
            conflict.code,
            savo_locations::CandidateMutationCode::
              kTagConflict);
        }


        TEST(LocationCatalog, AllowsSameTagAcrossMaps)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "north-candidate",
                "campus_north",
                3U,
                27,
                "N201")).success);

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "south-candidate",
                "campus_south",
                4U,
                27,
                "S201")).success);

          EXPECT_EQ(catalog.candidate_size(), 2U);
        }


        TEST(LocationCatalog, RejectsTagUsedByLocation)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.insert_location(
              make_location(
                "A201",
                "campus_main",
                7U,
                27)).success);

          const auto candidate =
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A202"));

          EXPECT_FALSE(candidate.success);

          EXPECT_EQ(
            candidate.code,
            savo_locations::CandidateMutationCode::
              kTagConflict);
        }


        TEST(LocationCatalog, ReplacesPendingCandidate)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          auto candidate =
            make_candidate(
              "candidate-27",
              "campus_main",
              7U,
              27,
              "A201");

          ASSERT_TRUE(
            catalog.register_candidate(
              candidate).success);

          candidate.detection_quality = 0.99;
          candidate.accepted_observations = 12U;

          const auto updated =
            catalog.replace_candidate(
              "candidate-27",
              candidate,
              1U);

          ASSERT_TRUE(updated.success);
          ASSERT_TRUE(updated.candidate.has_value());

          EXPECT_EQ(
            updated.candidate->candidate_revision,
            2U);

          EXPECT_DOUBLE_EQ(
            updated.candidate
              ->candidate
              .detection_quality,
            0.99);
        }


        TEST(LocationCatalog, RejectsStaleCandidateUpdate)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          auto candidate =
            make_candidate(
              "candidate-27",
              "campus_main",
              7U,
              27,
              "A201");

          ASSERT_TRUE(
            catalog.register_candidate(
              candidate).success);

          const auto stale =
            catalog.replace_candidate(
              "candidate-27",
              candidate,
              99U);

          EXPECT_FALSE(stale.success);

          EXPECT_EQ(
            stale.code,
            savo_locations::CandidateMutationCode::
              kStaleRevision);
        }


        TEST(LocationCatalog, RejectsCandidateWithReason)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto rejected =
            catalog.reject_candidate(
              "candidate-27",
              1U,
              "  approach pose is blocked  ");

          ASSERT_TRUE(rejected.success);
          ASSERT_TRUE(rejected.candidate.has_value());

          EXPECT_EQ(
            rejected.candidate->state,
            savo_locations::CandidateState::
              kRejected);

          EXPECT_EQ(
            rejected.candidate->candidate_revision,
            2U);

          EXPECT_EQ(
            rejected.candidate->review_reason,
            "approach pose is blocked");
        }


        TEST(LocationCatalog, RequiresRejectionReason)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto rejected =
            catalog.reject_candidate(
              "candidate-27",
              1U,
              "   ");

          EXPECT_FALSE(rejected.success);

          EXPECT_EQ(
            rejected.code,
            savo_locations::CandidateMutationCode::
              kEmptyReviewReason);
        }


        TEST(LocationCatalog, CannotApproveRejectedCandidate)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          ASSERT_TRUE(
            catalog.reject_candidate(
              "candidate-27",
              1U,
              "invalid landmark").success);

          const auto approval =
            catalog.approve_candidate(
              make_approval(
                "candidate-27",
                2U));

          EXPECT_FALSE(approval.success);

          EXPECT_EQ(
            approval.code,
            savo_locations::ApprovalCode::
              kCandidateNotPending);
        }


        TEST(LocationCatalog, ApprovesCandidateAtomically)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto approval =
            catalog.approve_candidate(
              make_approval(
                "candidate-27",
                1U));

          ASSERT_TRUE(approval.success);
          ASSERT_TRUE(approval.candidate.has_value());
          ASSERT_TRUE(approval.location.has_value());

          EXPECT_EQ(
            approval.code,
            savo_locations::ApprovalCode::
              kApproved);

          EXPECT_EQ(
            approval.candidate->state,
            savo_locations::CandidateState::
              kApproved);

          EXPECT_EQ(
            approval.candidate->candidate_revision,
            2U);

          EXPECT_EQ(
            approval.candidate
              ->approved_location_id,
            "A201");

          EXPECT_EQ(
            approval.location
              ->source_candidate_id,
            "candidate-27");

          EXPECT_EQ(catalog.location_size(), 1U);
          EXPECT_EQ(catalog.candidate_size(), 1U);

          const auto resolved =
            catalog.resolve_location("A201");

          ASSERT_TRUE(resolved.resolved);
        }


        TEST(LocationCatalog, CandidateOwnsMapAndTagOnApproval)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          auto candidate =
            make_candidate(
              "candidate-27",
              "campus_main",
              7U,
              27,
              "A201");

          candidate.tag_pose_map =
            make_pose(12.8, 8.1);

          ASSERT_TRUE(
            catalog.register_candidate(
              candidate).success);

          auto request =
            make_approval(
              "candidate-27",
              1U);

          request.location_id = "a 201";
          request.display_name = "Room A201";

          const auto approval =
            catalog.approve_candidate(request);

          ASSERT_TRUE(approval.success);
          ASSERT_TRUE(approval.location.has_value());

          EXPECT_EQ(
            approval.location
              ->location
              .location_id,
            "A_201");

          EXPECT_EQ(
            approval.location
              ->location
              .map
              .map_id,
            "campus_main");

          EXPECT_EQ(
            approval.location
              ->location
              .map
              .map_revision,
            7U);

          EXPECT_EQ(
            approval.location
              ->location
              .tag
              .id,
            27);

          ASSERT_TRUE(
            approval.location
              ->location
              .tag_pose_map
              .has_value());

          EXPECT_DOUBLE_EQ(
            approval.location
              ->location
              .tag_pose_map
              ->x,
            12.8);
        }


        TEST(LocationCatalog, ApprovalRequiresApproachPose)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          auto candidate =
            make_candidate(
              "candidate-27",
              "campus_main",
              7U,
              27,
              "A201");

          candidate.approach_pose.reset();

          ASSERT_TRUE(
            catalog.register_candidate(
              candidate).success);

          savo_locations::ApprovalRequest request;
          request.candidate_id = "candidate-27";
          request.expected_candidate_revision = 1U;

          const auto result =
            catalog.approve_candidate(request);

          EXPECT_FALSE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::ApprovalCode::
              kMissingApproachPose);
        }


        TEST(LocationCatalog, ApprovalRejectsStaleRevision)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          const auto result =
            catalog.approve_candidate(
              make_approval(
                "candidate-27",
                99U));

          EXPECT_FALSE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::ApprovalCode::
              kStaleRevision);
        }


        TEST(LocationCatalog, FailedApprovalLeavesCandidatePending)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.insert_location(
              make_location(
                "A201",
                "campus_main",
                7U,
                10,
                {"Student service"})).success);

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "B201")).success);

          auto request =
            make_approval(
              "candidate-27",
              1U);

          request.location_id = "B201";
          request.display_name = "Room B201";

          request.aliases = {
            "student-service",
          };

          const auto result =
            catalog.approve_candidate(request);

          EXPECT_FALSE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::ApprovalCode::
              kLocationConflict);

          EXPECT_EQ(catalog.location_size(), 1U);

          const auto candidate =
            catalog.get_candidate(
              "candidate-27");

          ASSERT_TRUE(candidate.has_value());

          EXPECT_EQ(
            candidate->state,
            savo_locations::CandidateState::
              kPendingReview);

          EXPECT_EQ(
            candidate->candidate_revision,
            1U);

          EXPECT_TRUE(
            candidate->approved_location_id.empty());
        }


        TEST(LocationCatalog, CandidateListIsDeterministic)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-c",
                "campus_main",
                7U,
                31,
                "C301")).success);

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-a",
                "campus_main",
                7U,
                27,
                "A201")).success);

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-b",
                "campus_main",
                7U,
                28,
                "B101")).success);

          const auto candidates =
            catalog.list_candidates();

          ASSERT_EQ(candidates.size(), 3U);

          EXPECT_EQ(
            candidates[0].candidate.candidate_id,
            "candidate-a");

          EXPECT_EQ(
            candidates[1].candidate.candidate_id,
            "candidate-b");

          EXPECT_EQ(
            candidates[2].candidate.candidate_id,
            "candidate-c");
        }


        TEST(LocationCatalog, ClearRemovesAllEntities)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.insert_location(
              make_location(
                "A201",
                "campus_main",
                7U,
                27)).success);

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-28",
                "campus_main",
                7U,
                28,
                "A202")).success);

          catalog.clear();

          EXPECT_EQ(catalog.location_size(), 0U);
          EXPECT_EQ(catalog.candidate_size(), 0U);
        }


        TEST(LocationCatalog, ResultStringsAreStable)
        {
          using savo_locations::ApprovalCode;
          using savo_locations::CandidateMutationCode;
          using savo_locations::to_string;

          EXPECT_EQ(
            to_string(
              CandidateMutationCode::
                kEmptyReviewReason),
            "empty_review_reason");

          EXPECT_EQ(
            to_string(
              ApprovalCode::
                kMissingApproachPose),
            "missing_approach_pose");

          EXPECT_EQ(
            to_string(
              ApprovalCode::kLocationConflict),
            "location_conflict");
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # LOC-1C contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase1c_contracts.py",
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


        def test_package_contains_loc1c_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 4, 0)

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert f'"{version}"' in constants


        def test_candidate_catalog_files_exist() -> None:
            for relative in (
                "include/savo_locations/location_catalog.hpp",
                "src/location_catalog.cpp",
                "test/unit/test_location_catalog.cpp",
            ):
                assert (ROOT / relative).is_file()


        def test_cmake_builds_location_catalog() -> None:
            cmake = read("CMakeLists.txt")

            assert "src/location_catalog.cpp" in cmake
            assert "test_location_catalog" in cmake
            assert "test_phase1c_contracts" in cmake


        def test_candidate_lifecycle_is_explicit() -> None:
            header = read(
                "include/savo_locations/location_catalog.hpp"
            )

            for required in (
                "kRegistered",
                "kUpdated",
                "kRejected",
                "kNotPending",
                "kStaleRevision",
                "kTagConflict",
                "register_candidate",
                "replace_candidate",
                "reject_candidate",
            ):
                assert required in header


        def test_approval_transaction_is_explicit() -> None:
            header = read(
                "include/savo_locations/location_catalog.hpp"
            )

            implementation = read(
                "src/location_catalog.cpp"
            )

            for required in (
                "kApproved",
                "kCandidateNotFound",
                "kCandidateNotPending",
                "kMissingApproachPose",
                "kLocationConflict",
                "approve_candidate",
            ):
                assert required in header

            assert (
                "location_record.source_candidate_id"
                in implementation
            )

            assert (
                "existing->second ="
                in implementation
            )

            assert (
                "locations_.insert(location_record)"
                in implementation
            )


        def test_candidate_owns_mapping_evidence() -> None:
            model = read(
                "include/savo_locations/model.hpp"
            )

            implementation = read(
                "src/location_catalog.cpp"
            )

            assert "MapContext map;" in model
            assert "TagBinding tag;" in model
            assert "PoseData tag_pose_map;" in model

            assert "location.map = source.map" in implementation
            assert "location.tag = source.tag" in implementation
            assert (
                "location.tag_pose_map ="
                in implementation
            )


        def test_loc1c_remains_without_ros_or_sqlite() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(rclcpp" not in cmake
            assert "find_package(SQLite3" not in cmake
            assert "add_executable(" not in cmake

            assert not (ROOT / "launch").exists()
            assert not (ROOT / "database").exists()
            assert not (
                ROOT
                / "src"
                / "location_registry_node.cpp"
            ).exists()
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

    policy = policy_path.read_text(
        encoding="utf-8"
    )

    if "\ncandidate_registry:\n" not in policy:
        policy += clean(
            r'''

            candidate_registry:
              initial_state: pending_review
              initial_revision: 1
              candidate_id_unique_globally: true

              pending_tag_unique_within_map_revision: true
              rejected_candidate_releases_tag: true
              approved_candidate_tag_owned_by_location: true

              update_requires_pending_state: true
              update_requires_expected_revision: true
              update_increments_revision: true

              rejection_requires_reason: true
              rejection_increments_revision: true

            approval:
              requires_pending_candidate: true
              requires_expected_candidate_revision: true
              requires_approach_pose: true

              map_context_source: candidate
              apriltag_binding_source: candidate
              tag_pose_source: candidate

              location_initial_state: approved
              location_initial_enabled: true
              location_initial_revision: 1

              candidate_revision_increment: 1
              atomic_on_location_conflict: true
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
    readme = readme_path.read_text(
        encoding="utf-8"
    )

    if "## LOC-1C candidate approval transaction" not in readme:
        readme += clean(
            r'''

            ## LOC-1C candidate approval transaction

            LOC-1C adds the deterministic candidate lifecycle and the
            in-memory candidate-to-location approval transaction.

            Candidate lifecycle:

            - newly registered candidates begin as `pending_review`;
            - candidate IDs are globally unique;
            - pending candidate tags are unique within a map revision;
            - updates require the expected candidate revision;
            - updates are allowed only while pending;
            - rejection requires a non-empty reason;
            - approval and rejection increment the candidate revision.

            Approval ownership:

            - the operator supplies the semantic identity and safe approach
              pose;
            - the candidate remains authoritative for `map_id`,
              `map_revision`, `map_release_id`, AprilTag family, AprilTag ID
              and `tag_pose_map`;
            - the approved location stores the originating candidate ID;
            - location records begin approved, enabled and at revision 1.

            Approval is atomic from the catalog API. If location insertion
            fails because of an ID, alias, identity or AprilTag conflict, the
            candidate remains pending at its existing revision.

            LOC-1C still does not introduce SQLite, a ROS node, ROS services,
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
        / f"LOC1C_savo_locations_{stamp}.sha256"
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
        "LOC-1C candidate lifecycle and approval transaction applied."
    )

    print(
        "No SQLite, ROS node or hardware runtime package was added."
    )


if __name__ == "__main__":
    main()
