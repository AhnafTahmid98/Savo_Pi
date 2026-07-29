#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape, quoteattr
import hashlib
import shutil
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


def read_existing_maintainer() -> tuple[str, str]:
    name = "Robot Savo Maintainers"
    email = "robot-savo@localhost.localdomain"

    package_xml = ROOT / "package.xml"

    try:
        tree = ET.parse(package_xml)
        maintainer = tree.getroot().find("maintainer")

        if maintainer is None:
            return name, email

        existing_name = (maintainer.text or "").strip()
        existing_email = maintainer.attrib.get("email", "").strip()

        if existing_name and "TODO" not in existing_name.upper():
            name = existing_name

        if (
            existing_email
            and "@" in existing_email
            and "TODO" not in existing_email.upper()
        ):
            email = existing_email

    except (ET.ParseError, OSError):
        pass

    return name, email


def main() -> None:
    if not (ROOT / "package.xml").is_file():
        raise SystemExit(f"savo_locations package not found: {ROOT}")

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    maintainer_name, maintainer_email = read_existing_maintainer()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    backup = (
        BACKUPS
        / f"pre_LOC0_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    # LOC-0 intentionally replaces the invalid mixed
    # ament_python/ament_cmake scaffold completely.
    shutil.rmtree(ROOT)
    ROOT.mkdir(parents=True, exist_ok=True)

    write(
        "CMakeLists.txt",
        r'''
        cmake_minimum_required(VERSION 3.16)
        project(savo_locations VERSION 0.1.0 LANGUAGES CXX)

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
        # Contract/foundation library
        # ---------------------------------------------------------------------------

        add_library(
          ${PROJECT_NAME}_contracts
          src/types.cpp
        )

        add_library(
          ${PROJECT_NAME}::contracts
          ALIAS ${PROJECT_NAME}_contracts
        )

        target_include_directories(
          ${PROJECT_NAME}_contracts
          PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )

        target_compile_features(
          ${PROJECT_NAME}_contracts
          PUBLIC
            cxx_std_17
        )

        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
          target_compile_options(
            ${PROJECT_NAME}_contracts
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
            ${PROJECT_NAME}_contracts
          EXPORT
            export_${PROJECT_NAME}
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
              ${PROJECT_NAME}_contracts
            )
          endif()

          ament_add_pytest_test(
            test_phase0_contracts
            test/contracts/test_phase0_contracts.py
            TIMEOUT 60
          )
        endif()

        # ---------------------------------------------------------------------------
        # Package exports
        # ---------------------------------------------------------------------------

        ament_export_targets(
          export_${PROJECT_NAME}
          HAS_LIBRARY_TARGET
        )

        ament_export_include_directories(include)
        ament_export_dependencies(savo_msgs)

        ament_package()
        ''',
    )

    write(
        "package.xml",
        f'''
        <?xml version="1.0"?>
        <package format="3">
          <name>savo_locations</name>
          <version>0.1.0</version>

          <description>
            Authoritative semantic location registry foundation for Robot Savo.
          </description>

          <maintainer email={quoteattr(maintainer_email)}>
            {escape(maintainer_name)}
          </maintainer>

          <license>Apache-2.0</license>

          <buildtool_depend>ament_cmake</buildtool_depend>

          <depend>savo_msgs</depend>

          <test_depend>ament_cmake_gtest</test_depend>
          <test_depend>ament_cmake_pytest</test_depend>
          <test_depend>python3-pytest</test_depend>

          <export>
            <build_type>ament_cmake</build_type>
          </export>
        </package>
        ''',
    )

    write(
        "LICENSE",
        r'''
        Apache License
        Version 2.0, January 2004
        http://www.apache.org/licenses/

        Copyright 2026 Robot Savo contributors

        Licensed under the Apache License, Version 2.0 (the "License");
        you may not use this file except in compliance with the License.
        You may obtain a copy of the License at

            http://www.apache.org/licenses/LICENSE-2.0

        Unless required by applicable law or agreed to in writing, software
        distributed under the License is distributed on an "AS IS" BASIS,
        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        See the License for the specific language governing permissions and
        limitations under the License.
        ''',
    )

    write(
        "README.md",
        r'''
        # Robot Savo — `savo_locations`

        `savo_locations` is the authoritative semantic-location registry for
        Robot Savo.

        ## Ownership

        This package owns:

        - canonical location IDs;
        - human-readable display names;
        - deterministic aliases;
        - semantic location types;
        - active-map and map-revision binding;
        - released-map provenance;
        - safe navigation approach poses;
        - optional confirmation poses;
        - expected AprilTag family and ID;
        - candidate and approved-location lifecycle;
        - location enable/disable state;
        - registry persistence and audit history.

        This package does not own:

        - AprilTag image detection;
        - camera-to-map transformations;
        - occupancy-grid validation;
        - path planning;
        - motor commands;
        - natural-language intent classification;
        - final robot-wide safety authorization.

        ## Package relationships

        - `savo_head` visually detects and confirms AprilTags.
        - `savo_mapping` georeferences observations and proposes candidates.
        - `savo_locations` stores and resolves approved locations.
        - `savo_nav` consumes a resolved approach pose and confirms arrival.
        - SavoMind or `savo_bridge` resolves natural-language destination intent.
        - `savo_supervisor` observes readiness and safety state.

        ## Locked LOC-0 terminology

        Map context uses:

        - `map_id`
        - `map_revision`
        - `map_release_id`

        `map_revision` is the active numeric map revision used by runtime
        contracts.

        `map_release_id` records released-map provenance and is not a substitute
        for `map_revision`.

        ## Pose separation

        The AprilTag pose is not automatically a safe navigation goal.

        A location may contain:

        - `tag_pose_map`: actual AprilTag pose in the map frame;
        - `approach_pose`: safe free-space robot navigation target;
        - `confirmation_pose`: optional robot pose/orientation for viewing the tag.

        Navigation must use the approved approach pose.

        ## Lifecycle

        Candidate states:

        - unknown
        - pending review
        - approved
        - rejected

        Location states:

        - unknown
        - approved
        - retired

        Only approved and enabled records may resolve for navigation.

        ## Storage policy

        SQLite will become the authoritative runtime store in LOC-2.

        YAML under `config/` is for schema policy and empty seed/import data.
        Installed package YAML must not be modified as a runtime database.

        ## Current phase

        LOC-0 provides:

        - valid C++17 `ament_cmake` package structure;
        - package ownership contract;
        - stable service and topic names;
        - lifecycle and semantic-type enums;
        - schema-versioned empty seed data;
        - static and C++ unit tests.

        LOC-0 intentionally does not provide:

        - a ROS node;
        - SQLite persistence;
        - candidate approval;
        - alias normalization;
        - location resolution;
        - mapping integration;
        - navigation integration;
        - AprilTag runtime integration.

        Those capabilities are introduced in later phases.
        ''',
    )

    write(
        "config/locations_seed.yaml",
        r'''
        schema_version: 1

        # Seed/import file only.
        # The authoritative LOC-2 runtime store will be SQLite.
        #
        # Do not place unvalidated example coordinates here.
        locations: []
        ''',
    )

    write(
        "config/location_policy.yaml",
        r'''
        schema_version: 1

        registry:
          authoritative_runtime_store: sqlite
          seed_file_is_authoritative: false
          require_active_map_match: true
          require_approved_state: true
          require_enabled_record: true

        identity:
          canonical_id_pattern: "^[A-Z0-9][A-Z0-9_-]{0,63}$"
          maximum_id_length: 64
          maximum_display_name_length: 128
          maximum_alias_count: 32
          maximum_alias_length: 128

        map_context:
          require_map_id: true
          require_map_revision: true
          preserve_map_release_id: true

        pose:
          frame_id: map
          approach_pose_required_for_approval: true
          separate_approach_pose_from_tag_pose: true
          confirmation_pose_optional: true

        apriltag:
          family_required: true
          id_required: true
          unique_within_map_revision: true

        arrival:
          confirmation_required_by_default: true

        semantic_types:
          - room
          - corridor
          - reception
          - laboratory
          - elevator
          - doorway
          - stairwell
          - restroom
          - office
          - classroom
          - service_point
          - charging_station
          - other
        ''',
    )

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

        inline constexpr std::string_view kPackageName{"savo_locations"};
        inline constexpr std::string_view kPackageVersion{"0.1.0"};

        inline constexpr std::uint32_t kSchemaVersion{1U};

        inline constexpr std::size_t kMaximumLocationIdLength{64U};
        inline constexpr std::size_t kMaximumDisplayNameLength{128U};
        inline constexpr std::size_t kMaximumAliasCount{32U};
        inline constexpr std::size_t kMaximumAliasLength{128U};

        inline constexpr std::string_view kCanonicalMapFrame{"map"};

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__CONSTANTS_HPP_
        ''',
    )

    write(
        "include/savo_locations/service_names.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__SERVICE_NAMES_HPP_
        #define SAVO_LOCATIONS__SERVICE_NAMES_HPP_

        #include <string_view>

        namespace savo_locations::service_names
        {

        inline constexpr std::string_view kResolve{
          "/savo_locations/resolve"};

        inline constexpr std::string_view kGet{
          "/savo_locations/get"};

        inline constexpr std::string_view kList{
          "/savo_locations/list"};

        inline constexpr std::string_view kRegisterCandidate{
          "/savo_locations/candidates/register"};

        inline constexpr std::string_view kApproveCandidate{
          "/savo_locations/candidates/approve"};

        inline constexpr std::string_view kSetEnabled{
          "/savo_locations/set_enabled"};

        }  // namespace savo_locations::service_names

        #endif  // SAVO_LOCATIONS__SERVICE_NAMES_HPP_
        ''',
    )

    write(
        "include/savo_locations/topic_names.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__TOPIC_NAMES_HPP_
        #define SAVO_LOCATIONS__TOPIC_NAMES_HPP_

        #include <string_view>

        namespace savo_locations::topic_names
        {

        inline constexpr std::string_view kStatus{
          "/savo_locations/status"};

        inline constexpr std::string_view kEvents{
          "/savo_locations/events"};

        inline constexpr std::string_view kHeartbeat{
          "/savo_locations/heartbeat"};

        inline constexpr std::string_view kSnapshot{
          "/savo_locations/snapshot"};

        }  // namespace savo_locations::topic_names

        #endif  // SAVO_LOCATIONS__TOPIC_NAMES_HPP_
        ''',
    )

    write(
        "include/savo_locations/types.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__TYPES_HPP_
        #define SAVO_LOCATIONS__TYPES_HPP_

        #include <cstdint>
        #include <optional>
        #include <string_view>

        namespace savo_locations
        {

        enum class CandidateState : std::uint8_t
        {
          kUnknown = 0U,
          kPendingReview = 1U,
          kApproved = 2U,
          kRejected = 3U,
        };

        enum class LocationState : std::uint8_t
        {
          kUnknown = 0U,
          kApproved = 1U,
          kRetired = 2U,
        };

        enum class SemanticType : std::uint8_t
        {
          kUnknown = 0U,
          kRoom,
          kCorridor,
          kReception,
          kLaboratory,
          kElevator,
          kDoorway,
          kStairwell,
          kRestroom,
          kOffice,
          kClassroom,
          kServicePoint,
          kChargingStation,
          kOther,
        };

        [[nodiscard]]
        std::string_view to_string(CandidateState state) noexcept;

        [[nodiscard]]
        std::string_view to_string(LocationState state) noexcept;

        [[nodiscard]]
        std::string_view to_string(SemanticType type) noexcept;

        [[nodiscard]]
        std::optional<SemanticType> semantic_type_from_string(
          std::string_view value) noexcept;

        [[nodiscard]]
        bool is_terminal(CandidateState state) noexcept;

        [[nodiscard]]
        bool is_navigable(
          LocationState state,
          bool enabled) noexcept;

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__TYPES_HPP_
        ''',
    )

    write(
        "src/types.cpp",
        r'''
        #include "savo_locations/types.hpp"

        namespace savo_locations
        {

        std::string_view to_string(
          const CandidateState state) noexcept
        {
          switch (state) {
            case CandidateState::kPendingReview:
              return "pending_review";

            case CandidateState::kApproved:
              return "approved";

            case CandidateState::kRejected:
              return "rejected";

            case CandidateState::kUnknown:
            default:
              return "unknown";
          }
        }


        std::string_view to_string(
          const LocationState state) noexcept
        {
          switch (state) {
            case LocationState::kApproved:
              return "approved";

            case LocationState::kRetired:
              return "retired";

            case LocationState::kUnknown:
            default:
              return "unknown";
          }
        }


        std::string_view to_string(
          const SemanticType type) noexcept
        {
          switch (type) {
            case SemanticType::kRoom:
              return "room";

            case SemanticType::kCorridor:
              return "corridor";

            case SemanticType::kReception:
              return "reception";

            case SemanticType::kLaboratory:
              return "laboratory";

            case SemanticType::kElevator:
              return "elevator";

            case SemanticType::kDoorway:
              return "doorway";

            case SemanticType::kStairwell:
              return "stairwell";

            case SemanticType::kRestroom:
              return "restroom";

            case SemanticType::kOffice:
              return "office";

            case SemanticType::kClassroom:
              return "classroom";

            case SemanticType::kServicePoint:
              return "service_point";

            case SemanticType::kChargingStation:
              return "charging_station";

            case SemanticType::kOther:
              return "other";

            case SemanticType::kUnknown:
            default:
              return "unknown";
          }
        }


        std::optional<SemanticType> semantic_type_from_string(
          const std::string_view value) noexcept
        {
          if (value == "room") {
            return SemanticType::kRoom;
          }

          if (value == "corridor") {
            return SemanticType::kCorridor;
          }

          if (value == "reception") {
            return SemanticType::kReception;
          }

          if (value == "laboratory") {
            return SemanticType::kLaboratory;
          }

          if (value == "elevator") {
            return SemanticType::kElevator;
          }

          if (value == "doorway") {
            return SemanticType::kDoorway;
          }

          if (value == "stairwell") {
            return SemanticType::kStairwell;
          }

          if (value == "restroom") {
            return SemanticType::kRestroom;
          }

          if (value == "office") {
            return SemanticType::kOffice;
          }

          if (value == "classroom") {
            return SemanticType::kClassroom;
          }

          if (value == "service_point") {
            return SemanticType::kServicePoint;
          }

          if (value == "charging_station") {
            return SemanticType::kChargingStation;
          }

          if (value == "other") {
            return SemanticType::kOther;
          }

          return std::nullopt;
        }


        bool is_terminal(
          const CandidateState state) noexcept
        {
          return
            state == CandidateState::kApproved ||
            state == CandidateState::kRejected;
        }


        bool is_navigable(
          const LocationState state,
          const bool enabled) noexcept
        {
          return
            state == LocationState::kApproved &&
            enabled;
        }

        }  // namespace savo_locations
        ''',
    )

    write(
        "test/unit/test_types.cpp",
        r'''
        #include <gtest/gtest.h>

        #include "savo_locations/constants.hpp"
        #include "savo_locations/service_names.hpp"
        #include "savo_locations/topic_names.hpp"
        #include "savo_locations/types.hpp"


        TEST(LocationTypes, CandidateStateStringsAreStable)
        {
          using savo_locations::CandidateState;
          using savo_locations::to_string;

          EXPECT_EQ(
            to_string(CandidateState::kUnknown),
            "unknown");

          EXPECT_EQ(
            to_string(CandidateState::kPendingReview),
            "pending_review");

          EXPECT_EQ(
            to_string(CandidateState::kApproved),
            "approved");

          EXPECT_EQ(
            to_string(CandidateState::kRejected),
            "rejected");
        }


        TEST(LocationTypes, CandidateTerminalPolicyIsStable)
        {
          using savo_locations::CandidateState;
          using savo_locations::is_terminal;

          EXPECT_FALSE(
            is_terminal(CandidateState::kUnknown));

          EXPECT_FALSE(
            is_terminal(CandidateState::kPendingReview));

          EXPECT_TRUE(
            is_terminal(CandidateState::kApproved));

          EXPECT_TRUE(
            is_terminal(CandidateState::kRejected));
        }


        TEST(LocationTypes, NavigationRequiresApprovalAndEnablement)
        {
          using savo_locations::LocationState;
          using savo_locations::is_navigable;

          EXPECT_TRUE(
            is_navigable(
              LocationState::kApproved,
              true));

          EXPECT_FALSE(
            is_navigable(
              LocationState::kApproved,
              false));

          EXPECT_FALSE(
            is_navigable(
              LocationState::kRetired,
              true));

          EXPECT_FALSE(
            is_navigable(
              LocationState::kUnknown,
              true));
        }


        TEST(LocationTypes, SemanticTypesRoundTrip)
        {
          using savo_locations::SemanticType;
          using savo_locations::semantic_type_from_string;
          using savo_locations::to_string;

          const auto room =
            semantic_type_from_string("room");

          ASSERT_TRUE(room.has_value());
          EXPECT_EQ(room.value(), SemanticType::kRoom);
          EXPECT_EQ(to_string(room.value()), "room");

          const auto charging_station =
            semantic_type_from_string(
              "charging_station");

          ASSERT_TRUE(charging_station.has_value());

          EXPECT_EQ(
            charging_station.value(),
            SemanticType::kChargingStation);

          EXPECT_FALSE(
            semantic_type_from_string(
              "unsupported_type").has_value());
        }


        TEST(LocationTypes, ContractNamesAreAbsolute)
        {
          using namespace savo_locations;

          EXPECT_EQ(
            service_names::kResolve,
            "/savo_locations/resolve");

          EXPECT_EQ(
            service_names::kRegisterCandidate,
            "/savo_locations/candidates/register");

          EXPECT_EQ(
            topic_names::kEvents,
            "/savo_locations/events");

          EXPECT_EQ(kSchemaVersion, 1U);
          EXPECT_EQ(kCanonicalMapFrame, "map");
        }
        ''',
    )

    write(
        "test/contracts/test_phase0_contracts.py",
        r'''
        from pathlib import Path
        import re
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[2]


        def read(relative: str) -> str:
            return (ROOT / relative).read_text(
                encoding="utf-8"
            )


        def test_package_is_pure_ament_cmake() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            build_type = package.find(
                "./export/build_type"
            )

            assert build_type is not None
            assert build_type.text == "ament_cmake"

            assert not (ROOT / "setup.py").exists()
            assert not (ROOT / "setup.cfg").exists()
            assert not (ROOT / "locations.yaml").exists()
            assert not (ROOT / "savo_locations").exists()


        def test_cmake_builds_cpp17_contract_library() -> None:
            cmake = read("CMakeLists.txt")

            assert "CMAKE_CXX_STANDARD 17" in cmake
            assert "savo_locations_contracts" in cmake
            assert "src/types.cpp" in cmake
            assert "ament_add_gtest" in cmake
            assert "ament_add_pytest_test" in cmake


        def test_seed_contains_no_fake_locations() -> None:
            seed = read("config/locations_seed.yaml")

            assert "schema_version: 1" in seed
            assert "locations: []" in seed

            assert "A201" not in seed
            assert "Info Desk" not in seed

            assert not re.search(
                r"^\s+x:\s*[-+]?\d",
                seed,
                re.MULTILINE,
            )

            assert not re.search(
                r"^\s+y:\s*[-+]?\d",
                seed,
                re.MULTILINE,
            )


        def test_map_contract_uses_revision_not_version() -> None:
            policy = read(
                "config/location_policy.yaml"
            )

            readme = read("README.md")

            assert "require_map_revision: true" in policy
            assert "preserve_map_release_id: true" in policy

            assert "`map_id`" in readme
            assert "`map_revision`" in readme
            assert "`map_release_id`" in readme


        def test_approach_pose_is_separate_from_tag_pose() -> None:
            policy = read(
                "config/location_policy.yaml"
            )

            readme = read("README.md")

            assert (
                "separate_approach_pose_from_tag_pose: true"
                in policy
            )

            assert "`tag_pose_map`" in readme
            assert "`approach_pose`" in readme
            assert "`confirmation_pose`" in readme


        def test_service_and_topic_names_are_locked() -> None:
            services = read(
                "include/savo_locations/service_names.hpp"
            )

            topics = read(
                "include/savo_locations/topic_names.hpp"
            )

            for name in (
                "/savo_locations/resolve",
                "/savo_locations/get",
                "/savo_locations/list",
                "/savo_locations/candidates/register",
                "/savo_locations/candidates/approve",
                "/savo_locations/set_enabled",
            ):
                assert name in services

            for name in (
                "/savo_locations/status",
                "/savo_locations/events",
                "/savo_locations/heartbeat",
                "/savo_locations/snapshot",
            ):
                assert name in topics
        ''',
    )

    changed_files = sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
    )

    manifest = (
        LOGS
        / f"LOC0_savo_locations_{stamp}.sha256"
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
        "LOC-0 savo_locations C++17 foundation applied."
    )
    print(
        "No runtime or hardware package was modified."
    )


if __name__ == "__main__":
    main()
