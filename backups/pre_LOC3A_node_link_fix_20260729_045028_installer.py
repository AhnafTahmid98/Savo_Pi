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
    path.write_text(
        clean(content),
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def update_package_xml() -> None:
    path = ROOT / "package.xml"

    tree = ET.parse(path)
    package = tree.getroot()

    version = package.find("version")

    if version is None:
        raise RuntimeError(
            "package.xml has no version element"
        )

    version.text = "0.8.0"

    def dependency_exists(name: str) -> bool:
        for tag in (
            "depend",
            "build_depend",
            "build_export_depend",
            "exec_depend",
            "test_depend",
        ):
            for element in package.findall(tag):
                if element.text == name:
                    return True

        return False

    for dependency in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
    ):
        if not dependency_exists(dependency):
            element = ET.SubElement(
                package,
                "depend",
            )

            element.text = dependency

    for dependency in (
        "launch",
        "launch_ros",
    ):
        if not dependency_exists(dependency):
            element = ET.SubElement(
                package,
                "exec_depend",
            )

            element.text = dependency

    ET.indent(tree, space="  ")

    tree.write(
        path,
        encoding="utf-8",
        xml_declaration=True,
    )


def find_build_testing_span(
    cmake: str,
) -> tuple[int, int]:
    lines = cmake.splitlines(
        keepends=True
    )

    start = None

    for index, line in enumerate(lines):
        if line.strip() == "if(BUILD_TESTING)":
            start = index
            break

    if start is None:
        raise RuntimeError(
            "Could not locate if(BUILD_TESTING)"
        )

    depth = 0

    for index in range(start, len(lines)):
        stripped = lines[index].strip()

        if re.match(r"^if\s*\(", stripped):
            depth += 1

        elif re.match(r"^endif\s*(\(|$)", stripped):
            depth -= 1

            if depth == 0:
                return start, index

    raise RuntimeError(
        "Could not locate BUILD_TESTING endif"
    )


def update_cmake() -> None:
    path = ROOT / "CMakeLists.txt"
    cmake = path.read_text(encoding="utf-8")

    cmake = re.sub(
        r"project\(savo_locations VERSION [0-9.]+",
        "project(savo_locations VERSION 0.8.0",
        cmake,
        count=1,
    )

    dependency_anchor = (
        "find_package(SQLite3 REQUIRED)"
    )

    if dependency_anchor not in cmake:
        raise RuntimeError(
            "Could not locate SQLite3 dependency"
        )

    dependencies = (
        "find_package(rclcpp REQUIRED)",
        "find_package(std_msgs REQUIRED)",
        "find_package(geometry_msgs REQUIRED)",
        "find_package(builtin_interfaces REQUIRED)",
    )

    missing_dependencies = [
        dependency
        for dependency in dependencies
        if dependency not in cmake
    ]

    if missing_dependencies:
        cmake = cmake.replace(
            dependency_anchor,
            dependency_anchor
            + "\n"
            + "\n".join(missing_dependencies),
            1,
        )

    if (
        "add_library(savo_locations_ros"
        not in cmake
    ):
        lines = cmake.splitlines(
            keepends=True
        )

        build_start, _ = (
            find_build_testing_span(cmake)
        )

        runtime_block = clean(
            r'''
            # ---------------------------------------------------------------------------
            # LOC-3A production read-only ROS runtime
            # ---------------------------------------------------------------------------

            add_library(
              savo_locations_ros
              STATIC
                src/read_only_catalog_view.cpp
                src/ros_conversions.cpp
                src/location_registry_node.cpp
            )

            target_compile_features(
              savo_locations_ros
              PUBLIC
                cxx_std_17
            )

            target_include_directories(
              savo_locations_ros
              PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
            )

            target_link_libraries(
              savo_locations_ros
              PUBLIC
                savo_locations_storage
            )

            ament_target_dependencies(
              savo_locations_ros
              PUBLIC
                rclcpp
                std_msgs
                geometry_msgs
                builtin_interfaces
                savo_msgs
            )

            add_executable(
              savo_locations_node
              src/location_registry_main.cpp
            )

            target_compile_features(
              savo_locations_node
              PRIVATE
                cxx_std_17
            )

            target_link_libraries(
              savo_locations_node
              PRIVATE
                savo_locations_ros
            )

            ament_target_dependencies(
              savo_locations_node
              rclcpp
            )

            install(
              TARGETS
                savo_locations_ros
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
            )

            install(
              TARGETS
                savo_locations_node
              DESTINATION
                lib/${PROJECT_NAME}
            )

            install(
              DIRECTORY
                launch
              DESTINATION
                share/${PROJECT_NAME}
            )

            '''
        )

        lines.insert(
            build_start,
            runtime_block + "\n",
        )

        cmake = "".join(lines)

    if "test_read_only_catalog_view" not in cmake:
        lines = cmake.splitlines(
            keepends=True
        )

        _, build_end = (
            find_build_testing_span(cmake)
        )

        test_block = clean(
            r'''
              ament_add_gtest(
                test_read_only_catalog_view
                test/ros/test_read_only_catalog_view.cpp
              )

              if(TARGET test_read_only_catalog_view)
                target_link_libraries(
                  test_read_only_catalog_view
                  savo_locations_ros
                )
              endif()

              ament_add_gtest(
                test_registry_node
                test/ros/test_registry_node.cpp
              )

              if(TARGET test_registry_node)
                target_link_libraries(
                  test_registry_node
                  savo_locations_ros
                  SQLite::SQLite3
                )

                ament_target_dependencies(
                  test_registry_node
                  rclcpp
                  std_msgs
                  savo_msgs
                )

                target_compile_definitions(
                  test_registry_node
                  PRIVATE
                    "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
                )
              endif()

              ament_add_pytest_test(
                test_phase3a_contracts
                test/contracts/test_phase3a_contracts.py
                TIMEOUT 60
              )

            '''
        )

        lines.insert(
            build_end,
            test_block,
        )

        cmake = "".join(lines)

    required = (
        "savo_locations_ros",
        "savo_locations_node",
        "test_read_only_catalog_view",
        "test_registry_node",
        "test_phase3a_contracts",
    )

    for fragment in required:
        if fragment not in cmake:
            raise RuntimeError(
                f"CMake verification failed: {fragment}"
            )

    path.write_text(
        cmake,
        encoding="utf-8",
    )


def update_constants_and_topics() -> None:
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
        '"0.8.0"',
        constants,
        count=1,
    )

    constants_path.write_text(
        constants,
        encoding="utf-8",
    )

    topics_path = (
        ROOT
        / "include"
        / "savo_locations"
        / "topic_names.hpp"
    )

    topics = topics_path.read_text(
        encoding="utf-8"
    )

    topics = topics.replace(
        "kApprovedSnapshot",
        "kSnapshot",
    )

    topics = topics.replace(
        "/savo_locations/approved_snapshot",
        "/savo_locations/snapshot",
    )

    if "kSnapshot" not in topics:
        marker = (
            "}  // namespace savo_locations::topics"
        )

        if marker not in topics:
            raise RuntimeError(
                "Could not locate topic namespace end"
            )

        topics = topics.replace(
            marker,
            (
                'inline constexpr char kSnapshot[] =\n'
                '  "/savo_locations/snapshot";\n\n'
                + marker
            ),
            1,
        )

    topics_path.write_text(
        topics,
        encoding="utf-8",
    )


def main() -> None:
    required = (
        ROOT / "package.xml",
        ROOT / "CMakeLists.txt",
        ROOT / "include/savo_locations/constants.hpp",
        ROOT / "include/savo_locations/topic_names.hpp",
        ROOT / "include/savo_locations/service_names.hpp",
        ROOT / "include/savo_locations/model.hpp",
        ROOT / "include/savo_locations/normalization.hpp",
        ROOT / "include/savo_locations/sqlite_store.hpp",
        ROOT / "include/savo_locations/sqlite_repository.hpp",
        ROOT / "src/sqlite_store.cpp",
        ROOT / "src/sqlite_repository.cpp",
    )

    for path in required:
        if not path.is_file():
            raise SystemExit(
                f"Required LOC-2C file missing: {path}"
            )

    BACKUPS.mkdir(
        parents=True,
        exist_ok=True,
    )

    LOGS.mkdir(
        parents=True,
        exist_ok=True,
    )

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC3A_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(
        backup,
        "w:gz",
    ) as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    update_package_xml()
    update_constants_and_topics()

    # -------------------------------------------------------------------------
    # Read-only catalog view
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/read_only_catalog_view.hpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #ifndef SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_
        #define SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_

        #include <cstdint>
        #include <string>
        #include <string_view>
        #include <vector>

        #include "savo_locations/sqlite_repository.hpp"


        namespace savo_locations
        {

        enum class ReadResolveCode : std::uint8_t
        {
          kResolved = 0U,
          kInvalidQuery,
          kNotFound,
          kAmbiguous,
          kDisabled,
          kRetired,
          kMapMismatch,
        };


        struct ReadResolveRequest
        {
          std::string query;

          bool enforce_map_context{false};
          std::string map_id;
          std::uint32_t map_revision{0U};
        };


        struct ReadResolveResult
        {
          ReadResolveCode code{
            ReadResolveCode::kNotFound};

          std::string reason;
          std::string normalized_query;

          ResolveMatchType match_type{
            ResolveMatchType::kNone};

          LocationRecordData location;

          std::vector<std::string>
            ambiguous_location_ids;
        };


        enum class ReadGetCode : std::uint8_t
        {
          kFound = 0U,
          kInvalidId,
          kNotFound,
          kDisabled,
          kRetired,
        };


        struct ReadGetResult
        {
          ReadGetCode code{
            ReadGetCode::kNotFound};

          std::string reason;
          LocationRecordData location;
        };


        struct ReadListRequest
        {
          std::string map_id;
          std::uint32_t map_revision{0U};
          bool enforce_map_context{false};

          std::string semantic_type;
          std::uint8_t state_filter{0U};

          bool enabled_only{false};
        };


        struct ReadListResult
        {
          bool valid{true};
          std::string reason;

          std::vector<LocationRecordData>
            locations;
        };


        class ReadOnlyCatalogView
        {
        public:
          ReadOnlyCatalogView() = default;

          explicit ReadOnlyCatalogView(
            CatalogSnapshot snapshot);

          void replace(
            CatalogSnapshot snapshot);

          [[nodiscard]]
          ReadResolveResult resolve(
            const ReadResolveRequest & request) const;

          [[nodiscard]]
          ReadGetResult get(
            std::string_view location_id,
            bool include_disabled,
            bool include_retired) const;

          [[nodiscard]]
          ReadListResult list(
            const ReadListRequest & request) const;

          [[nodiscard]]
          const std::vector<LocationRecordData> &
          locations() const noexcept;

        private:
          std::vector<LocationRecordData>
            locations_;
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_
        ''',
    )

    write(
        "src/read_only_catalog_view.cpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_locations/read_only_catalog_view.hpp"

        #include <algorithm>
        #include <cctype>
        #include <string>
        #include <utility>
        #include <vector>

        #include "savo_locations/normalization.hpp"


        namespace savo_locations
        {
        namespace
        {

        constexpr std::size_t
          kMaximumLocationIdLength{128U};


        bool is_ascii_alphanumeric(
          const char character)
        {
          const auto value =
            static_cast<unsigned char>(character);

          return std::isalnum(value) != 0;
        }


        bool is_valid_location_id(
          const std::string_view input)
        {
          if (
            input.empty() ||
            input.size() >
              kMaximumLocationIdLength ||
            !is_ascii_alphanumeric(input.front()))
          {
            return false;
          }

          for (
            const char character :
            input.substr(1U))
          {
            if (
              is_ascii_alphanumeric(character) ||
              character == '.' ||
              character == '_' ||
              character == ':' ||
              character == '-')
            {
              continue;
            }

            return false;
          }

          return true;
        }


        bool map_matches(
          const LocationRecordData & record,
          const std::string_view map_id,
          const std::uint32_t map_revision)
        {
          return
            record.location.map.map_id == map_id &&
            record.location.map.map_revision ==
              map_revision;
        }


        ReadResolveResult finalize_resolve(
          const LocationRecordData & record,
          const ResolveMatchType match_type,
          const std::string & normalized_query)
        {
          ReadResolveResult result;

          result.normalized_query =
            normalized_query;

          result.match_type = match_type;
          result.location = record;

          if (
            record.state ==
            LocationState::kRetired)
          {
            result.code =
              ReadResolveCode::kRetired;

            result.reason =
              "location is retired";

            return result;
          }

          if (!record.enabled) {
            result.code =
              ReadResolveCode::kDisabled;

            result.reason =
              "location is disabled";

            return result;
          }

          if (
            record.state !=
            LocationState::kApproved)
          {
            result.code =
              ReadResolveCode::kNotFound;

            result.reason =
              "location is not approved";

            return result;
          }

          result.code =
            ReadResolveCode::kResolved;

          result.reason =
            "location resolved";

          return result;
        }


        struct IdentityMatch
        {
          const LocationRecordData * record{
            nullptr};

          ResolveMatchType match_type{
            ResolveMatchType::kNone};
        };

        }  // namespace


        ReadOnlyCatalogView::ReadOnlyCatalogView(
          CatalogSnapshot snapshot)
        {
          replace(std::move(snapshot));
        }


        void ReadOnlyCatalogView::replace(
          CatalogSnapshot snapshot)
        {
          locations_ =
            std::move(snapshot.locations);

          std::sort(
            locations_.begin(),
            locations_.end(),
            [](
              const LocationRecordData & lhs,
              const LocationRecordData & rhs)
            {
              return
                lhs.location.location_id <
                rhs.location.location_id;
            });
        }


        ReadResolveResult
        ReadOnlyCatalogView::resolve(
          const ReadResolveRequest & request) const
        {
          ReadResolveResult result;

          const std::string query =
            trim_ascii(request.query);

          result.normalized_query =
            normalize_lookup_key(query);

          if (
            query.empty() ||
            result.normalized_query.empty())
          {
            result.code =
              ReadResolveCode::kInvalidQuery;

            result.reason =
              "location query is empty";

            return result;
          }

          if (
            request.enforce_map_context &&
            (
              trim_ascii(request.map_id).empty() ||
              request.map_revision == 0U
            ))
          {
            result.code =
              ReadResolveCode::kInvalidQuery;

            result.reason =
              "enforced map context requires "
              "map_id and map_revision";

            return result;
          }

          // Exact canonical-ID precedence.
          for (
            const auto & record :
            locations_)
          {
            if (
              normalize_lookup_key(
                record.location.location_id) !=
              result.normalized_query)
            {
              continue;
            }

            if (
              request.enforce_map_context &&
              !map_matches(
                record,
                request.map_id,
                request.map_revision))
            {
              result.code =
                ReadResolveCode::kMapMismatch;

              result.reason =
                "location exists in another "
                "map context";

              result.location = record;

              return result;
            }

            return finalize_resolve(
              record,
              ResolveMatchType::kLocationId,
              result.normalized_query);
          }

          std::vector<IdentityMatch> matches;

          for (
            const auto & record :
            locations_)
          {
            ResolveMatchType match_type{
              ResolveMatchType::kNone};

            if (
              normalize_lookup_key(
                record.location.display_name) ==
              result.normalized_query)
            {
              match_type =
                ResolveMatchType::kDisplayName;
            }
            else
            {
              for (
                const auto & alias :
                record.location.aliases)
              {
                if (
                  normalize_lookup_key(alias) ==
                  result.normalized_query)
                {
                  match_type =
                    ResolveMatchType::kAlias;

                  break;
                }
              }
            }

            if (
              match_type !=
              ResolveMatchType::kNone)
            {
              matches.push_back(
                IdentityMatch{
                  &record,
                  match_type});
            }
          }

          if (matches.empty()) {
            result.code =
              ReadResolveCode::kNotFound;

            result.reason =
              "location query did not match";

            return result;
          }

          if (request.enforce_map_context) {
            std::vector<IdentityMatch>
              scoped_matches;

            for (
              const auto & match :
              matches)
            {
              if (
                map_matches(
                  *match.record,
                  request.map_id,
                  request.map_revision))
              {
                scoped_matches.push_back(match);
              }
            }

            if (scoped_matches.empty()) {
              result.code =
                ReadResolveCode::kMapMismatch;

              result.reason =
                "matching identity exists in "
                "another map context";

              return result;
            }

            matches =
              std::move(scoped_matches);
          }

          if (matches.size() > 1U) {
            result.code =
              ReadResolveCode::kAmbiguous;

            result.reason =
              "location query is ambiguous";

            for (
              const auto & match :
              matches)
            {
              result.ambiguous_location_ids.push_back(
                match.record
                  ->location
                  .location_id);
            }

            std::sort(
              result.ambiguous_location_ids.begin(),
              result.ambiguous_location_ids.end());

            result.ambiguous_location_ids.erase(
              std::unique(
                result.ambiguous_location_ids.begin(),
                result.ambiguous_location_ids.end()),
              result.ambiguous_location_ids.end());

            return result;
          }

          return finalize_resolve(
            *matches.front().record,
            matches.front().match_type,
            result.normalized_query);
        }


        ReadGetResult ReadOnlyCatalogView::get(
          const std::string_view location_id,
          const bool include_disabled,
          const bool include_retired) const
        {
          ReadGetResult result;

          if (!is_valid_location_id(location_id)) {
            result.code =
              ReadGetCode::kInvalidId;

            result.reason =
              "location_id is not canonical";

            return result;
          }

          const auto iterator =
            std::find_if(
              locations_.begin(),
              locations_.end(),
              [location_id](
                const LocationRecordData & record)
              {
                return
                  record.location.location_id ==
                  location_id;
              });

          if (iterator == locations_.end()) {
            result.code =
              ReadGetCode::kNotFound;

            result.reason =
              "location was not found";

            return result;
          }

          result.location = *iterator;

          if (
            iterator->state ==
              LocationState::kRetired &&
            !include_retired)
          {
            result.code =
              ReadGetCode::kRetired;

            result.reason =
              "location is retired";

            return result;
          }

          if (
            !iterator->enabled &&
            !include_disabled)
          {
            result.code =
              ReadGetCode::kDisabled;

            result.reason =
              "location is disabled";

            return result;
          }

          result.code =
            ReadGetCode::kFound;

          result.reason =
            "location found";

          return result;
        }


        ReadListResult ReadOnlyCatalogView::list(
          const ReadListRequest & request) const
        {
          ReadListResult result;

          if (request.state_filter > 2U) {
            result.valid = false;
            result.reason =
              "state_filter is invalid";

            return result;
          }

          if (
            request.enforce_map_context &&
            (
              trim_ascii(request.map_id).empty() ||
              request.map_revision == 0U
            ))
          {
            result.valid = false;
            result.reason =
              "enforced map context requires "
              "map_id and map_revision";

            return result;
          }

          if (
            request.map_revision != 0U &&
            trim_ascii(request.map_id).empty())
          {
            result.valid = false;
            result.reason =
              "map_revision requires map_id";

            return result;
          }

          const std::string semantic_filter =
            normalize_lookup_key(
              trim_ascii(request.semantic_type));

          for (
            const auto & record :
            locations_)
          {
            if (
              !request.map_id.empty() &&
              record.location.map.map_id !=
                request.map_id)
            {
              continue;
            }

            if (
              request.map_revision != 0U &&
              record.location.map.map_revision !=
                request.map_revision)
            {
              continue;
            }

            if (
              !semantic_filter.empty() &&
              normalize_lookup_key(
                record.location.semantic_type) !=
                semantic_filter)
            {
              continue;
            }

            if (
              request.state_filter == 1U &&
              record.state !=
                LocationState::kApproved)
            {
              continue;
            }

            if (
              request.state_filter == 2U &&
              record.state !=
                LocationState::kRetired)
            {
              continue;
            }

            if (
              request.enabled_only &&
              !record.enabled)
            {
              continue;
            }

            result.locations.push_back(record);
          }

          result.reason =
            "locations listed";

          return result;
        }


        const std::vector<LocationRecordData> &
        ReadOnlyCatalogView::locations() const noexcept
        {
          return locations_;
        }

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # ROS conversions
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/ros_conversions.hpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #ifndef SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
        #define SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_

        #include "geometry_msgs/msg/pose_stamped.hpp"
        #include "savo_msgs/msg/location_record.hpp"

        #include "savo_locations/model.hpp"


        namespace savo_locations
        {

        [[nodiscard]]
        geometry_msgs::msg::PoseStamped
        to_ros_pose(
          const PoseData & pose);

        [[nodiscard]]
        savo_msgs::msg::LocationRecord
        to_ros_location_record(
          const LocationRecordData & record);

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__ROS_CONVERSIONS_HPP_
        ''',
    )

    write(
        "src/ros_conversions.cpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_locations/ros_conversions.hpp"

        #include <cstdint>


        namespace savo_locations
        {

        geometry_msgs::msg::PoseStamped
        to_ros_pose(
          const PoseData & pose)
        {
          geometry_msgs::msg::PoseStamped message;

          message.header.frame_id =
            pose.frame_id;

          message.pose.position.x = pose.x;
          message.pose.position.y = pose.y;
          message.pose.position.z = pose.z;

          message.pose.orientation.x = pose.qx;
          message.pose.orientation.y = pose.qy;
          message.pose.orientation.z = pose.qz;
          message.pose.orientation.w = pose.qw;

          return message;
        }


        savo_msgs::msg::LocationRecord
        to_ros_location_record(
          const LocationRecordData & record)
        {
          savo_msgs::msg::LocationRecord message;

          message.state =
            static_cast<std::uint8_t>(
              record.state);

          message.enabled = record.enabled;

          message.record_revision =
            record.record_revision;

          message.location_id =
            record.location.location_id;

          message.display_name =
            record.location.display_name;

          message.aliases =
            record.location.aliases;

          message.semantic_type =
            record.location.semantic_type;

          message.map_id =
            record.location.map.map_id;

          message.map_revision =
            record.location.map.map_revision;

          message.map_release_id =
            record.location.map.map_release_id;

          message.approach_pose =
            to_ros_pose(
              record.location.approach_pose);

          message.confirmation_pose_valid =
            record.location
              .confirmation_pose
              .has_value();

          if (
            record.location
              .confirmation_pose
              .has_value())
          {
            message.confirmation_pose =
              to_ros_pose(
                record.location
                  .confirmation_pose
                  .value());
          }

          message.tag_family =
            record.location.tag.family;

          message.tag_id =
            record.location.tag.id;

          message.tag_pose_map_valid =
            record.location
              .tag_pose_map
              .has_value();

          if (
            record.location
              .tag_pose_map
              .has_value())
          {
            message.tag_pose_map =
              to_ros_pose(
                record.location
                  .tag_pose_map
                  .value());
          }

          message.arrival_confirmation_required =
            record.location
              .arrival_confirmation_required;

          message.building =
            record.location.building;

          message.floor =
            record.location.floor;

          message.area =
            record.location.area;

          message.notes =
            record.location.notes;

          message.source_candidate_id =
            record.source_candidate_id;

          // LOC-2 persistence currently retains database timestamps
          // internally rather than in LocationRecordData. Do not invent
          // timestamps in the ROS response; zero means unavailable.
          message.created_at.sec = 0;
          message.created_at.nanosec = 0U;
          message.updated_at.sec = 0;
          message.updated_at.nanosec = 0U;

          return message;
        }

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # ROS node
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/location_registry_node.hpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #ifndef SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
        #define SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_

        #include <atomic>
        #include <memory>
        #include <shared_mutex>
        #include <string>

        #include "rclcpp/rclcpp.hpp"
        #include "std_msgs/msg/string.hpp"
        #include "std_msgs/msg/u_int64.hpp"

        #include "savo_msgs/srv/get_location.hpp"
        #include "savo_msgs/srv/list_locations.hpp"
        #include "savo_msgs/srv/resolve_location.hpp"

        #include "savo_locations/read_only_catalog_view.hpp"
        #include "savo_locations/sqlite_repository.hpp"
        #include "savo_locations/sqlite_store.hpp"


        namespace savo_locations
        {

        class LocationRegistryNode final :
          public rclcpp::Node
        {
        public:
          explicit LocationRegistryNode(
            const rclcpp::NodeOptions & options =
              rclcpp::NodeOptions());

          [[nodiscard]]
          bool registry_ready() const;

        private:
          using ResolveService =
            savo_msgs::srv::ResolveLocation;

          using GetService =
            savo_msgs::srv::GetLocation;

          using ListService =
            savo_msgs::srv::ListLocations;

          void initialize_storage();

          void publish_status();
          void publish_heartbeat();
          void publish_snapshot();

          void handle_resolve(
            const std::shared_ptr<
              ResolveService::Request> request,
            std::shared_ptr<
              ResolveService::Response> response);

          void handle_get(
            const std::shared_ptr<
              GetService::Request> request,
            std::shared_ptr<
              GetService::Response> response);

          void handle_list(
            const std::shared_ptr<
              ListService::Request> request,
            std::shared_ptr<
              ListService::Response> response);

          mutable std::shared_mutex state_mutex_;

          std::string database_path_;
          bool create_parent_directories_{false};
          bool auto_migrate_{true};
          bool publish_snapshot_enabled_{true};

          double status_publish_hz_{1.0};
          double heartbeat_publish_hz_{2.0};

          bool ready_{false};
          bool storage_healthy_{false};

          std::string state_{"starting"};
          std::string reason_{"startup pending"};

          BootstrapReport bootstrap_report_;
          ReadOnlyCatalogView catalog_view_;

          std::unique_ptr<SqliteStore> store_;
          std::unique_ptr<SqliteRepository> repository_;

          rclcpp::Publisher<
            std_msgs::msg::String>::SharedPtr
              status_publisher_;

          rclcpp::Publisher<
            std_msgs::msg::UInt64>::SharedPtr
              heartbeat_publisher_;

          rclcpp::Publisher<
            std_msgs::msg::String>::SharedPtr
              snapshot_publisher_;

          rclcpp::Service<
            ResolveService>::SharedPtr
              resolve_service_;

          rclcpp::Service<
            GetService>::SharedPtr
              get_service_;

          rclcpp::Service<
            ListService>::SharedPtr
              list_service_;

          rclcpp::TimerBase::SharedPtr
            status_timer_;

          rclcpp::TimerBase::SharedPtr
            heartbeat_timer_;

          std::atomic<std::uint64_t>
            heartbeat_sequence_{0U};
        };

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__LOCATION_REGISTRY_NODE_HPP_
        ''',
    )

    write(
        "src/location_registry_node.cpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_locations/location_registry_node.hpp"

        #include <chrono>
        #include <cmath>
        #include <filesystem>
        #include <sstream>
        #include <stdexcept>
        #include <string>
        #include <utility>

        #include "savo_locations/ros_conversions.hpp"
        #include "savo_locations/service_names.hpp"
        #include "savo_locations/sqlite_schema.hpp"
        #include "savo_locations/topic_names.hpp"


        namespace savo_locations
        {
        namespace
        {

        constexpr char kDefaultDatabasePath[] =
          "/var/lib/robot_savo/locations/locations.db";


        std::string json_escape(
          const std::string & value)
        {
          std::string escaped;
          escaped.reserve(value.size());

          for (const char character : value) {
            switch (character) {
              case '\\':
                escaped += "\\\\";
                break;

              case '"':
                escaped += "\\\"";
                break;

              case '\n':
                escaped += "\\n";
                break;

              case '\r':
                escaped += "\\r";
                break;

              case '\t':
                escaped += "\\t";
                break;

              default:
                escaped += character;
                break;
            }
          }

          return escaped;
        }


        std::chrono::nanoseconds period_from_hz(
          const double frequency_hz)
        {
          if (
            !std::isfinite(frequency_hz) ||
            frequency_hz <= 0.0)
          {
            throw std::invalid_argument(
              "publish frequency must be positive");
          }

          return std::chrono::duration_cast<
            std::chrono::nanoseconds>(
              std::chrono::duration<double>(
                1.0 / frequency_hz));
        }

        }  // namespace


        LocationRegistryNode::LocationRegistryNode(
          const rclcpp::NodeOptions & options)
        : Node("savo_locations", options)
        {
          database_path_ =
            declare_parameter<std::string>(
              "database_path",
              kDefaultDatabasePath);

          create_parent_directories_ =
            declare_parameter<bool>(
              "create_parent_directories",
              false);

          auto_migrate_ =
            declare_parameter<bool>(
              "auto_migrate",
              true);

          publish_snapshot_enabled_ =
            declare_parameter<bool>(
              "publish_snapshot",
              true);

          status_publish_hz_ =
            declare_parameter<double>(
              "status_publish_hz",
              1.0);

          heartbeat_publish_hz_ =
            declare_parameter<double>(
              "heartbeat_publish_hz",
              2.0);

          const auto latched_qos =
            rclcpp::QoS(
              rclcpp::KeepLast(1))
              .reliable()
              .transient_local();

          status_publisher_ =
            create_publisher<std_msgs::msg::String>(
              topics::kStatus,
              latched_qos);

          snapshot_publisher_ =
            create_publisher<std_msgs::msg::String>(
              topics::kSnapshot,
              latched_qos);

          heartbeat_publisher_ =
            create_publisher<std_msgs::msg::UInt64>(
              topics::kHeartbeat,
              rclcpp::QoS(
                rclcpp::KeepLast(10))
                .reliable()
                .durability_volatile());

          resolve_service_ =
            create_service<ResolveService>(
              services::kResolve,
              std::bind(
                &LocationRegistryNode::handle_resolve,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

          get_service_ =
            create_service<GetService>(
              services::kGet,
              std::bind(
                &LocationRegistryNode::handle_get,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

          list_service_ =
            create_service<ListService>(
              services::kList,
              std::bind(
                &LocationRegistryNode::handle_list,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

          initialize_storage();

          status_timer_ =
            create_wall_timer(
              period_from_hz(status_publish_hz_),
              std::bind(
                &LocationRegistryNode::publish_status,
                this));

          heartbeat_timer_ =
            create_wall_timer(
              period_from_hz(
                heartbeat_publish_hz_),
              std::bind(
                &LocationRegistryNode::
                  publish_heartbeat,
                this));

          publish_status();
          publish_heartbeat();

          if (publish_snapshot_enabled_) {
            publish_snapshot();
          }
        }


        bool
        LocationRegistryNode::registry_ready() const
        {
          std::shared_lock<std::shared_mutex> lock{
            state_mutex_};

          return ready_;
        }


        void
        LocationRegistryNode::initialize_storage()
        {
          std::unique_lock<std::shared_mutex> lock{
            state_mutex_};

          ready_ = false;
          storage_healthy_ = false;
          state_ = "starting";
          reason_ = "opening SQLite registry";

          try {
            if (
              database_path_.empty())
            {
              throw std::runtime_error(
                "database_path is empty");
            }

            if (
              create_parent_directories_ &&
              database_path_ != ":memory:")
            {
              const std::filesystem::path path{
                database_path_};

              const auto parent =
                path.parent_path();

              if (!parent.empty()) {
                std::filesystem::create_directories(
                  parent);
              }
            }

            store_ =
              std::make_unique<SqliteStore>(
                database_path_);

            const auto open_result =
              store_->open();

            if (!open_result.success) {
              throw std::runtime_error(
                "SQLite open failed: " +
                open_result.reason);
            }

            if (auto_migrate_) {
              SchemaStatus schema_status;

              const auto migration_result =
                store_->migrate(
                  &schema_status);

              if (!migration_result.success) {
                throw std::runtime_error(
                  "SQLite migration failed: " +
                  migration_result.reason);
              }
            }
            else
            {
              std::uint32_t schema_version = 0U;

              const auto version_result =
                store_->schema_version(
                  &schema_version);

              if (!version_result.success) {
                throw std::runtime_error(
                  "schema query failed: " +
                  version_result.reason);
              }

              if (
                schema_version !=
                kSupportedSqliteSchemaVersion)
              {
                throw std::runtime_error(
                  "database schema is not current");
              }
            }

            repository_ =
              std::make_unique<SqliteRepository>(
                *store_);

            CatalogSnapshot snapshot;
            BootstrapReport report;

            const auto bootstrap_result =
              repository_->bootstrap(
                &snapshot,
                &report);

            if (!bootstrap_result.success) {
              throw std::runtime_error(
                "catalog bootstrap failed: " +
                bootstrap_result.reason);
            }

            catalog_view_.replace(
              std::move(snapshot));

            bootstrap_report_ = report;

            ready_ = true;
            storage_healthy_ = true;
            state_ = "ready";
            reason_ =
              "persistent read-only registry ready";

            RCLCPP_INFO(
              get_logger(),
              "savo_locations ready: locations=%zu, "
              "candidates=%zu, events=%llu",
              report.location_count,
              report.candidate_count,
              static_cast<unsigned long long>(
                report.event_count));
          }
          catch (
            const std::exception & exception)
          {
            ready_ = false;
            storage_healthy_ = false;
            state_ = "degraded";
            reason_ = exception.what();

            RCLCPP_ERROR(
              get_logger(),
              "savo_locations startup degraded: %s",
              exception.what());
          }
        }


        void
        LocationRegistryNode::publish_status()
        {
          std_msgs::msg::String message;
          std::ostringstream stream;

          {
            std::shared_lock<std::shared_mutex> lock{
              state_mutex_};

            stream
              << "{"
              << "\"component\":\"savo_locations\","
              << "\"mode\":\"read_only\","
              << "\"state\":\""
              << json_escape(state_)
              << "\","
              << "\"ready\":"
              << (ready_ ? "true" : "false")
              << ","
              << "\"storage_healthy\":"
              << (
                storage_healthy_ ?
                "true" :
                "false")
              << ","
              << "\"schema_version\":"
              << bootstrap_report_.schema_version
              << ","
              << "\"location_count\":"
              << bootstrap_report_.location_count
              << ","
              << "\"candidate_count\":"
              << bootstrap_report_.candidate_count
              << ","
              << "\"event_count\":"
              << bootstrap_report_.event_count
              << ","
              << "\"last_event_sequence\":"
              << bootstrap_report_
                   .last_event_sequence
              << ","
              << "\"database_path\":\""
              << json_escape(database_path_)
              << "\","
              << "\"reason\":\""
              << json_escape(reason_)
              << "\""
              << "}";
          }

          message.data = stream.str();

          status_publisher_->publish(message);
        }


        void
        LocationRegistryNode::publish_heartbeat()
        {
          std_msgs::msg::UInt64 message;

          message.data =
            heartbeat_sequence_.fetch_add(
              1U) + 1U;

          heartbeat_publisher_->publish(message);
        }


        void
        LocationRegistryNode::publish_snapshot()
        {
          std_msgs::msg::String message;
          std::ostringstream stream;

          std::shared_lock<std::shared_mutex> lock{
            state_mutex_};

          stream
            << "{"
            << "\"schema_version\":1,"
            << "\"read_only\":true,"
            << "\"locations\":[";

          bool first = true;

          for (
            const auto & record :
            catalog_view_.locations())
          {
            if (!first) {
              stream << ",";
            }

            first = false;

            stream
              << "{"
              << "\"location_id\":\""
              << json_escape(
                   record.location.location_id)
              << "\","
              << "\"display_name\":\""
              << json_escape(
                   record.location.display_name)
              << "\","
              << "\"semantic_type\":\""
              << json_escape(
                   record.location.semantic_type)
              << "\","
              << "\"map_id\":\""
              << json_escape(
                   record.location.map.map_id)
              << "\","
              << "\"map_revision\":"
              << record.location.map.map_revision
              << ","
              << "\"state\":"
              << static_cast<unsigned int>(
                   record.state)
              << ","
              << "\"enabled\":"
              << (
                record.enabled ?
                "true" :
                "false")
              << "}";
          }

          stream << "]}";

          message.data = stream.str();

          snapshot_publisher_->publish(message);
        }


        void LocationRegistryNode::handle_resolve(
          const std::shared_ptr<
            ResolveService::Request> request,
          std::shared_ptr<
            ResolveService::Response> response)
        {
          std::shared_lock<std::shared_mutex> lock{
            state_mutex_};

          if (!ready_) {
            response->resolved = false;

            response->result_code =
              ResolveService::Response::
                RESULT_INTERNAL_ERROR;

            response->match_type =
              ResolveService::Response::
                MATCH_NONE;

            response->reason =
              "registry unavailable: " + reason_;

            return;
          }

          ReadResolveRequest view_request;

          view_request.query = request->query;

          view_request.enforce_map_context =
            request->enforce_map_context;

          view_request.map_id =
            request->map_id;

          view_request.map_revision =
            request->map_revision;

          const auto result =
            catalog_view_.resolve(view_request);

          response->reason = result.reason;

          response->normalized_query =
            result.normalized_query;

          response->match_type =
            static_cast<std::uint8_t>(
              result.match_type);

          response->ambiguous_location_ids =
            result.ambiguous_location_ids;

          switch (result.code) {
            case ReadResolveCode::kResolved:
              response->resolved = true;

              response->result_code =
                ResolveService::Response::
                  RESULT_RESOLVED;

              response->location =
                to_ros_location_record(
                  result.location);

              return;

            case ReadResolveCode::kInvalidQuery:
              response->result_code =
                ResolveService::Response::
                  RESULT_INVALID_QUERY;
              break;

            case ReadResolveCode::kNotFound:
              response->result_code =
                ResolveService::Response::
                  RESULT_NOT_FOUND;
              break;

            case ReadResolveCode::kAmbiguous:
              response->result_code =
                ResolveService::Response::
                  RESULT_AMBIGUOUS;
              break;

            case ReadResolveCode::kDisabled:
              response->result_code =
                ResolveService::Response::
                  RESULT_DISABLED;

              response->location =
                to_ros_location_record(
                  result.location);
              break;

            case ReadResolveCode::kRetired:
              response->result_code =
                ResolveService::Response::
                  RESULT_RETIRED;

              response->location =
                to_ros_location_record(
                  result.location);
              break;

            case ReadResolveCode::kMapMismatch:
              response->result_code =
                ResolveService::Response::
                  RESULT_MAP_MISMATCH;

              if (
                !result
                  .location
                  .location
                  .location_id
                  .empty())
              {
                response->location =
                  to_ros_location_record(
                    result.location);
              }
              break;
          }

          response->resolved = false;
        }


        void LocationRegistryNode::handle_get(
          const std::shared_ptr<
            GetService::Request> request,
          std::shared_ptr<
            GetService::Response> response)
        {
          std::shared_lock<std::shared_mutex> lock{
            state_mutex_};

          if (!ready_) {
            response->found = false;

            response->result_code =
              GetService::Response::
                RESULT_INTERNAL_ERROR;

            response->reason =
              "registry unavailable: " + reason_;

            return;
          }

          const auto result =
            catalog_view_.get(
              request->location_id,
              request->include_disabled,
              request->include_retired);

          response->reason = result.reason;

          switch (result.code) {
            case ReadGetCode::kFound:
              response->found = true;

              response->result_code =
                GetService::Response::
                  RESULT_FOUND;

              response->location =
                to_ros_location_record(
                  result.location);

              return;

            case ReadGetCode::kInvalidId:
              response->result_code =
                GetService::Response::
                  RESULT_INVALID_ID;
              break;

            case ReadGetCode::kNotFound:
              response->result_code =
                GetService::Response::
                  RESULT_NOT_FOUND;
              break;

            case ReadGetCode::kDisabled:
              response->result_code =
                GetService::Response::
                  RESULT_DISABLED;

              response->location =
                to_ros_location_record(
                  result.location);
              break;

            case ReadGetCode::kRetired:
              response->result_code =
                GetService::Response::
                  RESULT_RETIRED;

              response->location =
                to_ros_location_record(
                  result.location);
              break;
          }

          response->found = false;
        }


        void LocationRegistryNode::handle_list(
          const std::shared_ptr<
            ListService::Request> request,
          std::shared_ptr<
            ListService::Response> response)
        {
          std::shared_lock<std::shared_mutex> lock{
            state_mutex_};

          if (!ready_) {
            response->success = false;

            response->result_code =
              ListService::Response::
                RESULT_INTERNAL_ERROR;

            response->reason =
              "registry unavailable: " + reason_;

            return;
          }

          ReadListRequest view_request;

          view_request.map_id =
            request->map_id;

          view_request.map_revision =
            request->map_revision;

          view_request.enforce_map_context =
            request->enforce_map_context;

          view_request.semantic_type =
            request->semantic_type;

          view_request.state_filter =
            request->state_filter;

          view_request.enabled_only =
            request->enabled_only;

          const auto result =
            catalog_view_.list(view_request);

          if (!result.valid) {
            response->success = false;

            response->result_code =
              ListService::Response::
                RESULT_INVALID_FILTER;

            response->reason = result.reason;

            return;
          }

          response->success = true;

          response->result_code =
            ListService::Response::
              RESULT_OK;

          response->reason = result.reason;

          response->locations.reserve(
            result.locations.size());

          for (
            const auto & record :
            result.locations)
          {
            response->locations.push_back(
              to_ros_location_record(record));
          }
        }

        }  // namespace savo_locations
        ''',
    )

    write(
        "src/location_registry_main.cpp",
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include <memory>

        #include "rclcpp/rclcpp.hpp"

        #include "savo_locations/location_registry_node.hpp"


        int main(
          int argc,
          char ** argv)
        {
          rclcpp::init(argc, argv);

          auto node =
            std::make_shared<
              savo_locations::LocationRegistryNode>();

          rclcpp::spin(node);

          rclcpp::shutdown();

          return 0;
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # Configuration and launch
    # -------------------------------------------------------------------------

    write(
        "config/locations_node.yaml",
        r'''
        savo_locations:
          ros__parameters:
            # SQLite is the production authority. The parent directory must be
            # provisioned with ownership for the robot runtime user.
            database_path: /var/lib/robot_savo/locations/locations.db

            # Production startup may apply supported forward migrations.
            auto_migrate: true

            # System deployment should create /var/lib/robot_savo/locations.
            # Keep this false so the node does not silently alter permissions.
            create_parent_directories: false

            status_publish_hz: 1.0
            heartbeat_publish_hz: 2.0

            # Transient-local JSON diagnostic snapshot. Typed read access
            # remains ResolveLocation/GetLocation/ListLocations services.
            publish_snapshot: true
        ''',
    )

    write(
        "launch/locations_bringup.launch.py",
        r'''
        from pathlib import Path

        from ament_index_python.packages import (
            get_package_share_directory,
        )
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node


        def generate_launch_description() -> LaunchDescription:
            package_share = Path(
                get_package_share_directory("savo_locations")
            )

            default_config = (
                package_share
                / "config"
                / "locations_node.yaml"
            )

            config_file = LaunchConfiguration("config_file")

            return LaunchDescription(
                [
                    DeclareLaunchArgument(
                        "config_file",
                        default_value=str(default_config),
                    ),
                    Node(
                        package="savo_locations",
                        executable="savo_locations_node",
                        name="savo_locations",
                        output="screen",
                        parameters=[config_file],
                    ),
                ]
            )
        ''',
    )

    # -------------------------------------------------------------------------
    # Read-only view tests
    # -------------------------------------------------------------------------

    write(
        "test/ros/test_read_only_catalog_view.cpp",
        r'''
        #include <gtest/gtest.h>

        #include <string>
        #include <utility>

        #include "savo_locations/read_only_catalog_view.hpp"


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


        savo_locations::LocationRecordData make_record(
          std::string id,
          std::string name,
          std::string map_id,
          const std::uint32_t revision,
          const bool enabled = true)
        {
          savo_locations::LocationRecordData record;

          record.state =
            savo_locations::LocationState::
              kApproved;

          record.enabled = enabled;
          record.record_revision = 1U;

          record.location.location_id =
            std::move(id);

          record.location.display_name =
            std::move(name);

          record.location.semantic_type =
            "classroom";

          record.location.map.map_id =
            std::move(map_id);

          record.location.map.map_revision =
            revision;

          record.location.approach_pose =
            make_pose(1.0, 2.0);

          record.location.tag.family =
            "tag36h11";

          record.location.tag.id =
            static_cast<std::int32_t>(
              revision);

          return record;
        }


        savo_locations::ReadOnlyCatalogView make_view()
        {
          savo_locations::CatalogSnapshot snapshot;

          auto a201 = make_record(
            "A201",
            "Room A201",
            "campus_main",
            7U);

          a201.location.aliases = {
            "East classroom",
            "A 201",
          };

          auto b101 = make_record(
            "B101",
            "Room B101",
            "campus_main",
            7U,
            false);

          b101.location.aliases = {
            "Shared room",
          };

          auto c101 = make_record(
            "C101",
            "Room C101",
            "campus_annex",
            2U);

          c101.location.aliases = {
            "Shared room",
          };

          snapshot.locations = {
            c101,
            b101,
            a201,
          };

          return savo_locations::
            ReadOnlyCatalogView{
              std::move(snapshot)};
        }

        }  // namespace


        TEST(ReadOnlyCatalogView, ResolvesExactId)
        {
          const auto view = make_view();

          savo_locations::ReadResolveRequest request;
          request.query = "A201";

          const auto result =
            view.resolve(request);

          EXPECT_EQ(
            result.code,
            savo_locations::ReadResolveCode::
              kResolved);

          EXPECT_EQ(
            result.match_type,
            savo_locations::ResolveMatchType::
              kLocationId);

          EXPECT_EQ(
            result.location.location.location_id,
            "A201");
        }


        TEST(ReadOnlyCatalogView, ResolvesAlias)
        {
          const auto view = make_view();

          savo_locations::ReadResolveRequest request;
          request.query = "east classroom";

          const auto result =
            view.resolve(request);

          EXPECT_EQ(
            result.code,
            savo_locations::ReadResolveCode::
              kResolved);

          EXPECT_EQ(
            result.match_type,
            savo_locations::ResolveMatchType::
              kAlias);
        }


        TEST(ReadOnlyCatalogView, ReportsAmbiguityAcrossMaps)
        {
          const auto view = make_view();

          savo_locations::ReadResolveRequest request;
          request.query = "Shared room";

          const auto result =
            view.resolve(request);

          EXPECT_EQ(
            result.code,
            savo_locations::ReadResolveCode::
              kAmbiguous);

          ASSERT_EQ(
            result.ambiguous_location_ids.size(),
            2U);

          EXPECT_EQ(
            result.ambiguous_location_ids[0],
            "B101");

          EXPECT_EQ(
            result.ambiguous_location_ids[1],
            "C101");
        }


        TEST(ReadOnlyCatalogView, MapContextDisambiguates)
        {
          const auto view = make_view();

          savo_locations::ReadResolveRequest request;

          request.query = "Shared room";
          request.enforce_map_context = true;
          request.map_id = "campus_annex";
          request.map_revision = 2U;

          const auto result =
            view.resolve(request);

          EXPECT_EQ(
            result.code,
            savo_locations::ReadResolveCode::
              kResolved);

          EXPECT_EQ(
            result.location.location.location_id,
            "C101");
        }


        TEST(ReadOnlyCatalogView, DisabledResolveIsFailClosed)
        {
          const auto view = make_view();

          savo_locations::ReadResolveRequest request;
          request.query = "B101";

          const auto result =
            view.resolve(request);

          EXPECT_EQ(
            result.code,
            savo_locations::ReadResolveCode::
              kDisabled);
        }


        TEST(ReadOnlyCatalogView, GetHonoursDisabledPolicy)
        {
          const auto view = make_view();

          const auto blocked =
            view.get(
              "B101",
              false,
              false);

          EXPECT_EQ(
            blocked.code,
            savo_locations::ReadGetCode::
              kDisabled);

          const auto included =
            view.get(
              "B101",
              true,
              false);

          EXPECT_EQ(
            included.code,
            savo_locations::ReadGetCode::
              kFound);
        }


        TEST(ReadOnlyCatalogView, ListFiltersMapAndEnabled)
        {
          const auto view = make_view();

          savo_locations::ReadListRequest request;

          request.map_id = "campus_main";
          request.map_revision = 7U;
          request.enabled_only = true;

          const auto result =
            view.list(request);

          ASSERT_TRUE(result.valid);
          ASSERT_EQ(result.locations.size(), 1U);

          EXPECT_EQ(
            result.locations.front()
              .location
              .location_id,
            "A201");
        }


        TEST(ReadOnlyCatalogView, RejectsInvalidListContext)
        {
          const auto view = make_view();

          savo_locations::ReadListRequest request;

          request.enforce_map_context = true;
          request.map_id = "campus_main";

          const auto result =
            view.list(request);

          EXPECT_FALSE(result.valid);
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # ROS integration test
    # -------------------------------------------------------------------------

    write(
        "test/ros/test_registry_node.cpp",
        r'''
        #include <gtest/gtest.h>

        #include <chrono>
        #include <filesystem>
        #include <memory>
        #include <string>

        #include "rclcpp/rclcpp.hpp"
        #include "std_msgs/msg/string.hpp"
        #include "std_msgs/msg/u_int64.hpp"

        #include "savo_msgs/srv/get_location.hpp"
        #include "savo_msgs/srv/list_locations.hpp"
        #include "savo_msgs/srv/resolve_location.hpp"

        #include "savo_locations/location_registry_node.hpp"
        #include "savo_locations/sqlite_repository.hpp"
        #include "savo_locations/sqlite_store.hpp"


        namespace
        {

        using namespace std::chrono_literals;


        class RclcppGuard
        {
        public:
          RclcppGuard()
          {
            int argc = 0;
            char ** argv = nullptr;

            rclcpp::init(argc, argv);
          }

          ~RclcppGuard()
          {
            if (rclcpp::ok()) {
              rclcpp::shutdown();
            }
          }
        };


        savo_locations::PoseData make_pose()
        {
          savo_locations::PoseData pose;

          pose.frame_id = "map";
          pose.x = 4.0;
          pose.y = 2.0;
          pose.qw = 1.0;

          return pose;
        }


        savo_locations::LocationRecordData
        make_location()
        {
          savo_locations::LocationRecordData record;

          record.state =
            savo_locations::LocationState::
              kApproved;

          record.enabled = true;
          record.record_revision = 1U;

          record.location.location_id = "A201";
          record.location.display_name = "Room A201";

          record.location.aliases = {
            "East classroom",
          };

          record.location.semantic_type =
            "classroom";

          record.location.map.map_id =
            "campus_main";

          record.location.map.map_revision = 7U;

          record.location.approach_pose =
            make_pose();

          record.location.tag.family =
            "tag36h11";

          record.location.tag.id = 27;

          return record;
        }


        std::filesystem::path database_path()
        {
          const std::filesystem::path directory{
            SAVO_LOCATIONS_TEST_DB_DIR};

          std::filesystem::create_directories(
            directory);

          const auto path =
            directory /
            "loc3a_registry_node.sqlite3";

          std::filesystem::remove(path);

          std::filesystem::remove(
            path.string() + "-wal");

          std::filesystem::remove(
            path.string() + "-shm");

          return path;
        }


        void create_database(
          const std::filesystem::path & path)
        {
          savo_locations::SqliteStore store{
            path.string()};

          ASSERT_TRUE(store.open().success);

          savo_locations::SchemaStatus status;

          ASSERT_TRUE(
            store.migrate(&status).success);

          savo_locations::SqliteRepository
            repository{store};

          savo_locations::CatalogSnapshot snapshot;

          snapshot.locations.push_back(
            make_location());

          ASSERT_TRUE(
            repository
              .save_snapshot(snapshot)
              .success);
        }

        }  // namespace


        TEST(RegistryNode, BootstrapsAndServesReads)
        {
          RclcppGuard guard;

          const auto path = database_path();

          create_database(path);

          rclcpp::NodeOptions options;

          options.parameter_overrides(
            {
              rclcpp::Parameter(
                "database_path",
                path.string()),
              rclcpp::Parameter(
                "create_parent_directories",
                true),
              rclcpp::Parameter(
                "auto_migrate",
                true),
              rclcpp::Parameter(
                "status_publish_hz",
                20.0),
              rclcpp::Parameter(
                "heartbeat_publish_hz",
                20.0),
              rclcpp::Parameter(
                "publish_snapshot",
                true),
            });

          auto registry =
            std::make_shared<
              savo_locations::LocationRegistryNode>(
                options);

          ASSERT_TRUE(
            registry->registry_ready());

          auto client_node =
            std::make_shared<rclcpp::Node>(
              "savo_locations_test_client");

          auto resolve_client =
            client_node->create_client<
              savo_msgs::srv::ResolveLocation>(
                "/savo_locations/resolve");

          auto get_client =
            client_node->create_client<
              savo_msgs::srv::GetLocation>(
                "/savo_locations/get");

          auto list_client =
            client_node->create_client<
              savo_msgs::srv::ListLocations>(
                "/savo_locations/list");

          bool status_received = false;
          bool heartbeat_received = false;

          auto status_subscription =
            client_node->create_subscription<
              std_msgs::msg::String>(
                "/savo_locations/status",
                rclcpp::QoS(
                  rclcpp::KeepLast(1))
                  .reliable()
                  .transient_local(),
                [&status_received](
                  const std_msgs::msg::String & message)
                {
                  status_received =
                    message.data.find(
                      "\"ready\":true") !=
                    std::string::npos;
                });

          auto heartbeat_subscription =
            client_node->create_subscription<
              std_msgs::msg::UInt64>(
                "/savo_locations/heartbeat",
                rclcpp::QoS(
                  rclcpp::KeepLast(10))
                  .reliable(),
                [&heartbeat_received](
                  const std_msgs::msg::UInt64 & message)
                {
                  heartbeat_received =
                    message.data > 0U;
                });

          rclcpp::executors::
            SingleThreadedExecutor executor;

          executor.add_node(registry);
          executor.add_node(client_node);

          for (
            int attempt = 0;
            attempt < 100;
            ++attempt)
          {
            executor.spin_some();

            if (
              resolve_client->service_is_ready() &&
              get_client->service_is_ready() &&
              list_client->service_is_ready())
            {
              break;
            }

            rclcpp::sleep_for(10ms);
          }

          ASSERT_TRUE(
            resolve_client->service_is_ready());

          ASSERT_TRUE(
            get_client->service_is_ready());

          ASSERT_TRUE(
            list_client->service_is_ready());

          auto resolve_request =
            std::make_shared<
              savo_msgs::srv::
                ResolveLocation::Request>();

          resolve_request->query =
            "East classroom";

          const auto resolve_future =
            resolve_client->async_send_request(
              resolve_request);

          ASSERT_EQ(
            executor.spin_until_future_complete(
              resolve_future,
              2s),
            rclcpp::FutureReturnCode::SUCCESS);

          const auto resolve_response =
            resolve_future.get();

          EXPECT_TRUE(
            resolve_response->resolved);

          EXPECT_EQ(
            resolve_response->result_code,
            savo_msgs::srv::
              ResolveLocation::Response::
                RESULT_RESOLVED);

          EXPECT_EQ(
            resolve_response
              ->location
              .location_id,
            "A201");

          auto get_request =
            std::make_shared<
              savo_msgs::srv::
                GetLocation::Request>();

          get_request->location_id = "A201";

          const auto get_future =
            get_client->async_send_request(
              get_request);

          ASSERT_EQ(
            executor.spin_until_future_complete(
              get_future,
              2s),
            rclcpp::FutureReturnCode::SUCCESS);

          const auto get_response =
            get_future.get();

          EXPECT_TRUE(get_response->found);

          EXPECT_EQ(
            get_response->location.location_id,
            "A201");

          auto list_request =
            std::make_shared<
              savo_msgs::srv::
                ListLocations::Request>();

          list_request->map_id =
            "campus_main";

          list_request->map_revision = 7U;
          list_request->enabled_only = true;

          const auto list_future =
            list_client->async_send_request(
              list_request);

          ASSERT_EQ(
            executor.spin_until_future_complete(
              list_future,
              2s),
            rclcpp::FutureReturnCode::SUCCESS);

          const auto list_response =
            list_future.get();

          EXPECT_TRUE(list_response->success);

          ASSERT_EQ(
            list_response->locations.size(),
            1U);

          EXPECT_EQ(
            list_response
              ->locations
              .front()
              .location_id,
            "A201");

          for (
            int attempt = 0;
            attempt < 100 &&
            (
              !status_received ||
              !heartbeat_received
            );
            ++attempt)
          {
            executor.spin_some();
            rclcpp::sleep_for(10ms);
          }

          EXPECT_TRUE(status_received);
          EXPECT_TRUE(heartbeat_received);

          static_cast<void>(
            status_subscription);

          static_cast<void>(
            heartbeat_subscription);

          executor.remove_node(client_node);
          executor.remove_node(registry);
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # Contract tests
    # -------------------------------------------------------------------------

    write(
        "test/contracts/test_phase3a_contracts.py",
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
            return tuple(
                int(part)
                for part in value.split(".")
            )


        def test_package_is_loc3a_or_later() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            version = package.findtext("version")

            assert version is not None
            assert parse_version(version) >= (0, 8, 0)

            dependencies = {
                element.text
                for tag in (
                    "depend",
                    "build_depend",
                    "build_export_depend",
                    "exec_depend",
                )
                for element in package.findall(tag)
            }

            for dependency in (
                "rclcpp",
                "std_msgs",
                "geometry_msgs",
                "builtin_interfaces",
                "savo_msgs",
                "launch",
                "launch_ros",
            ):
                assert dependency in dependencies


        def test_ros_runtime_files_exist() -> None:
            for relative in (
                "include/savo_locations/"
                "read_only_catalog_view.hpp",
                "include/savo_locations/"
                "ros_conversions.hpp",
                "include/savo_locations/"
                "location_registry_node.hpp",
                "src/read_only_catalog_view.cpp",
                "src/ros_conversions.cpp",
                "src/location_registry_node.cpp",
                "src/location_registry_main.cpp",
                "config/locations_node.yaml",
                "launch/locations_bringup.launch.py",
            ):
                assert (ROOT / relative).is_file(), relative


        def test_cmake_builds_production_node() -> None:
            cmake = read("CMakeLists.txt")

            for fragment in (
                "add_library(",
                "savo_locations_ros",
                "add_executable(",
                "savo_locations_node",
                "src/location_registry_node.cpp",
                "src/location_registry_main.cpp",
                "rclcpp",
                "std_msgs",
                "test_registry_node",
                "test_phase3a_contracts",
            ):
                assert fragment in cmake


        def test_node_uses_persistent_bootstrap() -> None:
            source = read(
                "src/location_registry_node.cpp"
            )

            assert "SqliteStore" in source
            assert "SqliteRepository" in source
            assert "migrate(" in source
            assert "bootstrap(" in source
            assert "catalog_view_.replace(" in source

            assert (
                "/var/lib/robot_savo/locations/"
                "locations.db"
                in source
            )

            assert '"degraded"' in source
            assert "registry unavailable:" in source


        def test_node_exposes_only_read_services() -> None:
            source = read(
                "src/location_registry_node.cpp"
            )

            header = read(
                "include/savo_locations/"
                "location_registry_node.hpp"
            )

            combined = source + header

            for endpoint in (
                "services::kResolve",
                "services::kGet",
                "services::kList",
            ):
                assert endpoint in combined

            for forbidden in (
                "RegisterLocationCandidate",
                "ApproveLocation",
                "SetLocationEnabled",
                "services::kRegisterCandidate",
                "services::kApproveCandidate",
                "services::kSetEnabled",
            ):
                assert forbidden not in combined


        def test_status_heartbeat_and_snapshot_are_latched() -> None:
            source = read(
                "src/location_registry_node.cpp"
            )

            topics = read(
                "include/savo_locations/topic_names.hpp"
            )

            assert "/savo_locations/status" in topics
            assert "/savo_locations/heartbeat" in topics
            assert "/savo_locations/snapshot" in topics

            assert "transient_local()" in source
            assert '"mode":"read_only"' in source
            assert '"ready":' in source
            assert "storage_healthy" in source
            assert "heartbeat_sequence_" in source


        def test_resolution_remains_fail_closed() -> None:
            view = read(
                "src/read_only_catalog_view.cpp"
            )

            assert "Exact canonical-ID precedence" in view
            assert "location is disabled" in view
            assert "location is retired" in view
            assert "location query is ambiguous" in view
            assert "another map context" in view


        def test_launch_uses_installed_configuration() -> None:
            launch = read(
                "launch/locations_bringup.launch.py"
            )

            assert (
                'get_package_share_directory("savo_locations")'
                in launch
            )

            assert 'executable="savo_locations_node"' in launch
            assert 'parameters=[config_file]' in launch


        def test_loc3a_tests_are_registered() -> None:
            cmake = read("CMakeLists.txt")

            for target in (
                "test_read_only_catalog_view",
                "test_registry_node",
                "test_phase3a_contracts",
            ):
                assert target in cmake
        ''',
    )

    update_cmake()

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(
        encoding="utf-8"
    )

    if "## LOC-3A read-only ROS registry" not in readme:
        readme += clean(
            r'''

            ## LOC-3A read-only ROS registry

            LOC-3A adds the production C++ ROS registry node.

            Startup behavior:

            - opens the configured SQLite authority;
            - applies supported schema migrations when enabled;
            - performs SQLite integrity and foreign-key checks;
            - loads and validates the complete persistent catalog;
            - enters `ready` only after bootstrap succeeds;
            - remains alive but fail-closed in `degraded` state when storage
              cannot be trusted.

            Read services:

            - `/savo_locations/resolve`
            - `/savo_locations/get`
            - `/savo_locations/list`

            Runtime diagnostics:

            - `/savo_locations/status`
            - `/savo_locations/heartbeat`
            - `/savo_locations/snapshot`

            Status and snapshot use reliable transient-local JSON diagnostic
            messages. Typed location access remains the three `savo_msgs`
            read services.

            LOC-3A deliberately does not expose candidate registration,
            approval, rejection, enable/disable, import or other write paths.
            Those operations require a separately validated write phase.
            '''
        )

        readme_path.write_text(
            readme,
            encoding="utf-8",
        )

    verification_files = (
        ROOT / "CMakeLists.txt",
        ROOT / "package.xml",
        ROOT / "include/savo_locations/read_only_catalog_view.hpp",
        ROOT / "include/savo_locations/location_registry_node.hpp",
        ROOT / "src/read_only_catalog_view.cpp",
        ROOT / "src/location_registry_node.cpp",
        ROOT / "test/ros/test_registry_node.cpp",
        ROOT / "test/contracts/test_phase3a_contracts.py",
    )

    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in verification_files
    )

    required_fragments = (
        "savo_locations_node",
        "LocationRegistryNode",
        "ReadOnlyCatalogView",
        "SqliteRepository",
        "services::kResolve",
        "services::kGet",
        "services::kList",
        "test_registry_node",
        "test_phase3a_contracts",
    )

    for fragment in required_fragments:
        if fragment not in combined:
            raise RuntimeError(
                f"LOC-3A verification failed: {fragment}"
            )

    forbidden_fragments = (
        "create_service<"
        "savo_msgs::srv::RegisterLocationCandidate",
        "create_service<"
        "savo_msgs::srv::ApproveLocation",
        "create_service<"
        "savo_msgs::srv::SetLocationEnabled",
    )

    node_source = (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).read_text(encoding="utf-8")

    for fragment in forbidden_fragments:
        if fragment in node_source:
            raise RuntimeError(
                "LOC-3A accidentally added a write service"
            )

    manifest = (
        LOGS
        / f"LOC3A_savo_locations_{stamp}.sha256"
    )

    files = sorted(
        path
        for path in ROOT.rglob("*")
        if path.is_file()
    )

    manifest.write_text(
        "\n".join(
            f"{sha256(path)}  "
            f"{path.relative_to(ROOT)}"
            for path in files
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")

    print(
        "LOC-3A production read-only ROS registry "
        "node applied."
    )

    print(
        "Added persistent bootstrap, readiness, "
        "status, heartbeat, snapshot and "
        "resolve/get/list services."
    )

    print(
        "No write service or hardware runtime "
        "was added."
    )


if __name__ == "__main__":
    main()
