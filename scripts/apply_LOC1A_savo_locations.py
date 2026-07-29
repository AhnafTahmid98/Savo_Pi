#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import tarfile
import textwrap


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


def replace_once(
    path: Path,
    old: str,
    new: str,
) -> None:
    text = path.read_text(encoding="utf-8")

    if old not in text:
        if new in text:
            return

        raise RuntimeError(
            f"Expected text not found in {path}:\n{old}"
        )

    path.write_text(
        text.replace(old, new, 1),
        encoding="utf-8",
    )


def main() -> None:
    if not (ROOT / "package.xml").is_file():
        raise SystemExit(
            f"savo_locations package not found: {ROOT}"
        )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    backup = (
        BACKUPS
        / f"pre_LOC1A_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            ROOT,
            arcname="core/savo_locations",
        )

    # -------------------------------------------------------------------------
    # CMake
    # -------------------------------------------------------------------------

    write(
        "CMakeLists.txt",
        r'''
        cmake_minimum_required(VERSION 3.16)
        project(savo_locations VERSION 0.2.0 LANGUAGES CXX)

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
    # Package version
    # -------------------------------------------------------------------------

    package_xml = ROOT / "package.xml"

    replace_once(
        package_xml,
        "<version>0.1.0</version>",
        "<version>0.2.0</version>",
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
          "0.2.0"};

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
    # Pure location models
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
    # Normalization API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/normalization.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__NORMALIZATION_HPP_
        #define SAVO_LOCATIONS__NORMALIZATION_HPP_

        #include <string>
        #include <string_view>

        namespace savo_locations
        {

        [[nodiscard]]
        bool is_ascii_whitespace(char value) noexcept;

        [[nodiscard]]
        std::string trim_ascii(
          std::string_view value);

        [[nodiscard]]
        std::string collapse_ascii_whitespace(
          std::string_view value);

        [[nodiscard]]
        std::string normalize_lookup_key(
          std::string_view value);

        [[nodiscard]]
        std::string canonicalize_location_id(
          std::string_view value);

        [[nodiscard]]
        bool is_canonical_location_id(
          std::string_view value) noexcept;

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__NORMALIZATION_HPP_
        ''',
    )

    write(
        "src/normalization.cpp",
        r'''
        #include "savo_locations/normalization.hpp"

        #include "savo_locations/constants.hpp"

        namespace savo_locations
        {
        namespace
        {

        bool is_ascii_lower(
          const unsigned char value) noexcept
        {
          return value >= static_cast<unsigned char>('a') &&
                 value <= static_cast<unsigned char>('z');
        }


        bool is_ascii_upper(
          const unsigned char value) noexcept
        {
          return value >= static_cast<unsigned char>('A') &&
                 value <= static_cast<unsigned char>('Z');
        }


        bool is_ascii_digit(
          const unsigned char value) noexcept
        {
          return value >= static_cast<unsigned char>('0') &&
                 value <= static_cast<unsigned char>('9');
        }


        bool is_ascii_alphanumeric(
          const unsigned char value) noexcept
        {
          return
            is_ascii_lower(value) ||
            is_ascii_upper(value) ||
            is_ascii_digit(value);
        }


        char ascii_to_lower(
          const unsigned char value) noexcept
        {
          if (!is_ascii_upper(value)) {
            return static_cast<char>(value);
          }

          const auto offset =
            static_cast<unsigned char>(
              value -
              static_cast<unsigned char>('A'));

          return static_cast<char>(
            static_cast<unsigned char>('a') +
            offset);
        }


        char ascii_to_upper(
          const unsigned char value) noexcept
        {
          if (!is_ascii_lower(value)) {
            return static_cast<char>(value);
          }

          const auto offset =
            static_cast<unsigned char>(
              value -
              static_cast<unsigned char>('a'));

          return static_cast<char>(
            static_cast<unsigned char>('A') +
            offset);
        }

        }  // namespace


        bool is_ascii_whitespace(
          const char value) noexcept
        {
          switch (value) {
            case ' ':
            case '\t':
            case '\n':
            case '\r':
            case '\f':
            case '\v':
              return true;

            default:
              return false;
          }
        }


        std::string trim_ascii(
          const std::string_view value)
        {
          std::size_t begin = 0U;
          std::size_t end = value.size();

          while (
            begin < end &&
            is_ascii_whitespace(value[begin]))
          {
            ++begin;
          }

          while (
            end > begin &&
            is_ascii_whitespace(value[end - 1U]))
          {
            --end;
          }

          return std::string{
            value.substr(begin, end - begin)};
        }


        std::string collapse_ascii_whitespace(
          const std::string_view value)
        {
          const auto trimmed = trim_ascii(value);

          std::string output;
          output.reserve(trimmed.size());

          bool pending_space = false;

          for (const char character : trimmed) {
            if (is_ascii_whitespace(character)) {
              pending_space = !output.empty();
              continue;
            }

            if (pending_space) {
              output.push_back(' ');
              pending_space = false;
            }

            output.push_back(character);
          }

          return output;
        }


        std::string normalize_lookup_key(
          const std::string_view value)
        {
          const auto trimmed = trim_ascii(value);

          std::string output;
          output.reserve(trimmed.size());

          bool pending_separator = false;

          for (const char character : trimmed) {
            const auto byte =
              static_cast<unsigned char>(character);

            const bool is_non_ascii = byte >= 128U;

            if (
              is_ascii_alphanumeric(byte) ||
              is_non_ascii)
            {
              if (
                pending_separator &&
                !output.empty())
              {
                output.push_back(' ');
              }

              output.push_back(
                ascii_to_lower(byte));

              pending_separator = false;
              continue;
            }

            pending_separator = !output.empty();
          }

          return output;
        }


        std::string canonicalize_location_id(
          const std::string_view value)
        {
          const auto trimmed = trim_ascii(value);

          std::string output;
          output.reserve(trimmed.size());

          bool previous_separator = false;

          for (const char character : trimmed) {
            const auto byte =
              static_cast<unsigned char>(character);

            if (is_ascii_whitespace(character)) {
              if (
                !output.empty() &&
                !previous_separator)
              {
                output.push_back('_');
                previous_separator = true;
              }

              continue;
            }

            output.push_back(
              ascii_to_upper(byte));

            previous_separator =
              character == '_' ||
              character == '-';
          }

          while (
            !output.empty() &&
            output.back() == '_')
          {
            output.pop_back();
          }

          return output;
        }


        bool is_canonical_location_id(
          const std::string_view value) noexcept
        {
          if (
            value.empty() ||
            value.size() >
              kMaximumLocationIdLength)
          {
            return false;
          }

          const auto first =
            static_cast<unsigned char>(
              value.front());

          if (!is_ascii_alphanumeric(first)) {
            return false;
          }

          for (const char character : value) {
            const auto byte =
              static_cast<unsigned char>(character);

            const bool allowed =
              is_ascii_upper(byte) ||
              is_ascii_digit(byte) ||
              character == '_' ||
              character == '-';

            if (!allowed) {
              return false;
            }
          }

          return true;
        }

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # Validation API
    # -------------------------------------------------------------------------

    write(
        "include/savo_locations/validation.hpp",
        r'''
        #ifndef SAVO_LOCATIONS__VALIDATION_HPP_
        #define SAVO_LOCATIONS__VALIDATION_HPP_

        #include <cstdint>
        #include <string>
        #include <string_view>
        #include <vector>

        #include "savo_locations/model.hpp"

        namespace savo_locations
        {

        enum class ValidationCode : std::uint8_t
        {
          kNone = 0U,
          kEmptyValue,
          kTooLong,
          kInvalidFormat,
          kUnsupportedValue,
          kDuplicateNormalizedKey,
          kNonFiniteNumber,
          kOutOfRange,
          kMapRevisionZero,
          kWrongFrame,
          kInvalidQuaternion,
          kMissingRequiredPose,
        };


        struct ValidationIssue
        {
          ValidationCode code{ValidationCode::kNone};
          std::string field;
          std::string message;
        };


        class ValidationResult
        {
        public:
          [[nodiscard]]
          bool valid() const noexcept;

          [[nodiscard]]
          bool has(
            ValidationCode code) const noexcept;

          [[nodiscard]]
          const std::vector<ValidationIssue> &
          issues() const noexcept;

          void add(
            ValidationCode code,
            std::string field,
            std::string message);

        private:
          std::vector<ValidationIssue> issues_;
        };


        [[nodiscard]]
        std::string_view to_string(
          ValidationCode code) noexcept;

        [[nodiscard]]
        ValidationResult validate_map_context(
          const MapContext & context);

        [[nodiscard]]
        ValidationResult validate_pose(
          const PoseData & pose,
          std::string_view field);

        [[nodiscard]]
        ValidationResult validate_tag_binding(
          const TagBinding & tag);

        [[nodiscard]]
        ValidationResult validate_location_draft(
          const LocationDraft & location);

        [[nodiscard]]
        ValidationResult validate_candidate_draft(
          const CandidateDraft & candidate);

        }  // namespace savo_locations

        #endif  // SAVO_LOCATIONS__VALIDATION_HPP_
        ''',
    )

    write(
        "src/validation.cpp",
        r'''
        #include "savo_locations/validation.hpp"

        #include <cmath>
        #include <set>
        #include <utility>

        #include "savo_locations/constants.hpp"
        #include "savo_locations/normalization.hpp"
        #include "savo_locations/types.hpp"

        namespace savo_locations
        {
        namespace
        {

        void append(
          ValidationResult & destination,
          const ValidationResult & source)
        {
          for (const auto & issue : source.issues()) {
            destination.add(
              issue.code,
              issue.field,
              issue.message);
          }
        }


        void validate_required_text(
          ValidationResult & result,
          const std::string_view value,
          const std::string_view field,
          const std::size_t maximum_length)
        {
          const auto trimmed = trim_ascii(value);

          if (trimmed.empty()) {
            result.add(
              ValidationCode::kEmptyValue,
              std::string{field},
              "value is required");

            return;
          }

          if (trimmed.size() > maximum_length) {
            result.add(
              ValidationCode::kTooLong,
              std::string{field},
              "value exceeds maximum length");
          }
        }


        void validate_optional_text(
          ValidationResult & result,
          const std::string_view value,
          const std::string_view field,
          const std::size_t maximum_length)
        {
          if (value.empty()) {
            return;
          }

          if (
            trim_ascii(value).size() >
            maximum_length)
          {
            result.add(
              ValidationCode::kTooLong,
              std::string{field},
              "value exceeds maximum length");
          }
        }


        bool finite(
          const double value) noexcept
        {
          return std::isfinite(value);
        }


        void validate_non_negative_finite(
          ValidationResult & result,
          const double value,
          const std::string_view field)
        {
          if (!finite(value)) {
            result.add(
              ValidationCode::kNonFiniteNumber,
              std::string{field},
              "value must be finite");

            return;
          }

          if (value < 0.0) {
            result.add(
              ValidationCode::kOutOfRange,
              std::string{field},
              "value must be non-negative");
          }
        }


        void validate_identity_aliases(
          ValidationResult & result,
          const std::string_view location_id,
          const std::string_view display_name,
          const std::vector<std::string> & aliases,
          const std::string_view field_prefix)
        {
          if (aliases.size() > kMaximumAliasCount) {
            result.add(
              ValidationCode::kTooLong,
              std::string{field_prefix},
              "alias count exceeds maximum");

            return;
          }

          std::set<std::string> normalized_keys;

          const auto normalized_id =
            normalize_lookup_key(location_id);

          const auto normalized_display_name =
            normalize_lookup_key(display_name);

          if (!normalized_id.empty()) {
            normalized_keys.insert(normalized_id);
          }

          if (!normalized_display_name.empty()) {
            normalized_keys.insert(
              normalized_display_name);
          }

          for (
            std::size_t index = 0U;
            index < aliases.size();
            ++index)
          {
            const auto field =
              std::string{field_prefix} +
              "[" +
              std::to_string(index) +
              "]";

            const auto trimmed =
              trim_ascii(aliases[index]);

            if (trimmed.empty()) {
              result.add(
                ValidationCode::kEmptyValue,
                field,
                "alias must not be empty");

              continue;
            }

            if (
              trimmed.size() >
              kMaximumAliasLength)
            {
              result.add(
                ValidationCode::kTooLong,
                field,
                "alias exceeds maximum length");

              continue;
            }

            const auto key =
              normalize_lookup_key(trimmed);

            if (key.empty()) {
              result.add(
                ValidationCode::kInvalidFormat,
                field,
                "alias does not contain a searchable key");

              continue;
            }

            const auto inserted =
              normalized_keys.insert(key);

            if (!inserted.second) {
              result.add(
                ValidationCode::
                  kDuplicateNormalizedKey,
                field,
                "alias duplicates another identity key");
            }
          }
        }


        void validate_semantic_type(
          ValidationResult & result,
          const std::string_view value,
          const std::string_view field,
          const bool required)
        {
          const auto trimmed = trim_ascii(value);

          if (trimmed.empty()) {
            if (required) {
              result.add(
                ValidationCode::kEmptyValue,
                std::string{field},
                "semantic type is required");
            }

            return;
          }

          const auto parsed =
            semantic_type_from_string(trimmed);

          if (!parsed.has_value()) {
            result.add(
              ValidationCode::kUnsupportedValue,
              std::string{field},
              "semantic type is not supported");
          }
        }

        }  // namespace


        bool ValidationResult::valid() const noexcept
        {
          return issues_.empty();
        }


        bool ValidationResult::has(
          const ValidationCode code) const noexcept
        {
          for (const auto & issue : issues_) {
            if (issue.code == code) {
              return true;
            }
          }

          return false;
        }


        const std::vector<ValidationIssue> &
        ValidationResult::issues() const noexcept
        {
          return issues_;
        }


        void ValidationResult::add(
          const ValidationCode code,
          std::string field,
          std::string message)
        {
          issues_.push_back(
            ValidationIssue{
              code,
              std::move(field),
              std::move(message)});
        }


        std::string_view to_string(
          const ValidationCode code) noexcept
        {
          switch (code) {
            case ValidationCode::kNone:
              return "none";

            case ValidationCode::kEmptyValue:
              return "empty_value";

            case ValidationCode::kTooLong:
              return "too_long";

            case ValidationCode::kInvalidFormat:
              return "invalid_format";

            case ValidationCode::kUnsupportedValue:
              return "unsupported_value";

            case ValidationCode::
              kDuplicateNormalizedKey:
              return "duplicate_normalized_key";

            case ValidationCode::kNonFiniteNumber:
              return "non_finite_number";

            case ValidationCode::kOutOfRange:
              return "out_of_range";

            case ValidationCode::kMapRevisionZero:
              return "map_revision_zero";

            case ValidationCode::kWrongFrame:
              return "wrong_frame";

            case ValidationCode::kInvalidQuaternion:
              return "invalid_quaternion";

            case ValidationCode::kMissingRequiredPose:
              return "missing_required_pose";

            default:
              return "unknown";
          }
        }


        ValidationResult validate_map_context(
          const MapContext & context)
        {
          ValidationResult result;

          validate_required_text(
            result,
            context.map_id,
            "map.map_id",
            kMaximumMapIdLength);

          if (context.map_revision == 0U) {
            result.add(
              ValidationCode::kMapRevisionZero,
              "map.map_revision",
              "map revision must be greater than zero");
          }

          validate_optional_text(
            result,
            context.map_release_id,
            "map.map_release_id",
            kMaximumMapReleaseIdLength);

          return result;
        }


        ValidationResult validate_pose(
          const PoseData & pose,
          const std::string_view field)
        {
          ValidationResult result;

          if (pose.frame_id != kCanonicalMapFrame) {
            result.add(
              ValidationCode::kWrongFrame,
              std::string{field} + ".frame_id",
              "pose frame must be map");
          }

          const bool all_finite =
            finite(pose.x) &&
            finite(pose.y) &&
            finite(pose.z) &&
            finite(pose.qx) &&
            finite(pose.qy) &&
            finite(pose.qz) &&
            finite(pose.qw);

          if (!all_finite) {
            result.add(
              ValidationCode::kNonFiniteNumber,
              std::string{field},
              "pose values must all be finite");

            return result;
          }

          const double norm_squared =
            pose.qx * pose.qx +
            pose.qy * pose.qy +
            pose.qz * pose.qz +
            pose.qw * pose.qw;

          const double norm =
            std::sqrt(norm_squared);

          if (
            std::abs(norm - 1.0) >
            kQuaternionNormTolerance)
          {
            result.add(
              ValidationCode::kInvalidQuaternion,
              std::string{field} + ".orientation",
              "quaternion must be normalized");
          }

          return result;
        }


        ValidationResult validate_tag_binding(
          const TagBinding & tag)
        {
          ValidationResult result;

          validate_required_text(
            result,
            tag.family,
            "tag.family",
            kMaximumTagFamilyLength);

          if (tag.id < 0) {
            result.add(
              ValidationCode::kOutOfRange,
              "tag.id",
              "AprilTag ID must be non-negative");
          }

          return result;
        }


        ValidationResult validate_location_draft(
          const LocationDraft & location)
        {
          ValidationResult result;

          validate_required_text(
            result,
            location.location_id,
            "location_id",
            kMaximumLocationIdLength);

          if (
            !location.location_id.empty() &&
            !is_canonical_location_id(
              location.location_id))
          {
            result.add(
              ValidationCode::kInvalidFormat,
              "location_id",
              "location ID is not canonical");
          }

          validate_required_text(
            result,
            location.display_name,
            "display_name",
            kMaximumDisplayNameLength);

          validate_semantic_type(
            result,
            location.semantic_type,
            "semantic_type",
            true);

          validate_identity_aliases(
            result,
            location.location_id,
            location.display_name,
            location.aliases,
            "aliases");

          append(
            result,
            validate_map_context(location.map));

          append(
            result,
            validate_pose(
              location.approach_pose,
              "approach_pose"));

          if (location.confirmation_pose.has_value()) {
            append(
              result,
              validate_pose(
                location.confirmation_pose.value(),
                "confirmation_pose"));
          }

          if (location.tag_pose_map.has_value()) {
            append(
              result,
              validate_pose(
                location.tag_pose_map.value(),
                "tag_pose_map"));
          }

          append(
            result,
            validate_tag_binding(location.tag));

          return result;
        }


        ValidationResult validate_candidate_draft(
          const CandidateDraft & candidate)
        {
          ValidationResult result;

          validate_required_text(
            result,
            candidate.candidate_id,
            "candidate_id",
            kMaximumCandidateIdLength);

          append(
            result,
            validate_map_context(candidate.map));

          append(
            result,
            validate_tag_binding(candidate.tag));

          append(
            result,
            validate_pose(
              candidate.tag_pose_map,
              "tag_pose_map"));

          if (!finite(candidate.detection_quality)) {
            result.add(
              ValidationCode::kNonFiniteNumber,
              "detection_quality",
              "detection quality must be finite");
          } else if (
            candidate.detection_quality < 0.0 ||
            candidate.detection_quality > 1.0)
          {
            result.add(
              ValidationCode::kOutOfRange,
              "detection_quality",
              "detection quality must be within [0, 1]");
          }

          if (candidate.accepted_observations == 0U) {
            result.add(
              ValidationCode::kOutOfRange,
              "accepted_observations",
              "at least one observation is required");
          }

          validate_non_negative_finite(
            result,
            candidate.position_stddev_m,
            "position_stddev_m");

          validate_non_negative_finite(
            result,
            candidate.yaw_stddev_rad,
            "yaw_stddev_rad");

          if (candidate.approach_pose.has_value()) {
            append(
              result,
              validate_pose(
                candidate.approach_pose.value(),
                "approach_pose"));
          }

          if (
            candidate.confirmation_pose.has_value())
          {
            append(
              result,
              validate_pose(
                candidate.confirmation_pose.value(),
                "confirmation_pose"));
          }

          if (
            !candidate.suggested_location_id.empty() &&
            !is_canonical_location_id(
              candidate.suggested_location_id))
          {
            result.add(
              ValidationCode::kInvalidFormat,
              "suggested_location_id",
              "suggested location ID is not canonical");
          }

          validate_optional_text(
            result,
            candidate.suggested_display_name,
            "suggested_display_name",
            kMaximumDisplayNameLength);

          validate_semantic_type(
            result,
            candidate.suggested_semantic_type,
            "suggested_semantic_type",
            false);

          validate_identity_aliases(
            result,
            candidate.suggested_location_id,
            candidate.suggested_display_name,
            candidate.suggested_aliases,
            "suggested_aliases");

          return result;
        }

        }  // namespace savo_locations
        ''',
    )

    # -------------------------------------------------------------------------
    # Normalization tests
    # -------------------------------------------------------------------------

    write(
        "test/unit/test_normalization.cpp",
        r'''
        #include <gtest/gtest.h>

        #include "savo_locations/normalization.hpp"


        TEST(LocationNormalization, TrimsAsciiWhitespace)
        {
          EXPECT_EQ(
            savo_locations::trim_ascii(
              " \t Room A201 \n"),
            "Room A201");
        }


        TEST(LocationNormalization, CollapsesWhitespace)
        {
          EXPECT_EQ(
            savo_locations::
              collapse_ascii_whitespace(
                "  Room \t A201 \n East  "),
            "Room A201 East");
        }


        TEST(LocationNormalization, BuildsStableLookupKeys)
        {
          EXPECT_EQ(
            savo_locations::normalize_lookup_key(
              "  Room-A_201 / East  "),
            "room a 201 east");

          EXPECT_EQ(
            savo_locations::normalize_lookup_key(
              "A201"),
            "a201");
        }


        TEST(LocationNormalization, CanonicalizesLocationIds)
        {
          EXPECT_EQ(
            savo_locations::
              canonicalize_location_id(
                "  a 201  "),
            "A_201");

          EXPECT_EQ(
            savo_locations::
              canonicalize_location_id(
                "lab-west"),
            "LAB-WEST");
        }


        TEST(LocationNormalization, ValidatesCanonicalIds)
        {
          EXPECT_TRUE(
            savo_locations::
              is_canonical_location_id(
                "A201"));

          EXPECT_TRUE(
            savo_locations::
              is_canonical_location_id(
                "LAB-WEST_2"));

          EXPECT_FALSE(
            savo_locations::
              is_canonical_location_id(
                "a201"));

          EXPECT_FALSE(
            savo_locations::
              is_canonical_location_id(
                "_A201"));

          EXPECT_FALSE(
            savo_locations::
              is_canonical_location_id(
                "A 201"));
        }


        TEST(LocationNormalization, PreservesUtf8Bytes)
        {
          EXPECT_EQ(
            savo_locations::normalize_lookup_key(
              "  Käytävä 2  "),
            "käytävä 2");
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # Validation tests
    # -------------------------------------------------------------------------

    write(
        "test/unit/test_validation.cpp",
        r'''
        #include <gtest/gtest.h>

        #include <limits>

        #include "savo_locations/model.hpp"
        #include "savo_locations/validation.hpp"


        namespace
        {

        savo_locations::LocationDraft
        make_valid_location()
        {
          savo_locations::LocationDraft location;

          location.location_id = "A201";
          location.display_name = "Room A201";
          location.aliases = {
            "A 201",
            "Classroom A201",
          };

          location.semantic_type = "classroom";

          location.map.map_id = "campus_main";
          location.map.map_revision = 7U;
          location.map.map_release_id =
            "campus_main_release_2026_07";

          location.approach_pose.frame_id = "map";
          location.approach_pose.x = 12.4;
          location.approach_pose.y = 7.8;
          location.approach_pose.qw = 1.0;

          savo_locations::PoseData tag_pose;
          tag_pose.frame_id = "map";
          tag_pose.x = 12.8;
          tag_pose.y = 8.1;
          tag_pose.qw = 1.0;

          location.tag_pose_map = tag_pose;

          location.tag.family = "tag36h11";
          location.tag.id = 27;

          return location;
        }


        savo_locations::CandidateDraft
        make_valid_candidate()
        {
          savo_locations::CandidateDraft candidate;

          candidate.candidate_id =
            "candidate-campus-main-27";

          candidate.map.map_id = "campus_main";
          candidate.map.map_revision = 7U;

          candidate.tag.family = "tag36h11";
          candidate.tag.id = 27;

          candidate.tag_pose_map.frame_id = "map";
          candidate.tag_pose_map.x = 12.8;
          candidate.tag_pose_map.y = 8.1;
          candidate.tag_pose_map.qw = 1.0;

          candidate.detection_quality = 0.95;
          candidate.accepted_observations = 8U;
          candidate.position_stddev_m = 0.02;
          candidate.yaw_stddev_rad = 0.03;

          candidate.suggested_location_id = "A201";
          candidate.suggested_display_name =
            "Room A201";

          candidate.suggested_aliases = {
            "A 201",
          };

          candidate.suggested_semantic_type =
            "classroom";

          return candidate;
        }

        }  // namespace


        TEST(LocationValidation, AcceptsValidLocation)
        {
          const auto result =
            savo_locations::validate_location_draft(
              make_valid_location());

          EXPECT_TRUE(result.valid());
          EXPECT_TRUE(result.issues().empty());
        }


        TEST(LocationValidation, RejectsNonCanonicalId)
        {
          auto location = make_valid_location();
          location.location_id = "a 201";

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_FALSE(result.valid());

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kInvalidFormat));
        }


        TEST(LocationValidation, RejectsZeroMapRevision)
        {
          auto location = make_valid_location();
          location.map.map_revision = 0U;

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kMapRevisionZero));
        }


        TEST(LocationValidation, RejectsWrongPoseFrame)
        {
          auto location = make_valid_location();

          location.approach_pose.frame_id =
            "base_link";

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kWrongFrame));
        }


        TEST(LocationValidation, RejectsInvalidQuaternion)
        {
          auto location = make_valid_location();

          location.approach_pose.qx = 0.0;
          location.approach_pose.qy = 0.0;
          location.approach_pose.qz = 0.0;
          location.approach_pose.qw = 0.0;

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kInvalidQuaternion));
        }


        TEST(LocationValidation, RejectsDuplicateAliasKeys)
        {
          auto location = make_valid_location();

          location.aliases.push_back(
            "room-a201");

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kDuplicateNormalizedKey));
        }


        TEST(LocationValidation, RejectsNonFinitePose)
        {
          auto location = make_valid_location();

          location.approach_pose.x =
            std::numeric_limits<double>::
              quiet_NaN();

          const auto result =
            savo_locations::validate_location_draft(
              location);

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kNonFiniteNumber));
        }


        TEST(LocationValidation, AcceptsValidCandidate)
        {
          const auto result =
            savo_locations::validate_candidate_draft(
              make_valid_candidate());

          EXPECT_TRUE(result.valid());
        }


        TEST(LocationValidation, RejectsInvalidCandidateQuality)
        {
          auto candidate = make_valid_candidate();

          candidate.detection_quality = 1.5;
          candidate.accepted_observations = 0U;

          const auto result =
            savo_locations::validate_candidate_draft(
              candidate);

          EXPECT_FALSE(result.valid());

          EXPECT_TRUE(
            result.has(
              savo_locations::ValidationCode::
                kOutOfRange));
        }


        TEST(LocationValidation, ReasonStringsAreStable)
        {
          using savo_locations::ValidationCode;
          using savo_locations::to_string;

          EXPECT_EQ(
            to_string(
              ValidationCode::
                kDuplicateNormalizedKey),
            "duplicate_normalized_key");

          EXPECT_EQ(
            to_string(
              ValidationCode::
                kInvalidQuaternion),
            "invalid_quaternion");
        }
        ''',
    )

    # -------------------------------------------------------------------------
    # Contract tests
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


        def test_package_version_is_loc1a() -> None:
            package = ET.parse(
                ROOT / "package.xml"
            ).getroot()

            assert package.findtext("version") == "0.2.0"

            constants = read(
                "include/savo_locations/constants.hpp"
            )

            assert '"0.2.0"' in constants


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


        def test_loc1a_remains_runtime_independent() -> None:
            cmake = read("CMakeLists.txt")

            assert "find_package(rclcpp" not in cmake
            assert "find_package(SQLite3" not in cmake
            assert "add_executable(" not in cmake

            assert not (ROOT / "launch").exists()
            assert not (ROOT / "src/location_registry_node.cpp").exists()


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
    # Policy
    # -------------------------------------------------------------------------

    policy_path = ROOT / "config" / "location_policy.yaml"
    policy = policy_path.read_text(encoding="utf-8")

    if "\nnormalization:\n" not in policy:
        policy += clean(
            r'''

            normalization:
              trim_ascii_whitespace: true
              collapse_ascii_whitespace: true
              lookup_ascii_case: lowercase
              lookup_ascii_punctuation_as_separator: true
              location_id_ascii_case: uppercase
              location_id_spaces_to_underscore: true

              # LOC-1A deliberately avoids locale-dependent C/C++ case
              # conversion. Non-ASCII UTF-8 bytes are preserved exactly.
              unicode_casefold_enabled: false
            '''
        )

        policy_path.write_text(
            policy,
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # README phase note
    # -------------------------------------------------------------------------

    readme_path = ROOT / "README.md"
    readme = readme_path.read_text(encoding="utf-8")

    if "## LOC-1A normalization and validation" not in readme:
        readme += clean(
            r'''

            ## LOC-1A normalization and validation

            LOC-1A adds a ROS-independent deterministic C++ domain core.

            It provides:

            - ASCII whitespace trimming and collapsing;
            - deterministic lookup-key normalization;
            - canonical location-ID conversion and validation;
            - pure C++ map, pose, tag, location and candidate models;
            - finite numeric-value validation;
            - mandatory `map` frame validation;
            - normalized quaternion validation;
            - alias collision detection;
            - semantic-type validation;
            - map-revision and AprilTag validation;
            - stable machine-readable validation reason codes.

            LOC-1A does not introduce a database, ROS node, service server,
            launch file, mapping integration or navigation integration.

            Lookup normalization performs ASCII case conversion only.
            Non-ASCII UTF-8 bytes are preserved exactly so behavior does not
            depend on the operating-system locale. Full Unicode case folding
            may be added later through an explicitly selected dependency.
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
        / f"LOC1A_savo_locations_{stamp}.sha256"
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
        "LOC-1A normalization and validation core applied."
    )
    print(
        "No SQLite, ROS node or hardware runtime package was added."
    )


if __name__ == "__main__":
    main()
