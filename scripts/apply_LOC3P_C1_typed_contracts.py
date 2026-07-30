#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import py_compile
import tarfile
import textwrap
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path.home() / "Savo_Pi"
WS = ROOT / "savo_ws"
SRC = WS / "src"
SCRIPTS = ROOT / "scripts"
BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


def find_package(name: str) -> Path:
    matches: list[Path] = []
    for package_xml in SRC.rglob("package.xml"):
        try:
            root = ET.parse(package_xml).getroot()
        except (ET.ParseError, OSError):
            continue
        if root.findtext("name") == name:
            matches.append(package_xml.parent)
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {name} package, found {len(matches)}: {matches}"
        )
    return matches[0]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def update_xml_version(path: Path, expected: str, new: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    node = root.find("version")
    if node is None or node.text != expected:
        raise RuntimeError(
            f"{path}: expected version {expected}, found {None if node is None else node.text}"
        )
    node.text = new
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def ensure_xml_depend(path: Path, dependency: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    existing = {
        (node.text or "").strip()
        for tag in ("depend", "build_depend", "exec_depend")
        for node in root.findall(tag)
    }
    if dependency in existing:
        return

    insertion_index = len(root)
    for index, node in enumerate(list(root)):
        if node.tag in {"test_depend", "export"}:
            insertion_index = index
            break

    element = ET.Element("depend")
    element.text = dependency
    root.insert(insertion_index, element)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    if not SRC.is_dir():
        raise RuntimeError(f"workspace source directory missing: {SRC}")

    SCRIPTS.mkdir(parents=True, exist_ok=True)
    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    msgs = find_package("savo_msgs")
    head = find_package("savo_head")
    mapping = find_package("savo_mapping")
    nav = find_package("savo_nav")
    supervisor = find_package("savo_supervisor")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = BACKUPS / f"pre_LOC3P_C1_typed_contracts_{stamp}.tar.gz"
    manifest = LOGS / f"LOC3P_C1_typed_contracts_{stamp}.sha256"

    with tarfile.open(backup, "w:gz") as tar:
        for package in (msgs, head, mapping, nav, supervisor):
            tar.add(package, arcname=str(package.relative_to(SRC)))

    # ------------------------------------------------------------------
    # savo_msgs 0.5.0: typed umbrella integration contracts.
    # ------------------------------------------------------------------
    update_xml_version(msgs / "package.xml", "0.4.0", "0.5.0")

    register_action = textwrap.dedent(
        """\
        # Robot Savo — mapping-owned typed semantic-location candidate workflow.
        #
        # Ownership:
        # - savo_head confirms camera-derived AprilTag evidence.
        # - savo_mapping validates/georeferences the candidate and calls
        #   savo_locations/candidates/register.
        # - savo_locations remains the only persistent location authority.

        int32 ANY_TAG_ID=-1

        string request_id
        string actor_id
        string candidate_id

        string expected_family
        int32 expected_tag_id

        string map_id
        uint32 map_revision
        string map_release_id

        string suggested_location_id
        string suggested_display_name
        string[] suggested_aliases
        string suggested_semantic_type

        bool approach_pose_valid
        geometry_msgs/PoseStamped approach_pose

        bool confirmation_pose_valid
        geometry_msgs/PoseStamped confirmation_pose

        string building
        string floor
        string area
        string notes

        string source_session_id
        builtin_interfaces/Duration timeout
        ---
        uint8 RESULT_REGISTERED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_SUPERVISOR_DENIED=2
        uint8 RESULT_HEAD_UNAVAILABLE=3
        uint8 RESULT_CONFIRMATION_FAILED=4
        uint8 RESULT_LOCATION_REGISTRY_UNAVAILABLE=5
        uint8 RESULT_REGISTRATION_REJECTED=6
        uint8 RESULT_TIMED_OUT=7
        uint8 RESULT_CANCELED=8
        uint8 RESULT_INTERNAL_ERROR=9

        bool registered
        uint8 result_code
        string reason
        savo_msgs/LocationCandidate candidate
        ---
        uint8 STATE_VALIDATING=0
        uint8 STATE_REQUESTING_AUTHORIZATION=1
        uint8 STATE_CONFIRMING_TAG=2
        uint8 STATE_BUILDING_CANDIDATE=3
        uint8 STATE_PERSISTING_CANDIDATE=4

        uint8 state
        string state_text
        uint32 accepted_observations
        float32 detection_quality
        int32 current_tag_id
        """
    )
    write(msgs / "action" / "RegisterMappedLocation.action", register_action)

    navigate_action = textwrap.dedent(
        """\
        # Robot Savo — navigation-owned semantic destination workflow.
        #
        # The server must resolve the query through savo_locations and forward
        # only LocationRecord.approach_pose to the existing validated navigation
        # gateway. tag_pose_map is landmark evidence and is never a motion goal.

        string request_id
        string actor_id
        string query

        bool enforce_map_context
        string map_id
        uint32 map_revision

        bool require_arrival_confirmation
        builtin_interfaces/Duration timeout
        ---
        uint8 RESULT_SUCCEEDED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_LOCATION_NOT_FOUND=2
        uint8 RESULT_LOCATION_AMBIGUOUS=3
        uint8 RESULT_LOCATION_DISABLED=4
        uint8 RESULT_MAP_MISMATCH=5
        uint8 RESULT_SUPERVISOR_DENIED=6
        uint8 RESULT_NAVIGATION_UNAVAILABLE=7
        uint8 RESULT_NAVIGATION_REJECTED=8
        uint8 RESULT_NAVIGATION_FAILED=9
        uint8 RESULT_HEAD_UNAVAILABLE=10
        uint8 RESULT_ARRIVAL_CONFIRMATION_FAILED=11
        uint8 RESULT_TIMED_OUT=12
        uint8 RESULT_CANCELED=13
        uint8 RESULT_INTERNAL_ERROR=14

        bool succeeded
        uint8 result_code
        string reason

        savo_msgs/LocationRecord location
        bool navigation_succeeded
        bool arrival_confirmed
        savo_msgs/AprilTagObservation final_observation
        ---
        uint8 STATE_RESOLVING_LOCATION=0
        uint8 STATE_REQUESTING_AUTHORIZATION=1
        uint8 STATE_NAVIGATING_TO_APPROACH_POSE=2
        uint8 STATE_CONFIRMING_ARRIVAL=3
        uint8 STATE_COMPLETING=4

        uint8 state
        string state_text
        string location_id
        float32 distance_remaining_m
        uint32 navigation_recoveries
        uint32 accepted_tag_observations
        """
    )
    write(msgs / "action" / "NavigateToLocation.action", navigate_action)

    authorize_service = textwrap.dedent(
        """\
        # Robot Savo — supervisor-owned operation authorization.
        #
        # This service grants permission only. It never performs mapping,
        # persistence, head motion, tag confirmation, or navigation.

        uint8 OP_REGISTER_LOCATION_CANDIDATE=1
        uint8 OP_APPROVE_LOCATION=2
        uint8 OP_NAVIGATE_TO_LOCATION=3
        uint8 OP_CONFIRM_LOCATION_ARRIVAL=4

        uint8 operation
        string request_id
        string actor_id
        string candidate_id
        string location_id
        string map_id
        uint32 map_revision
        bool motion_required
        ---
        uint8 RESULT_AUTHORIZED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_SUPERVISOR_NOT_READY=2
        uint8 RESULT_HEALTH_BLOCKED=3
        uint8 RESULT_SAFETY_BLOCKED=4
        uint8 RESULT_MAP_CONTEXT_BLOCKED=5
        uint8 RESULT_OPERATION_DISABLED=6
        uint8 RESULT_INTERNAL_ERROR=7

        bool authorized
        uint8 result_code
        string reason
        string supervisor_lifecycle
        string supervisor_health
        string supervisor_safety
        builtin_interfaces/Time evaluated_at
        """
    )
    write(msgs / "srv" / "AuthorizeLocationOperation.srv", authorize_service)

    msgs_cmake_path = msgs / "CMakeLists.txt"
    msgs_cmake = msgs_cmake_path.read_text(encoding="utf-8")
    msgs_cmake = replace_once(
        msgs_cmake,
        '  "action/RotateToHeading.action"\n)',
        '  "action/RotateToHeading.action"\n'
        '  "action/RegisterMappedLocation.action"\n'
        '  "action/NavigateToLocation.action"\n)',
        "savo_msgs action registration",
    )
    msgs_cmake = replace_once(
        msgs_cmake,
        '  "srv/RecoverLocationStorage.srv"\n)',
        '  "srv/RecoverLocationStorage.srv"\n'
        '  "srv/AuthorizeLocationOperation.srv"\n)',
        "savo_msgs service registration",
    )
    msgs_cmake = replace_once(
        msgs_cmake,
        'ament_add_pytest_test(\n    test_location_interfaces\n    test/test_location_interfaces.py\n    TIMEOUT 60\n  )\n',
        'ament_add_pytest_test(\n'
        '    test_location_interfaces\n'
        '    test/test_location_interfaces.py\n'
        '    TIMEOUT 60\n'
        '  )\n\n'
        '  ament_add_pytest_test(\n'
        '    test_location_integration_interfaces\n'
        '    test/test_location_integration_interfaces.py\n'
        '    TIMEOUT 60\n'
        '  )\n',
        "savo_msgs integration test registration",
    )
    write(msgs_cmake_path, msgs_cmake)

    apriltag_test_path = msgs / "test" / "test_apriltag_interfaces.py"
    apriltag_test = apriltag_test_path.read_text(encoding="utf-8")
    apriltag_test = replace_once(
        apriltag_test,
        'assert root.findtext("version") == "0.4.0"',
        'assert root.findtext("version") == "0.5.0"',
        "savo_msgs metadata version",
    )
    write(apriltag_test_path, apriltag_test)

    integration_test = textwrap.dedent(
        """\
        from pathlib import Path


        ROOT = Path(__file__).resolve().parents[1]


        def read(path: str) -> str:
            return (ROOT / path).read_text(encoding="utf-8")


        def test_typed_location_integration_interfaces_are_registered() -> None:
            cmake = read("CMakeLists.txt")

            for path in (
                "action/RegisterMappedLocation.action",
                "action/NavigateToLocation.action",
                "srv/AuthorizeLocationOperation.srv",
            ):
                assert f'"{path}"' in cmake


        def test_mapping_registration_ownership_contract() -> None:
            text = read("action/RegisterMappedLocation.action")

            for token in (
                "string candidate_id",
                "string expected_family",
                "int32 expected_tag_id",
                "string map_id",
                "uint32 map_revision",
                "geometry_msgs/PoseStamped approach_pose",
                "savo_msgs/LocationCandidate candidate",
                "RESULT_SUPERVISOR_DENIED=2",
                "RESULT_REGISTRATION_REJECTED=6",
            ):
                assert token in text

            assert "LocationRecord" not in text


        def test_navigation_uses_semantic_record_and_arrival_confirmation() -> None:
            text = read("action/NavigateToLocation.action")

            for token in (
                "string query",
                "bool enforce_map_context",
                "savo_msgs/LocationRecord location",
                "bool navigation_succeeded",
                "bool arrival_confirmed",
                "savo_msgs/AprilTagObservation final_observation",
                "STATE_NAVIGATING_TO_APPROACH_POSE=2",
                "STATE_CONFIRMING_ARRIVAL=3",
            ):
                assert token in text

            assert "geometry_msgs/PoseStamped tag_pose_map" not in text


        def test_supervisor_authorization_is_permission_only() -> None:
            text = read("srv/AuthorizeLocationOperation.srv")

            for token in (
                "OP_REGISTER_LOCATION_CANDIDATE=1",
                "OP_APPROVE_LOCATION=2",
                "OP_NAVIGATE_TO_LOCATION=3",
                "OP_CONFIRM_LOCATION_ARRIVAL=4",
                "bool motion_required",
                "bool authorized",
                "RESULT_SAFETY_BLOCKED=4",
                "builtin_interfaces/Time evaluated_at",
            ):
                assert token in text
        """
    )
    write(msgs / "test" / "test_location_integration_interfaces.py", integration_test)

    # ------------------------------------------------------------------
    # savo_head: typed action/topic ownership names, legacy JSON preserved.
    # ------------------------------------------------------------------
    ensure_xml_depend(head / "package.xml", "savo_msgs")
    ensure_xml_depend(head / "package.xml", "rclcpp_action")

    head_contract = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: Apache-2.0

        #pragma once

        #include <cstdint>
        #include <string_view>

        namespace savo_head::apriltag_contract
        {

        inline constexpr std::string_view kTypedObservationTopic{
          "/savo_head/apriltag/observations"};

        inline constexpr std::string_view kConfirmAction{
          "/savo_head/apriltag/confirm"};

        enum class Duty : std::uint8_t
        {
          kRegisterLocation = 1U,
          kConfirmArrival = 2U,
        };

        [[nodiscard]] constexpr bool IsValidDuty(const std::uint8_t value)
        {
          return value == static_cast<std::uint8_t>(Duty::kRegisterLocation) ||
                 value == static_cast<std::uint8_t>(Duty::kConfirmArrival);
        }

        }  // namespace savo_head::apriltag_contract
        """
    )
    write(
        head / "include" / "savo_head" / "core" / "apriltag_action_contract.hpp",
        head_contract,
    )

    head_cmake_path = head / "CMakeLists.txt"
    head_cmake = head_cmake_path.read_text(encoding="utf-8")
    head_cmake = replace_once(
        head_cmake,
        "find_package(rclcpp REQUIRED)\n",
        "find_package(rclcpp REQUIRED)\nfind_package(rclcpp_action REQUIRED)\nfind_package(savo_msgs REQUIRED)\n",
        "savo_head typed dependencies",
    )
    head_cmake = replace_once(
        head_cmake,
        "  ament_add_pytest_test(\n    test_controller_diagnostic_ownership\n    test/test_controller_diagnostic_ownership.py\n  )\n",
        "  ament_add_pytest_test(\n"
        "    test_controller_diagnostic_ownership\n"
        "    test/test_controller_diagnostic_ownership.py\n"
        "  )\n\n"
        "  ament_add_pytest_test(\n"
        "    test_apriltag_typed_contract\n"
        "    test/test_apriltag_typed_contract.py\n"
        "  )\n",
        "savo_head typed contract test",
    )
    write(head_cmake_path, head_cmake)

    head_test = textwrap.dedent(
        """\
        from pathlib import Path
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[1]


        def test_typed_apriltag_contract_names_and_duties() -> None:
            text = (
                ROOT
                / "include/savo_head/core/apriltag_action_contract.hpp"
            ).read_text(encoding="utf-8")

            for token in (
                "/savo_head/apriltag/observations",
                "/savo_head/apriltag/confirm",
                "kRegisterLocation = 1U",
                "kConfirmArrival = 2U",
                "IsValidDuty",
            ):
                assert token in text


        def test_typed_dependencies_are_declared_without_removing_legacy_node() -> None:
            root = ET.parse(ROOT / "package.xml").getroot()
            dependencies = {
                (node.text or "").strip()
                for node in root.findall("depend")
            }

            assert {"savo_msgs", "rclcpp_action"} <= dependencies

            cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
            assert "find_package(savo_msgs REQUIRED)" in cmake
            assert "find_package(rclcpp_action REQUIRED)" in cmake
            assert "apriltag_confirm_node src/nodes/apriltag_confirm_node.cpp" in cmake
        """
    )
    write(head / "test" / "test_apriltag_typed_contract.py", head_test)

    # ------------------------------------------------------------------
    # savo_mapping: deterministic candidate-draft validation and ownership.
    # ------------------------------------------------------------------
    semantic_landmark_hpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #pragma once

        #include <cstdint>
        #include <string>
        #include <vector>

        namespace savo_mapping
        {

        struct SemanticPlanarPose
        {
          std::string frame_id{"map"};
          double x{0.0};
          double y{0.0};
          double yaw{0.0};
        };

        struct SemanticLandmarkDraft
        {
          std::string candidate_id{};
          std::string map_id{};
          std::uint32_t map_revision{0U};
          std::string map_release_id{};

          std::string tag_family{"tag36h11"};
          std::int32_t tag_id{-1};
          SemanticPlanarPose tag_pose_map{};

          double detection_quality{0.0};
          std::uint32_t accepted_observations{0U};
          double position_stddev_m{0.0};
          double yaw_stddev_rad{0.0};

          bool approach_pose_valid{false};
          SemanticPlanarPose approach_pose{};

          bool confirmation_pose_valid{false};
          SemanticPlanarPose confirmation_pose{};

          std::string suggested_location_id{};
          std::string suggested_display_name{};
          std::vector<std::string> suggested_aliases{};
          std::string suggested_semantic_type{};

          std::string building{};
          std::string floor{};
          std::string area{};
          std::string notes{};
          std::string source_session_id{};
        };

        }  // namespace savo_mapping
        """
    )
    write(mapping / "include" / "savo_mapping" / "semantic_landmark.hpp", semantic_landmark_hpp)

    recorder_hpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #pragma once

        #include <cstdint>
        #include <string>

        #include "savo_mapping/semantic_landmark.hpp"

        namespace savo_mapping
        {

        enum class SemanticLandmarkValidationCode : std::uint8_t
        {
          kValid = 0U,
          kInvalidIdentity,
          kInvalidMapContext,
          kInvalidTag,
          kInvalidEvidence,
          kInvalidTagPose,
          kInvalidApproachPose,
          kDirectTagPoseTarget,
          kInvalidConfirmationPose,
        };

        struct SemanticLandmarkValidationResult
        {
          bool valid{false};
          SemanticLandmarkValidationCode code{
            SemanticLandmarkValidationCode::kInvalidIdentity};
          std::string reason{"not_evaluated"};
        };

        class SemanticLandmarkRecorder
        {
        public:
          explicit SemanticLandmarkRecorder(
            double minimum_tag_to_approach_distance_m = 0.20);

          [[nodiscard]] SemanticLandmarkValidationResult Validate(
            const SemanticLandmarkDraft & draft) const;

          [[nodiscard]] double minimum_tag_to_approach_distance_m() const;

        private:
          double minimum_tag_to_approach_distance_m_{0.20};
        };

        }  // namespace savo_mapping
        """
    )
    write(
        mapping / "include" / "savo_mapping" / "semantic_landmark_recorder.hpp",
        recorder_hpp,
    )

    semantic_cpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_mapping/semantic_landmark.hpp"
        """
    )
    write(mapping / "src" / "semantic" / "semantic_landmark.cpp", semantic_cpp)

    recorder_cpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_mapping/semantic_landmark_recorder.hpp"

        #include <cmath>
        #include <utility>

        namespace savo_mapping
        {
        namespace
        {

        bool finite_pose(const SemanticPlanarPose & pose)
        {
          return !pose.frame_id.empty() &&
                 std::isfinite(pose.x) &&
                 std::isfinite(pose.y) &&
                 std::isfinite(pose.yaw);
        }

        SemanticLandmarkValidationResult reject(
          const SemanticLandmarkValidationCode code,
          std::string reason)
        {
          return {false, code, std::move(reason)};
        }

        }  // namespace

        SemanticLandmarkRecorder::SemanticLandmarkRecorder(
          const double minimum_tag_to_approach_distance_m)
        : minimum_tag_to_approach_distance_m_(minimum_tag_to_approach_distance_m)
        {
          if (!std::isfinite(minimum_tag_to_approach_distance_m_) ||
            minimum_tag_to_approach_distance_m_ < 0.0)
          {
            minimum_tag_to_approach_distance_m_ = 0.20;
          }
        }

        SemanticLandmarkValidationResult SemanticLandmarkRecorder::Validate(
          const SemanticLandmarkDraft & draft) const
        {
          if (draft.candidate_id.empty()) {
            return reject(
              SemanticLandmarkValidationCode::kInvalidIdentity,
              "candidate_id_required");
          }

          if (draft.map_id.empty() || draft.map_revision == 0U) {
            return reject(
              SemanticLandmarkValidationCode::kInvalidMapContext,
              "map_id_and_revision_required");
          }

          if (draft.tag_family.empty() || draft.tag_id < 0) {
            return reject(
              SemanticLandmarkValidationCode::kInvalidTag,
              "valid_tag_identity_required");
          }

          if (!std::isfinite(draft.detection_quality) ||
            draft.detection_quality < 0.0 ||
            draft.detection_quality > 1.0 ||
            draft.accepted_observations == 0U ||
            !std::isfinite(draft.position_stddev_m) ||
            draft.position_stddev_m < 0.0 ||
            !std::isfinite(draft.yaw_stddev_rad) ||
            draft.yaw_stddev_rad < 0.0)
          {
            return reject(
              SemanticLandmarkValidationCode::kInvalidEvidence,
              "stable_detection_evidence_required");
          }

          if (!finite_pose(draft.tag_pose_map) ||
            draft.tag_pose_map.frame_id != "map")
          {
            return reject(
              SemanticLandmarkValidationCode::kInvalidTagPose,
              "tag_pose_map_must_be_finite_in_map_frame");
          }

          if (draft.approach_pose_valid) {
            if (!finite_pose(draft.approach_pose) ||
              draft.approach_pose.frame_id != draft.tag_pose_map.frame_id)
            {
              return reject(
                SemanticLandmarkValidationCode::kInvalidApproachPose,
                "approach_pose_must_be_finite_in_map_frame");
            }

            const double distance = std::hypot(
              draft.approach_pose.x - draft.tag_pose_map.x,
              draft.approach_pose.y - draft.tag_pose_map.y);

            if (distance < minimum_tag_to_approach_distance_m_) {
              return reject(
                SemanticLandmarkValidationCode::kDirectTagPoseTarget,
                "approach_pose_must_not_be_tag_pose_map");
            }
          }

          if (draft.confirmation_pose_valid &&
            (!finite_pose(draft.confirmation_pose) ||
            draft.confirmation_pose.frame_id != draft.tag_pose_map.frame_id))
          {
            return reject(
              SemanticLandmarkValidationCode::kInvalidConfirmationPose,
              "confirmation_pose_must_be_finite_in_map_frame");
          }

          return {
            true,
            SemanticLandmarkValidationCode::kValid,
            "semantic_landmark_draft_valid"};
        }

        double SemanticLandmarkRecorder::minimum_tag_to_approach_distance_m() const
        {
          return minimum_tag_to_approach_distance_m_;
        }

        }  // namespace savo_mapping
        """
    )
    write(
        mapping / "src" / "semantic" / "semantic_landmark_recorder.cpp",
        recorder_cpp,
    )

    mapping_test = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include <gtest/gtest.h>

        #include "savo_mapping/semantic_landmark_recorder.hpp"

        namespace
        {

        savo_mapping::SemanticLandmarkDraft valid_draft()
        {
          savo_mapping::SemanticLandmarkDraft draft;
          draft.candidate_id = "candidate-a201-tag27";
          draft.map_id = "campus_main";
          draft.map_revision = 7U;
          draft.tag_family = "tag36h11";
          draft.tag_id = 27;
          draft.tag_pose_map = {"map", 4.0, 2.0, 0.0};
          draft.detection_quality = 0.95;
          draft.accepted_observations = 8U;
          draft.position_stddev_m = 0.01;
          draft.yaw_stddev_rad = 0.02;
          draft.approach_pose_valid = true;
          draft.approach_pose = {"map", 3.2, 2.0, 0.0};
          draft.confirmation_pose_valid = true;
          draft.confirmation_pose = {"map", 3.5, 2.0, 0.0};
          return draft;
        }

        TEST(SemanticLandmarkRecorder, AcceptsSafeMappedCandidateDraft)
        {
          savo_mapping::SemanticLandmarkRecorder recorder;
          const auto result = recorder.Validate(valid_draft());
          EXPECT_TRUE(result.valid);
          EXPECT_EQ(
            result.code,
            savo_mapping::SemanticLandmarkValidationCode::kValid);
        }

        TEST(SemanticLandmarkRecorder, RejectsTagPoseAsApproachPose)
        {
          savo_mapping::SemanticLandmarkRecorder recorder;
          auto draft = valid_draft();
          draft.approach_pose = draft.tag_pose_map;

          const auto result = recorder.Validate(draft);
          EXPECT_FALSE(result.valid);
          EXPECT_EQ(
            result.code,
            savo_mapping::SemanticLandmarkValidationCode::kDirectTagPoseTarget);
        }

        TEST(SemanticLandmarkRecorder, AllowsPendingCandidateWithoutApproachPose)
        {
          savo_mapping::SemanticLandmarkRecorder recorder;
          auto draft = valid_draft();
          draft.approach_pose_valid = false;

          const auto result = recorder.Validate(draft);
          EXPECT_TRUE(result.valid);
        }

        }  // namespace
        """
    )
    write(mapping / "test" / "test_semantic_landmark.cpp", mapping_test)

    mapping_cmake_path = mapping / "CMakeLists.txt"
    mapping_cmake = mapping_cmake_path.read_text(encoding="utf-8")
    mapping_cmake = replace_once(
        mapping_cmake,
        "  src/scan360/scan360_quality.cpp\n",
        "  src/scan360/scan360_quality.cpp\n"
        "  src/semantic/semantic_landmark.cpp\n"
        "  src/semantic/semantic_landmark_recorder.cpp\n",
        "savo_mapping semantic sources",
    )
    mapping_cmake = replace_once(
        mapping_cmake,
        "  include/savo_mapping/map_quality_placeholder.hpp\n",
        "  include/savo_mapping/map_quality_placeholder.hpp\n"
        "  include/savo_mapping/semantic_landmark.hpp\n"
        "  include/savo_mapping/semantic_landmark_recorder.hpp\n",
        "savo_mapping semantic headers",
    )
    mapping_cmake = replace_once(
        mapping_cmake,
        "  ament_add_gtest(test_exploration_goal_handoff\n"
        "    test/test_exploration_goal_handoff.cpp\n"
        "  )\n\n"
        "  target_link_libraries(\n"
        "    test_exploration_goal_handoff\n"
        "    ${PROJECT_NAME}_core\n"
        "  )\n\n"
        "endif()\n",
        "  ament_add_gtest(test_exploration_goal_handoff\n"
        "    test/test_exploration_goal_handoff.cpp\n"
        "  )\n\n"
        "  target_link_libraries(\n"
        "    test_exploration_goal_handoff\n"
        "    ${PROJECT_NAME}_core\n"
        "  )\n\n"
        "  ament_add_gtest(test_semantic_landmark\n"
        "    test/test_semantic_landmark.cpp\n"
        "  )\n\n"
        "  target_link_libraries(\n"
        "    test_semantic_landmark\n"
        "    ${PROJECT_NAME}_core\n"
        "  )\n\n"
        "endif()\n",
        "savo_mapping semantic test",
    )
    write(mapping_cmake_path, mapping_cmake)

    # ------------------------------------------------------------------
    # savo_nav: deterministic semantic-target selection. Never tag pose.
    # ------------------------------------------------------------------
    nav_hpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #pragma once

        #include <cstdint>
        #include <string>

        namespace savo_nav
        {

        struct LocationPlanarPose
        {
          std::string frame_id{"map"};
          double x{0.0};
          double y{0.0};
          double yaw{0.0};
          bool valid{false};
        };

        struct LocationRecordView
        {
          bool approved{false};
          bool enabled{false};
          bool retired{false};
          std::string location_id{};
          std::string map_id{};
          std::uint32_t map_revision{0U};
          LocationPlanarPose approach_pose{};
          LocationPlanarPose tag_pose_map{};
          bool tag_pose_map_valid{false};
          bool arrival_confirmation_required{false};
        };

        struct LocationNavigationRequestView
        {
          std::string query{};
          bool enforce_map_context{false};
          std::string map_id{};
          std::uint32_t map_revision{0U};
          bool require_arrival_confirmation{false};
        };

        enum class LocationTargetCode : std::uint8_t
        {
          kAccepted = 0U,
          kInvalidRequest,
          kNotApproved,
          kDisabled,
          kRetired,
          kMapMismatch,
          kInvalidApproachPose,
          kDirectTagPoseTarget,
        };

        enum class LocationTargetSource : std::uint8_t
        {
          kNone = 0U,
          kApproachPose = 1U,
        };

        struct LocationTargetDecision
        {
          bool accepted{false};
          LocationTargetCode code{LocationTargetCode::kInvalidRequest};
          std::string reason{"not_evaluated"};
          LocationTargetSource target_source{LocationTargetSource::kNone};
          LocationPlanarPose target{};
          bool arrival_confirmation_required{false};
        };

        class LocationNavigationContract
        {
        public:
          explicit LocationNavigationContract(
            double direct_tag_pose_epsilon_m = 0.05);

          [[nodiscard]] LocationTargetDecision SelectTarget(
            const LocationNavigationRequestView & request,
            const LocationRecordView & record) const;

        private:
          double direct_tag_pose_epsilon_m_{0.05};
        };

        }  // namespace savo_nav
        """
    )
    write(nav / "include" / "savo_nav" / "location_navigation_contract.hpp", nav_hpp)

    nav_cpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_nav/location_navigation_contract.hpp"

        #include <cmath>
        #include <utility>

        namespace savo_nav
        {
        namespace
        {

        bool finite_pose(const LocationPlanarPose & pose)
        {
          return pose.valid &&
                 pose.frame_id == "map" &&
                 std::isfinite(pose.x) &&
                 std::isfinite(pose.y) &&
                 std::isfinite(pose.yaw);
        }

        LocationTargetDecision reject(
          const LocationTargetCode code,
          std::string reason)
        {
          LocationTargetDecision decision;
          decision.code = code;
          decision.reason = std::move(reason);
          return decision;
        }

        }  // namespace

        LocationNavigationContract::LocationNavigationContract(
          const double direct_tag_pose_epsilon_m)
        : direct_tag_pose_epsilon_m_(direct_tag_pose_epsilon_m)
        {
          if (!std::isfinite(direct_tag_pose_epsilon_m_) ||
            direct_tag_pose_epsilon_m_ < 0.0)
          {
            direct_tag_pose_epsilon_m_ = 0.05;
          }
        }

        LocationTargetDecision LocationNavigationContract::SelectTarget(
          const LocationNavigationRequestView & request,
          const LocationRecordView & record) const
        {
          if (request.query.empty()) {
            return reject(
              LocationTargetCode::kInvalidRequest,
              "location_query_required");
          }

          if (!record.approved) {
            return reject(
              LocationTargetCode::kNotApproved,
              "location_not_approved");
          }

          if (record.retired) {
            return reject(
              LocationTargetCode::kRetired,
              "location_retired");
          }

          if (!record.enabled) {
            return reject(
              LocationTargetCode::kDisabled,
              "location_disabled");
          }

          if (request.enforce_map_context &&
            (request.map_id.empty() || request.map_revision == 0U ||
            record.map_id != request.map_id ||
            record.map_revision != request.map_revision))
          {
            return reject(
              LocationTargetCode::kMapMismatch,
              "location_map_context_mismatch");
          }

          if (!finite_pose(record.approach_pose)) {
            return reject(
              LocationTargetCode::kInvalidApproachPose,
              "approved_approach_pose_required");
          }

          if (record.tag_pose_map_valid && finite_pose(record.tag_pose_map)) {
            const double distance = std::hypot(
              record.approach_pose.x - record.tag_pose_map.x,
              record.approach_pose.y - record.tag_pose_map.y);

            if (distance <= direct_tag_pose_epsilon_m_) {
              return reject(
                LocationTargetCode::kDirectTagPoseTarget,
                "tag_pose_map_must_never_be_navigation_target");
            }
          }

          LocationTargetDecision decision;
          decision.accepted = true;
          decision.code = LocationTargetCode::kAccepted;
          decision.reason = "approved_approach_pose_selected";
          decision.target_source = LocationTargetSource::kApproachPose;
          decision.target = record.approach_pose;
          decision.arrival_confirmation_required =
            record.arrival_confirmation_required ||
            request.require_arrival_confirmation;
          return decision;
        }

        }  // namespace savo_nav
        """
    )
    write(nav / "src" / "core" / "location_navigation_contract.cpp", nav_cpp)

    nav_test = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include <gtest/gtest.h>

        #include "savo_nav/location_navigation_contract.hpp"

        namespace
        {

        savo_nav::LocationRecordView approved_record()
        {
          savo_nav::LocationRecordView record;
          record.approved = true;
          record.enabled = true;
          record.location_id = "A201";
          record.map_id = "campus_main";
          record.map_revision = 7U;
          record.approach_pose = {"map", 3.2, 2.0, 0.0, true};
          record.tag_pose_map = {"map", 4.0, 2.0, 0.0, true};
          record.tag_pose_map_valid = true;
          record.arrival_confirmation_required = true;
          return record;
        }

        savo_nav::LocationNavigationRequestView request()
        {
          savo_nav::LocationNavigationRequestView value;
          value.query = "room a201";
          value.enforce_map_context = true;
          value.map_id = "campus_main";
          value.map_revision = 7U;
          return value;
        }

        TEST(LocationNavigationContract, SelectsOnlyApprovedApproachPose)
        {
          savo_nav::LocationNavigationContract contract;
          const auto decision = contract.SelectTarget(request(), approved_record());

          ASSERT_TRUE(decision.accepted);
          EXPECT_EQ(
            decision.target_source,
            savo_nav::LocationTargetSource::kApproachPose);
          EXPECT_DOUBLE_EQ(decision.target.x, 3.2);
          EXPECT_TRUE(decision.arrival_confirmation_required);
        }

        TEST(LocationNavigationContract, RejectsDirectTagPoseTarget)
        {
          savo_nav::LocationNavigationContract contract;
          auto record = approved_record();
          record.approach_pose = record.tag_pose_map;

          const auto decision = contract.SelectTarget(request(), record);
          EXPECT_FALSE(decision.accepted);
          EXPECT_EQ(
            decision.code,
            savo_nav::LocationTargetCode::kDirectTagPoseTarget);
        }

        TEST(LocationNavigationContract, RejectsDisabledAndWrongMap)
        {
          savo_nav::LocationNavigationContract contract;
          auto record = approved_record();
          record.enabled = false;
          EXPECT_EQ(
            contract.SelectTarget(request(), record).code,
            savo_nav::LocationTargetCode::kDisabled);

          record = approved_record();
          auto wrong_map = request();
          wrong_map.map_revision = 8U;
          EXPECT_EQ(
            contract.SelectTarget(wrong_map, record).code,
            savo_nav::LocationTargetCode::kMapMismatch);
        }

        }  // namespace
        """
    )
    write(nav / "test" / "unit" / "test_location_navigation_contract.cpp", nav_test)

    nav_cmake_path = nav / "CMakeLists.txt"
    nav_cmake = nav_cmake_path.read_text(encoding="utf-8")
    nav_cmake = replace_once(
        nav_cmake,
        "  src/core/localization_monitor.cpp\n",
        "  src/core/localization_monitor.cpp\n"
        "  src/core/location_navigation_contract.cpp\n",
        "savo_nav location navigation source",
    )
    nav_cmake = replace_once(
        nav_cmake,
        "  ament_lint_auto_find_test_dependencies()\nendif()\n",
        "  ament_add_gtest(\n"
        "    test_location_navigation_contract\n"
        "    test/unit/test_location_navigation_contract.cpp\n"
        "  )\n\n"
        "  if(TARGET test_location_navigation_contract)\n"
        "    target_link_libraries(\n"
        "      test_location_navigation_contract\n"
        "      savo_nav_core\n"
        "    )\n"
        "  endif()\n\n"
        "  ament_lint_auto_find_test_dependencies()\n"
        "endif()\n",
        "savo_nav location navigation test",
    )
    write(nav_cmake_path, nav_cmake)

    # ------------------------------------------------------------------
    # savo_supervisor: deterministic permission policy foundation.
    # ------------------------------------------------------------------
    ensure_xml_depend(supervisor / "package.xml", "savo_msgs")

    auth_hpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #pragma once

        #include <cstdint>
        #include <string>

        #include "savo_supervisor/supervisor_state.hpp"

        namespace savo_supervisor
        {

        enum class LocationOperation : std::uint8_t
        {
          kRegisterCandidate = 1U,
          kApproveLocation = 2U,
          kNavigateToLocation = 3U,
          kConfirmArrival = 4U,
        };

        enum class LocationAuthorizationCode : std::uint8_t
        {
          kAuthorized = 0U,
          kInvalidRequest,
          kSupervisorNotReady,
          kHealthBlocked,
          kSafetyBlocked,
          kMapContextBlocked,
          kOperationDisabled,
        };

        struct LocationAuthorizationRequest
        {
          LocationOperation operation{LocationOperation::kRegisterCandidate};
          std::string request_id{};
          std::string actor_id{};
          std::string candidate_id{};
          std::string location_id{};
          std::string map_id{};
          std::uint32_t map_revision{0U};
          bool motion_required{false};
        };

        struct LocationAuthorizationPolicy
        {
          bool allow_registration{true};
          bool allow_approval{true};
          bool allow_navigation{true};
          bool allow_arrival_confirmation{true};
          bool allow_degraded_non_motion{true};
          bool allow_degraded_motion{false};
          bool require_known_safety_for_motion{false};
        };

        struct LocationAuthorizationDecision
        {
          bool authorized{false};
          LocationAuthorizationCode code{
            LocationAuthorizationCode::kInvalidRequest};
          std::string reason{"not_evaluated"};
        };

        class LocationAuthorizationEvaluator
        {
        public:
          explicit LocationAuthorizationEvaluator(
            LocationAuthorizationPolicy policy = {});

          [[nodiscard]] LocationAuthorizationDecision Evaluate(
            const LocationAuthorizationRequest & request,
            const SupervisorState & state) const;

        private:
          LocationAuthorizationPolicy policy_{};
        };

        }  // namespace savo_supervisor
        """
    )
    write(
        supervisor / "include" / "savo_supervisor" / "location_authorization_policy.hpp",
        auth_hpp,
    )

    auth_cpp = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include "savo_supervisor/location_authorization_policy.hpp"

        #include <utility>

        namespace savo_supervisor
        {
        namespace
        {

        LocationAuthorizationDecision reject(
          const LocationAuthorizationCode code,
          std::string reason)
        {
          return {false, code, std::move(reason)};
        }

        bool operation_enabled(
          const LocationAuthorizationPolicy & policy,
          const LocationOperation operation)
        {
          switch (operation) {
            case LocationOperation::kRegisterCandidate:
              return policy.allow_registration;
            case LocationOperation::kApproveLocation:
              return policy.allow_approval;
            case LocationOperation::kNavigateToLocation:
              return policy.allow_navigation;
            case LocationOperation::kConfirmArrival:
              return policy.allow_arrival_confirmation;
          }
          return false;
        }

        }  // namespace

        LocationAuthorizationEvaluator::LocationAuthorizationEvaluator(
          LocationAuthorizationPolicy policy)
        : policy_(std::move(policy))
        {
        }

        LocationAuthorizationDecision LocationAuthorizationEvaluator::Evaluate(
          const LocationAuthorizationRequest & request,
          const SupervisorState & state) const
        {
          if (request.request_id.empty() || request.actor_id.empty() ||
            request.map_id.empty() || request.map_revision == 0U)
          {
            return reject(
              LocationAuthorizationCode::kInvalidRequest,
              "request_actor_and_map_context_required");
          }

          if (!operation_enabled(policy_, request.operation)) {
            return reject(
              LocationAuthorizationCode::kOperationDisabled,
              "location_operation_disabled_by_policy");
          }

          if (state.lifecycle != Lifecycle::RUNNING || !state.ready) {
            return reject(
              LocationAuthorizationCode::kSupervisorNotReady,
              "supervisor_not_ready");
          }

          if (state.health == AggregateHealth::ERROR ||
            state.health == AggregateHealth::UNKNOWN)
          {
            return reject(
              LocationAuthorizationCode::kHealthBlocked,
              "aggregate_health_blocks_operation");
          }

          if (state.health == AggregateHealth::DEGRADED) {
            const bool allowed = request.motion_required ?
              policy_.allow_degraded_motion :
              policy_.allow_degraded_non_motion;

            if (!allowed) {
              return reject(
                LocationAuthorizationCode::kHealthBlocked,
                "degraded_health_not_authorized");
            }
          }

          if (request.motion_required) {
            if (state.safety == SafetyObservation::STOPPED) {
              return reject(
                LocationAuthorizationCode::kSafetyBlocked,
                "safety_stop_active");
            }

            if (policy_.require_known_safety_for_motion &&
              state.safety == SafetyObservation::UNKNOWN)
            {
              return reject(
                LocationAuthorizationCode::kSafetyBlocked,
                "safety_state_unknown");
            }
          }

          return {
            true,
            LocationAuthorizationCode::kAuthorized,
            "location_operation_authorized"};
        }

        }  // namespace savo_supervisor
        """
    )
    write(supervisor / "src" / "location_authorization_policy.cpp", auth_cpp)

    auth_test = textwrap.dedent(
        """\
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include <gtest/gtest.h>

        #include "savo_supervisor/location_authorization_policy.hpp"

        namespace
        {

        savo_supervisor::SupervisorState ready_state()
        {
          savo_supervisor::SupervisorState state;
          state.lifecycle = savo_supervisor::Lifecycle::RUNNING;
          state.health = savo_supervisor::AggregateHealth::OK;
          state.safety = savo_supervisor::SafetyObservation::CLEAR;
          state.ready = true;
          return state;
        }

        savo_supervisor::LocationAuthorizationRequest navigation_request()
        {
          savo_supervisor::LocationAuthorizationRequest request;
          request.operation =
            savo_supervisor::LocationOperation::kNavigateToLocation;
          request.request_id = "nav-a201-1";
          request.actor_id = "operator";
          request.location_id = "A201";
          request.map_id = "campus_main";
          request.map_revision = 7U;
          request.motion_required = true;
          return request;
        }

        TEST(LocationAuthorizationPolicy, AuthorizesReadyHealthyNavigation)
        {
          savo_supervisor::LocationAuthorizationEvaluator evaluator;
          const auto decision = evaluator.Evaluate(
            navigation_request(), ready_state());
          EXPECT_TRUE(decision.authorized);
        }

        TEST(LocationAuthorizationPolicy, RejectsSafetyStop)
        {
          savo_supervisor::LocationAuthorizationEvaluator evaluator;
          auto state = ready_state();
          state.safety = savo_supervisor::SafetyObservation::STOPPED;
          const auto decision = evaluator.Evaluate(
            navigation_request(), state);
          EXPECT_FALSE(decision.authorized);
          EXPECT_EQ(
            decision.code,
            savo_supervisor::LocationAuthorizationCode::kSafetyBlocked);
        }

        TEST(LocationAuthorizationPolicy, RejectsDegradedMotionByDefault)
        {
          savo_supervisor::LocationAuthorizationEvaluator evaluator;
          auto state = ready_state();
          state.health = savo_supervisor::AggregateHealth::DEGRADED;
          const auto decision = evaluator.Evaluate(
            navigation_request(), state);
          EXPECT_FALSE(decision.authorized);
          EXPECT_EQ(
            decision.code,
            savo_supervisor::LocationAuthorizationCode::kHealthBlocked);
        }

        }  // namespace
        """
    )
    write(supervisor / "test" / "test_location_authorization_policy.cpp", auth_test)

    supervisor_cmake_path = supervisor / "CMakeLists.txt"
    supervisor_cmake = supervisor_cmake_path.read_text(encoding="utf-8")
    supervisor_cmake = replace_once(
        supervisor_cmake,
        "find_package(rclcpp REQUIRED)\n",
        "find_package(rclcpp REQUIRED)\nfind_package(savo_msgs REQUIRED)\n",
        "savo_supervisor savo_msgs dependency",
    )
    supervisor_cmake = replace_once(
        supervisor_cmake,
        "  src/localization_payload_parser.cpp\n",
        "  src/localization_payload_parser.cpp\n"
        "  src/location_authorization_policy.cpp\n",
        "savo_supervisor authorization source",
    )
    supervisor_cmake = replace_once(
        supervisor_cmake,
        "  ament_add_gtest(test_supervisor_contract\n"
        "    test/test_supervisor_contract.cpp\n"
        "  )\n"
        "  target_link_libraries(test_supervisor_contract\n"
        "    savo_supervisor_core\n"
        "  )\n"
        "  ament_target_dependencies(test_supervisor_contract\n"
        "    rclcpp\n"
        "    builtin_interfaces\n"
        "  )\n"
        "endif()\n",
        "  ament_add_gtest(test_supervisor_contract\n"
        "    test/test_supervisor_contract.cpp\n"
        "  )\n"
        "  target_link_libraries(test_supervisor_contract\n"
        "    savo_supervisor_core\n"
        "  )\n"
        "  ament_target_dependencies(test_supervisor_contract\n"
        "    rclcpp\n"
        "    builtin_interfaces\n"
        "  )\n\n"
        "  ament_add_gtest(test_location_authorization_policy\n"
        "    test/test_location_authorization_policy.cpp\n"
        "  )\n"
        "  target_link_libraries(test_location_authorization_policy\n"
        "    savo_supervisor_core\n"
        "  )\n"
        "endif()\n",
        "savo_supervisor authorization test",
    )
    supervisor_cmake = replace_once(
        supervisor_cmake,
        "  nlohmann_json\n)\n\nament_package()",
        "  nlohmann_json\n  savo_msgs\n)\n\nament_package()",
        "savo_supervisor export dependency",
    )
    write(supervisor_cmake_path, supervisor_cmake)

    # ------------------------------------------------------------------
    # Static verification.
    # ------------------------------------------------------------------
    for path in (
        msgs / "test" / "test_apriltag_interfaces.py",
        msgs / "test" / "test_location_integration_interfaces.py",
        head / "test" / "test_apriltag_typed_contract.py",
    ):
        py_compile.compile(str(path), doraise=True)

    required_tokens = {
        msgs / "CMakeLists.txt": (
            "action/RegisterMappedLocation.action",
            "action/NavigateToLocation.action",
            "srv/AuthorizeLocationOperation.srv",
        ),
        mapping / "CMakeLists.txt": (
            "src/semantic/semantic_landmark_recorder.cpp",
            "test_semantic_landmark",
        ),
        nav / "CMakeLists.txt": (
            "src/core/location_navigation_contract.cpp",
            "test_location_navigation_contract",
        ),
        supervisor / "CMakeLists.txt": (
            "src/location_authorization_policy.cpp",
            "test_location_authorization_policy",
        ),
    }

    for path, tokens in required_tokens.items():
        text = path.read_text(encoding="utf-8")
        for token in tokens:
            if token not in text:
                raise RuntimeError(f"verification failed: {token} missing from {path}")

    with manifest.open("w", encoding="utf-8") as stream:
        for package in (msgs, head, mapping, nav, supervisor):
            for path in sorted(p for p in package.rglob("*") if p.is_file()):
                stream.write(f"{sha256(path)}  {path}\n")

    print("LOC-3P-C1 typed integration contracts installed and verified.")
    print(f"Backup : {backup}")
    print(f"Manifest: {manifest}")
    print("savo_msgs version: 0.5.0")
    print("Shared contracts: RegisterMappedLocation, NavigateToLocation, AuthorizeLocationOperation")
    print("Head typed action/topic ownership contract: added")
    print("Mapping semantic candidate guard: added")
    print("Navigation approach-pose-only guard: added")
    print("Supervisor authorization policy foundation: added")
    print("Legacy runtime nodes remain available; live typed adapters are LOC-3P-C2.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
