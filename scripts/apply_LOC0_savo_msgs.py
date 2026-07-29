#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import tarfile
import textwrap

ROOT = Path.home() / "Savo_Pi" / "savo_ws" / "src" / "shared" / "savo_msgs"
BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


def clean(text: str) -> str:
    return textwrap.dedent(text).lstrip()


def write(relative: str, content: str) -> None:
    path = ROOT / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(clean(content), encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if not (ROOT / "package.xml").is_file():
        raise SystemExit(f"savo_msgs not found: {ROOT}")

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = BACKUPS / f"pre_LOC0_savo_msgs_{stamp}.tar.gz"

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(ROOT, arcname="shared/savo_msgs")

    write(
        "msg/LocationRecord.msg",
        """
        uint8 STATE_UNKNOWN=0
        uint8 STATE_APPROVED=1
        uint8 STATE_RETIRED=2

        uint8 state
        bool enabled
        uint64 record_revision

        string location_id
        string display_name
        string[] aliases
        string semantic_type

        string map_id
        uint32 map_revision
        string map_release_id

        geometry_msgs/PoseStamped approach_pose

        bool confirmation_pose_valid
        geometry_msgs/PoseStamped confirmation_pose

        string tag_family
        int32 tag_id

        bool tag_pose_map_valid
        geometry_msgs/PoseStamped tag_pose_map

        bool arrival_confirmation_required

        string building
        string floor
        string area
        string notes

        string source_candidate_id

        builtin_interfaces/Time created_at
        builtin_interfaces/Time updated_at
        """,
    )

    write(
        "msg/LocationCandidate.msg",
        """
        uint8 STATE_UNKNOWN=0
        uint8 STATE_PENDING_REVIEW=1
        uint8 STATE_APPROVED=2
        uint8 STATE_REJECTED=3

        uint8 state
        uint64 candidate_revision
        string candidate_id

        string map_id
        uint32 map_revision
        string map_release_id

        string tag_family
        int32 tag_id
        geometry_msgs/PoseStamped tag_pose_map

        float32 detection_quality
        uint32 accepted_observations
        float32 position_stddev_m
        float32 yaw_stddev_rad

        bool approach_pose_valid
        geometry_msgs/PoseStamped approach_pose

        bool confirmation_pose_valid
        geometry_msgs/PoseStamped confirmation_pose

        string suggested_location_id
        string suggested_display_name
        string[] suggested_aliases
        string suggested_semantic_type

        string building
        string floor
        string area
        string notes

        string source_session_id
        string source_component
        string review_reason

        builtin_interfaces/Time created_at
        builtin_interfaces/Time updated_at
        """,
    )

    write(
        "msg/LocationEvent.msg",
        """
        uint8 EVENT_UNKNOWN=0
        uint8 EVENT_CANDIDATE_REGISTERED=1
        uint8 EVENT_LOCATION_APPROVED=2
        uint8 EVENT_CANDIDATE_REJECTED=3
        uint8 EVENT_LOCATION_UPDATED=4
        uint8 EVENT_LOCATION_ENABLED=5
        uint8 EVENT_LOCATION_DISABLED=6
        uint8 EVENT_LOCATION_RETIRED=7
        uint8 EVENT_IMPORT_COMPLETED=8
        uint8 EVENT_STORAGE_DEGRADED=9

        uint64 event_sequence
        builtin_interfaces/Time stamp
        uint8 event_type

        string candidate_id
        string location_id
        uint64 entity_revision

        string actor_id
        string reason
        """,
    )

    write(
        "srv/ResolveLocation.srv",
        """
        string query
        bool enforce_map_context
        string map_id
        uint32 map_revision
        ---
        uint8 RESULT_RESOLVED=0
        uint8 RESULT_INVALID_QUERY=1
        uint8 RESULT_NOT_FOUND=2
        uint8 RESULT_AMBIGUOUS=3
        uint8 RESULT_DISABLED=4
        uint8 RESULT_RETIRED=5
        uint8 RESULT_MAP_MISMATCH=6
        uint8 RESULT_INTERNAL_ERROR=7

        uint8 MATCH_NONE=0
        uint8 MATCH_LOCATION_ID=1
        uint8 MATCH_DISPLAY_NAME=2
        uint8 MATCH_ALIAS=3

        bool resolved
        uint8 result_code
        string reason
        string normalized_query
        uint8 match_type

        savo_msgs/LocationRecord location
        string[] ambiguous_location_ids
        """,
    )

    write(
        "srv/GetLocation.srv",
        """
        string location_id
        bool include_disabled
        bool include_retired
        ---
        uint8 RESULT_FOUND=0
        uint8 RESULT_INVALID_ID=1
        uint8 RESULT_NOT_FOUND=2
        uint8 RESULT_DISABLED=3
        uint8 RESULT_RETIRED=4
        uint8 RESULT_INTERNAL_ERROR=5

        bool found
        uint8 result_code
        string reason
        savo_msgs/LocationRecord location
        """,
    )

    write(
        "srv/ListLocations.srv",
        """
        uint8 STATE_ANY=0
        uint8 STATE_APPROVED=1
        uint8 STATE_RETIRED=2

        string map_id
        uint32 map_revision
        bool enforce_map_context
        string semantic_type
        uint8 state_filter
        bool enabled_only
        ---
        uint8 RESULT_OK=0
        uint8 RESULT_INVALID_FILTER=1
        uint8 RESULT_INTERNAL_ERROR=2

        bool success
        uint8 result_code
        string reason
        savo_msgs/LocationRecord[] locations
        """,
    )

    write(
        "srv/RegisterLocationCandidate.srv",
        """
        savo_msgs/LocationCandidate candidate
        string actor_id
        ---
        uint8 RESULT_REGISTERED=0
        uint8 RESULT_INVALID_CANDIDATE=1
        uint8 RESULT_DUPLICATE_CANDIDATE_ID=2
        uint8 RESULT_TAG_CONFLICT=3
        uint8 RESULT_MAP_MISMATCH=4
        uint8 RESULT_STORAGE_UNAVAILABLE=5
        uint8 RESULT_INTERNAL_ERROR=6

        bool registered
        uint8 result_code
        string reason
        savo_msgs/LocationCandidate stored_candidate
        """,
    )

    write(
        "srv/ApproveLocation.srv",
        """
        string candidate_id
        uint64 expected_candidate_revision
        string actor_id

        string location_id
        string display_name
        string[] aliases
        string semantic_type

        geometry_msgs/PoseStamped approach_pose

        bool confirmation_pose_valid
        geometry_msgs/PoseStamped confirmation_pose

        bool arrival_confirmation_required

        string building
        string floor
        string area
        string notes
        ---
        uint8 RESULT_APPROVED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_CANDIDATE_NOT_FOUND=2
        uint8 RESULT_CANDIDATE_NOT_PENDING=3
        uint8 RESULT_STALE_REVISION=4
        uint8 RESULT_LOCATION_ID_CONFLICT=5
        uint8 RESULT_ALIAS_CONFLICT=6
        uint8 RESULT_INVALID_APPROACH_POSE=7
        uint8 RESULT_STORAGE_UNAVAILABLE=8
        uint8 RESULT_INTERNAL_ERROR=9

        bool approved
        uint8 result_code
        string reason
        savo_msgs/LocationRecord location
        """,
    )

    write(
        "srv/SetLocationEnabled.srv",
        """
        string location_id
        uint64 expected_record_revision
        bool enabled
        string actor_id
        string reason
        ---
        uint8 RESULT_UPDATED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_NOT_FOUND=2
        uint8 RESULT_RETIRED=3
        uint8 RESULT_STALE_REVISION=4
        uint8 RESULT_STORAGE_UNAVAILABLE=5
        uint8 RESULT_INTERNAL_ERROR=6

        bool updated
        uint8 result_code
        string reason
        savo_msgs/LocationRecord location
        """,
    )

    cmake_path = ROOT / "CMakeLists.txt"
    cmake = cmake_path.read_text(encoding="utf-8")

    if '"msg/LocationRecord.msg"' not in cmake:
        cmake = cmake.replace(
            '  "msg/AprilTagObservation.msg"\n',
            '  "msg/AprilTagObservation.msg"\n'
            '  "msg/LocationRecord.msg"\n'
            '  "msg/LocationCandidate.msg"\n'
            '  "msg/LocationEvent.msg"\n',
        )

    if "set(srv_files" not in cmake:
        action_block = (
            'set(action_files\n'
            '  "action/ConfirmAprilTag.action"\n'
            '  "action/ExecuteCoveragePath.action"\n'
            '  "action/RotateToHeading.action"\n'
            ')\n'
        )

        service_block = action_block + clean(
            """

            set(srv_files
              "srv/ResolveLocation.srv"
              "srv/GetLocation.srv"
              "srv/ListLocations.srv"
              "srv/RegisterLocationCandidate.srv"
              "srv/ApproveLocation.srv"
              "srv/SetLocationEnabled.srv"
            )
            """
        )

        if action_block not in cmake:
            raise RuntimeError(
                "Could not find expected action_files block in CMakeLists.txt"
            )

        cmake = cmake.replace(action_block, service_block)

    if "  ${srv_files}\n" not in cmake:
        cmake = cmake.replace(
            "  ${action_files}\n  DEPENDENCIES",
            "  ${action_files}\n  ${srv_files}\n  DEPENDENCIES",
        )

    if "test_location_interfaces" not in cmake:
        marker = (
            "endif()\n\n"
            "# ---------------------------------------------------------------------------\n"
            "# Export and package"
        )

        replacement = clean(
            """

              ament_add_pytest_test(
                test_location_interfaces
                test/test_location_interfaces.py
                TIMEOUT 60
              )
            endif()

            # ---------------------------------------------------------------------------
            # Export and package
            """
        )

        if marker not in cmake:
            raise RuntimeError(
                "Could not find expected BUILD_TESTING end block in CMakeLists.txt"
            )

        cmake = cmake.replace(marker, replacement)

    cmake_path.write_text(cmake, encoding="utf-8")

    package_path = ROOT / "package.xml"
    package = package_path.read_text(encoding="utf-8")

    package = package.replace(
        "<version>0.2.0</version>",
        "<version>0.3.0</version>",
    )

    package = package.replace(
        "messages plus typed AprilTag observation and confirmation contracts.",
        "messages, typed AprilTag contracts and semantic-location registry "
        "interfaces.",
    )

    package_path.write_text(package, encoding="utf-8")

    action_path = ROOT / "action" / "ConfirmAprilTag.action"
    action = action_path.read_text(encoding="utf-8")
    action = action.replace("map_version", "map_revision")
    action_path.write_text(action, encoding="utf-8")

    apriltag_test = ROOT / "test" / "test_apriltag_interfaces.py"
    test_text = apriltag_test.read_text(encoding="utf-8")
    test_text = test_text.replace("map_version", "map_revision")
    test_text = test_text.replace(
        '== "0.2.0"',
        '== "0.3.0"',
    )
    apriltag_test.write_text(test_text, encoding="utf-8")

    write(
        "test/test_location_interfaces.py",
        r'''
        from pathlib import Path


        ROOT = Path(__file__).resolve().parents[1]


        def read(path: str) -> str:
            return (ROOT / path).read_text(encoding="utf-8")


        def test_location_interfaces_registered() -> None:
            cmake = read("CMakeLists.txt")

            for path in (
                "msg/LocationRecord.msg",
                "msg/LocationCandidate.msg",
                "msg/LocationEvent.msg",
                "srv/ResolveLocation.srv",
                "srv/GetLocation.srv",
                "srv/ListLocations.srv",
                "srv/RegisterLocationCandidate.srv",
                "srv/ApproveLocation.srv",
                "srv/SetLocationEnabled.srv",
            ):
                assert f'"{path}"' in cmake


        def test_map_revision_is_canonical() -> None:
            for path in (
                "msg/LocationRecord.msg",
                "msg/LocationCandidate.msg",
                "srv/ResolveLocation.srv",
                "srv/ListLocations.srv",
                "action/ConfirmAprilTag.action",
            ):
                text = read(path)

                assert "map_revision" in text
                assert "map_version" not in text


        def test_navigation_resolution_guards() -> None:
            text = read("srv/ResolveLocation.srv")

            assert "bool enforce_map_context" in text
            assert "RESULT_AMBIGUOUS=3" in text
            assert "RESULT_MAP_MISMATCH=6" in text
            assert "string[] ambiguous_location_ids" in text


        def test_revision_guards() -> None:
            assert (
                "uint64 expected_candidate_revision"
                in read("srv/ApproveLocation.srv")
            )

            assert (
                "uint64 expected_record_revision"
                in read("srv/SetLocationEnabled.srv")
            )
        ''',
    )

    changed = [
        ROOT / "CMakeLists.txt",
        ROOT / "package.xml",
        ROOT / "action" / "ConfirmAprilTag.action",
        ROOT / "msg" / "LocationRecord.msg",
        ROOT / "msg" / "LocationCandidate.msg",
        ROOT / "msg" / "LocationEvent.msg",
        ROOT / "srv" / "ResolveLocation.srv",
        ROOT / "srv" / "GetLocation.srv",
        ROOT / "srv" / "ListLocations.srv",
        ROOT / "srv" / "RegisterLocationCandidate.srv",
        ROOT / "srv" / "ApproveLocation.srv",
        ROOT / "srv" / "SetLocationEnabled.srv",
        ROOT / "test" / "test_apriltag_interfaces.py",
        ROOT / "test" / "test_location_interfaces.py",
    ]

    manifest = LOGS / f"LOC0_savo_msgs_{stamp}.sha256"

    manifest.write_text(
        "\n".join(
            f"{sha256(path)}  {path.relative_to(ROOT)}"
            for path in changed
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")
    print("LOC-0 savo_msgs contracts applied.")


if __name__ == "__main__":
    main()
