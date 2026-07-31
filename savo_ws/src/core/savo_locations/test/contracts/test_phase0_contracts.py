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
        "/savo_locations/candidates/get",
        "/savo_locations/candidates/list",
        "/savo_locations/list",
        "/savo_locations/candidates/register",
        "/savo_locations/candidates/approve",
        "/savo_locations/candidates/reject",
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
