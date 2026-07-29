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


def test_loc1a_core_remains_dependency_isolated() -> None:
    core_sources = "\n".join(
        read(relative)
        for relative in (
            "include/savo_locations/model.hpp",
            "include/savo_locations/normalization.hpp",
            "include/savo_locations/validation.hpp",
            "src/normalization.cpp",
            "src/validation.cpp",
        )
    )

    # Later phases may add separate storage and ROS
    # targets to this package. The LOC-1A domain
    # implementation itself must remain isolated.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "sqlite3",
        "sqlite_store",
        "sqlite_repository",
    ):
        assert forbidden not in core_sources



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
