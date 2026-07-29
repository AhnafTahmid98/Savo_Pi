#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import tarfile


ROOT = (
    Path.home()
    / "Savo_Pi"
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

CONTRACTS = ROOT / "test" / "contracts"

PHASE1A = CONTRACTS / "test_phase1a_contracts.py"
PHASE1B = CONTRACTS / "test_phase1b_contracts.py"
PHASE1C = CONTRACTS / "test_phase1c_contracts.py"

BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


PHASE1A_FUNCTION = '''def test_loc1a_core_remains_dependency_isolated() -> None:
    cmake = read("CMakeLists.txt")

    core_sources = "\\n".join(
        read(relative)
        for relative in (
            "include/savo_locations/model.hpp",
            "include/savo_locations/normalization.hpp",
            "include/savo_locations/validation.hpp",
            "src/normalization.cpp",
            "src/validation.cpp",
        )
    )

    # LOC-1A remains a ROS-independent deterministic
    # domain layer even when later phases add a separate
    # SQLite storage library to the package.
    assert "find_package(rclcpp" not in cmake
    assert "rclcpp::rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert "#include <sqlite3.h>" not in core_sources
    assert "sqlite3_" not in core_sources

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()
'''


PHASE1B_FUNCTION = '''def test_loc1b_registry_remains_dependency_isolated() -> None:
    cmake = read("CMakeLists.txt")

    registry_layer = "\\n".join(
        (
            read("include/savo_locations/registry.hpp"),
            read("src/registry.cpp"),
        )
    )

    # Later storage phases may add SQLite to a separate
    # library. The LOC-1B in-memory registry itself must
    # remain independent of ROS and SQLite.
    assert "find_package(rclcpp" not in cmake
    assert "rclcpp::rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert "#include <sqlite3.h>" not in registry_layer
    assert "sqlite3_" not in registry_layer

    assert not (ROOT / "launch").exists()

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()
'''


PHASE1C_FUNCTION = '''def test_loc1c_catalog_remains_dependency_isolated() -> None:
    cmake = read("CMakeLists.txt")

    catalog_layer = "\\n".join(
        (
            read(
                "include/savo_locations/"
                "location_catalog.hpp"
            ),
            read("src/location_catalog.cpp"),
        )
    )

    # SQLite is allowed only in the dedicated LOC-2
    # storage layer. The candidate and approval catalog
    # must remain a pure in-memory domain component.
    assert "find_package(rclcpp" not in cmake
    assert "rclcpp::rclcpp" not in cmake
    assert "add_executable(" not in cmake

    assert "#include <sqlite3.h>" not in catalog_layer
    assert "sqlite3_" not in catalog_layer

    assert not (ROOT / "launch").exists()

    assert not (
        ROOT
        / "src"
        / "location_registry_node.cpp"
    ).exists()
'''


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def replace_function(
    path: Path,
    old_function_name: str,
    new_function_name: str,
    replacement: str,
) -> None:
    text = path.read_text(encoding="utf-8")

    if f"def {new_function_name}(" in text:
        print(f"Already corrected: {path.name}")
        return

    lines = text.splitlines(keepends=True)

    start = None

    for index, line in enumerate(lines):
        if line.startswith(
            f"def {old_function_name}("
        ):
            start = index
            break

    if start is None:
        raise RuntimeError(
            f"Could not find {old_function_name} "
            f"in {path}"
        )

    end = len(lines)

    for index in range(start + 1, len(lines)):
        if lines[index].startswith("def "):
            end = index
            break

    replacement_text = replacement.rstrip() + "\n\n\n"

    updated = (
        "".join(lines[:start])
        + replacement_text
        + "".join(lines[end:])
    )

    if f"def {new_function_name}(" not in updated:
        raise RuntimeError(
            f"Replacement verification failed: "
            f"{new_function_name}"
        )

    path.write_text(
        updated,
        encoding="utf-8",
    )


def main() -> None:
    for path in (
        PHASE1A,
        PHASE1B,
        PHASE1C,
    ):
        if not path.is_file():
            raise SystemExit(
                f"Required contract test missing: {path}"
            )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC2A_historical_contract_fix_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        for path in (
            PHASE1A,
            PHASE1B,
            PHASE1C,
        ):
            archive.add(
                path,
                arcname=(
                    "core/savo_locations/"
                    "test/contracts/"
                    f"{path.name}"
                ),
            )

    replace_function(
        PHASE1A,
        "test_loc1a_core_remains_ros_independent",
        "test_loc1a_core_remains_dependency_isolated",
        PHASE1A_FUNCTION,
    )

    replace_function(
        PHASE1B,
        "test_loc1b_has_no_ros_or_sqlite_runtime",
        "test_loc1b_registry_remains_dependency_isolated",
        PHASE1B_FUNCTION,
    )

    replace_function(
        PHASE1C,
        "test_loc1c_remains_without_ros_or_sqlite",
        "test_loc1c_catalog_remains_dependency_isolated",
        PHASE1C_FUNCTION,
    )

    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            PHASE1A,
            PHASE1B,
            PHASE1C,
        )
    )

    forbidden_obsolete_assertion = (
        'assert "find_package(SQLite3" not in cmake'
    )

    if forbidden_obsolete_assertion in combined:
        raise RuntimeError(
            "An obsolete package-wide SQLite "
            "assertion remains."
        )

    required_fragments = (
        "test_loc1a_core_remains_dependency_isolated",
        "test_loc1b_registry_remains_dependency_isolated",
        "test_loc1c_catalog_remains_dependency_isolated",
        '"#include <sqlite3.h>" not in core_sources',
        '"#include <sqlite3.h>" not in registry_layer',
        '"#include <sqlite3.h>" not in catalog_layer',
    )

    for fragment in required_fragments:
        if fragment not in combined:
            raise RuntimeError(
                f"Verification failed: {fragment}"
            )

    manifest = (
        LOGS
        / f"LOC2A_historical_contract_fix_{stamp}.sha256"
    )

    manifest.write_text(
        "\n".join(
            (
                f"{sha256(path)}  "
                "core/savo_locations/"
                "test/contracts/"
                f"{path.name}"
            )
            for path in (
                PHASE1A,
                PHASE1B,
                PHASE1C,
            )
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")

    print(
        "LOC-1A/1B/1C historical contracts "
        "updated for the LOC-2 storage layer."
    )

    print(
        "Domain, registry and catalog layers "
        "remain ROS- and SQLite-independent."
    )


if __name__ == "__main__":
    main()
