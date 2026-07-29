#!/usr/bin/env python3
from __future__ import annotations

import ast
from datetime import datetime
import hashlib
from pathlib import Path
import shutil
import tarfile


ROOT = Path.home() / "Savo_Pi"

PACKAGE = (
    ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

CONTRACTS = PACKAGE / "test" / "contracts"

INSTALLER = (
    ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


REPLACEMENTS = {
    CONTRACTS / "test_phase1a_contracts.py": {
        "test_loc1a_core_remains_dependency_isolated":
'''def test_loc1a_core_remains_dependency_isolated() -> None:
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
''',
    },

    CONTRACTS / "test_phase1b_contracts.py": {
        "test_loc1b_registry_remains_dependency_isolated":
'''def test_loc1b_registry_remains_dependency_isolated() -> None:
    registry_layer = "\\n".join(
        (
            read("include/savo_locations/registry.hpp"),
            read("src/registry.cpp"),
        )
    )

    # The in-memory registry remains independent even
    # though later targets in the package use ROS/SQLite.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "sqlite3",
        "sqlite_store",
        "sqlite_repository",
    ):
        assert forbidden not in registry_layer
''',
    },

    CONTRACTS / "test_phase1c_contracts.py": {
        "test_loc1c_catalog_remains_dependency_isolated":
'''def test_loc1c_catalog_remains_dependency_isolated() -> None:
    catalog_layer = "\\n".join(
        (
            read(
                "include/savo_locations/"
                "location_catalog.hpp"
            ),
            read("src/location_catalog.cpp"),
        )
    )

    # Candidate and approval behavior remains a pure
    # in-memory domain component.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "sqlite3",
        "sqlite_store",
        "sqlite_repository",
    ):
        assert forbidden not in catalog_layer
''',
    },

    CONTRACTS / "test_phase2a_contracts.py": {
        "test_loc2a_has_no_ros_runtime_node":
'''def test_loc2a_storage_layer_remains_ros_independent() -> None:
    storage_layer = "\\n".join(
        (
            read(
                "include/savo_locations/"
                "sqlite_schema.hpp"
            ),
            read(
                "include/savo_locations/"
                "sqlite_store.hpp"
            ),
            read("src/sqlite_store.cpp"),
        )
    )

    # LOC-3A may add a separate ROS target. LOC-2A
    # storage implementation must not depend on ROS.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
    ):
        assert forbidden not in storage_layer
''',
    },

    CONTRACTS / "test_phase2b_contracts.py": {
        "test_loc2b_still_has_no_ros_runtime_node":
'''def test_loc2b_repository_remains_ros_independent() -> None:
    repository_layer = "\\n".join(
        (
            read(
                "include/savo_locations/"
                "sqlite_repository.hpp"
            ),
            read("src/sqlite_repository.cpp"),
        )
    )

    # Typed persistence remains below and independent
    # from the later LOC-3A ROS adapter.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
    ):
        assert forbidden not in repository_layer
''',
    },

    CONTRACTS / "test_phase2c_contracts.py": {
        "test_loc2c_remains_without_ros_runtime":
'''def test_loc2c_persistence_remains_ros_independent() -> None:
    persistence_layer = "\\n".join(
        (
            read(
                "include/savo_locations/"
                "sqlite_schema.hpp"
            ),
            read(
                "include/savo_locations/"
                "sqlite_store.hpp"
            ),
            read(
                "include/savo_locations/"
                "sqlite_repository.hpp"
            ),
            read("src/sqlite_store.cpp"),
            read("src/sqlite_repository.cpp"),
        )
    )

    # Bootstrap, transactions and the append-only event
    # journal remain ROS-independent storage behavior.
    for forbidden in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
    ):
        assert forbidden not in persistence_layer
''',
    },

    CONTRACTS / "test_phase3a_contracts.py": {
        "test_node_exposes_only_read_services":
'''def test_node_exposes_only_read_services() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    header = read(
        "include/savo_locations/"
        "location_registry_node.hpp"
    )

    combined = source + header

    for endpoint in (
        "service_names::kResolve",
        "service_names::kGet",
        "service_names::kList",
    ):
        assert endpoint in combined

    # LOC-3A is intentionally read-only. Write service
    # interfaces must not be instantiated by the node.
    for forbidden in (
        "RegisterLocationCandidate",
        "ApproveLocation",
        "SetLocationEnabled",
        "handle_register",
        "handle_approve",
        "handle_set_enabled",
    ):
        assert forbidden not in combined
''',

        "test_status_heartbeat_and_snapshot_are_latched":
'''def test_status_snapshot_latched_heartbeat_volatile() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    topics = read(
        "include/savo_locations/topic_names.hpp"
    )

    assert "/savo_locations/status" in topics
    assert "/savo_locations/heartbeat" in topics
    assert "/savo_locations/snapshot" in topics

    # Status and snapshot retain their latest values.
    assert "transient_local()" in source

    # Heartbeat is a continuous volatile signal.
    assert "durability_volatile()" in source

    # Status/snapshot identify LOC-3A as read-only.
    assert "mode" in source
    assert "read_only" in source
''',
    },
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def replace_functions(
    path: Path,
    replacements: dict[str, str],
) -> int:
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text)

    targets: list[
        tuple[int, int, str, str]
    ] = []

    for node in tree.body:
        if not isinstance(
            node,
            (ast.FunctionDef, ast.AsyncFunctionDef),
        ):
            continue

        if node.name not in replacements:
            continue

        if node.end_lineno is None:
            raise RuntimeError(
                f"Missing end line for {node.name}"
            )

        targets.append(
            (
                node.lineno,
                node.end_lineno,
                node.name,
                replacements[node.name],
            )
        )

    found = {item[2] for item in targets}
    expected = set(replacements)

    if found != expected:
        missing = sorted(expected - found)

        raise RuntimeError(
            f"Functions missing from {path}: "
            f"{', '.join(missing)}"
        )

    lines = text.splitlines(
        keepends=True
    )

    for start, end, _, replacement in sorted(
        targets,
        reverse=True,
    ):
        lines[start - 1:end] = [
            replacement.rstrip() + "\n\n"
        ]

    updated = "".join(lines)

    # Verify the edited test module is syntactically valid.
    ast.parse(updated)

    path.write_text(
        updated,
        encoding="utf-8",
    )

    return len(targets)


def patch_loc3a_installer(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    original = text

    replacements = {
        '"services::kResolve"':
            '"service_names::kResolve"',
        '"services::kGet"':
            '"service_names::kGet"',
        '"services::kList"':
            '"service_names::kList"',
    }

    for old, new in replacements.items():
        text = text.replace(old, new)

    old_mode_assertions = (
        '''assert '"mode":"read_only"' in source''',
        '''assert '\\"mode\\":\\"read_only\\"' in source''',
    )

    new_mode_assertion = '''assert "mode" in source
    assert "read_only" in source
    assert "durability_volatile()" in source'''

    for old in old_mode_assertions:
        if old in text:
            text = text.replace(
                old,
                new_mode_assertion,
                1,
            )
            break

    if text == original:
        return "already_correct"

    # Verify the installer itself remains valid Python.
    ast.parse(text)

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def main() -> None:
    required = tuple(REPLACEMENTS) + (
        INSTALLER,
    )

    for path in required:
        if not path.is_file():
            raise SystemExit(
                f"Required file missing: {path}"
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

    contracts_backup = (
        BACKUPS
        / f"pre_LOC3A_contract_migration_"
          f"{stamp}.tar.gz"
    )

    with tarfile.open(
        contracts_backup,
        "w:gz",
    ) as archive:
        for path in REPLACEMENTS:
            archive.add(
                path,
                arcname=(
                    "test/contracts/"
                    + path.name
                ),
            )

    installer_backup = (
        BACKUPS
        / f"pre_LOC3A_contract_migration_"
          f"{stamp}_installer.py"
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    total_functions = 0

    for path, replacements in REPLACEMENTS.items():
        count = replace_functions(
            path,
            replacements,
        )

        total_functions += count

        print(
            f"Updated {path.name}: "
            f"{count} function(s)"
        )

    installer_result = (
        patch_loc3a_installer(INSTALLER)
    )

    manifest = (
        LOGS
        / f"LOC3A_contract_migration_"
          f"{stamp}.sha256"
    )

    manifest_lines = []

    for path in REPLACEMENTS:
        manifest_lines.append(
            f"{sha256(path)}  "
            f"savo_ws/src/core/savo_locations/"
            f"test/contracts/{path.name}"
        )

    manifest_lines.append(
        f"{sha256(INSTALLER)}  "
        "scripts/apply_LOC3A_savo_locations.py"
    )

    manifest.write_text(
        "\n".join(manifest_lines) + "\n",
        encoding="utf-8",
    )

    print(
        f"Updated contract functions: "
        f"{total_functions}"
    )

    print(
        f"LOC-3A installer          : "
        f"{installer_result}"
    )

    print(
        f"Permanent contract backup: "
        f"{contracts_backup}"
    )

    print(
        f"Installer backup         : "
        f"{installer_backup}"
    )

    print(
        f"Permanent manifest       : "
        f"{manifest}"
    )

    print(
        "Historical contracts now validate layer "
        "isolation instead of forbidding LOC-3A."
    )


if __name__ == "__main__":
    main()
