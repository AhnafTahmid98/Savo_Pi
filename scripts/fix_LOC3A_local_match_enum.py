#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
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

VIEW_HEADER = (
    PACKAGE
    / "include"
    / "savo_locations"
    / "read_only_catalog_view.hpp"
)

NODE_SOURCE = (
    PACKAGE
    / "src"
    / "location_registry_node.cpp"
)

INSTALLER = (
    ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


ENUM_BLOCK = """enum class ResolveMatchType
{
  kNone = 0,
  kLocationId = 1,
  kDisplayName = 2,
  kAlias = 3,
};

"""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def insert_enum(path: Path) -> str:
    text = path.read_text(encoding="utf-8")

    if "enum class ResolveMatchType" in text:
        return "already_correct"

    anchor = "struct ReadResolveResult"

    position = text.find(anchor)

    if position < 0:
        raise RuntimeError(
            "Could not locate ReadResolveResult "
            f"in {path}"
        )

    line_start = text.rfind(
        "\n",
        0,
        position,
    ) + 1

    indentation = text[
        line_start:position
    ]

    if indentation.strip():
        raise RuntimeError(
            "Unexpected content before "
            f"ReadResolveResult in {path}"
        )

    indented_enum = "".join(
        (
            indentation + line
            if line.strip()
            else line
        )
        for line in ENUM_BLOCK.splitlines(
            keepends=True
        )
    )

    updated = (
        text[:line_start]
        + indented_enum
        + text[line_start:]
    )

    path.write_text(
        updated,
        encoding="utf-8",
    )

    return "corrected"


def repair_namespaces(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    original = text

    replacements = {
        (
            "::savo_locations::::savo_locations::"
            "topic_names::"
        ): (
            "::savo_locations::topic_names::"
        ),
        (
            "::savo_locations::::savo_locations::"
            "service_names::"
        ): (
            "::savo_locations::service_names::"
        ),
    }

    for old, new in replacements.items():
        text = text.replace(old, new)

    if text == original:
        return "already_correct"

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def verify(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    if "enum class ResolveMatchType" not in text:
        raise RuntimeError(
            f"ResolveMatchType enum missing in {path}"
        )

    for enumerator in (
        "kNone",
        "kLocationId",
        "kDisplayName",
        "kAlias",
    ):
        if enumerator not in text:
            raise RuntimeError(
                f"Missing enum value {enumerator} "
                f"in {path}"
            )


def verify_namespaces(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    malformed = (
        "::savo_locations::::savo_locations::"
    )

    if malformed in text:
        raise RuntimeError(
            f"Duplicated namespace remains in {path}"
        )

    required = (
        "::savo_locations::topic_names::kStatus",
        "::savo_locations::topic_names::kSnapshot",
        "::savo_locations::topic_names::kHeartbeat",
        "::savo_locations::service_names::kResolve",
        "::savo_locations::service_names::kGet",
        "::savo_locations::service_names::kList",
    )

    for fragment in required:
        if fragment not in text:
            raise RuntimeError(
                f"Required contract name missing: "
                f"{fragment} in {path}"
            )


def main() -> None:
    for path in (
        PACKAGE,
        VIEW_HEADER,
        NODE_SOURCE,
        INSTALLER,
    ):
        if not path.exists():
            raise SystemExit(
                f"Required path missing: {path}"
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

    package_backup = (
        BACKUPS
        / f"partial_LOC3A_before_local_enum_fix_"
          f"{stamp}.tar.gz"
    )

    with tarfile.open(
        package_backup,
        "w:gz",
    ) as archive:
        archive.add(
            PACKAGE,
            arcname="savo_locations",
        )

    installer_backup = (
        BACKUPS
        / f"pre_LOC3A_local_enum_fix_"
          f"{stamp}_installer.py"
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    active_enum_result = insert_enum(
        VIEW_HEADER
    )

    installer_enum_result = insert_enum(
        INSTALLER
    )

    active_namespace_result = (
        repair_namespaces(NODE_SOURCE)
    )

    installer_namespace_result = (
        repair_namespaces(INSTALLER)
    )

    verify(VIEW_HEADER)
    verify(INSTALLER)

    verify_namespaces(NODE_SOURCE)
    verify_namespaces(INSTALLER)

    manifest = (
        LOGS
        / f"LOC3A_local_match_enum_fix_"
          f"{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(VIEW_HEADER)}  "
            "savo_ws/src/core/savo_locations/"
            "include/savo_locations/"
            "read_only_catalog_view.hpp\n"
            f"{sha256(NODE_SOURCE)}  "
            "savo_ws/src/core/savo_locations/"
            "src/location_registry_node.cpp\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(
        f"Active ResolveMatchType : "
        f"{active_enum_result}"
    )

    print(
        f"Installer ResolveMatchType: "
        f"{installer_enum_result}"
    )

    print(
        f"Active namespaces       : "
        f"{active_namespace_result}"
    )

    print(
        f"Installer namespaces    : "
        f"{installer_namespace_result}"
    )

    print(
        f"Partial package backup  : "
        f"{package_backup}"
    )

    print(
        f"Installer backup        : "
        f"{installer_backup}"
    )

    print(
        f"Permanent manifest      : "
        f"{manifest}"
    )

    print(
        "LOC-3A local match enum and namespace "
        "repair complete."
    )


if __name__ == "__main__":
    main()
