#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
import shutil


ROOT = Path.home() / "Savo_Pi"

CMAKE = (
    ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
    / "CMakeLists.txt"
)

INSTALLER = (
    ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


KEYWORD_PATTERN = re.compile(
    r"(?m)^(?P<indent>[ \t]*)target_link_libraries\(\s*\n"
    r"[ \t]*savo_locations_node\s*\n"
    r"[ \t]*PRIVATE\s*\n"
    r"[ \t]*savo_locations_ros\s*\n"
    r"[ \t]*\)"
)

PLAIN_PATTERN = re.compile(
    r"(?m)^(?P<indent>[ \t]*)target_link_libraries\(\s*\n"
    r"[ \t]*savo_locations_node\s*\n"
    r"[ \t]*savo_locations_ros\s*\n"
    r"[ \t]*\)"
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


def patch(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(
            f"Required file is missing: {path}"
        )

    text = path.read_text(encoding="utf-8")

    match = KEYWORD_PATTERN.search(text)

    if match is not None:
        indent = match.group("indent")

        replacement = (
            f"{indent}target_link_libraries(\n"
            f"{indent}  savo_locations_node\n"
            f"{indent}  savo_locations_ros\n"
            f"{indent})"
        )

        updated, count = KEYWORD_PATTERN.subn(
            replacement,
            text,
            count=1,
        )

        if count != 1:
            raise RuntimeError(
                f"Expected one replacement in {path}; "
                f"performed {count}"
            )

        path.write_text(
            updated,
            encoding="utf-8",
        )

        return "corrected"

    if PLAIN_PATTERN.search(text) is not None:
        return "already_correct"

    raise RuntimeError(
        "Could not locate either the keyword or plain "
        f"savo_locations_node link block in {path}"
    )


def verify(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    if KEYWORD_PATTERN.search(text) is not None:
        raise RuntimeError(
            f"PRIVATE link signature remains in {path}"
        )

    if PLAIN_PATTERN.search(text) is None:
        raise RuntimeError(
            f"Plain link signature is missing from {path}"
        )


def main() -> None:
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

    cmake_backup = (
        BACKUPS
        / f"pre_LOC3A_link_fix_v2_{stamp}_CMakeLists.txt"
    )

    installer_backup = (
        BACKUPS
        / f"pre_LOC3A_link_fix_v2_{stamp}_installer.py"
    )

    shutil.copy2(
        CMAKE,
        cmake_backup,
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    cmake_result = patch(CMAKE)
    installer_result = patch(INSTALLER)

    verify(CMAKE)
    verify(INSTALLER)

    manifest = (
        LOGS
        / f"LOC3A_node_link_signature_fix_v2_{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(CMAKE)}  "
            "savo_ws/src/core/savo_locations/CMakeLists.txt\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(f"CMakeLists.txt  : {cmake_result}")
    print(f"LOC-3A installer: {installer_result}")
    print(f"CMake backup    : {cmake_backup}")
    print(f"Installer backup: {installer_backup}")
    print(f"Manifest        : {manifest}")
    print(
        "LOC-3A node link signature correction complete."
    )


if __name__ == "__main__":
    main()
