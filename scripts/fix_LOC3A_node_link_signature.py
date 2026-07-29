#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import shutil


PACKAGE = (
    Path.home()
    / "Savo_Pi"
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

CMAKE = PACKAGE / "CMakeLists.txt"

INSTALLER = (
    Path.home()
    / "Savo_Pi"
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


OLD_BLOCK = """target_link_libraries(
  savo_locations_node
  PRIVATE
    savo_locations_ros
)"""

NEW_BLOCK = """target_link_libraries(
  savo_locations_node
  savo_locations_ros
)"""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def patch_file(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(
            f"Required file missing: {path}"
        )

    text = path.read_text(
        encoding="utf-8"
    )

    if OLD_BLOCK in text:
        updated = text.replace(
            OLD_BLOCK,
            NEW_BLOCK,
            1,
        )

        path.write_text(
            updated,
            encoding="utf-8",
        )

        return "corrected"

    if NEW_BLOCK in text:
        return "already_correct"

    raise RuntimeError(
        "Could not locate the savo_locations_node "
        f"link block in {path}"
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
        / f"pre_LOC3A_node_link_fix_{stamp}_CMakeLists.txt"
    )

    installer_backup = (
        BACKUPS
        / f"pre_LOC3A_node_link_fix_{stamp}_installer.py"
    )

    shutil.copy2(
        CMAKE,
        cmake_backup,
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    cmake_result = patch_file(CMAKE)
    installer_result = patch_file(INSTALLER)

    cmake_text = CMAKE.read_text(
        encoding="utf-8"
    )

    if OLD_BLOCK in cmake_text:
        raise RuntimeError(
            "Keyword target-link signature remains "
            "in active CMakeLists.txt"
        )

    if NEW_BLOCK not in cmake_text:
        raise RuntimeError(
            "Corrected plain target-link signature "
            "is missing"
        )

    manifest = (
        LOGS
        / f"LOC3A_node_link_signature_fix_{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(CMAKE)}  "
            "core/savo_locations/CMakeLists.txt\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(f"CMakeLists.txt : {cmake_result}")
    print(f"LOC-3A installer: {installer_result}")
    print(f"Permanent backup : {cmake_backup}")
    print(f"Installer backup : {installer_backup}")
    print(f"Permanent manifest: {manifest}")

    print(
        "LOC-3A node target now uses the plain "
        "target_link_libraries signature required "
        "with ament_target_dependencies."
    )


if __name__ == "__main__":
    main()
