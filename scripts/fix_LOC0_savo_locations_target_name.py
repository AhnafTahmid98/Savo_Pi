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
BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def main() -> None:
    if not CMAKE.is_file():
        raise SystemExit(f"Missing CMakeLists.txt: {CMAKE}")

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    backup = (
        BACKUPS
        / f"pre_LOC0_target_name_CMakeLists_{stamp}.txt"
    )

    shutil.copy2(CMAKE, backup)

    original = CMAKE.read_text(encoding="utf-8")

    old = "${PROJECT_NAME}_contracts"
    new = "savo_locations_contracts"

    count = original.count(old)

    if count == 0:
        if new in original:
            print("Correction already applied.")
            print(f"Current SHA-256: {sha256(CMAKE)}")
            return

        raise RuntimeError(
            "Expected target expression was not found."
        )

    updated = original.replace(old, new)

    CMAKE.write_text(updated, encoding="utf-8")

    verified = CMAKE.read_text(encoding="utf-8")

    if old in verified:
        raise RuntimeError(
            "Old variable-based target name remains."
        )

    required_fragments = (
        "add_library(\n  savo_locations_contracts",
        "target_include_directories(\n  savo_locations_contracts",
        "target_compile_features(\n  savo_locations_contracts",
        "TARGETS\n    savo_locations_contracts",
        "target_link_libraries(\n      test_location_types\n      savo_locations_contracts",
    )

    missing = [
        fragment
        for fragment in required_fragments
        if fragment not in verified
    ]

    if missing:
        shutil.copy2(backup, CMAKE)

        raise RuntimeError(
            "Verification failed; original CMakeLists.txt restored.\n"
            + "\n".join(missing)
        )

    manifest = (
        LOGS
        / f"LOC0_target_name_fix_{stamp}.sha256"
    )

    manifest.write_text(
        f"{sha256(CMAKE)}  "
        "core/savo_locations/CMakeLists.txt\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")
    print(
        f"Replaced {count} target-name occurrences."
    )
    print("LOC-0 target-name correction verified.")


if __name__ == "__main__":
    main()
