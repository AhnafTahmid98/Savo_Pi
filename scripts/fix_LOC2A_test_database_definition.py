#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
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


CORRECT_BLOCK = r'''          target_compile_definitions(
            test_sqlite_store
            PRIVATE
              "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
          )'''


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def main() -> None:
    if not CMAKE.is_file():
        raise SystemExit(
            f"Missing CMakeLists.txt: {CMAKE}"
        )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC2A_test_db_definition_{stamp}.txt"
    )

    shutil.copy2(CMAKE, backup)

    text = CMAKE.read_text(encoding="utf-8")

    correct_definition = (
        '"SAVO_LOCATIONS_TEST_DB_DIR='
        '\\"${CMAKE_CURRENT_BINARY_DIR}/'
        'storage_test_runtime\\""'
    )

    if correct_definition in text:
        print("LOC-2A test database definition is already correct.")
        print(f"Permanent backup : {backup}")
        return

    pattern = re.compile(
        r'''
        target_compile_definitions
        \s*\(
        \s*test_sqlite_store
        \s*PRIVATE
        \s*SAVO_LOCATIONS_TEST_DB_DIR
        \s*=
        \s*"\$\{CMAKE_CURRENT_BINARY_DIR\}
        /storage_test_runtime"
        \s*\)
        ''',
        re.VERBOSE,
    )

    updated, count = pattern.subn(
        CORRECT_BLOCK,
        text,
        count=1,
    )

    if count != 1:
        raise RuntimeError(
            "Could not locate the malformed "
            "SAVO_LOCATIONS_TEST_DB_DIR definition."
        )

    if correct_definition not in updated:
        raise RuntimeError(
            "Corrected compiler definition failed verification."
        )

    CMAKE.write_text(
        updated,
        encoding="utf-8",
    )

    manifest = (
        LOGS
        / f"LOC2A_test_db_definition_fix_{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(CMAKE)}  "
            "core/savo_locations/CMakeLists.txt\n"
        ),
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")
    print("LOC-2A test database compiler definition corrected.")
    print(
        "The permanent test database remains under "
        "build/savo_locations/storage_test_runtime."
    )


if __name__ == "__main__":
    main()
