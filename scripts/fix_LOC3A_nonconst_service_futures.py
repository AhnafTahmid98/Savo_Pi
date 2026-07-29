#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import shutil


ROOT = Path.home() / "Savo_Pi"

PACKAGE = (
    ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

ACTIVE_TEST = (
    PACKAGE
    / "test"
    / "ros"
    / "test_registry_node.cpp"
)

INSTALLER = (
    ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


REPLACEMENTS = {
    "const auto resolve_future =":
        "auto resolve_future =",

    "const auto get_future =":
        "auto get_future =",

    "const auto list_future =":
        "auto list_future =",
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


def patch(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(
            f"Required file missing: {path}"
        )

    text = path.read_text(encoding="utf-8")
    original = text

    for old, new in REPLACEMENTS.items():
        if old in text:
            text = text.replace(old, new, 1)
        elif new not in text:
            raise RuntimeError(
                f"Could not locate service future "
                f"declaration '{old}' in {path}"
            )

    if text == original:
        return "already_correct"

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def verify(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    for old, new in REPLACEMENTS.items():
        if old in text:
            raise RuntimeError(
                f"Const future remains in {path}: {old}"
            )

        if new not in text:
            raise RuntimeError(
                f"Corrected future missing in {path}: "
                f"{new}"
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

    active_backup = (
        BACKUPS
        / f"pre_LOC3A_future_fix_{stamp}_"
          "test_registry_node.cpp"
    )

    installer_backup = (
        BACKUPS
        / f"pre_LOC3A_future_fix_{stamp}_"
          "installer.py"
    )

    shutil.copy2(
        ACTIVE_TEST,
        active_backup,
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    active_result = patch(ACTIVE_TEST)
    installer_result = patch(INSTALLER)

    verify(ACTIVE_TEST)
    verify(INSTALLER)

    manifest = (
        LOGS
        / f"LOC3A_nonconst_service_futures_"
          f"{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(ACTIVE_TEST)}  "
            "savo_ws/src/core/savo_locations/"
            "test/ros/test_registry_node.cpp\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(f"Active ROS test : {active_result}")
    print(f"LOC-3A installer: {installer_result}")
    print(f"Active backup   : {active_backup}")
    print(f"Installer backup: {installer_backup}")
    print(f"Manifest        : {manifest}")

    print(
        "LOC-3A ROS service futures are now "
        "non-const for Jazzy FutureAndRequestId::get()."
    )


if __name__ == "__main__":
    main()
