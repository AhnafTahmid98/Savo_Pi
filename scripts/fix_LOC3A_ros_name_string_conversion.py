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


NAME_CONSTANTS = (
    "::savo_locations::topic_names::kStatus",
    "::savo_locations::topic_names::kSnapshot",
    "::savo_locations::topic_names::kHeartbeat",
    "::savo_locations::service_names::kResolve",
    "::savo_locations::service_names::kGet",
    "::savo_locations::service_names::kList",
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


def patch_ros_names(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(
            f"Required file missing: {path}"
        )

    text = path.read_text(encoding="utf-8")
    original = text

    for constant in NAME_CONSTANTS:
        wrapped = f"std::string({constant})"

        if wrapped in text:
            continue

        argument = constant + ","

        if argument not in text:
            raise RuntimeError(
                "Could not locate unconverted ROS-name "
                f"argument {constant} in {path}"
            )

        text = text.replace(
            argument,
            wrapped + ",",
            1,
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

    for constant in NAME_CONSTANTS:
        wrapped = f"std::string({constant})"

        if wrapped not in text:
            raise RuntimeError(
                f"Converted argument missing: "
                f"{wrapped} in {path}"
            )

        unwrapped_argument = constant + ","

        if unwrapped_argument in text.replace(
            wrapped + ",",
            "",
        ):
            raise RuntimeError(
                f"Unconverted argument remains: "
                f"{constant} in {path}"
            )


def main() -> None:
    for path in (
        PACKAGE,
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
        / f"partial_LOC3A_before_ros_name_string_fix_"
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
        / f"pre_LOC3A_ros_name_string_fix_"
          f"{stamp}_installer.py"
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    active_result = patch_ros_names(
        NODE_SOURCE
    )

    installer_result = patch_ros_names(
        INSTALLER
    )

    verify(NODE_SOURCE)
    verify(INSTALLER)

    manifest = (
        LOGS
        / f"LOC3A_ros_name_string_conversion_"
          f"{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(NODE_SOURCE)}  "
            "savo_ws/src/core/savo_locations/"
            "src/location_registry_node.cpp\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(
        f"Active ROS names    : {active_result}"
    )

    print(
        f"Installer ROS names : {installer_result}"
    )

    print(
        f"Partial package backup: {package_backup}"
    )

    print(
        f"Installer backup      : {installer_backup}"
    )

    print(
        f"Permanent manifest    : {manifest}"
    )

    print(
        "LOC-3A ROS topic/service names now use "
        "explicit std::string conversion."
    )


if __name__ == "__main__":
    main()
