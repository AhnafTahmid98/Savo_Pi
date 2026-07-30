#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import subprocess
import tarfile
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path.home() / "Savo_Pi"
SOURCE_ROOT = ROOT / "savo_ws" / "src"
BACKUP_ROOT = ROOT / "backups"
LOG_ROOT = ROOT / "change_logs"

REQUIRED_PACKAGES = {
    "savo_msgs",
    "savo_locations",
    "savo_head",
    "savo_mapping",
    "savo_nav",
    "savo_supervisor",
}

OPTIONAL_PACKAGES = {
    "savo_bringup",
    "savo_core_bringup",
    "savo_robot_bringup",
}


def find_packages() -> dict[str, Path]:
    packages: dict[str, Path] = {}

    for package_xml in SOURCE_ROOT.rglob("package.xml"):
        try:
            root = ET.parse(package_xml).getroot()
            name = (root.findtext("name") or "").strip()
        except (ET.ParseError, OSError):
            continue

        if name:
            packages[name] = package_xml.parent

    return packages


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def main() -> int:
    if not SOURCE_ROOT.is_dir():
        raise SystemExit(
            f"Workspace source directory missing: {SOURCE_ROOT}"
        )

    BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
    LOG_ROOT.mkdir(parents=True, exist_ok=True)

    packages = find_packages()

    missing = sorted(
        name
        for name in REQUIRED_PACKAGES
        if name not in packages
    )

    if missing:
        print("ERROR: required packages are missing:")
        for name in missing:
            print(f"  {name}")
        return 1

    selected_names = sorted(
        REQUIRED_PACKAGES
        | {
            name
            for name in OPTIONAL_PACKAGES
            if name in packages
        }
    )

    selected_paths = [
        packages[name]
        for name in selected_names
    ]

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    archive = (
        BACKUP_ROOT
        / f"LOC3P_input_current_{stamp}.tar.gz"
    )

    manifest = (
        LOG_ROOT
        / f"LOC3P_input_current_{stamp}.sha256"
    )

    with tarfile.open(archive, "w:gz") as tar:
        for name, package_path in zip(
            selected_names,
            selected_paths,
            strict=True,
        ):
            relative = package_path.relative_to(SOURCE_ROOT)

            print(f"Including {name}: {relative}")

            tar.add(
                package_path,
                arcname=str(relative),
                recursive=True,
            )

    digest = sha256(archive)

    manifest.write_text(
        f"{digest}  {archive}\n",
        encoding="utf-8",
    )

    print()
    print("LOC-3P umbrella input snapshot created.")
    print(f"Archive : {archive}")
    print(f"SHA-256: {digest}")
    print(f"Manifest: {manifest}")
    print()
    print("Included packages:")

    for name in selected_names:
        relative = packages[name].relative_to(SOURCE_ROOT)
        print(f"  {name}: {relative}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
