#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REQUIRED_FILES = {
    "VL53L1X_api.h": "third_party/vl53l1x_uld/include/VL53L1X_api.h",
    "VL53L1X_calibration.h": "third_party/vl53l1x_uld/include/VL53L1X_calibration.h",
    "VL53L1X_types.h": "third_party/vl53l1x_uld/include/VL53L1X_types.h",
    "VL53L1X_api.c": "third_party/vl53l1x_uld/src/VL53L1X_api.c",
    "VL53L1X_calibration.c": "third_party/vl53l1x_uld/src/VL53L1X_calibration.c",
}


def find_file(source_root: Path, filename: str) -> Path | None:
    matches = sorted(source_root.rglob(filename))
    if not matches:
        return None
    return matches[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Import ST VL53L1X ULD core files into savo_perception."
    )
    parser.add_argument(
        "--source",
        required=True,
        help="Path to extracted STSW-IMG009 / VL53L1X ULD source folder.",
    )
    parser.add_argument(
        "--package-root",
        default=".",
        help="Path to savo_perception package root. Default: current directory.",
    )
    args = parser.parse_args()

    source_root = Path(args.source).expanduser().resolve()
    package_root = Path(args.package_root).expanduser().resolve()

    if not source_root.exists():
        raise FileNotFoundError(f"source folder does not exist: {source_root}")

    if not (package_root / "package.xml").exists():
        raise FileNotFoundError(f"not a ROS package root: {package_root}")

    missing: list[str] = []
    copied: list[tuple[Path, Path]] = []

    for filename, relative_dest in REQUIRED_FILES.items():
        source_file = find_file(source_root, filename)
        if source_file is None:
            missing.append(filename)
            continue

        dest_file = package_root / relative_dest
        dest_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_file, dest_file)
        copied.append((source_file, dest_file))

    print("Copied files:")
    for source_file, dest_file in copied:
        print(f"  {source_file} -> {dest_file}")

    if missing:
        print()
        print("Missing files:")
        for filename in missing:
            print(f"  {filename}")
        return 1

    print()
    print("VL53L1X ULD import: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
