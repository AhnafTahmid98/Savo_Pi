#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


COPY_FILES = {
    "VL53L1X_api.h": "third_party/vl53l1x_uld/include/VL53L1X_api.h",
    "VL53L1X_calibration.h": "third_party/vl53l1x_uld/include/VL53L1X_calibration.h",
    "vl53l1_types.h": "third_party/vl53l1x_uld/include/vl53l1_types.h",
    "vl53l1_error_codes.h": "third_party/vl53l1x_uld/include/vl53l1_error_codes.h",
    "VL53L1X_api.cpp": "third_party/vl53l1x_uld/src/VL53L1X_api.cpp",
    "VL53L1X_calibration.cpp": "third_party/vl53l1x_uld/src/VL53L1X_calibration.cpp",
}


COMPAT_TYPES_HEADER = """#ifndef SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_INCLUDE_VL53L1X_TYPES_H_
#define SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_INCLUDE_VL53L1X_TYPES_H_

#include "vl53l1_types.h"

#endif  // SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_INCLUDE_VL53L1X_TYPES_H_
"""

PLATFORM_TYPES_HEADER = """#ifndef SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_TYPES_H_
#define SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_TYPES_H_

#include "../include/vl53l1_types.h"

#endif  // SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_TYPES_H_
"""


def find_file(source_root: Path, filename: str) -> Path | None:
    matches = sorted(source_root.rglob(filename))
    if not matches:
        return None
    return matches[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Import VL53L1X ULD core files into savo_perception."
    )
    parser.add_argument(
        "--source",
        required=True,
        help="Path to extracted/cloned VL53L1X ULD source folder.",
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

    for filename, relative_dest in COPY_FILES.items():
        source_file = find_file(source_root, filename)
        if source_file is None:
            missing.append(filename)
            continue

        dest_file = package_root / relative_dest
        dest_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_file, dest_file)
        copied.append((source_file, dest_file))

    compat_header = package_root / "third_party/vl53l1x_uld/include/VL53L1X_types.h"
    compat_header.parent.mkdir(parents=True, exist_ok=True)
    compat_header.write_text(COMPAT_TYPES_HEADER, encoding="utf-8")

    platform_types_header = package_root / "third_party/vl53l1x_uld/platform/vl53l1_types.h"
    platform_types_header.parent.mkdir(parents=True, exist_ok=True)
    platform_types_header.write_text(PLATFORM_TYPES_HEADER, encoding="utf-8")

    print("Copied files:")
    for source_file, dest_file in copied:
        print(f"  {source_file} -> {dest_file}")

    print(f"  generated -> {compat_header}")
    print(f"  generated -> {platform_types_header}")

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
