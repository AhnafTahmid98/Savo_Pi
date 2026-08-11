#!/usr/bin/env python3
"""Print the physical-lock status and derived Robot SAVO geometry."""

import argparse
from pathlib import Path

from geometry_profile import canonical_digest, footprint, load_profile, validate_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    profile = load_profile(args.profile)
    validate_profile(profile)
    metadata = profile["metadata"]
    print(f"Profile: {metadata['profile_id']} revision {metadata['geometry_revision']}")
    print(f"Measurement state: {metadata['measurement_state'].upper()}")
    print(f"SHA-256: {canonical_digest(profile)}")
    chassis = profile["chassis"]
    wheels = profile["wheels"]
    wheelbase = float(wheels["front_x_m"]) - float(wheels["rear_x_m"])
    track = float(wheels["left_y_m"]) - float(wheels["right_y_m"])
    print(
        "Wheel geometry: "
        f"wheelbase={wheelbase:.3f} m track={track:.3f} m "
        f"k={(wheelbase + track) / 2.0:.3f} m radius={wheels['radius_m']:.4f} m"
    )
    print(
        "base_link convention: axle plane at "
        f"{chassis['base_footprint_to_base_link_z_m']:.4f} m"
    )
    print(f"Measured plate-only footprint: {footprint(profile)}")
    print(f"Production footprint policy: {profile['navigation']['production_footprint_policy']}")
    if metadata["measurement_state"] == "provisional":
        print("Physical measurement lock: NOT COMPLETE")
        print("Remaining calibration:")
        for item in profile["calibration_remaining"]:
            print(f"  - {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
