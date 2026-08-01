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
    print(f"Nav2 footprint: {footprint(profile)}")
    if metadata["measurement_state"] == "provisional":
        print("Physical measurement lock: NOT COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
