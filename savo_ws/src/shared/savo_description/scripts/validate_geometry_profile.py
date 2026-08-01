#!/usr/bin/env python3
"""Validate a Robot SAVO production geometry profile."""

import argparse
from pathlib import Path

from geometry_profile import GeometryProfileError, load_profile, validate_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", type=Path)
    parser.add_argument("--require-locked", action="store_true")
    parser.add_argument("--allow-provisional", action="store_true")
    args = parser.parse_args()
    try:
        profile = load_profile(args.profile)
        validate_profile(
            profile,
            require_locked=args.require_locked,
            allow_provisional=args.allow_provisional,
        )
    except (OSError, GeometryProfileError) as error:
        print(f"INVALID: {error}")
        return 2
    print(f"VALID: {args.profile} ({profile['metadata']['measurement_state']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
