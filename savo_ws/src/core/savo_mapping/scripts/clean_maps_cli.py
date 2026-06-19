#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Clean saved/session/test map files for Robot Savo mapping."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Iterable, Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.map_metadata import sanitize_map_name  # noqa: E402


# =============================================================================
# Constants
# =============================================================================
DEFAULT_MAP_DIRS = (
    "maps/sessions",
    "maps/test",
)

SAFE_MAP_SUFFIXES = (
    ".yaml",
    ".pgm",
    ".png",
    ".metadata.json",
    ".json",
    ".log",
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Clean Robot Savo temporary map files safely.",
    )

    parser.add_argument(
        "--maps-dir",
        action="append",
        default=[],
        help=(
            "Map directory to clean. Can be used multiple times. "
            "Defaults to maps/sessions and maps/test."
        ),
    )
    parser.add_argument(
        "--map-name",
        default="",
        help="Clean files matching one sanitized map name only.",
    )
    parser.add_argument(
        "--include-saved",
        action="store_true",
        help="Also allow cleaning maps/saved. Dangerous without --confirm.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Clean all safe map files in selected directories.",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Actually delete files. Without this, only dry-run output is shown.",
    )
    parser.add_argument(
        "--keep-gitkeep",
        action="store_true",
        default=True,
        help="Keep .gitkeep files.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON-like Python dict output.",
    )

    return parser


# =============================================================================
# File selection
# =============================================================================
def _selected_dirs(args: argparse.Namespace) -> list[Path]:
    dirs = [Path(item) for item in args.maps_dir] if args.maps_dir else [
        Path(item) for item in DEFAULT_MAP_DIRS
    ]

    if args.include_saved:
        dirs.append(Path("maps/saved"))

    return [path.expanduser() for path in dirs]


def _is_safe_map_file(path: Path) -> bool:
    if not path.is_file():
        return False

    name = path.name

    if name == ".gitkeep":
        return False

    return any(name.endswith(suffix) for suffix in SAFE_MAP_SUFFIXES)


def _matches_map_name(path: Path, map_name: str) -> bool:
    if not map_name:
        return True

    clean = sanitize_map_name(map_name)

    return (
        path.name == clean
        or path.name.startswith(f"{clean}.")
        or path.name.startswith(f"{clean}_")
        or path.stem == clean
    )


def _collect_files(
    directories: Iterable[Path],
    map_name: str = "",
    clean_all: bool = False,
) -> list[Path]:
    files: list[Path] = []

    for directory in directories:
        if not directory.exists() or not directory.is_dir():
            continue

        for path in sorted(directory.rglob("*")):
            if not _is_safe_map_file(path):
                continue

            if not clean_all and not map_name:
                continue

            if not _matches_map_name(path, map_name):
                continue

            files.append(path)

    return files


# =============================================================================
# Deletion
# =============================================================================
def _delete_files(files: Iterable[Path]) -> tuple[int, list[str]]:
    deleted = 0
    errors: list[str] = []

    for path in files:
        try:
            path.unlink()
            deleted += 1
        except Exception as exc:
            errors.append(f"{path}: {type(exc).__name__}: {exc}")

    return deleted, errors


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    directories = _selected_dirs(args)

    if not args.all and not args.map_name:
        print(
            "Nothing selected. Use --map-name <name> or --all. "
            "This tool is dry-run by default."
        )
        return 0

    if args.include_saved and not args.confirm:
        print(
            "maps/saved selected, but --confirm was not given. "
            "Dry-run only."
        )

    files = _collect_files(
        directories=directories,
        map_name=args.map_name,
        clean_all=args.all,
    )

    deleted = 0
    errors: list[str] = []

    if args.confirm:
        deleted, errors = _delete_files(files)

    result = {
        "dry_run": not args.confirm,
        "selected_dirs": [str(path) for path in directories],
        "map_name": sanitize_map_name(args.map_name) if args.map_name else "",
        "all": bool(args.all),
        "matched_count": len(files),
        "deleted_count": deleted,
        "error_count": len(errors),
        "matched_files": [str(path) for path in files],
        "errors": errors,
    }

    if args.json:
        import json

        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(
            format_key_value_block(
                "Robot Savo map cleanup",
                {
                    "dry_run": result["dry_run"],
                    "selected_dirs": ", ".join(result["selected_dirs"]),
                    "map_name": result["map_name"] or "not set",
                    "all": result["all"],
                    "matched_count": result["matched_count"],
                    "deleted_count": result["deleted_count"],
                    "error_count": result["error_count"],
                },
            )
        )

        if files:
            print()
            print("Matched files:")
            for path in files:
                print(f"- {path}")

        if errors:
            print()
            print("Errors:")
            for error in errors:
                print(f"- {error}")

        if not args.confirm:
            print()
            print("Dry-run only. Add --confirm to delete matched files.")

    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))