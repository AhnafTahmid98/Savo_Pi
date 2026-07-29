#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
import shutil
import tarfile


SAVO_ROOT = Path.home() / "Savo_Pi"

PACKAGE = (
    SAVO_ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

INCLUDE_DIR = (
    PACKAGE
    / "include"
    / "savo_locations"
)

VIEW_HEADER = (
    INCLUDE_DIR
    / "read_only_catalog_view.hpp"
)

NODE_SOURCE = (
    PACKAGE
    / "src"
    / "location_registry_node.cpp"
)

INSTALLER = (
    SAVO_ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = SAVO_ROOT / "backups"
LOGS = SAVO_ROOT / "change_logs"


TOPIC_SYMBOLS = (
    "kStatus",
    "kSnapshot",
    "kHeartbeat",
)

SERVICE_SYMBOLS = (
    "kResolve",
    "kGet",
    "kList",
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


def find_resolve_match_header() -> Path:
    declaration = re.compile(
        r"\b(?:"
        r"enum\s+(?:class\s+|struct\s+)?"
        r"|class\s+"
        r"|struct\s+"
        r"|using\s+"
        r")ResolveMatchType\b"
    )

    matches: list[Path] = []

    for path in sorted(INCLUDE_DIR.glob("*.hpp")):
        text = path.read_text(
            encoding="utf-8"
        )

        if declaration.search(text):
            matches.append(path)

    if len(matches) != 1:
        visible = ", ".join(
            str(path.name)
            for path in matches
        )

        raise RuntimeError(
            "Expected exactly one header defining "
            f"ResolveMatchType; found {len(matches)}: "
            f"{visible or 'none'}"
        )

    return matches[0]


def ensure_include(
    path: Path,
    include_name: str,
    *,
    first_occurrence_only: bool,
) -> str:
    text = path.read_text(encoding="utf-8")

    include_line = (
        f'#include "savo_locations/{include_name}"'
    )

    if not first_occurrence_only:
        if include_line in text:
            return "already_correct"

        anchor = (
            '#include "savo_locations/types.hpp"'
        )

        if anchor not in text:
            anchor = (
                '#include '
                '"savo_locations/sqlite_repository.hpp"'
            )

        if anchor not in text:
            raise RuntimeError(
                f"Could not locate include insertion "
                f"anchor in {path}"
            )

        text = text.replace(
            anchor,
            anchor + "\n" + include_line,
            1,
        )

        path.write_text(
            text,
            encoding="utf-8",
        )

        return "corrected"

    # Installer contains generated source bodies. Insert
    # the defining header beside the first generated
    # read-only-view types include.
    anchor = (
        '#include "savo_locations/types.hpp"'
    )

    position = text.find(anchor)

    if position < 0:
        raise RuntimeError(
            "Could not locate the generated "
            "read-only-view types include in installer"
        )

    nearby = text[
        position:
        position + 400
    ]

    if include_line in nearby:
        return "already_correct"

    text = (
        text[:position]
        + text[position:].replace(
            anchor,
            anchor + "\n" + include_line,
            1,
        )
    )

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def normalize_contract_names(
    path: Path,
) -> str:
    text = path.read_text(encoding="utf-8")
    original = text

    for symbol in TOPIC_SYMBOLS:
        pattern = re.compile(
            r"(?<![A-Za-z0-9_])"
            r"(?:(?:::)?savo_locations::|:+)*"
            r"(?:topic_names|topics)::"
            + re.escape(symbol)
        )

        text, count = pattern.subn(
            (
                "::savo_locations::"
                f"topic_names::{symbol}"
            ),
            text,
        )

        if count == 0:
            raise RuntimeError(
                f"Could not locate topic symbol "
                f"{symbol} in {path}"
            )

    for symbol in SERVICE_SYMBOLS:
        pattern = re.compile(
            r"(?<![A-Za-z0-9_])"
            r"(?:(?:::)?savo_locations::|:+)*"
            r"(?:service_names|services)::"
            + re.escape(symbol)
        )

        text, count = pattern.subn(
            (
                "::savo_locations::"
                f"service_names::{symbol}"
            ),
            text,
        )

        if count == 0:
            raise RuntimeError(
                f"Could not locate service symbol "
                f"{symbol} in {path}"
            )

    if text == original:
        return "already_correct"

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def verify(
    resolve_header: Path,
) -> None:
    view_text = VIEW_HEADER.read_text(
        encoding="utf-8"
    )

    required_include = (
        '#include "savo_locations/'
        f'{resolve_header.name}"'
    )

    if required_include not in view_text:
        raise RuntimeError(
            "ResolveMatchType defining header "
            "was not included"
        )

    for path in (NODE_SOURCE, INSTALLER):
        text = path.read_text(encoding="utf-8")

        malformed = (
            "::savo_locations::::savo_locations::"
        )

        if malformed in text:
            raise RuntimeError(
                f"Duplicated namespace remains in {path}"
            )

        for symbol in TOPIC_SYMBOLS:
            required = (
                "::savo_locations::"
                f"topic_names::{symbol}"
            )

            if required not in text:
                raise RuntimeError(
                    f"Missing {required} in {path}"
                )

        for symbol in SERVICE_SYMBOLS:
            required = (
                "::savo_locations::"
                f"service_names::{symbol}"
            )

            if required not in text:
                raise RuntimeError(
                    f"Missing {required} in {path}"
                )


def main() -> None:
    for path in (
        PACKAGE,
        VIEW_HEADER,
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
        / f"partial_LOC3A_before_symbol_fix_"
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
        / f"pre_LOC3A_symbol_fix_"
          f"{stamp}_installer.py"
    )

    shutil.copy2(
        INSTALLER,
        installer_backup,
    )

    resolve_header = (
        find_resolve_match_header()
    )

    active_include = ensure_include(
        VIEW_HEADER,
        resolve_header.name,
        first_occurrence_only=False,
    )

    installer_include = ensure_include(
        INSTALLER,
        resolve_header.name,
        first_occurrence_only=True,
    )

    active_names = normalize_contract_names(
        NODE_SOURCE
    )

    installer_names = normalize_contract_names(
        INSTALLER
    )

    verify(resolve_header)

    manifest = (
        LOGS
        / f"LOC3A_symbols_qualification_fix_"
          f"{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(VIEW_HEADER)}  "
            "savo_ws/src/core/savo_locations/"
            "include/savo_locations/"
            "read_only_catalog_view.hpp\n"
            f"{sha256(NODE_SOURCE)}  "
            "savo_ws/src/core/savo_locations/"
            "src/location_registry_node.cpp\n"
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(
        "ResolveMatchType header : "
        f"{resolve_header.name}"
    )

    print(
        "Active header include   : "
        f"{active_include}"
    )

    print(
        "Installer header include: "
        f"{installer_include}"
    )

    print(
        "Active contract names   : "
        f"{active_names}"
    )

    print(
        "Installer contract names: "
        f"{installer_names}"
    )

    print(
        f"Partial package backup   : "
        f"{package_backup}"
    )

    print(
        f"Installer backup         : "
        f"{installer_backup}"
    )

    print(
        f"Permanent manifest       : "
        f"{manifest}"
    )

    print(
        "LOC-3A symbol and namespace "
        "qualification correction complete."
    )


if __name__ == "__main__":
    main()
