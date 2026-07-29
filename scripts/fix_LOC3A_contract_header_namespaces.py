#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
import shutil


SAVO_ROOT = Path.home() / "Savo_Pi"

PACKAGE = (
    SAVO_ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

VIEW_HEADER = (
    PACKAGE
    / "include"
    / "savo_locations"
    / "read_only_catalog_view.hpp"
)

NODE_SOURCE = (
    PACKAGE
    / "src"
    / "location_registry_node.cpp"
)

SERVICE_HEADER = (
    PACKAGE
    / "include"
    / "savo_locations"
    / "service_names.hpp"
)

TOPIC_HEADER = (
    PACKAGE
    / "include"
    / "savo_locations"
    / "topic_names.hpp"
)

INSTALLER = (
    SAVO_ROOT
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = SAVO_ROOT / "backups"
LOGS = SAVO_ROOT / "change_logs"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def detect_namespace(
    path: Path,
    required_symbol: str,
) -> str:
    text = path.read_text(encoding="utf-8")

    if required_symbol not in text:
        raise RuntimeError(
            f"{required_symbol} is missing from {path}"
        )

    direct = re.search(
        r"namespace\s+savo_locations::"
        r"([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)"
        r"\s*\{",
        text,
    )

    if direct is not None:
        return (
            "::savo_locations::"
            + direct.group(1)
        )

    nested = re.search(
        r"namespace\s+savo_locations\s*\{\s*"
        r"namespace\s+"
        r"([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)"
        r"\s*\{",
        text,
        flags=re.DOTALL,
    )

    if nested is not None:
        return (
            "::savo_locations::"
            + nested.group(1)
        )

    root_namespace = re.search(
        r"namespace\s+savo_locations\s*\{",
        text,
    )

    if root_namespace is not None:
        return "::savo_locations"

    raise RuntimeError(
        f"Could not determine namespace in {path}"
    )


def add_types_include(
    path: Path,
    *,
    installer: bool,
) -> str:
    text = path.read_text(encoding="utf-8")

    include_line = (
        '#include "savo_locations/types.hpp"'
    )

    if include_line in text:
        return "already_correct"

    anchor = (
        '#include '
        '"savo_locations/sqlite_repository.hpp"'
    )

    if anchor not in text:
        raise RuntimeError(
            "Could not locate sqlite_repository.hpp "
            f"include in {path}"
        )

    # The active header has one occurrence. In the installer,
    # the first occurrence belongs to the generated
    # read_only_catalog_view.hpp body.
    updated = text.replace(
        anchor,
        anchor + "\n" + include_line,
        1,
    )

    path.write_text(
        updated,
        encoding="utf-8",
    )

    return "corrected"


def qualify_node_names(
    path: Path,
    topic_namespace: str,
    service_namespace: str,
) -> str:
    text = path.read_text(encoding="utf-8")
    original = text

    topic_symbols = (
        "kStatus",
        "kSnapshot",
        "kHeartbeat",
    )

    service_symbols = (
        "kResolve",
        "kGet",
        "kList",
    )

    for symbol in topic_symbols:
        replacements = (
            f"topics::{symbol}",
            f"topic_names::{symbol}",
        )

        qualified = (
            f"{topic_namespace}::{symbol}"
        )

        for candidate in replacements:
            text = text.replace(
                candidate,
                qualified,
            )

    for symbol in service_symbols:
        replacements = (
            f"services::{symbol}",
            f"service_names::{symbol}",
        )

        qualified = (
            f"{service_namespace}::{symbol}"
        )

        for candidate in replacements:
            text = text.replace(
                candidate,
                qualified,
            )

    required = tuple(
        f"{topic_namespace}::{symbol}"
        for symbol in topic_symbols
    ) + tuple(
        f"{service_namespace}::{symbol}"
        for symbol in service_symbols
    )

    for fragment in required:
        if fragment not in text:
            raise RuntimeError(
                f"Qualified contract name missing: "
                f"{fragment} in {path}"
            )

    if text == original:
        return "already_correct"

    path.write_text(
        text,
        encoding="utf-8",
    )

    return "corrected"


def main() -> None:
    required_files = (
        VIEW_HEADER,
        NODE_SOURCE,
        SERVICE_HEADER,
        TOPIC_HEADER,
        INSTALLER,
    )

    for path in required_files:
        if not path.is_file():
            raise SystemExit(
                f"Required file missing: {path}"
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

    backup_map = {
        VIEW_HEADER: (
            BACKUPS
            / f"pre_LOC3A_contract_fix_{stamp}_"
              "read_only_catalog_view.hpp"
        ),
        NODE_SOURCE: (
            BACKUPS
            / f"pre_LOC3A_contract_fix_{stamp}_"
              "location_registry_node.cpp"
        ),
        INSTALLER: (
            BACKUPS
            / f"pre_LOC3A_contract_fix_{stamp}_"
              "installer.py"
        ),
    }

    for source, backup in backup_map.items():
        shutil.copy2(source, backup)

    topic_namespace = detect_namespace(
        TOPIC_HEADER,
        "kStatus",
    )

    service_namespace = detect_namespace(
        SERVICE_HEADER,
        "kResolve",
    )

    active_include_result = add_types_include(
        VIEW_HEADER,
        installer=False,
    )

    installer_include_result = add_types_include(
        INSTALLER,
        installer=True,
    )

    active_namespace_result = qualify_node_names(
        NODE_SOURCE,
        topic_namespace,
        service_namespace,
    )

    installer_namespace_result = (
        qualify_node_names(
            INSTALLER,
            topic_namespace,
            service_namespace,
        )
    )

    view_text = VIEW_HEADER.read_text(
        encoding="utf-8"
    )

    if (
        '#include "savo_locations/types.hpp"'
        not in view_text
    ):
        raise RuntimeError(
            "types.hpp include verification failed"
        )

    node_text = NODE_SOURCE.read_text(
        encoding="utf-8"
    )

    for stale in (
        "topics::kStatus",
        "topics::kSnapshot",
        "topics::kHeartbeat",
        "services::kResolve",
        "services::kGet",
        "services::kList",
    ):
        if stale in node_text:
            raise RuntimeError(
                f"Unqualified name remains: {stale}"
            )

    manifest = (
        LOGS
        / f"LOC3A_contract_header_namespace_fix_"
          f"{stamp}.sha256"
    )

    changed_files = (
        VIEW_HEADER,
        NODE_SOURCE,
        INSTALLER,
    )

    manifest.write_text(
        "\n".join(
            f"{sha256(path)}  "
            f"{path.relative_to(SAVO_ROOT)}"
            for path in changed_files
        )
        + "\n",
        encoding="utf-8",
    )

    print(
        f"Detected topic namespace  : "
        f"{topic_namespace}"
    )

    print(
        f"Detected service namespace: "
        f"{service_namespace}"
    )

    print(
        f"Active types include      : "
        f"{active_include_result}"
    )

    print(
        f"Installer types include   : "
        f"{installer_include_result}"
    )

    print(
        f"Active namespace names    : "
        f"{active_namespace_result}"
    )

    print(
        f"Installer namespace names : "
        f"{installer_namespace_result}"
    )

    for backup in backup_map.values():
        print(f"Permanent backup          : {backup}")

    print(f"Permanent manifest        : {manifest}")

    print(
        "LOC-3A contract header and namespace "
        "correction complete."
    )


if __name__ == "__main__":
    main()
