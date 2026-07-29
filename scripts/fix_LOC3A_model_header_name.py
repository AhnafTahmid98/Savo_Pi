#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import shutil


INSTALLER = (
    Path.home()
    / "Savo_Pi"
    / "scripts"
    / "apply_LOC3A_savo_locations.py"
)

BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def main() -> None:
    if not INSTALLER.is_file():
        raise SystemExit(
            f"LOC-3A installer missing: {INSTALLER}"
        )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC3A_model_header_fix_{stamp}.py"
    )

    shutil.copy2(INSTALLER, backup)

    text = INSTALLER.read_text(
        encoding="utf-8"
    )

    old = "location_models.hpp"
    new = "model.hpp"

    occurrence_count = text.count(old)

    if occurrence_count == 0:
        if new not in text:
            raise RuntimeError(
                "Neither location_models.hpp nor model.hpp "
                "was found in the LOC-3A installer."
            )

        print(
            "LOC-3A installer already uses model.hpp."
        )
    else:
        text = text.replace(old, new)

        if old in text:
            raise RuntimeError(
                "Not all incorrect model header references "
                "were replaced."
            )

        INSTALLER.write_text(
            text,
            encoding="utf-8",
        )

        print(
            f"Corrected {occurrence_count} model-header "
            "reference(s)."
        )

    manifest = (
        LOGS
        / f"LOC3A_model_header_fix_{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(INSTALLER)}  "
            "scripts/apply_LOC3A_savo_locations.py\n"
        ),
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")
    print(
        "LOC-3A installer now uses the existing "
        "savo_locations/model.hpp header."
    )


if __name__ == "__main__":
    main()
