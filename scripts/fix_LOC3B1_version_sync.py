#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
import hashlib
from pathlib import Path
import shutil
import tarfile

ROOT = Path.home() / "Savo_Pi"
PACKAGE = ROOT / "savo_ws" / "src" / "core" / "savo_locations"
CONSTANTS = PACKAGE / "include" / "savo_locations" / "constants.hpp"
PACKAGE_XML = PACKAGE / "package.xml"
INSTALLER = ROOT / "scripts" / "apply_LOC3B1_savo_locations.py"
BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"

OLD_CONSTANTS_HASH = "ed903a2117fae60a404c9e943aa16034fd130722f9446ca9cd93976a1f03ecc2"
NEW_CONSTANTS_HASH = "c2b7932306927699fd5cca20b7832da374696a158cb094b3d1a94abaabc69d3a"
OLD_INSTALLER_HASH = "23e23b2e8bf35f4701b8a81edfedf2e1947388594854fec1370a000cfbd5a595"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one anchor, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    for path in (CONSTANTS, PACKAGE_XML, INSTALLER):
        if not path.is_file():
            raise SystemExit(f"Required file missing: {path}")

    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    if "<version>0.9.0</version>" not in package_xml:
        raise SystemExit("package.xml is not at LOC-3B1 version 0.9.0")

    constants_hash = sha256(CONSTANTS)
    installer_hash = sha256(INSTALLER)

    if constants_hash == NEW_CONSTANTS_HASH:
        print("LOC-3B1 version synchronization is already applied.")
        print(f"constants.hpp SHA-256: {constants_hash}")
        return

    if constants_hash != OLD_CONSTANTS_HASH:
        raise SystemExit(
            "Unexpected constants.hpp input\n"
            f"expected: {OLD_CONSTANTS_HASH}\n"
            f"actual  : {constants_hash}"
        )

    if installer_hash != OLD_INSTALLER_HASH:
        raise SystemExit(
            "Unexpected LOC-3B1 installer input\n"
            f"expected: {OLD_INSTALLER_HASH}\n"
            f"actual  : {installer_hash}"
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    backup = BACKUPS / f"pre_LOC3B1_version_sync_{stamp}.tar.gz"
    installer_backup = BACKUPS / f"pre_LOC3B1_version_sync_{stamp}_installer.py"
    manifest = LOGS / f"LOC3B1_version_sync_{stamp}.sha256"

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            CONSTANTS,
            arcname="core/savo_locations/include/savo_locations/constants.hpp",
        )
        archive.add(
            PACKAGE_XML,
            arcname="core/savo_locations/package.xml",
        )

    shutil.copy2(INSTALLER, installer_backup)

    constants_text = CONSTANTS.read_text(encoding="utf-8")
    constants_text = replace_once(
        constants_text,
        '"0.8.0"',
        '"0.9.0"',
        "active package version constant",
    )
    CONSTANTS.write_text(constants_text, encoding="utf-8")

    installer_text = INSTALLER.read_text(encoding="utf-8")

    installer_text = replace_once(
        installer_text,
        "EXPECTED_HASHES = {'CMakeLists.txt': 'ac691210c2e2e7e440d51d15750e5507ac77a355b571a7eb1c80754415c93ba6',\n",
        "EXPECTED_HASHES = {'CMakeLists.txt': 'ac691210c2e2e7e440d51d15750e5507ac77a355b571a7eb1c80754415c93ba6',\n"
        " 'include/savo_locations/constants.hpp': 'ed903a2117fae60a404c9e943aa16034fd130722f9446ca9cd93976a1f03ecc2',\n",
        "installer expected constants hash",
    )

    installer_text = replace_once(
        installer_text,
        "FINAL_HASHES = {'CMakeLists.txt': 'c2b6c4ba930b07a35a48b846c0d7a212bed4b5a70f3beb72a9687f4daa2fbfda',\n",
        "FINAL_HASHES = {'CMakeLists.txt': 'c2b6c4ba930b07a35a48b846c0d7a212bed4b5a70f3beb72a9687f4daa2fbfda',\n"
        " 'include/savo_locations/constants.hpp': 'c2b7932306927699fd5cca20b7832da374696a158cb094b3d1a94abaabc69d3a',\n",
        "installer final constants hash",
    )

    patch_function = '''\n\ndef patch_constants() -> None:\n    path = (\n        PACKAGE\n        / "include"\n        / "savo_locations"\n        / "constants.hpp"\n    )\n\n    text = path.read_text(encoding="utf-8")\n\n    text = replace_once(\n        text,\n        '\"0.8.0\"',\n        '\"0.9.0\"',\n        "package version constant",\n    )\n\n    path.write_text(text, encoding="utf-8")\n\n\n'''

    installer_text = replace_once(
        installer_text,
        "\ndef patch_cmake() -> None:\n",
        patch_function + "def patch_cmake() -> None:\n",
        "installer constants patch function",
    )

    installer_text = replace_once(
        installer_text,
        "    patch_repository()\n    patch_cmake()\n",
        "    patch_repository()\n    patch_constants()\n    patch_cmake()\n",
        "installer constants patch call",
    )

    INSTALLER.write_text(installer_text, encoding="utf-8")
    INSTALLER.chmod(0o755)

    new_constants_hash = sha256(CONSTANTS)
    if new_constants_hash != NEW_CONSTANTS_HASH:
        raise RuntimeError(
            "constants.hpp output hash mismatch\n"
            f"expected: {NEW_CONSTANTS_HASH}\n"
            f"actual  : {new_constants_hash}"
        )

    compile(INSTALLER.read_text(encoding="utf-8"), str(INSTALLER), "exec")

    manifest.write_text(
        f"{sha256(CONSTANTS)}  {CONSTANTS}\n"
        f"{sha256(PACKAGE_XML)}  {PACKAGE_XML}\n"
        f"{sha256(INSTALLER)}  {INSTALLER}\n",
        encoding="utf-8",
    )

    print("LOC-3B1 package version synchronization fixed.")
    print(f"Backup          : {backup}")
    print(f"Installer backup: {installer_backup}")
    print(f"Manifest        : {manifest}")
    print("constants.hpp   : 0.9.0")
    print("LOC-3B1 installer: corrected and syntax-checked")


if __name__ == "__main__":
    main()
