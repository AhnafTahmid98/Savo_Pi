#!/usr/bin/env python3
from __future__ import annotations

import ast
import base64
from datetime import datetime
import hashlib
import io
from pathlib import Path
import re
import shutil
import tarfile
import textwrap


ROOT = Path.home() / "Savo_Pi"
PACKAGE = ROOT / "savo_ws" / "src" / "core" / "savo_locations"
SOURCE = PACKAGE / "src" / "location_registry_node.cpp"
INSTALLER = ROOT / "scripts" / "apply_LOC3B2_savo_locations.py"
BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"

OLD_LINE = b"  message.stamp = now().to_msg();"
NEW_LINE = b"  message.stamp = now();"
OLD_SOURCE_HASH = (
    "0211bd63fe4edd32e12860b1c0e24e3471321225ad4a773a919c1814d85fb4ed"
)
NEW_SOURCE_HASH = (
    "7751f0d64ecc01701ed5626c5413c2ec09fd955edd969511b2fe64244edb010a"
)
PAYLOAD_MEMBER = "src/location_registry_node.cpp"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, content: bytes, mode: int) -> None:
    staging = path.with_name(path.name + ".loc3b2_timefix.new")
    staging.write_bytes(content)
    staging.chmod(mode)
    staging.replace(path)


def find_assignment_value(tree: ast.Module, name: str):
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
            return ast.literal_eval(node.value)
    raise RuntimeError(f"Installer assignment missing: {name}")


def rebuild_payload(installer_text: str) -> tuple[str, str]:
    tree = ast.parse(installer_text)
    payload = find_assignment_value(tree, "PAYLOAD")
    final_hashes = find_assignment_value(tree, "FINAL_HASHES")

    if final_hashes.get(PAYLOAD_MEMBER) != OLD_SOURCE_HASH:
        raise RuntimeError(
            "Retained LOC-3B2 installer has an unexpected source hash for "
            f"{PAYLOAD_MEMBER}"
        )

    archive_bytes = base64.b64decode("".join(payload.split()))
    files: dict[str, tuple[bytes, int]] = {}

    with tarfile.open(fileobj=io.BytesIO(archive_bytes), mode="r:gz") as archive:
        for member in archive.getmembers():
            if member.isdir():
                continue
            stream = archive.extractfile(member)
            if stream is None:
                raise RuntimeError(f"Unreadable installer payload member: {member.name}")
            files[member.name] = (stream.read(), member.mode or 0o644)

    if PAYLOAD_MEMBER not in files:
        raise RuntimeError(f"Installer payload member missing: {PAYLOAD_MEMBER}")

    source_content, source_mode = files[PAYLOAD_MEMBER]
    if sha256_bytes(source_content) != OLD_SOURCE_HASH:
        raise RuntimeError("Installer payload source hash does not match its manifest")
    if source_content.count(OLD_LINE) != 1:
        raise RuntimeError("Installer payload does not contain exactly one Jazzy time defect")

    corrected_source = source_content.replace(OLD_LINE, NEW_LINE, 1)
    if sha256_bytes(corrected_source) != NEW_SOURCE_HASH:
        raise RuntimeError("Corrected installer payload source hash is unexpected")
    files[PAYLOAD_MEMBER] = (corrected_source, source_mode)

    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w:gz", format=tarfile.PAX_FORMAT) as archive:
        for name in sorted(files):
            content, mode = files[name]
            info = tarfile.TarInfo(name=name)
            info.size = len(content)
            info.mode = mode
            info.mtime = 0
            info.uid = 0
            info.gid = 0
            info.uname = ""
            info.gname = ""
            archive.addfile(info, io.BytesIO(content))

    encoded = base64.b64encode(output.getvalue()).decode("ascii")
    wrapped = "\n".join(textwrap.wrap(encoded, width=100))

    payload_pattern = re.compile(
        r'PAYLOAD\s*=\s*r"""\n.*?\n"""',
        flags=re.DOTALL,
    )
    if len(payload_pattern.findall(installer_text)) != 1:
        raise RuntimeError("Could not identify exactly one installer PAYLOAD block")

    corrected = payload_pattern.sub(
        'PAYLOAD = r"""\n' + wrapped + '\n"""',
        installer_text,
        count=1,
    )

    if corrected.count(OLD_SOURCE_HASH) != 1:
        raise RuntimeError("Expected exactly one old source hash in installer")
    corrected = corrected.replace(OLD_SOURCE_HASH, NEW_SOURCE_HASH, 1)

    ast.parse(corrected)
    return corrected, sha256_bytes(corrected.encode("utf-8"))


def main() -> None:
    if not PACKAGE.is_dir():
        raise RuntimeError(f"Package missing: {PACKAGE}")
    if not SOURCE.is_file():
        raise RuntimeError(f"Source missing: {SOURCE}")
    if not INSTALLER.is_file():
        raise RuntimeError(f"Retained installer missing: {INSTALLER}")

    source_content = SOURCE.read_bytes()
    source_hash = sha256_bytes(source_content)

    if source_hash == NEW_SOURCE_HASH:
        if source_content.count(NEW_LINE) != 1:
            raise RuntimeError("Source hash is corrected but expected line is missing")
        print("LOC-3B2 Jazzy time conversion is already fixed in the active source.")
        return

    if source_hash != OLD_SOURCE_HASH:
        raise RuntimeError(
            "Unexpected active LOC-3B2 source hash\n"
            f"expected: {OLD_SOURCE_HASH}\n"
            f"actual  : {source_hash}"
        )
    if source_content.count(OLD_LINE) != 1:
        raise RuntimeError("Active source does not contain exactly one Jazzy time defect")

    installer_text = INSTALLER.read_text(encoding="utf-8")
    corrected_installer, corrected_installer_hash = rebuild_payload(installer_text)
    corrected_source = source_content.replace(OLD_LINE, NEW_LINE, 1)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    package_backup = BACKUPS / f"pre_LOC3B2_jazzy_time_fix_{stamp}.tar.gz"
    installer_backup = BACKUPS / f"pre_LOC3B2_jazzy_time_fix_{stamp}_installer.py"
    manifest = LOGS / f"LOC3B2_jazzy_time_fix_{stamp}.sha256"

    with tarfile.open(package_backup, "w:gz") as archive:
        archive.add(PACKAGE, arcname="core/savo_locations")
    shutil.copy2(INSTALLER, installer_backup)

    try:
        atomic_write(SOURCE, corrected_source, 0o644)
        atomic_write(INSTALLER, corrected_installer.encode("utf-8"), 0o755)

        if sha256_file(SOURCE) != NEW_SOURCE_HASH:
            raise RuntimeError("Active source verification failed")
        if sha256_file(INSTALLER) != corrected_installer_hash:
            raise RuntimeError("Retained installer verification failed")
        ast.parse(INSTALLER.read_text(encoding="utf-8"))

    except Exception:
        with tarfile.open(package_backup, "r:gz") as archive:
            archive.extractall(PACKAGE.parent.parent)
        shutil.copy2(installer_backup, INSTALLER)
        raise

    with manifest.open("w", encoding="utf-8") as stream:
        stream.write(f"{sha256_file(SOURCE)}  {SOURCE}\n")
        stream.write(f"{sha256_file(INSTALLER)}  {INSTALLER}\n")
        stream.write(f"{sha256_file(package_backup)}  {package_backup}\n")
        stream.write(f"{sha256_file(installer_backup)}  {installer_backup}\n")

    print("LOC-3B2 Jazzy time conversion fixed.")
    print(f"Backup          : {package_backup}")
    print(f"Installer backup: {installer_backup}")
    print(f"Manifest        : {manifest}")
    print("Active source   : message.stamp = now();")
    print("LOC-3B2 installer: corrected and syntax-checked")


if __name__ == "__main__":
    main()
