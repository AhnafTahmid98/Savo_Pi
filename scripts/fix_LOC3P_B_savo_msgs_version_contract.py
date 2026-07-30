#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import py_compile
import shutil
import tarfile
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path.home() / "Savo_Pi"
SRC = ROOT / "savo_ws" / "src"
SCRIPTS = ROOT / "scripts"
BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"
INSTALLER = SCRIPTS / "apply_LOC3P_B_hardening.py"

OLD_ASSERTION = '    assert root.findtext("version") == "0.3.0"'
NEW_ASSERTION = '    assert root.findtext("version") == "0.4.0"'


def find_package(name: str) -> Path:
    matches: list[Path] = []
    for package_xml in SRC.rglob("package.xml"):
        try:
            root = ET.parse(package_xml).getroot()
        except (ET.ParseError, OSError):
            continue
        if root.findtext("name") == name:
            matches.append(package_xml.parent)
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {name} package, found {len(matches)}: {matches}"
        )
    return matches[0]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def main() -> int:
    if not SRC.is_dir():
        raise RuntimeError(f"workspace source directory missing: {SRC}")
    if not INSTALLER.is_file():
        raise RuntimeError(f"retained LOC-3P-B installer missing: {INSTALLER}")

    SCRIPTS.mkdir(parents=True, exist_ok=True)
    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    msgs = find_package("savo_msgs")
    package_root = ET.parse(msgs / "package.xml").getroot()
    active_version = package_root.findtext("version")
    if active_version != "0.4.0":
        raise RuntimeError(
            f"expected active savo_msgs version 0.4.0, found {active_version}"
        )

    test_path = msgs / "test" / "test_apriltag_interfaces.py"
    test_text = test_path.read_text(encoding="utf-8")
    if test_text.count(OLD_ASSERTION) != 1:
        raise RuntimeError(
            f"{test_path}: expected exactly one stale 0.3.0 assertion"
        )
    if NEW_ASSERTION in test_text:
        raise RuntimeError(f"{test_path}: corrected assertion already present")

    installer_text = INSTALLER.read_text(encoding="utf-8")

    version_anchor = (
        '    update_xml_version(msgs / "package.xml", "0.3.0", "0.4.0")\n'
        "\n"
        "    recover_srv = textwrap.dedent(\n"
    )
    version_replacement = (
        '    update_xml_version(msgs / "package.xml", "0.3.0", "0.4.0")\n'
        "\n"
        '    apriltag_test_path = msgs / "test" / "test_apriltag_interfaces.py"\n'
        "    apriltag_test = apriltag_test_path.read_text(encoding=\"utf-8\")\n"
        "    apriltag_test = replace_once(\n"
        "        apriltag_test,\n"
        "        '    assert root.findtext(\"version\") == \"0.3.0\"',\n"
        "        '    assert root.findtext(\"version\") == \"0.4.0\"',\n"
        "        \"historical AprilTag interface version contract\",\n"
        "    )\n"
        "    write(apriltag_test_path, apriltag_test)\n"
        "\n"
        "    recover_srv = textwrap.dedent(\n"
    )
    installer_text = replace_once(
        installer_text,
        version_anchor,
        version_replacement,
        "installer active-test migration insertion",
    )

    compile_anchor = (
        "    py_compile.compile(str(test_interfaces_path), doraise=True)\n"
        "    py_compile.compile(str(phase_contract_path), doraise=True)\n"
    )
    compile_replacement = (
        "    py_compile.compile(str(apriltag_test_path), doraise=True)\n"
        "    py_compile.compile(str(test_interfaces_path), doraise=True)\n"
        "    py_compile.compile(str(phase_contract_path), doraise=True)\n"
    )
    installer_text = replace_once(
        installer_text,
        compile_anchor,
        compile_replacement,
        "installer syntax-verification insertion",
    )

    manifest_anchor = (
        '        msgs / "srv" / "RecoverLocationStorage.srv",\n'
        "        test_interfaces_path,\n"
    )
    manifest_replacement = (
        '        msgs / "srv" / "RecoverLocationStorage.srv",\n'
        "        apriltag_test_path,\n"
        "        test_interfaces_path,\n"
    )
    installer_text = replace_once(
        installer_text,
        manifest_anchor,
        manifest_replacement,
        "installer manifest insertion",
    )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = BACKUPS / f"pre_LOC3P_B_msgs_version_contract_{stamp}.tar.gz"
    installer_backup = (
        BACKUPS / f"pre_LOC3P_B_msgs_version_contract_{stamp}_installer.py"
    )
    manifest = LOGS / f"LOC3P_B_msgs_version_contract_{stamp}.sha256"

    with tarfile.open(backup, "w:gz") as tar:
        tar.add(msgs, arcname=str(msgs.relative_to(SRC)))
    shutil.copy2(INSTALLER, installer_backup)

    test_path.write_text(
        replace_once(
            test_text,
            OLD_ASSERTION,
            NEW_ASSERTION,
            "active AprilTag interface version contract",
        ),
        encoding="utf-8",
    )
    INSTALLER.write_text(installer_text, encoding="utf-8")

    py_compile.compile(str(test_path), doraise=True)
    py_compile.compile(str(INSTALLER), doraise=True)

    corrected_test = test_path.read_text(encoding="utf-8")
    if OLD_ASSERTION in corrected_test or corrected_test.count(NEW_ASSERTION) != 1:
        raise RuntimeError("active test verification failed")

    corrected_installer = INSTALLER.read_text(encoding="utf-8")
    required_installer_tokens = [
        "apriltag_test_path = msgs / \"test\" / \"test_apriltag_interfaces.py\"",
        "historical AprilTag interface version contract",
        "py_compile.compile(str(apriltag_test_path), doraise=True)",
        "        apriltag_test_path,",
    ]
    for token in required_installer_tokens:
        if token not in corrected_installer:
            raise RuntimeError(f"corrected installer missing token: {token}")

    manifest.write_text(
        "\n".join(
            [
                f"{sha256(backup)}  {backup}",
                f"{sha256(installer_backup)}  {installer_backup}",
                f"{sha256(test_path)}  {test_path}",
                f"{sha256(INSTALLER)}  {INSTALLER}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print("LOC-3P-B savo_msgs version contract fixed.")
    print(f"Backup          : {backup}")
    print(f"Installer backup: {installer_backup}")
    print(f"Manifest        : {manifest}")
    print("Active assertion: assert root.findtext(\"version\") == \"0.4.0\"")
    print("LOC-3P-B installer: corrected and syntax-checked")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
