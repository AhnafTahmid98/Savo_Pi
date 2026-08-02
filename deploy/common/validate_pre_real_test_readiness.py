#!/usr/bin/env python3
"""Read-only Robot SAVO pre-real-test repository validator."""

from __future__ import annotations

import ast
import json
import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
REQUIRED = (
    "deploy/network/core_netplan.yaml",
    "deploy/network/edge_netplan.yaml",
    "deploy/network/chrony_core.conf",
    "deploy/network/chrony_edge.conf",
    "deploy/systemd/savo_core.service",
    "deploy/systemd/savo_edge.service",
    "deploy/systemd/savo.service",
    "deploy/systemd/savo_mapping.service",
    "deploy/common/backup_robot_state.sh",
    "deploy/common/restore_robot_state.sh",
    "deploy/common/preflight_storage_check.sh",
    "deploy/logrotate/robot-savo",
    "docs/setup/dependency_matrix.md",
    "docs/testing/full_robot_test_plan.md",
)

PHYSICAL_BLOCKERS = (
    "AM-0B geometry measurement and lock",
    "/var/lib/robot_savo preparation on core",
    "target Pi dependency installation",
    "core/edge safe-idle validation",
    "real sensor validation",
    "wheels-raised motor validation",
    "safety-stop validation",
    "localization and TF validation",
    "supervisor authorization validation",
    "real mapping",
    "real AprilTag registration",
    "real AM-8 release",
    "LiDAR-only saved-map navigation",
    "Mac/PC observer validation",
    "phone dashboard validation",
    "D435 filtering before voxel enablement",
)

EXTERNAL_BLOCKERS = (
    "SavoMind TTS endpoint does not return audio bytes/path in the inspected contract",
)


@dataclass
class Check:
    name: str
    status: str
    detail: str


checks: list[Check] = []


def add(name: str, status: str, detail: str) -> None:
    checks.append(Check(name, status, detail))


def command(name: str, argv: list[str], *, blocked_ok: bool = False) -> None:
    result = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, check=False)
    output = (result.stdout + result.stderr).strip()[-2000:]
    if result.returncode == 0:
        add(name, "PASS", output or "ok")
    elif blocked_ok and re.search(r"dbus|not available|not found|operation not permitted", output, re.I):
        add(name, "BLOCKED", output)
    else:
        add(name, "FAIL", output or f"exit {result.returncode}")


def validate_required() -> None:
    missing = [path for path in REQUIRED if not (ROOT / path).is_file() or (ROOT / path).stat().st_size == 0]
    add("required_files", "FAIL" if missing else "PASS", ", ".join(missing) if missing else "all required files are nonempty")


def validate_empty_files() -> None:
    runtime_suffixes = {".cpp", ".hpp", ".py", ".sh", ".yaml", ".yml", ".xml", ".service", ".conf"}
    bad: list[str] = []
    intentional = 0
    for root_name in ("deploy", "docs", "savo_ws/src", "tools"):
        for path in (ROOT / root_name).rglob("*"):
            if not path.is_file() or path.stat().st_size:
                continue
            relative = path.relative_to(ROOT).as_posix()
            if path.name in {"__init__.py", ".gitkeep"} or "resource" in path.parts:
                intentional += 1
            elif path.suffix in runtime_suffixes and not relative.startswith("docs/"):
                bad.append(relative)
    add("zero_byte_runtime", "FAIL" if bad else "PASS", ", ".join(bad) if bad else f"none; {intentional} intentional markers")


def validate_launch_references() -> None:
    forbidden = ("localization_dashboard.py", "ekf_state_publisher_node.py", "wheel_odom_fallback_node.py")
    offenders = []
    for path in (ROOT / "savo_ws/src").rglob("*"):
        if "launch" not in path.parts:
            continue
        if path.is_file() and path.suffix in {".py", ".xml", ".yaml"}:
            text = path.read_text(encoding="utf-8", errors="replace")
            if any(item in text for item in forbidden):
                offenders.append(path.relative_to(ROOT).as_posix())
    add("launch_empty_executables", "FAIL" if offenders else "PASS", ", ".join(offenders) if offenders else "no removed/empty localization executables referenced")


def validate_parsers() -> None:
    errors: dict[str, list[str]] = {"yaml": [], "xml": [], "python": []}
    for base in (ROOT / "deploy", ROOT / "savo_ws/src"):
        for path in base.rglob("*"):
            if not path.is_file() or path.stat().st_size == 0:
                continue
            relative = path.relative_to(ROOT).as_posix()
            kind = ""
            try:
                if path.suffix in {".yaml", ".yml"}:
                    kind = "yaml"
                    yaml.safe_load(path.read_text(encoding="utf-8"))
                elif path.suffix == ".xml" or path.name == "package.xml":
                    kind = "xml"
                    ET.parse(path)
                elif path.suffix == ".py":
                    kind = "python"
                    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except Exception as exc:
                errors[kind].append(f"{relative}: {exc}")
    for kind in ("yaml", "xml", "python"):
        found = errors[kind]
        add(f"{kind}_parse", "FAIL" if found else "PASS", "; ".join(found) if found else f"all nonempty {kind} parsed")


def validate_network_and_units() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        temp = Path(temporary)
        env = temp / "network.env"
        env.write_text(
            "CORE_ETH_IFACE=eth0\nEDGE_ETH_IFACE=eth0\nCORE_WIFI_IFACE=wlan0\nEDGE_WIFI_IFACE=wlan0\n"
            "CORE_ETH_IP=192.168.50.1\nEDGE_ETH_IP=192.168.50.2\nLINK_PREFIX=24\n"
            "LINK_SUBNET=192.168.50.0/24\nWIFI_SSID=validator-only\nWIFI_PASSWORD=validator-only\n",
            encoding="utf-8",
        )
        for role in ("core", "edge"):
            command(f"netplan_{role}", [str(ROOT / "deploy/network/render_network_config.sh"), "--role", role, "--env", str(env), "--output-dir", str(temp / role)], blocked_ok=True)
        command("systemd_units", [str(ROOT / "deploy/systemd/render_units.sh"), "--user", os.environ.get("USER", "robot-savo"), "--group", os.environ.get("USER", "robot-savo"), "--root", str(ROOT), "--output-dir", str(temp / "units")], blocked_ok=True)


def validate_safety_contracts() -> None:
    corpus = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in (ROOT / "deploy").rglob("*") if path.is_file() and path.stat().st_size)
    add("control_startup_stop", "PASS" if "SAVO_CONTROL_STARTUP_MODE=STOP" in corpus and "control_startup_mode:=STOP" in corpus else "FAIL", "STOP defaults present")
    bringup = (ROOT / "deploy/core/run_core.sh").read_text(encoding="utf-8")
    add("geometry_provisional", "PASS" if "allow_provisional_geometry:=\"${SAVO_ALLOW_PROVISIONAL_GEOMETRY:-false}\"" in bringup else "FAIL", "BLOCKED_FOR_MOTION: geometry_not_locked")
    add("voxel_default", "PASS" if "d435_voxel_validated:=\"${SAVO_D435_VOXEL_VALIDATED:-false}\"" in bringup else "FAIL", "D435 voxel remains false")
    mapping_path = ROOT / "savo_ws/src/core/savo_mapping/config/autonomous_mapping_orchestrator.yaml"
    mapping = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
    release = mapping.get("autonomous_mapping_orchestrator_node", {}).get(
        "ros__parameters", {}
    ).get("release", {})
    review_launch = ROOT / "savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py"
    review_text = review_launch.read_text(encoding="utf-8")
    am8_required = (
        release.get("require_locked_geometry") is True
        and release.get("allow_provisional_geometry") is False
        and '"start_review_gateway": "true"' in review_text
    )
    add(
        "am8_required",
        "PASS" if am8_required else "FAIL",
        "locked geometry plus enabled review gateway remain required",
    )


def write_report() -> int:
    failures = [check for check in checks if check.status == "FAIL"]
    code_blockers = [check for check in checks if check.status == "BLOCKED"]
    overall = "FAIL" if failures else "BLOCKED"
    report = {
        "schema_version": 1,
        "generated_utc": datetime.now(UTC).isoformat(),
        "status": overall,
        "checks": [asdict(check) for check in checks],
        "physical_blockers": list(PHYSICAL_BLOCKERS),
        "external_blockers": list(EXTERNAL_BLOCKERS),
        "blocked_for_motion": "geometry_not_locked",
    }
    log_dir = ROOT / "log"
    log_dir.mkdir(exist_ok=True)
    (log_dir / "pre_real_test_readiness.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = ["# Pre-real-test readiness", "", f"Status: **{overall}**", "", "| Check | Status | Detail |", "| --- | --- | --- |"]
    lines.extend(f"| {c.name} | {c.status} | {c.detail.replace('|', '/').replace(chr(10), ' ')} |" for c in checks)
    lines.extend(["", "## Physical blockers", ""] + [f"- {item}" for item in PHYSICAL_BLOCKERS])
    lines.extend(["", "## External blockers", ""] + [f"- {item}" for item in EXTERNAL_BLOCKERS] + [""])
    (log_dir / "pre_real_test_readiness.md").write_text("\n".join(lines), encoding="utf-8")
    print(overall)
    for check in checks:
        print(f"{check.status:7} {check.name}: {check.detail}")
    if code_blockers:
        print(f"External validation blockers: {len(code_blockers)}")
    return 1 if failures else 2


def main() -> int:
    validate_required()
    command("git_diff_whitespace", ["git", "diff", "--check"])
    validate_empty_files()
    validate_launch_references()
    validate_parsers()
    command("bash_syntax", ["bash", "-n", str(ROOT / "deploy/common/validate_pre_real_test_readiness.sh")])
    validate_network_and_units()
    command("observer_read_only", [str(ROOT / "deploy/observer/validate_observer.sh")])
    validate_safety_contracts()
    return write_report()


if __name__ == "__main__":
    raise SystemExit(main())
