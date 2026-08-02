#!/usr/bin/env python3
"""Read-only Robot SAVO pre-real-test repository validator.

The validator distinguishes repository failures from unavailable host tools and
external/physical blockers. It never starts robot hardware, applies Netplan, or
installs systemd units.
"""

from __future__ import annotations

import argparse
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
from typing import Iterable

import yaml


DEFAULT_ROOT = Path(__file__).resolve().parents[2]

REQUIRED = (
    "deploy/network/core_netplan.yaml",
    "deploy/network/edge_netplan.yaml",
    "deploy/network/chrony_core.conf",
    "deploy/network/chrony_edge.conf",
    "deploy/network/render_network_config.sh",
    "deploy/systemd/savo_core.service",
    "deploy/systemd/savo_edge.service",
    "deploy/systemd/savo.service",
    "deploy/systemd/savo_mapping.service",
    "deploy/systemd/render_units.sh",
    "deploy/systemd/robot-savo-tmpfiles.conf",
    "deploy/edge/prepare_runtime_sockets.sh",
    "deploy/edge/savomind_speech_contract.yaml",
    "deploy/common/backup_robot_state.sh",
    "deploy/common/restore_robot_state.sh",
    "deploy/common/check_disk_space.sh",
    "deploy/common/check_service_health.sh",
    "deploy/common/preflight_storage_check.sh",
    "deploy/common/run_pre_real_test_regression.sh",
    "deploy/logrotate/robot-savo",
    "docs/setup/dependency_matrix.md",
    "docs/testing/full_robot_test_plan.md",
    "docs/testing/base_test_plan.md",
    "docs/testing/perception_test_plan.md",
    "docs/testing/localization_test_plan.md",
    "docs/testing/speech_test_plan.md",
    "docs/testing/vo_test_plan.md",
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

INTENTIONAL_EMPTY_NAMES = {"__init__.py", ".gitkeep"}
RUNTIME_SUFFIXES = {
    ".cpp", ".c", ".hpp", ".h", ".py", ".sh", ".yaml", ".yml",
    ".xml", ".service", ".conf", ".launch", ".action", ".srv", ".msg",
}


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str


class Validator:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.checks: list[Check] = []

    def add(self, name: str, status: str, detail: str) -> None:
        if status not in {"PASS", "BLOCKED", "FAIL"}:
            raise ValueError(f"invalid check status: {status}")
        self.checks.append(Check(name, status, detail.strip() or "ok"))

    def command(
        self,
        name: str,
        argv: list[str],
        *,
        blocked_patterns: Iterable[str] = (),
        cwd: Path | None = None,
    ) -> None:
        try:
            result = subprocess.run(
                argv,
                cwd=cwd or self.root,
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError as exc:
            self.add(name, "BLOCKED", f"tool unavailable: {exc.filename}")
            return
        output = (result.stdout + result.stderr).strip()[-4000:]
        if result.returncode == 0:
            self.add(name, "PASS", output or "ok")
            return
        if any(re.search(pattern, output, re.I) for pattern in blocked_patterns):
            self.add(name, "BLOCKED", output or f"exit {result.returncode}")
            return
        self.add(name, "FAIL", output or f"exit {result.returncode}")

    def validate_required(self) -> None:
        missing = [
            path for path in REQUIRED
            if not (self.root / path).is_file() or (self.root / path).stat().st_size == 0
        ]
        self.add(
            "required_files",
            "FAIL" if missing else "PASS",
            ", ".join(missing) if missing else "all required files are nonempty",
        )

    @staticmethod
    def _intentional_empty(path: Path) -> bool:
        return (
            path.name in INTENTIONAL_EMPTY_NAMES
            or path.parent.name == "resource"
            or "resource" in path.parts and path.parent.name == "resource"
        )

    def validate_empty_files(self) -> None:
        bad: list[str] = []
        intentional = 0
        deferred_docs = 0
        for root_name in ("deploy", "docs", "savo_ws/src", "tools"):
            base = self.root / root_name
            if not base.exists():
                continue
            for path in base.rglob("*"):
                if not path.is_file() or path.stat().st_size != 0:
                    continue
                relative = path.relative_to(self.root).as_posix()
                if self._intentional_empty(path):
                    intentional += 1
                elif relative.startswith("docs/"):
                    deferred_docs += 1
                elif path.suffix in RUNTIME_SUFFIXES or os.access(path, os.X_OK):
                    bad.append(relative)
        detail = (
            ", ".join(bad) if bad
            else f"none; intentional_markers={intentional}; deferred_empty_docs={deferred_docs}"
        )
        self.add("zero_byte_runtime", "FAIL" if bad else "PASS", detail)

    def validate_launch_references(self) -> None:
        forbidden = {
            "localization_dashboard.py",
            "ekf_state_publisher_node.py",
            "wheel_odom_fallback_node.py",
        }
        offenders: list[str] = []
        for launch_dir in (self.root / "savo_ws/src").rglob("launch"):
            if not launch_dir.is_dir():
                continue
            for path in launch_dir.rglob("*"):
                if not path.is_file() or path.suffix not in {".py", ".xml", ".yaml", ".yml"}:
                    continue
                text = path.read_text(encoding="utf-8", errors="replace")
                if any(item in text for item in forbidden):
                    offenders.append(path.relative_to(self.root).as_posix())
        self.add(
            "launch_empty_executables",
            "FAIL" if offenders else "PASS",
            ", ".join(offenders) if offenders
            else "no removed/empty localization executables referenced by launch files",
        )

    def validate_parsers(self) -> None:
        errors: dict[str, list[str]] = {"yaml": [], "xml": [], "python": []}
        for base in (self.root / "deploy", self.root / "savo_ws/src", self.root / "tools"):
            if not base.exists():
                continue
            for path in base.rglob("*"):
                if not path.is_file() or path.stat().st_size == 0:
                    continue
                relative = path.relative_to(self.root).as_posix()
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
                except Exception as exc:  # validator must report all parser failures
                    errors[kind].append(f"{relative}: {exc}")
        for kind in ("yaml", "xml", "python"):
            found = errors[kind]
            self.add(
                f"{kind}_parse",
                "FAIL" if found else "PASS",
                "; ".join(found) if found else f"all nonempty {kind} parsed",
            )

    def validate_git(self) -> None:
        if not (self.root / ".git").exists():
            self.add(
                "git_diff_whitespace",
                "BLOCKED",
                "Git metadata is absent (for example, an exported ZIP); run git diff --check in the live checkout",
            )
            return
        self.command("git_diff_whitespace", ["git", "diff", "--check"])

    def validate_network_and_units(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            env = temp / "network.env"
            env.write_text(
                "CORE_ETH_IFACE=eth0\nEDGE_ETH_IFACE=eth0\n"
                "CORE_WIFI_IFACE=wlan0\nEDGE_WIFI_IFACE=wlan0\n"
                "CORE_ETH_IP=192.168.50.1\nEDGE_ETH_IP=192.168.50.2\n"
                "LINK_PREFIX=24\nLINK_SUBNET=192.168.50.0/24\n"
                "WIFI_SSID=validator-only\nWIFI_PASSWORD=validator-only\n",
                encoding="utf-8",
            )
            for role in ("core", "edge"):
                self.command(
                    f"netplan_{role}",
                    [
                        str(self.root / "deploy/network/render_network_config.sh"),
                        "--role", role,
                        "--env", str(env),
                        "--output-dir", str(temp / role),
                    ],
                )
            unit_dir = temp / "units"
            self.command(
                "systemd_render",
                [
                    str(self.root / "deploy/systemd/render_units.sh"),
                    "--user", os.environ.get("USER", "robot-savo"),
                    "--group", os.environ.get("USER", "robot-savo"),
                    "--root", str(self.root),
                    "--output-dir", str(unit_dir),
                ],
            )
            rendered = sorted(unit_dir.glob("*.service"))
            if not rendered:
                self.add("systemd_verify", "FAIL", "no systemd services were rendered")
            else:
                self.command(
                    "systemd_verify",
                    ["systemd-analyze", "verify", *[str(path) for path in rendered]],
                    blocked_patterns=(r"systemd-analyze.*not found", r"Failed to connect to bus"),
                )

    def validate_persistent_operations(self) -> None:
        test_script = self.root / "deploy/common/test_state_backup_restore.sh"
        if not test_script.is_file():
            self.add("backup_restore", "FAIL", "backup/restore test script missing")
            return
        self.command(
            "backup_restore",
            [str(test_script)],
            blocked_patterns=(
                r"python3.*not found",
                r"sqlite3 CLI or Python sqlite3 module is required",
            ),
        )

    def validate_rosdep(self) -> None:
        source_root = self.root / "savo_ws/src"
        self.command(
            "rosdep_check",
            ["rosdep", "check", "--from-paths", str(source_root), "--ignore-src"],
            blocked_patterns=(
                r"rosdep.*not found",
                r"command not found",
                r"rosdep database is not initialized",
                r"No definition of \[.*\] for OS",
            ),
        )

    def validate_safety_contracts(self) -> None:
        deploy_corpus = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in (self.root / "deploy").rglob("*")
            if path.is_file() and path.stat().st_size
        )
        self.add(
            "control_startup_stop",
            "PASS" if (
                "SAVO_CONTROL_STARTUP_MODE=STOP" in deploy_corpus
                and "control_startup_mode:=\"${SAVO_CONTROL_STARTUP_MODE:-STOP}\"" in deploy_corpus
            ) else "FAIL",
            "STOP is the deployment default",
        )

        run_core_path = self.root / "deploy/core/run_core.sh"
        run_core = run_core_path.read_text(encoding="utf-8") if run_core_path.is_file() else ""
        self.add(
            "geometry_provisional",
            "PASS" if "allow_provisional_geometry:=\"${SAVO_ALLOW_PROVISIONAL_GEOMETRY:-false}\"" in run_core
            else "FAIL",
            "BLOCKED_FOR_MOTION: geometry_not_locked",
        )
        self.add(
            "voxel_default",
            "PASS" if "d435_voxel_validated:=\"${SAVO_D435_VOXEL_VALIDATED:-false}\"" in run_core
            else "FAIL",
            "D435 voxel validation defaults to false",
        )

        bringup_path = (
            self.root / "savo_ws/src/shared/savo_bringup/launch/autonomous_mapping.launch.py"
        )
        bringup_readme_path = (
            self.root / "savo_ws/src/shared/savo_bringup/README.md"
        )
        action_path = (
            self.root / "savo_ws/src/shared/savo_msgs/action/RunAutonomousMapping.action"
        )
        bringup = bringup_path.read_text(encoding="utf-8") if bringup_path.is_file() else ""
        bringup_readme = (
            bringup_readme_path.read_text(encoding="utf-8")
            if bringup_readme_path.is_file() else ""
        )
        action_contract = action_path.read_text(encoding="utf-8") if action_path.is_file() else ""
        quality_ok = "require_quality_approval: true" in bringup_readme
        review_ok = bool(re.search(
            r"['\"]start_review_gateway['\"]\s*:\s*['\"]true['\"]",
            bringup,
        ))
        contract_ok = bool(re.search(
            r"^uint32\s+CONTRACT_VERSION=2\s*$",
            action_contract,
            re.MULTILINE,
        )) and "contract_version: 2" in bringup_readme
        self.add(
            "am8_required",
            "PASS" if quality_ok and review_ok and contract_ok else "FAIL",
            f"quality={quality_ok}; review_gateway={review_ok}; contract_v2={contract_ok}",
        )

    def validate_scripts(self) -> None:
        failures: list[str] = []
        count = 0
        for base in (self.root / "deploy", self.root / "tools/diag"):
            if not base.exists():
                continue
            for path in base.rglob("*.sh"):
                if not path.is_file() or path.stat().st_size == 0:
                    continue
                count += 1
                result = subprocess.run(
                    ["bash", "-n", str(path)], capture_output=True, text=True, check=False
                )
                if result.returncode != 0:
                    failures.append(f"{path.relative_to(self.root)}: {(result.stderr or result.stdout).strip()}")
        self.add(
            "bash_syntax",
            "FAIL" if failures else "PASS",
            "; ".join(failures) if failures else f"{count} shell scripts parsed",
        )

    def validate_diagnostic_safety(self) -> None:
        prohibited = (
            "import smbus",
            "from smbus",
            "PCA9685(",
            "Adafruit_PCA9685",
            "/dev/gpiochip",
            "RPi.GPIO",
        )
        relative_paths = (
            "tools/diag/motion/automode.py",
            "tools/diag/motion/drive_automode.py",
            "tools/diag/motion/drive_manual_direct.py",
            "tools/diag/motion/motor_direction_test.py",
            "tools/diag/motion/pantilt_camera_view.py",
            "tools/diag/motion/pantilt_servo.py",
            "tools/diag/ui/head_pan_tilt_test.py",
        )
        offenders: list[str] = []
        for relative in relative_paths:
            path = self.root / relative
            if not path.is_file():
                offenders.append(f"{relative}:missing")
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            for token in prohibited:
                if token in text:
                    offenders.append(f"{relative}:{token}")
        motor = (self.root / "tools/diag/motion/motor_direction_test.py").read_text(
            encoding="utf-8", errors="replace")
        head = (self.root / "tools/diag/ui/head_pan_tilt_test.py").read_text(
            encoding="utf-8", errors="replace")
        boundaries_ok = all((
            "--allow-motion" in motor,
            "--wheels-raised" in motor,
            "/cmd_vel_manual" in motor,
            "/cmd_vel_safe" not in motor,
            "parser(__doc__, motion=True)" in head,
            "args.allow_motion" in head,
            "--head-clear" in head,
            "/savo_head/pan_tilt_cmd" in head,
        ))
        if not boundaries_ok:
            offenders.append("approved_motion_boundary_missing")
        self.add(
            "diagnostic_motion_safety",
            "FAIL" if offenders else "PASS",
            ", ".join(offenders) if offenders else
            "moving diagnostics use approved ROS boundaries and explicit physical opt-ins",
        )

    def validate_ui_integration(self) -> None:
        package = self.root / "savo_ws/src/edge/savo_ui/package.xml"
        source = self.root / "savo_ws/src/edge/savo_ui/src/app/ui_node.cpp"
        config = self.root / "savo_ws/src/edge/savo_ui/config/ui.yaml"
        if not all(path.is_file() for path in (package, source, config)):
            self.add("ui_read_only_integration", "FAIL", "required UI files missing")
            return
        corpus = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in (package, source, config))
        required = (
            "savo_msgs",
            "AutonomousMappingStatus",
            "LocationEvent",
            "/savo_mapping/autonomous/status",
            "/savo_speech/state",
            "/savo_speech/transcript",
            "/savo_speech/response",
        )
        missing = [token for token in required if token not in corpus]
        unsafe = any(token in source.read_text(encoding="utf-8", errors="replace") for token in (
            "create_publisher", "/cmd_vel", "NavigateToLocation::Goal", "ReviewAutonomousMappingRelease",
        ))
        self.add(
            "ui_read_only_integration",
            "FAIL" if missing or unsafe else "PASS",
            f"missing={missing}; unsafe_authority={unsafe}" if missing or unsafe else
            "typed mapping/location and live speech/system feeds are read-only",
        )

    def validate_bridge_command_boundary(self) -> None:
        protocol = self.root / "savo_ws/src/shared/savo_bridge/src/command_protocol.cpp"
        dispatcher = self.root / "savo_ws/src/shared/savo_bridge/src/ros_command_dispatcher.cpp"
        readme = self.root / "savo_ws/src/shared/savo_bridge/README.md"
        if not all(path.is_file() for path in (protocol, dispatcher, readme)):
            self.add("bridge_command_boundary", "FAIL", "bridge command files missing")
            return
        corpus = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in (protocol, dispatcher, readme))
        corpus_lower = corpus.lower()
        required = (
            "start_autonomous_mapping",
            "request_scan360",
            "query_mapping_state",
            "query_supervisor_state",
            "require_quality_approval",
            "operator approval is never accepted from savomind",
        )
        missing = [token for token in required if token.lower() not in corpus_lower]
        forbidden = ("generic ROS", "arbitrary poses")
        documented_closed = all(token in readme.read_text(encoding="utf-8", errors="replace") for token in forbidden)
        self.add(
            "bridge_command_boundary",
            "FAIL" if missing or not documented_closed else "PASS",
            f"missing={missing}; closed_boundary_documented={documented_closed}" if missing or not documented_closed else
            "typed STOP/teleop/navigation/mapping/query boundary present; approval remains operator-only",
        )

    def validate_external_contract(self) -> None:
        contract = (
            self.root / "savo_ws/src/edge/savo_speech/docs/savomind_speech_transport_v2.md"
        )
        client = (
            self.root / "savo_ws/src/edge/savo_speech/src/transport/savomind_transport.cpp"
        )
        deployment = self.root / "deploy/edge/savomind_speech_contract.yaml"
        if not all(path.is_file() and path.stat().st_size for path in (contract, client, deployment)):
            self.add(
                "savomind_speech_contract_v2",
                "FAIL",
                "Robot-side v2 client, documentation, or deployment contract is missing",
            )
            return
        corpus = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in (contract, client, deployment)
        )
        required = (
            "contract_version: 2",
            "SAVOSPRQ",
            "SAVOSPRS",
            "SAVOSPAK",
            "SAVOSPAR",
            "acknowledge_playback",
            "physical_playback_ack_required_for_pending_navigation: true",
        )
        missing = [token for token in required if token not in corpus]
        self.add(
            "savomind_speech_contract_v2",
            "FAIL" if missing else "PASS",
            f"missing={missing}" if missing else
            "bounded authenticated v2 speech and physical-playback acknowledgement contract present",
        )

    def run(self) -> None:
        self.validate_required()
        self.validate_git()
        self.validate_empty_files()
        self.validate_launch_references()
        self.validate_parsers()
        self.validate_scripts()
        self.validate_network_and_units()
        self.validate_persistent_operations()
        self.validate_rosdep()
        observer = self.root / "deploy/observer/validate_observer.sh"
        if observer.is_file():
            self.command("observer_read_only", [str(observer)])
        else:
            self.add("observer_read_only", "FAIL", "observer validator missing")
        self.validate_safety_contracts()
        self.validate_diagnostic_safety()
        self.validate_ui_integration()
        self.validate_bridge_command_boundary()
        self.validate_external_contract()

    def write_report(self, output_dir: Path) -> int:
        failures = [check for check in self.checks if check.status == "FAIL"]
        blockers = [check for check in self.checks if check.status == "BLOCKED"]
        if failures:
            overall = "FAIL"
            exit_code = 1
        elif blockers:
            overall = "BLOCKED"
            exit_code = 2
        else:
            overall = "PASS"
            exit_code = 0

        report = {
            "schema_version": 2,
            "generated_utc": datetime.now(UTC).isoformat(),
            "status": overall,
            "checks": [asdict(check) for check in self.checks],
            "physical_blockers": list(PHYSICAL_BLOCKERS),
            "external_blockers": [check.detail for check in blockers],
            "blocked_for_motion": "geometry_not_locked",
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "pre_real_test_readiness.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        lines = [
            "# Pre-real-test readiness",
            "",
            f"Status: **{overall}**",
            "",
            "| Check | Status | Detail |",
            "| --- | --- | --- |",
        ]
        lines.extend(
            f"| {check.name} | {check.status} | "
            f"{check.detail.replace('|', '/').replace(chr(10), ' ')} |"
            for check in self.checks
        )
        lines.extend(["", "## Physical blockers", ""])
        lines.extend(f"- {item}" for item in PHYSICAL_BLOCKERS)
        lines.append("")
        (output_dir / "pre_real_test_readiness.md").write_text(
            "\n".join(lines), encoding="utf-8"
        )

        print(overall)
        for check in self.checks:
            print(f"{check.status:7} {check.name}: {check.detail}")
        return exit_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(os.environ.get("SAVO_VALIDATION_ROOT", DEFAULT_ROOT)),
        help="Robot SAVO repository root",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report directory; defaults to <root>/log",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    validator = Validator(args.root)
    validator.run()
    output_dir = args.output_dir or (validator.root / "log")
    return validator.write_report(output_dir)


if __name__ == "__main__":
    raise SystemExit(main())
