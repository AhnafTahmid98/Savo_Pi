#!/usr/bin/env python3
"""
Validate Robot Savo deployment assets without installing or activating them.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import tempfile


@dataclass(frozen=True)
class Check:
    """One deployment validation result."""

    name: str
    status: str
    detail: str


class DeploymentValidator:
    """Perform read-only source and rendered deployment checks."""

    REQUIRED = (
        "deploy/common/runtime_ownership.py",
        "deploy/common/validate_deployment_assets.py",
        "deploy/core/run_core.sh",
        "deploy/core/run_autonomous_mapping.sh",
        "deploy/core/run_mapping_service.sh",
        "deploy/systemd/install_services.sh",
        "deploy/systemd/robot-savo-core-tmpfiles.conf.in",
        "deploy/systemd/render_units.sh",
        "deploy/systemd/robot-savo.paths.env.in",
        "deploy/systemd/savo-location-stack@.service",
        "deploy/systemd/savo.service",
        "deploy/systemd/savo_core.service",
        "deploy/systemd/savo_edge.service",
        "deploy/systemd/savo_mapping.service",
        "savo_ws/src/edge/savo_ui/scripts/install_savo_ui_service.sh",
    )

    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.checks: list[Check] = []
        self.rendered: list[Path] = []

    def add(self, name: str, status: str, detail: str) -> None:
        self.checks.append(Check(name, status, detail))

    def required_files(self) -> None:
        missing = [item for item in self.REQUIRED if not self.root.joinpath(item).is_file()]
        self.add(
            "required_files",
            "FAIL" if missing else "PASS",
            ", ".join(missing) if missing else "all deployment assets present",
        )

    def bash_syntax(self) -> None:
        failures = []
        scripts = sorted(self.root.joinpath("deploy").rglob("*.sh"))
        scripts.append(
            self.root / "savo_ws/src/edge/savo_ui/scripts/install_savo_ui_service.sh"
        )
        for script in scripts:
            result = subprocess.run(
                ["bash", "-n", str(script)], capture_output=True, text=True, check=False
            )
            if result.returncode:
                failures.append(f"{script.relative_to(self.root)}: {result.stderr.strip()}")
        self.add(
            "bash_syntax",
            "FAIL" if failures else "PASS",
            "; ".join(failures) if failures else f"{len(scripts)} scripts parsed",
        )

    def render(self, temporary: Path, deployment_root: str, name: str) -> Path | None:
        output = temporary / name
        result = subprocess.run(
            [
                "bash",
                str(self.root / "deploy/systemd/render_units.sh"),
                "--user",
                "savo",
                "--group",
                "savo",
                "--source-root",
                str(self.root),
                "--root",
                deployment_root,
                "--output-dir",
                str(output),
                "--skip-systemd-verify",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            self.add(name, "FAIL", (result.stdout + result.stderr).strip())
            return None
        self.add(name, "PASS", f"rendered root={deployment_root}")
        return output

    def root_consistency(self, outputs: list[tuple[Path, str]]) -> None:
        failures = []
        for output, root in outputs:
            paths = output.joinpath("robot-savo.paths.env").read_text(encoding="utf-8")
            if f"SAVO_ROOT={root}\n" not in paths or f"SAVO_WS={root}/savo_ws\n" not in paths:
                failures.append(f"{root}: generated path environment mismatch")
            for unit in output.glob("*.service"):
                text = unit.read_text(encoding="utf-8")
                if "@SAVO_" in text:
                    failures.append(f"{unit.name}: unresolved placeholder")
                if "EnvironmentFile=/etc/robot-savo/robot-savo.paths.env" not in text:
                    failures.append(f"{unit.name}: generated paths environment not required")
                if f"WorkingDirectory={root}" not in text:
                    failures.append(f"{unit.name}: working directory mismatch")
        self.add(
            "root_environment_consistency",
            "FAIL" if failures else "PASS",
            "; ".join(failures) if failures else "rendered roots and runtime environment agree",
        )

    def ownership_contracts(self) -> None:
        core = self.root.joinpath("deploy/core/run_core.sh").read_text(encoding="utf-8")
        mapping_runner = self.root.joinpath(
            "deploy/core/run_mapping_service.sh"
        ).read_text(encoding="utf-8")
        autonomous_runner = self.root.joinpath(
            "deploy/core/run_autonomous_mapping.sh"
        ).read_text(encoding="utf-8")
        mapping_unit = self.root.joinpath(
            "deploy/systemd/savo_mapping.service"
        ).read_text(encoding="utf-8")
        generic_unit = self.root.joinpath("deploy/systemd/savo.service").read_text(
            encoding="utf-8"
        )
        core_unit = self.root.joinpath("deploy/systemd/savo_core.service").read_text(
            encoding="utf-8"
        )
        tmpfiles = self.root.joinpath(
            "deploy/systemd/robot-savo-core-tmpfiles.conf.in"
        ).read_text(encoding="utf-8")
        core_ok = all(
            token in core and token in mapping_runner and token in autonomous_runner
            for token in (
                "runtime_ownership.py",
                "preflight",
                "run-core-owner",
                "/run/robot-savo/core-owner.lock",
            )
        ) and all(
            "SAVO_CORE_OWNER_LOCK" not in runner
            for runner in (core, mapping_runner, autonomous_runner)
        ) and all(
            "RuntimeDirectoryPreserve=yes" in unit
            for unit in (core_unit, mapping_unit, generic_unit)
        ) and all(
            "RestartPreventExitStatus=4" in unit
            for unit in (core_unit, mapping_unit, generic_unit)
        ) and "ExecCondition=/usr/bin/python3" in core_unit and (
            "ExecCondition=/usr/bin/python3" in mapping_unit
        ) and "ExecStartPre=/usr/bin/python3" not in core_unit and (
            "ExecStartPre=/usr/bin/python3" not in mapping_unit
        ) and "manual_mapping must use deploy/core/run_mapping_service.sh" in core and (
            "autonomous_mapping must use deploy/core/run_autonomous_mapping.sh" in core
        ) and "d /run/robot-savo 0750 @SAVO_USER@ @SAVO_GROUP@ -" in tmpfiles and (
            "After=network-online.target savo_core.service" not in mapping_unit
        )
        self.add(
            "core_owner_contract",
            "PASS" if core_ok else "FAIL",
            "shared lifetime lock and non-overlay mapping ordering" if core_ok
            else "Core/mapping ownership contract incomplete",
        )

        edge = self.root.joinpath("deploy/systemd/savo_edge.service").read_text(
            encoding="utf-8"
        )
        ui = self.root.joinpath(
            "savo_ws/src/edge/savo_ui/systemd/savo-ui-runtime.service"
        ).read_text(encoding="utf-8")
        ui_installer = self.root.joinpath(
            "savo_ws/src/edge/savo_ui/scripts/install_savo_ui_service.sh"
        ).read_text(encoding="utf-8")
        edge_ok = all(
            token in edge
            for token in (
                "savo-ui-runtime.service",
                "Environment=SAVO_START_UI=false",
            )
        ) and all(
            token in ui + ui_installer
            for token in (
                "Before=savo_edge.service",
                "robot-savo.paths.env",
                "is-active",
            )
        )
        self.add(
            "edge_ui_owner_contract",
            "PASS" if edge_ok else "FAIL",
            "savo-ui-runtime.service is the production UI owner" if edge_ok
            else "Edge UI ownership mismatch",
        )

        installer = self.root.joinpath("deploy/systemd/install_services.sh").read_text(
            encoding="utf-8"
        )
        forbidden = (
            "systemctl enable",
            "systemctl start",
            "systemctl stop",
            "systemctl restart",
        )
        install_ok = not any(token in installer for token in forbidden)
        self.add(
            "install_only_contract",
            "PASS" if install_ok else "FAIL",
            "installer performs no activation" if install_ok else "installer contains activation",
        )


    def systemd_verify(self) -> None:
        tool = shutil.which("systemd-analyze")
        if tool is None:
            self.add("systemd_verify", "BLOCKED", "systemd-analyze unavailable on this host")
            return
        result = subprocess.run(
            [tool, "verify", *[str(path) for path in self.rendered]],
            capture_output=True,
            text=True,
            check=False,
        )
        self.add(
            "systemd_verify",
            "PASS" if result.returncode == 0 else "FAIL",
            (result.stdout + result.stderr).strip() or "all rendered units verified",
        )

    def run(self) -> int:
        self.required_files()
        self.bash_syntax()
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            outputs = []
            for deployment_root, name in (
                ("/home/savo/Savo_Pi", "render_home_root"),
                ("/opt/robot-savo", "render_opt_root"),
            ):
                output = self.render(temporary, deployment_root, name)
                if output is not None:
                    outputs.append((output, deployment_root))
            source_output = self.render(
                temporary, str(self.root), "render_source_root"
            )
            if source_output is not None:
                outputs.append((source_output, str(self.root)))
                self.rendered = sorted(source_output.glob("*.service"))
            self.root_consistency(outputs)
            self.ownership_contracts()
            self.systemd_verify()

        for check in self.checks:
            print(f"{check.status:7} {check.name}: {check.detail}")
        return 1 if any(check.status == "FAIL" for check in self.checks) else 0


def parse_args() -> argparse.Namespace:
    """Parse validator arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    return parser.parse_args()


def main() -> int:
    """Run deployment validation."""
    return DeploymentValidator(parse_args().root).run()


if __name__ == "__main__":
    raise SystemExit(main())
