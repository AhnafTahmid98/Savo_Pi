"""Production deployment validator integration tests."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = ROOT / "deploy" / "common" / "validate_deployment_assets.py"


def test_deployment_validator_checks_rendering_ownership_and_shells() -> None:
    result = subprocess.run(
        [sys.executable, str(VALIDATOR), "--root", str(ROOT)],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    for check in (
        "required_files",
        "bash_syntax",
        "render_home_root",
        "render_opt_root",
        "render_source_root",
        "root_environment_consistency",
        "core_owner_contract",
        "edge_ui_owner_contract",
        "install_only_contract",
        "systemd_verify",
    ):
        assert check in result.stdout
    assert "FAIL" not in result.stdout


def test_core_production_test_path_calls_deployment_validator() -> None:
    build = ROOT.joinpath("deploy/core/build_core.sh").read_text(encoding="utf-8")

    assert "validate_deployment_assets.py" in build
    assert 'python3 -m pytest -q "${SAVO_ROOT}/deploy/test"' in build
    assert 'if [[ "${RUN_TESTS}" == "1" ]]' in build
    assert build.index("colcon test-result --verbose") < build.index(
        "validate_deployment_assets.py"
    )
