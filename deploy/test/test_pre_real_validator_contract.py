"""Pre-real-test validator contract regression tests."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = ROOT / "deploy/common/validate_pre_real_test_readiness.py"


def _module():
    spec = importlib.util.spec_from_file_location("pre_real_validator", VALIDATOR)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_autonomous_mapping_contract_version_is_derived_from_action() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    action = ROOT.joinpath(
        "savo_ws/src/shared/savo_msgs/action/RunAutonomousMapping.action"
    ).read_text(encoding="utf-8")

    assert "uint32 CONTRACT_VERSION=3" in action
    assert "autonomous_action_contract_version" in source
    mapping_section = source[source.index("def validate_safety_contracts"):source.index(
        "def validate_scripts"
    )]
    assert "CONTRACT_VERSION=2" not in mapping_section
    assert "contract_version: 2" not in mapping_section


def test_generated_source_paths_are_excluded() -> None:
    module = _module()

    for path in (
        Path("savo_ws/src/pkg/__pycache__/node.cpython-312.pyc"),
        Path("savo_ws/build/pkg/generated.py"),
        Path("savo_ws/install/pkg/node.py"),
        Path("savo_ws/log/latest/node.py"),
        Path(".pytest_cache/v/cache/nodeids"),
        Path("pkg/.mypy_cache/data.json"),
        Path("pkg/.ruff_cache/cache"),
    ):
        assert module.is_generated_path(path), path
    assert not module.is_generated_path(Path("deploy/common/real_source.py"))


def test_observer_script_prunes_bytecode_and_cache_directories() -> None:
    observer = ROOT.joinpath("deploy/observer/validate_observer.sh").read_text(
        encoding="utf-8"
    )

    assert "__pycache__" in observer
    assert "*.pyc" in observer
    assert "-prune" in observer


def test_observer_validator_falls_back_when_ripgrep_is_unavailable() -> None:
    observer = ROOT.joinpath("deploy/observer/validate_observer.sh").read_text(
        encoding="utf-8"
    )

    assert "command -v rg" in observer
    assert "grep -RInE" in observer
    assert "observer source scan failed" in observer.lower()


def test_main_validator_requires_new_deployment_assets() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")

    for path in (
        "deploy/common/runtime_ownership.py",
        "deploy/common/validate_deployment_assets.py",
        "deploy/systemd/install_services.sh",
        "deploy/systemd/robot-savo.paths.env.in",
        "deploy/systemd/savo-location-stack@.service",
    ):
        assert f'"{path}"' in source


def test_canonical_locked_geometry_passes_without_motion_block() -> None:
    module = _module()
    profile = ROOT / (
        "savo_ws/src/shared/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.name == "geometry_locked"
    assert check.status == "PASS"
    assert check.detail == "locked production geometry revision 5"


def test_provisional_or_invalid_geometry_fails_and_blocks_motion(
    tmp_path: Path,
) -> None:
    module = _module()
    canonical = ROOT / (
        "savo_ws/src/shared/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )
    profile_data = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    profile_data["metadata"]["measurement_state"] = "provisional"
    profile = tmp_path / "provisional.yaml"
    profile.write_text(yaml.safe_dump(profile_data), encoding="utf-8")

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.name == "geometry_locked"
    assert check.status == "FAIL"
    assert "production requires locked geometry" in check.detail


def test_malformed_geometry_fails_closed(tmp_path: Path) -> None:
    module = _module()
    profile = tmp_path / "malformed.yaml"
    profile.write_text("metadata: [", encoding="utf-8")

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.name == "geometry_locked"
    assert check.status == "FAIL"
    assert check.detail


def test_missing_geometry_profile_fails_closed(tmp_path: Path) -> None:
    module = _module()

    check = module.validate_production_geometry(
        tmp_path / "missing.yaml",
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.name == "geometry_locked"
    assert check.status == "FAIL"
    assert "No such file" in check.detail


def test_wrong_geometry_profile_id_fails_closed(tmp_path: Path) -> None:
    module = _module()
    canonical = ROOT / (
        "savo_ws/src/shared/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )
    profile_data = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    profile_data["metadata"]["profile_id"] = "wrong_robot"
    profile = tmp_path / "wrong-id.yaml"
    profile.write_text(yaml.safe_dump(profile_data), encoding="utf-8")

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.status == "FAIL"
    assert "unexpected production geometry profile_id" in check.detail


def test_locked_geometry_with_remaining_calibration_fails_closed(
    tmp_path: Path,
) -> None:
    module = _module()
    canonical = ROOT / (
        "savo_ws/src/shared/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )
    profile_data = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    profile_data["calibration_remaining"] = ["wheel_radius"]
    profile = tmp_path / "unfinished.yaml"
    profile.write_text(yaml.safe_dump(profile_data), encoding="utf-8")

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.status == "FAIL"
    assert "locked profile cannot retain calibration blockers" in check.detail


def test_geometry_profile_validator_failure_fails_closed(tmp_path: Path) -> None:
    module = _module()
    canonical = ROOT / (
        "savo_ws/src/shared/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )
    profile_data = yaml.safe_load(canonical.read_text(encoding="utf-8"))
    profile_data["wheels"]["radius_m"] = -0.1
    profile = tmp_path / "invalid-geometry.yaml"
    profile.write_text(yaml.safe_dump(profile_data), encoding="utf-8")

    check = module.validate_production_geometry(
        profile,
        ROOT / "savo_ws/src/shared/savo_description/scripts/geometry_profile.py",
    )

    assert check.status == "FAIL"
    assert "wheels.radius_m must be positive" in check.detail


def test_report_motion_block_matches_geometry_gate(tmp_path: Path) -> None:
    module = _module()
    validator = module.Validator(ROOT)
    validator.checks = [
        module.Check(
            name="geometry_locked",
            status="PASS",
            detail="locked production geometry revision 5",
        )
    ]

    assert validator.write_report(tmp_path / "pass") == 0
    passed = yaml.safe_load(
        (tmp_path / "pass/pre_real_test_readiness.json").read_text(
            encoding="utf-8"
        )
    )
    assert passed["status"] == "PASS"
    assert passed["blocked_for_motion"] is None

    validator.checks = [
        module.Check(
            name="geometry_locked",
            status="FAIL",
            detail="production requires locked geometry",
        )
    ]
    assert validator.write_report(tmp_path / "fail") == 1
    failed = yaml.safe_load(
        (tmp_path / "fail/pre_real_test_readiness.json").read_text(
            encoding="utf-8"
        )
    )
    assert failed["status"] == "FAIL"
    assert failed["blocked_for_motion"] == "geometry_not_locked"
