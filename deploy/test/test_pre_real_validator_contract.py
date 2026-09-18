"""Pre-real-test validator contract regression tests."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


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
