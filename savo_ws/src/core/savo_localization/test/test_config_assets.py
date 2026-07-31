"""Validate installed localization configuration references."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VO_OVERLAY = "config/ekf_vo_input_optional.yaml"
STALE_OVERLAY = "vo_fusion_optional.yaml"


def test_optional_vo_overlay_exists_and_is_valid_yaml() -> None:
    """Require the actual VO overlay consumed by launch and diagnostics."""
    path = PACKAGE_ROOT / VO_OVERLAY
    assert path.is_file()
    assert path.stat().st_size > 0

    with path.open(encoding="utf-8") as stream:
        parsed = yaml.safe_load(stream)
    assert isinstance(parsed, dict)
    assert parsed


def test_launch_and_dump_script_reference_the_installed_overlay() -> None:
    """Prevent a stale filename from breaking VO-enabled localization."""
    launch_text = (PACKAGE_ROOT / "launch/ekf.launch.py").read_text()
    dump_text = (PACKAGE_ROOT / "scripts/dump_effective_params.py").read_text()
    bringup_text = (PACKAGE_ROOT / "launch/localization_bringup.launch.py").read_text()

    for text in (launch_text, dump_text, bringup_text):
        assert "ekf_vo_input_optional.yaml" in text
        assert STALE_OVERLAY not in text


def test_cmake_installs_localization_config_directory() -> None:
    """Ensure the corrected overlay is available from package share."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "config/" in cmake
    assert "DESTINATION share/${PROJECT_NAME}/config" in cmake
