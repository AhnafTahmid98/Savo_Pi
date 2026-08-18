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


def test_vo_fusion_contract_matches_planar_ekf_base() -> None:
    """VO must measure odom -> base_footprint without taking TF ownership."""
    overlay = yaml.safe_load((PACKAGE_ROOT / VO_OVERLAY).read_text(encoding="utf-8"))
    params = overlay["ekf_filter_node"]["ros__parameters"]

    assert params["odom1"] == "/vo/odom"
    assert params["odom1_config"] == [
        True,
        True,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
    ]
    assert params["odom1_differential"] is False
    assert params["odom1_relative"] is True

    frames = yaml.safe_load(
        (PACKAGE_ROOT / "config/frames.yaml").read_text(encoding="utf-8")
    )
    ekf_frames = frames["ekf_filter_node"]["ros__parameters"]
    assert ekf_frames["base_link_frame"] == "base_footprint"

    health_source = (
        PACKAGE_ROOT / "src/localization_health_node.cpp"
    ).read_text(encoding="utf-8")
    assert "record_odom(vo_tracker_, message, odom_frame_id_, base_frame_id_)" in health_source
