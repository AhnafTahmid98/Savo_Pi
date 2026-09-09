"""Prevent the VO camera extrinsic from drifting from locked description."""

from pathlib import Path

import pytest
import yaml


VO_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = VO_ROOT.parents[3]
DESCRIPTION_PROFILE = (
    PROJECT_ROOT
    / "savo_ws/src/shared/savo_description/config/profiles"
    / "robot_savo_core_v1.yaml"
)


def test_real_vo_extrinsic_matches_locked_description_chain() -> None:
    """VO duplicates the composed footprint-to-color-optical transform."""
    description = yaml.safe_load(DESCRIPTION_PROFILE.read_text(encoding="utf-8"))
    vo = yaml.safe_load(
        (VO_ROOT / "config/profiles/real_robot_v1.yaml").read_text(
            encoding="utf-8"
        )
    )["rgbd_odometry_node"]["ros__parameters"]

    mount = description["mounts"]["realsense_d435"]
    base_z = description["chassis"]["base_footprint_to_base_link_z_m"]
    expected_xyz = [
        mount["xyz_m"][0],
        mount["xyz_m"][1],
        base_z + mount["xyz_m"][2],
    ]

    assert mount["parent"] == "base_link"
    assert mount["frame"] == "camera_link"
    assert vo["base_frame"] == description["frames"]["base_footprint"]
    assert vo["camera_frame"] == description["frames"]["camera_color_optical"]
    assert [
        vo["base_to_camera_x_m"],
        vo["base_to_camera_y_m"],
        vo["base_to_camera_z_m"],
    ] == pytest.approx(expected_xyz)
    assert [
        vo["base_to_camera_roll_rad"],
        vo["base_to_camera_pitch_rad"],
        vo["base_to_camera_yaw_rad"],
    ] == pytest.approx([-1.5707963267948966, 0.0, -1.5707963267948966])
