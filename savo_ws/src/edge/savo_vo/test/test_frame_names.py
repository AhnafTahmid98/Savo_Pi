"""Tests for savo_vo frame name contracts."""

from savo_vo.contracts.frame_names import (
    BASE_LINK_FRAME,
    CAMERA_COLOR_FRAME,
    CAMERA_DEPTH_FRAME,
    CAMERA_LINK_FRAME,
    ODOM_FRAME,
    VO_CAMERA_FRAME,
    VO_ODOM_FRAME,
)


def test_core_robot_frames_match_expected_defaults() -> None:
    assert ODOM_FRAME == "odom"
    assert BASE_LINK_FRAME == "base_link"


def test_vo_internal_frames_match_expected_defaults() -> None:
    assert VO_ODOM_FRAME == "vo_odom"
    assert VO_CAMERA_FRAME == "vo_camera_link"


def test_camera_frames_match_expected_defaults() -> None:
    assert CAMERA_LINK_FRAME == "camera_link"
    assert CAMERA_COLOR_FRAME == "camera_color_optical_frame"
    assert CAMERA_DEPTH_FRAME == "camera_depth_optical_frame"


def test_frame_names_are_not_empty() -> None:
    frames = [
        ODOM_FRAME,
        BASE_LINK_FRAME,
        VO_ODOM_FRAME,
        VO_CAMERA_FRAME,
        CAMERA_LINK_FRAME,
        CAMERA_COLOR_FRAME,
        CAMERA_DEPTH_FRAME,
    ]

    for frame in frames:
        assert frame
        assert frame.strip() == frame
        assert " " not in frame