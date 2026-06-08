import pytest

from savo_realsense.ros.frame_contract import (
    CAMERA_OPTICAL_FRAMES,
    REQUIRED_TF_FRAMES,
    all_realsense_frames,
    camera_optical_frames,
)
from savo_realsense.utils.frame_names import (
    has_optical_frame_suffix,
    is_camera_frame,
    normalize_frame_name,
    unique_frame_names,
)


def test_normalize_frame_name_removes_leading_slash() -> None:
    assert normalize_frame_name("/camera_link") == "camera_link"


def test_normalize_frame_name_rejects_empty_value() -> None:
    with pytest.raises(ValueError):
        normalize_frame_name("")


def test_optical_frame_suffix_detection() -> None:
    assert has_optical_frame_suffix("camera_color_optical_frame")
    assert not has_optical_frame_suffix("camera_link")


def test_camera_frame_detection() -> None:
    assert is_camera_frame("camera_link")
    assert is_camera_frame("camera_depth_optical_frame")
    assert not is_camera_frame("base_link")


def test_unique_frame_names_preserves_order() -> None:
    assert unique_frame_names(
        "base_link",
        "/camera_link",
        "camera_link",
        "camera_color_optical_frame",
    ) == (
        "base_link",
        "camera_link",
        "camera_color_optical_frame",
    )


def test_camera_optical_frames_match_contract() -> None:
    assert camera_optical_frames() == CAMERA_OPTICAL_FRAMES


def test_all_realsense_frames_contains_required_frames() -> None:
    frames = all_realsense_frames()

    for frame in REQUIRED_TF_FRAMES:
        assert frame in frames