"""Tests for visual odometry motion sanity checks."""

import pytest

from savo_vo.core.motion_quality import (
    MotionDelta,
    check_motion_quality,
    compute_planar_motion_delta,
    is_motion_jump,
    normalize_angle_rad,
)


def test_normalize_angle_wraps_positive_angle() -> None:
    assert normalize_angle_rad(3.5) == pytest.approx(-2.7831853071795862)


def test_normalize_angle_wraps_negative_angle() -> None:
    assert normalize_angle_rad(-3.5) == pytest.approx(2.7831853071795862)


def test_normalize_angle_keeps_angle_inside_range() -> None:
    assert normalize_angle_rad(1.2) == pytest.approx(1.2)


def test_compute_planar_motion_delta() -> None:
    delta = compute_planar_motion_delta(
        previous_x_m=0.0,
        previous_y_m=0.0,
        previous_yaw_rad=0.0,
        current_x_m=3.0,
        current_y_m=4.0,
        current_yaw_rad=0.5,
    )

    assert delta.translation_m == pytest.approx(5.0)
    assert delta.rotation_rad == pytest.approx(0.5)


def test_compute_planar_motion_delta_wraps_yaw() -> None:
    delta = compute_planar_motion_delta(
        previous_x_m=0.0,
        previous_y_m=0.0,
        previous_yaw_rad=3.10,
        current_x_m=0.0,
        current_y_m=0.0,
        current_yaw_rad=-3.10,
    )

    assert delta.translation_m == pytest.approx(0.0)
    assert delta.rotation_rad == pytest.approx(0.08318530717958605)


def test_is_motion_jump_detects_translation_jump() -> None:
    delta = MotionDelta(
        translation_m=0.50,
        rotation_rad=0.10,
    )

    assert is_motion_jump(
        delta=delta,
        max_translation_jump_m=0.30,
        max_rotation_jump_rad=0.35,
    )


def test_is_motion_jump_detects_rotation_jump() -> None:
    delta = MotionDelta(
        translation_m=0.10,
        rotation_rad=0.50,
    )

    assert is_motion_jump(
        delta=delta,
        max_translation_jump_m=0.30,
        max_rotation_jump_rad=0.35,
    )


def test_is_motion_jump_accepts_valid_motion() -> None:
    delta = MotionDelta(
        translation_m=0.10,
        rotation_rad=0.10,
    )

    assert not is_motion_jump(
        delta=delta,
        max_translation_jump_m=0.30,
        max_rotation_jump_rad=0.35,
    )


def test_is_motion_jump_rejects_negative_translation_limit() -> None:
    with pytest.raises(ValueError):
        is_motion_jump(
            delta=MotionDelta(translation_m=0.1, rotation_rad=0.1),
            max_translation_jump_m=-0.1,
            max_rotation_jump_rad=0.35,
        )


def test_is_motion_jump_rejects_negative_rotation_limit() -> None:
    with pytest.raises(ValueError):
        is_motion_jump(
            delta=MotionDelta(translation_m=0.1, rotation_rad=0.1),
            max_translation_jump_m=0.30,
            max_rotation_jump_rad=-0.1,
        )


def test_check_motion_quality_rejects_jump() -> None:
    result = check_motion_quality(
        previous_x_m=0.0,
        previous_y_m=0.0,
        previous_yaw_rad=0.0,
        current_x_m=1.0,
        current_y_m=0.0,
        current_yaw_rad=0.0,
        max_translation_jump_m=0.30,
        max_rotation_jump_rad=0.35,
    )

    assert not result.is_valid
    assert result.delta.translation_m == pytest.approx(1.0)
    assert "motion jump rejected" in result.message


def test_check_motion_quality_accepts_valid_motion() -> None:
    result = check_motion_quality(
        previous_x_m=0.0,
        previous_y_m=0.0,
        previous_yaw_rad=0.0,
        current_x_m=0.10,
        current_y_m=0.05,
        current_yaw_rad=0.1,
        max_translation_jump_m=0.30,
        max_rotation_jump_rad=0.35,
    )

    assert result.is_valid
    assert result.delta.translation_m == pytest.approx(0.1118033988749895)
    assert result.delta.rotation_rad == pytest.approx(0.1)
    assert result.message == "visual odometry motion delta is valid"