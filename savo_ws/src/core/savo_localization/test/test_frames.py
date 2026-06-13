#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for frame helpers used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_MAP,
    FRAME_ODOM,
)
from savo_localization.utils.frames import (
    frame_id_valid,
    frame_pair_valid,
    make_tf_edge_label,
    normalize_frame_id,
    require_frame_id,
    require_frame_pair,
)


def test_normalize_frame_id_keeps_plain_frame() -> None:
    assert normalize_frame_id("base_link") == "base_link"
    assert normalize_frame_id("odom") == "odom"
    assert normalize_frame_id("imu_link") == "imu_link"


def test_normalize_frame_id_strips_spaces() -> None:
    assert normalize_frame_id("  base_link  ") == "base_link"
    assert normalize_frame_id("\todom\n") == "odom"


def test_normalize_frame_id_strips_leading_slash() -> None:
    assert normalize_frame_id("/base_link") == "base_link"
    assert normalize_frame_id("///imu_link") == "imu_link"


def test_normalize_frame_id_does_not_strip_internal_slash() -> None:
    assert normalize_frame_id("/robot/base_link") == "robot/base_link"


def test_frame_id_valid_true_for_robot_savo_frames() -> None:
    assert frame_id_valid(FRAME_MAP)
    assert frame_id_valid(FRAME_ODOM)
    assert frame_id_valid(FRAME_BASE_LINK)
    assert frame_id_valid(FRAME_IMU)


@pytest.mark.parametrize(
    "frame_id",
    [
        "",
        " ",
        "\t",
        None,
    ],
)
def test_frame_id_valid_false_for_empty_values(frame_id: str | None) -> None:
    assert not frame_id_valid(frame_id)


def test_require_frame_id_returns_normalized_value() -> None:
    assert require_frame_id(" /base_link ") == "base_link"
    assert require_frame_id("/imu_link") == "imu_link"


def test_require_frame_id_rejects_empty_value() -> None:
    with pytest.raises(ValueError):
        require_frame_id("")

    with pytest.raises(ValueError):
        require_frame_id("   ")


def test_frame_pair_valid_true_for_expected_edges() -> None:
    assert frame_pair_valid(FRAME_ODOM, FRAME_BASE_LINK)
    assert frame_pair_valid(FRAME_BASE_LINK, FRAME_IMU)
    assert frame_pair_valid(FRAME_MAP, FRAME_ODOM)


def test_frame_pair_valid_normalizes_inputs() -> None:
    assert frame_pair_valid("/odom", " /base_link ")
    assert frame_pair_valid(" /base_link ", "/imu_link")


def test_frame_pair_valid_false_for_empty_parent() -> None:
    assert not frame_pair_valid("", FRAME_BASE_LINK)


def test_frame_pair_valid_false_for_empty_child() -> None:
    assert not frame_pair_valid(FRAME_ODOM, "")


def test_frame_pair_valid_false_for_same_frame() -> None:
    assert not frame_pair_valid(FRAME_BASE_LINK, FRAME_BASE_LINK)
    assert not frame_pair_valid("/odom", "odom")


def test_require_frame_pair_returns_normalized_pair() -> None:
    parent, child = require_frame_pair(" /odom ", "/base_link")

    assert parent == FRAME_ODOM
    assert child == FRAME_BASE_LINK


def test_require_frame_pair_rejects_empty_parent() -> None:
    with pytest.raises(ValueError):
        require_frame_pair("", FRAME_BASE_LINK)


def test_require_frame_pair_rejects_empty_child() -> None:
    with pytest.raises(ValueError):
        require_frame_pair(FRAME_ODOM, "")


def test_require_frame_pair_rejects_same_frame() -> None:
    with pytest.raises(ValueError):
        require_frame_pair(FRAME_BASE_LINK, "/base_link")


def test_make_tf_edge_label_normalizes_frames() -> None:
    assert make_tf_edge_label("/odom", " /base_link ") == "odom -> base_link"
    assert make_tf_edge_label("/base_link", "/imu_link") == "base_link -> imu_link"


def test_robot_savo_localization_frame_chain() -> None:
    assert make_tf_edge_label(FRAME_MAP, FRAME_ODOM) == "map -> odom"
    assert make_tf_edge_label(FRAME_ODOM, FRAME_BASE_LINK) == "odom -> base_link"
    assert make_tf_edge_label(FRAME_BASE_LINK, FRAME_IMU) == "base_link -> imu_link"