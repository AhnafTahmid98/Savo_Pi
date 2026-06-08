import math

import numpy as np
import pytest

from savo_realsense.utils.depth_image import (
    crop_roi,
    depth_image_to_meters,
    front_roi_depth_m,
    percentile_depth_m,
    valid_depth_values,
)


def test_depth_image_to_meters_converts_16uc1_mm_to_meters() -> None:
    image = np.array([[1000, 1500], [2000, 0]], dtype=np.uint16)

    depth_m = depth_image_to_meters(image, "16UC1")

    assert np.allclose(depth_m, np.array([[1.0, 1.5], [2.0, 0.0]], dtype=np.float32))


def test_depth_image_to_meters_keeps_32fc1_as_meters() -> None:
    image = np.array([[0.5, 1.25], [2.0, math.nan]], dtype=np.float32)

    depth_m = depth_image_to_meters(image, "32FC1")

    assert np.allclose(depth_m[:1, :], image[:1, :], equal_nan=True)


def test_depth_image_to_meters_rejects_unknown_encoding() -> None:
    image = np.zeros((2, 2), dtype=np.uint16)

    with pytest.raises(ValueError):
        depth_image_to_meters(image, "mono16")


def test_crop_roi_returns_expected_region() -> None:
    image = np.arange(16, dtype=np.float32).reshape((4, 4))

    roi = crop_roi(
        image,
        x_min_ratio=0.25,
        x_max_ratio=0.75,
        y_min_ratio=0.25,
        y_max_ratio=0.75,
    )

    assert roi.shape == (2, 2)
    assert np.array_equal(roi, np.array([[5, 6], [9, 10]], dtype=np.float32))


def test_crop_roi_rejects_non_depth_image() -> None:
    image = np.zeros((4, 4, 3), dtype=np.float32)

    with pytest.raises(ValueError):
        crop_roi(image, 0.0, 1.0, 0.0, 1.0)


def test_valid_depth_values_filters_nan_zero_and_far_values() -> None:
    depth_m = np.array([0.0, 0.1, 1.0, 5.0, math.nan], dtype=np.float32)

    values = valid_depth_values(depth_m, min_valid_m=0.02, max_valid_m=3.0)

    assert np.allclose(values, np.array([0.1, 1.0], dtype=np.float32))


def test_percentile_depth_returns_nan_when_no_valid_values() -> None:
    depth_m = np.array([0.0, math.nan, 5.0], dtype=np.float32)

    distance_m = percentile_depth_m(
        depth_m,
        percentile=10.0,
        min_valid_m=0.02,
        max_valid_m=3.0,
    )

    assert math.isnan(distance_m)


def test_front_roi_depth_m_returns_percentile_depth() -> None:
    image = np.array(
        [
            [1000, 1000, 1000, 1000],
            [1000, 500, 600, 1000],
            [1000, 700, 800, 1000],
            [1000, 1000, 1000, 1000],
        ],
        dtype=np.uint16,
    )

    distance_m = front_roi_depth_m(
        image=image,
        encoding="16UC1",
        x_min_ratio=0.25,
        x_max_ratio=0.75,
        y_min_ratio=0.25,
        y_max_ratio=0.75,
        percentile=50.0,
        min_valid_m=0.02,
        max_valid_m=3.0,
    )

    assert math.isclose(distance_m, 0.65, rel_tol=1e-6)