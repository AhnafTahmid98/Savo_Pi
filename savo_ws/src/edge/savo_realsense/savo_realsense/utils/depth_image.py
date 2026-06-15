# Copyright 2026 Ahnaf Tahmid
from __future__ import annotations

import math

import numpy as np


SUPPORTED_DEPTH_ENCODINGS = ("16UC1", "32FC1")


def depth_image_to_meters(
    image: np.ndarray,
    encoding: str,
    depth_scale: float = 0.001,
) -> np.ndarray:
    if encoding == "16UC1":
        return image.astype(np.float32) * depth_scale

    if encoding == "32FC1":
        return image.astype(np.float32)

    raise ValueError(f"Unsupported depth encoding: {encoding}")


def crop_roi(
    image: np.ndarray,
    x_min_ratio: float,
    x_max_ratio: float,
    y_min_ratio: float,
    y_max_ratio: float,
) -> np.ndarray:
    if image.ndim != 2:
        raise ValueError("Depth image must be single-channel")

    height, width = image.shape

    x0 = _ratio_to_index(x_min_ratio, width)
    x1 = _ratio_to_index(x_max_ratio, width)
    y0 = _ratio_to_index(y_min_ratio, height)
    y1 = _ratio_to_index(y_max_ratio, height)

    if x1 <= x0 or y1 <= y0:
        raise ValueError("Invalid ROI bounds")

    return image[y0:y1, x0:x1]


def valid_depth_values(
    depth_m: np.ndarray,
    min_valid_m: float,
    max_valid_m: float,
) -> np.ndarray:
    if min_valid_m <= 0.0:
        raise ValueError("min_valid_m must be positive")
    if max_valid_m <= min_valid_m:
        raise ValueError("max_valid_m must be greater than min_valid_m")

    return depth_m[
        np.isfinite(depth_m)
        & (depth_m >= min_valid_m)
        & (depth_m <= max_valid_m)
    ]


def percentile_depth_m(
    depth_m: np.ndarray,
    percentile: float,
    min_valid_m: float,
    max_valid_m: float,
) -> float:
    values = valid_depth_values(depth_m, min_valid_m, max_valid_m)
    if values.size == 0:
        return math.nan

    return float(np.percentile(values, percentile))


def front_roi_depth_m(
    image: np.ndarray,
    encoding: str,
    x_min_ratio: float,
    x_max_ratio: float,
    y_min_ratio: float,
    y_max_ratio: float,
    percentile: float,
    min_valid_m: float,
    max_valid_m: float,
    depth_scale: float = 0.001,
) -> float:
    depth_m = depth_image_to_meters(image, encoding, depth_scale)
    roi = crop_roi(depth_m, x_min_ratio, x_max_ratio, y_min_ratio, y_max_ratio)

    return percentile_depth_m(
        roi,
        percentile=percentile,
        min_valid_m=min_valid_m,
        max_valid_m=max_valid_m,
    )


def _ratio_to_index(ratio: float, size: int) -> int:
    clamped = min(1.0, max(0.0, ratio))
    return int(round(clamped * size))
