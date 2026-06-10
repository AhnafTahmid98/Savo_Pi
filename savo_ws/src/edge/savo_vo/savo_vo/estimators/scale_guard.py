"""Scale and translation sanity checks for RGB-D visual odometry."""

from dataclasses import dataclass
from math import sqrt

import numpy as np


@dataclass(frozen=True)
class ScaleGuardResult:
    is_valid: bool
    translation_norm_m: float
    message: str = ""


def translation_norm_m(translation_vector: np.ndarray) -> float:
    vector = np.asarray(translation_vector, dtype=np.float64).reshape(-1)

    if vector.size < 3:
        raise ValueError("translation_vector must contain at least 3 values")

    return sqrt(
        float(vector[0]) * float(vector[0])
        + float(vector[1]) * float(vector[1])
        + float(vector[2]) * float(vector[2])
    )


def check_translation_scale(
    translation_vector: np.ndarray,
    max_translation_m: float,
) -> ScaleGuardResult:
    if max_translation_m < 0.0:
        raise ValueError("max_translation_m must be non-negative")

    norm_m = translation_norm_m(translation_vector)

    if not np.isfinite(norm_m):
        return ScaleGuardResult(
            is_valid=False,
            translation_norm_m=0.0,
            message="translation scale is not finite",
        )

    if norm_m > max_translation_m:
        return ScaleGuardResult(
            is_valid=False,
            translation_norm_m=norm_m,
            message=(
                "translation scale rejected: "
                f"{norm_m:.3f}m exceeds limit {max_translation_m:.3f}m"
            ),
        )

    return ScaleGuardResult(
        is_valid=True,
        translation_norm_m=norm_m,
        message="translation scale is valid",
    )


def is_depth_distribution_usable(
    depth_values_m: np.ndarray,
    min_valid_count: int = 30,
    min_depth_m: float = 0.10,
    max_depth_m: float = 5.00,
) -> bool:
    if min_valid_count <= 0:
        raise ValueError("min_valid_count must be positive")

    if depth_values_m is None:
        return False

    depth = np.asarray(depth_values_m, dtype=np.float32).reshape(-1)
    valid = depth[
        np.isfinite(depth)
        & (depth >= min_depth_m)
        & (depth <= max_depth_m)
    ]

    return int(valid.size) >= min_valid_count