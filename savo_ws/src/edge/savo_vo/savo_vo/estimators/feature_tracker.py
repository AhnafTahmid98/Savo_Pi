"""Feature detection and tracking helpers for RGB-D visual odometry."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FeatureTrackingResult:
    previous_points: np.ndarray
    current_points: np.ndarray
    status: np.ndarray

    @property
    def matched_count(self) -> int:
        return int(len(self.current_points))

    @property
    def has_matches(self) -> bool:
        return self.matched_count > 0


def detect_features(
    gray_image: np.ndarray,
    max_features: int = 800,
    quality_level: float = 0.01,
    min_distance_px: float = 8.0,
) -> np.ndarray:
    if gray_image is None:
        return np.empty((0, 1, 2), dtype=np.float32)

    features = cv2.goodFeaturesToTrack(
        gray_image,
        maxCorners=max_features,
        qualityLevel=quality_level,
        minDistance=min_distance_px,
        blockSize=7,
    )

    if features is None:
        return np.empty((0, 1, 2), dtype=np.float32)

    return features.astype(np.float32)


def track_features(
    previous_gray: np.ndarray,
    current_gray: np.ndarray,
    previous_points: np.ndarray,
) -> FeatureTrackingResult:
    if previous_points is None or len(previous_points) == 0:
        empty = np.empty((0, 2), dtype=np.float32)
        return FeatureTrackingResult(
            previous_points=empty,
            current_points=empty,
            status=np.empty((0,), dtype=np.uint8),
        )

    current_points, status, _error = cv2.calcOpticalFlowPyrLK(
        previous_gray,
        current_gray,
        previous_points,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            30,
            0.01,
        ),
    )

    if current_points is None or status is None:
        empty = np.empty((0, 2), dtype=np.float32)
        return FeatureTrackingResult(
            previous_points=empty,
            current_points=empty,
            status=np.empty((0,), dtype=np.uint8),
        )

    good_mask = status.reshape(-1) == 1

    return FeatureTrackingResult(
        previous_points=previous_points.reshape(-1, 2)[good_mask],
        current_points=current_points.reshape(-1, 2)[good_mask],
        status=status.reshape(-1)[good_mask],
    )


def count_features(points: np.ndarray | None) -> int:
    if points is None:
        return 0

    return int(len(points))