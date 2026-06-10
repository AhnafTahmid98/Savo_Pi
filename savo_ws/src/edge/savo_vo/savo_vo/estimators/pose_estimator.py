"""Pose estimation helpers for RGB-D visual odometry."""

from dataclasses import dataclass

import cv2
import numpy as np

from savo_vo.adapters.camera_info_adapter import CameraIntrinsics


@dataclass(frozen=True)
class PoseEstimate:
    success: bool
    rotation_vector: np.ndarray
    translation_vector: np.ndarray
    inlier_mask: np.ndarray
    message: str = ""

    @property
    def inlier_count(self) -> int:
        if self.inlier_mask is None:
            return 0
        return int(np.count_nonzero(self.inlier_mask))

    @classmethod
    def failed(cls, message: str) -> "PoseEstimate":
        return cls(
            success=False,
            rotation_vector=np.zeros((3, 1), dtype=np.float64),
            translation_vector=np.zeros((3, 1), dtype=np.float64),
            inlier_mask=np.empty((0, 1), dtype=np.uint8),
            message=message,
        )


def pixel_depth_to_point_3d(
    u_px: float,
    v_px: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    x_m = (u_px - intrinsics.cx) * depth_m / intrinsics.fx
    y_m = (v_px - intrinsics.cy) * depth_m / intrinsics.fy
    return x_m, y_m, depth_m


def build_3d_2d_correspondences(
    previous_points_px: np.ndarray,
    current_points_px: np.ndarray,
    previous_depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    min_depth_m: float = 0.10,
    max_depth_m: float = 5.00,
) -> tuple[np.ndarray, np.ndarray]:
    object_points = []
    image_points = []

    height, width = previous_depth_m.shape[:2]

    for previous_point, current_point in zip(previous_points_px, current_points_px):
        u_prev = int(round(float(previous_point[0])))
        v_prev = int(round(float(previous_point[1])))

        if u_prev < 0 or v_prev < 0 or u_prev >= width or v_prev >= height:
            continue

        depth_m = float(previous_depth_m[v_prev, u_prev])

        if not np.isfinite(depth_m):
            continue

        if depth_m < min_depth_m or depth_m > max_depth_m:
            continue

        object_points.append(
            pixel_depth_to_point_3d(
                u_px=float(previous_point[0]),
                v_px=float(previous_point[1]),
                depth_m=depth_m,
                intrinsics=intrinsics,
            )
        )
        image_points.append(
            [
                float(current_point[0]),
                float(current_point[1]),
            ]
        )

    if not object_points:
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 2), dtype=np.float32),
        )

    return (
        np.asarray(object_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
    )


def estimate_pose_pnp(
    object_points: np.ndarray,
    image_points: np.ndarray,
    intrinsics: CameraIntrinsics,
    min_points: int = 8,
    reprojection_error_px: float = 5.0,
    confidence: float = 0.99,
    iterations_count: int = 100,
) -> PoseEstimate:
    if object_points is None or image_points is None:
        return PoseEstimate.failed("pose estimation input is missing")

    if len(object_points) < min_points or len(image_points) < min_points:
        return PoseEstimate.failed(
            f"not enough correspondences for PnP: {len(object_points)}"
        )

    camera_matrix = np.array(
        [
            [intrinsics.fx, 0.0, intrinsics.cx],
            [0.0, intrinsics.fy, intrinsics.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.zeros((4, 1), dtype=np.float64)

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points.astype(np.float64),
        image_points.astype(np.float64),
        camera_matrix,
        distortion,
        iterationsCount=iterations_count,
        reprojectionError=reprojection_error_px,
        confidence=confidence,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success or inliers is None:
        return PoseEstimate.failed("PnP pose estimation failed")

    return PoseEstimate(
        success=True,
        rotation_vector=rvec,
        translation_vector=tvec,
        inlier_mask=inliers,
        message="PnP pose estimation succeeded",
    )


def rotation_vector_to_yaw(rotation_vector: np.ndarray) -> float:
    rotation_matrix, _jacobian = cv2.Rodrigues(rotation_vector)
    return float(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))