"""RGB-D visual odometry estimator."""

import cv2
import numpy as np

from savo_vo.adapters.camera_info_adapter import CameraIntrinsics
from savo_vo.core.covariance_builder import covariance_for_tracking_quality
from savo_vo.core.motion_quality import normalize_angle_rad
from savo_vo.core.tracking_quality import build_tracking_report
from savo_vo.core.vo_state_machine import VOStateThresholds, evaluate_vo_state
from savo_vo.estimators.base_estimator import BaseVOEstimator
from savo_vo.estimators.feature_tracker import (
    count_features,
    detect_features,
    track_features,
)
from savo_vo.estimators.pose_estimator import (
    build_3d_2d_correspondences,
    estimate_pose_pnp,
    rotation_vector_to_yaw,
)
from savo_vo.estimators.scale_guard import check_translation_scale
from savo_vo.models.vo_sample import VOSample


class RGBDVOEstimator(BaseVOEstimator):
    def __init__(
        self,
        min_features: int = 80,
        good_features_target: int = 300,
        max_features: int = 800,
        min_tracking_quality: float = 0.35,
        min_inliers: int = 8,
        max_translation_jump_m: float = 0.30,
        stale_timeout_s: float = 0.30,
        depth_min_m: float = 0.10,
        depth_max_m: float = 5.00,
    ) -> None:
        self._min_features = min_features
        self._good_features_target = good_features_target
        self._max_features = max_features
        self._min_tracking_quality = min_tracking_quality
        self._min_inliers = min_inliers
        self._max_translation_jump_m = max_translation_jump_m
        self._stale_timeout_s = stale_timeout_s
        self._depth_min_m = depth_min_m
        self._depth_max_m = depth_max_m

        self._previous_timestamp_s: float | None = None
        self._previous_gray: np.ndarray | None = None
        self._previous_depth_m: np.ndarray | None = None
        self._previous_features: np.ndarray | None = None

        self._x_m = 0.0
        self._y_m = 0.0
        self._z_m = 0.0
        self._roll_rad = 0.0
        self._pitch_rad = 0.0
        self._yaw_rad = 0.0

    def reset(self) -> None:
        self._previous_timestamp_s = None
        self._previous_gray = None
        self._previous_depth_m = None
        self._previous_features = None

        self._x_m = 0.0
        self._y_m = 0.0
        self._z_m = 0.0
        self._roll_rad = 0.0
        self._pitch_rad = 0.0
        self._yaw_rad = 0.0

    def process_frame(
        self,
        timestamp_s: float,
        gray_image: object,
        depth_m: object,
        intrinsics: CameraIntrinsics,
    ) -> VOSample | None:
        gray = np.asarray(gray_image)
        depth = np.asarray(depth_m, dtype=np.float32)

        if gray.ndim != 2:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

        features = detect_features(
            gray_image=gray,
            max_features=self._max_features,
        )

        if self._previous_gray is None:
            self._store_reference_frame(
                timestamp_s=timestamp_s,
                gray=gray,
                depth=depth,
                features=features,
            )
            return None

        if self._previous_features is None or count_features(self._previous_features) < self._min_features:
            self._store_reference_frame(
                timestamp_s=timestamp_s,
                gray=gray,
                depth=depth,
                features=features,
            )
            return None

        tracking = track_features(
            previous_gray=self._previous_gray,
            current_gray=gray,
            previous_points=self._previous_features,
        )

        object_points, image_points = build_3d_2d_correspondences(
            previous_points_px=tracking.previous_points,
            current_points_px=tracking.current_points,
            previous_depth_m=self._previous_depth_m,
            intrinsics=intrinsics,
            min_depth_m=self._depth_min_m,
            max_depth_m=self._depth_max_m,
        )

        pose = estimate_pose_pnp(
            object_points=object_points,
            image_points=image_points,
            intrinsics=intrinsics,
            min_points=self._min_inliers,
        )

        tracking_report = build_tracking_report(
            feature_count=count_features(self._previous_features),
            matched_count=tracking.matched_count,
            inlier_count=pose.inlier_count,
            good_features_target=self._good_features_target,
        )

        quality = covariance_for_tracking_quality(tracking_report.tracking_quality)

        dt_s = self._delta_time_s(timestamp_s)
        age_s = 0.0

        status = evaluate_vo_state(
            tracking=tracking_report,
            age_s=age_s,
            thresholds=VOStateThresholds(
                min_tracking_quality=self._min_tracking_quality,
                min_inliers=self._min_inliers,
                stale_timeout_s=self._stale_timeout_s,
            ),
        )

        if not pose.success:
            self._store_reference_frame(
                timestamp_s=timestamp_s,
                gray=gray,
                depth=depth,
                features=features,
            )
            return VOSample(
                timestamp_s=timestamp_s,
                x_m=self._x_m,
                y_m=self._y_m,
                z_m=self._z_m,
                roll_rad=self._roll_rad,
                pitch_rad=self._pitch_rad,
                yaw_rad=self._yaw_rad,
                vx_mps=0.0,
                vy_mps=0.0,
                vz_mps=0.0,
                yaw_rate_radps=0.0,
                tracking=tracking_report,
                quality=quality,
                status=status,
            )

        scale_result = check_translation_scale(
            translation_vector=pose.translation_vector,
            max_translation_m=self._max_translation_jump_m,
        )

        if not scale_result.is_valid:
            self._store_reference_frame(
                timestamp_s=timestamp_s,
                gray=gray,
                depth=depth,
                features=features,
            )
            return VOSample(
                timestamp_s=timestamp_s,
                x_m=self._x_m,
                y_m=self._y_m,
                z_m=self._z_m,
                roll_rad=self._roll_rad,
                pitch_rad=self._pitch_rad,
                yaw_rad=self._yaw_rad,
                vx_mps=0.0,
                vy_mps=0.0,
                vz_mps=0.0,
                yaw_rate_radps=0.0,
                tracking=tracking_report,
                quality=quality,
                status=status,
            )

        dx_camera_m = float(pose.translation_vector[0])
        dy_camera_m = float(pose.translation_vector[1])
        dz_camera_m = float(pose.translation_vector[2])
        dyaw_rad = rotation_vector_to_yaw(pose.rotation_vector)

        previous_x_m = self._x_m
        previous_y_m = self._y_m
        previous_z_m = self._z_m
        previous_yaw_rad = self._yaw_rad

        self._x_m += dz_camera_m
        self._y_m += -dx_camera_m
        self._z_m += -dy_camera_m
        self._yaw_rad = normalize_angle_rad(self._yaw_rad + dyaw_rad)

        vx_mps = (self._x_m - previous_x_m) / dt_s if dt_s > 0.0 else 0.0
        vy_mps = (self._y_m - previous_y_m) / dt_s if dt_s > 0.0 else 0.0
        vz_mps = (self._z_m - previous_z_m) / dt_s if dt_s > 0.0 else 0.0
        yaw_rate_radps = normalize_angle_rad(self._yaw_rad - previous_yaw_rad) / dt_s if dt_s > 0.0 else 0.0

        sample = VOSample(
            timestamp_s=timestamp_s,
            x_m=self._x_m,
            y_m=self._y_m,
            z_m=self._z_m,
            roll_rad=self._roll_rad,
            pitch_rad=self._pitch_rad,
            yaw_rad=self._yaw_rad,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            vz_mps=vz_mps,
            yaw_rate_radps=yaw_rate_radps,
            tracking=tracking_report,
            quality=quality,
            status=status,
        )

        self._store_reference_frame(
            timestamp_s=timestamp_s,
            gray=gray,
            depth=depth,
            features=features,
        )

        return sample

    def _delta_time_s(self, timestamp_s: float) -> float:
        if self._previous_timestamp_s is None:
            return 0.0

        return max(0.0, timestamp_s - self._previous_timestamp_s)

    def _store_reference_frame(
        self,
        timestamp_s: float,
        gray: np.ndarray,
        depth: np.ndarray,
        features: np.ndarray,
    ) -> None:
        self._previous_timestamp_s = timestamp_s
        self._previous_gray = gray
        self._previous_depth_m = depth
        self._previous_features = features