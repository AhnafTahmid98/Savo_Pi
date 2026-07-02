# -*- coding: utf-8 -*-

"""Semantic confirmation models for AprilTag-based known locations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Optional, Tuple

from savo_head.constants import (
    APRILTAG_FAMILY_DEFAULT,
    APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT,
    APRILTAG_MIN_STABLE_FRAMES_DEFAULT,
    FRAME_BASE,
    FRAME_PI_CAMERA_OPTICAL,
    SEMANTIC_CONFIRMATION_SOURCE,
    SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION,
    SEMANTIC_CONFIRMATION_TYPE_SUMMON_POINT,
)


CONFIRMATION_PENDING: Final[str] = "pending"
CONFIRMATION_CONFIRMED: Final[str] = "confirmed"
CONFIRMATION_REJECTED: Final[str] = "rejected"

VALID_CONFIRMATION_STATES: Final[tuple[str, ...]] = (
    CONFIRMATION_PENDING,
    CONFIRMATION_CONFIRMED,
    CONFIRMATION_REJECTED,
)

VALID_CONFIRMATION_TYPES: Final[tuple[str, ...]] = (
    SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION,
    SEMANTIC_CONFIRMATION_TYPE_SUMMON_POINT,
)


@dataclass(frozen=True)
class TagRegistration:
    tag_id: int
    label: str
    confirmation_type: str = SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION
    enabled: bool = True
    save_as_summon_point: bool = False
    aliases: Tuple[str, ...] = ()

    def normalized(self) -> "TagRegistration":
        confirmation_type = (
            self.confirmation_type
            if self.confirmation_type in VALID_CONFIRMATION_TYPES
            else SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION
        )

        return TagRegistration(
            tag_id=int(self.tag_id),
            label=str(self.label).strip(),
            confirmation_type=confirmation_type,
            enabled=bool(self.enabled),
            save_as_summon_point=bool(self.save_as_summon_point),
            aliases=tuple(str(v).strip() for v in self.aliases if str(v).strip()),
        )

    def validation_errors(self) -> list[str]:
        errors: list[str] = []

        if int(self.tag_id) < 0:
            errors.append("tag_id must be non-negative")

        if not str(self.label).strip():
            errors.append("label must not be empty")

        if self.confirmation_type not in VALID_CONFIRMATION_TYPES:
            errors.append(f"invalid confirmation_type: {self.confirmation_type!r}")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()

    def to_dict(self) -> dict:
        item = self.normalized()
        return {
            "tag_id": item.tag_id,
            "label": item.label,
            "confirmation_type": item.confirmation_type,
            "enabled": item.enabled,
            "save_as_summon_point": item.save_as_summon_point,
            "aliases": list(item.aliases),
        }


@dataclass(frozen=True)
class AprilTagObservation:
    tag_id: int
    family: str = APRILTAG_FAMILY_DEFAULT
    confidence: float = 0.0
    distance_m: float = 0.0
    stable_frames: int = 0
    stamp_s: float = 0.0
    frame_id: str = FRAME_PI_CAMERA_OPTICAL
    pose_camera_xyz_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    pose_camera_rpy_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def normalized(self) -> "AprilTagObservation":
        return AprilTagObservation(
            tag_id=int(self.tag_id),
            family=str(self.family).strip() or APRILTAG_FAMILY_DEFAULT,
            confidence=max(0.0, min(1.0, float(self.confidence))),
            distance_m=max(0.0, float(self.distance_m)),
            stable_frames=max(0, int(self.stable_frames)),
            stamp_s=float(self.stamp_s),
            frame_id=str(self.frame_id).strip() or FRAME_PI_CAMERA_OPTICAL,
            pose_camera_xyz_m=tuple(float(v) for v in self.pose_camera_xyz_m[:3]),
            pose_camera_rpy_rad=tuple(float(v) for v in self.pose_camera_rpy_rad[:3]),
        )

    def validation_errors(self) -> list[str]:
        errors: list[str] = []

        if int(self.tag_id) < 0:
            errors.append("tag_id must be non-negative")

        if not 0.0 <= float(self.confidence) <= 1.0:
            errors.append("confidence must be in range 0..1")

        if float(self.distance_m) < 0.0:
            errors.append("distance_m must be non-negative")

        if int(self.stable_frames) < 0:
            errors.append("stable_frames must be non-negative")

        if len(self.pose_camera_xyz_m) != 3:
            errors.append("pose_camera_xyz_m must have 3 values")

        if len(self.pose_camera_rpy_rad) != 3:
            errors.append("pose_camera_rpy_rad must have 3 values")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()

    def age_s(self, now_s: float) -> float:
        if self.stamp_s <= 0.0:
            return float("inf")
        return max(0.0, float(now_s) - float(self.stamp_s))

    def to_dict(self) -> dict:
        item = self.normalized()
        return {
            "tag_id": item.tag_id,
            "family": item.family,
            "confidence": item.confidence,
            "distance_m": item.distance_m,
            "stable_frames": item.stable_frames,
            "stamp_s": item.stamp_s,
            "frame_id": item.frame_id,
            "pose_camera_xyz_m": list(item.pose_camera_xyz_m),
            "pose_camera_rpy_rad": list(item.pose_camera_rpy_rad),
        }


@dataclass(frozen=True)
class RobotPoseSnapshot:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    frame_id: str = "map"
    base_frame: str = FRAME_BASE
    stamp_s: float = 0.0

    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0

    pose_covariance_xy: float = 0.0
    yaw_covariance: float = 0.0

    localization_ok: bool = True
    lidar_map_pose_ok: bool = True
    tf_ok: bool = True

    def stationary(self, max_linear_mps: float, max_angular_radps: float) -> bool:
        return (
            abs(float(self.linear_speed_mps)) <= float(max_linear_mps)
            and abs(float(self.angular_speed_radps)) <= float(max_angular_radps)
        )

    def covariance_ok(self, max_xy: float, max_yaw: float) -> bool:
        return (
            float(self.pose_covariance_xy) <= float(max_xy)
            and float(self.yaw_covariance) <= float(max_yaw)
        )

    def to_dict(self) -> dict:
        return {
            "x_m": float(self.x_m),
            "y_m": float(self.y_m),
            "yaw_rad": float(self.yaw_rad),
            "frame_id": self.frame_id,
            "base_frame": self.base_frame,
            "stamp_s": float(self.stamp_s),
            "linear_speed_mps": float(self.linear_speed_mps),
            "angular_speed_radps": float(self.angular_speed_radps),
            "pose_covariance_xy": float(self.pose_covariance_xy),
            "yaw_covariance": float(self.yaw_covariance),
            "localization_ok": bool(self.localization_ok),
            "lidar_map_pose_ok": bool(self.lidar_map_pose_ok),
            "tf_ok": bool(self.tf_ok),
        }


@dataclass(frozen=True)
class SemanticConfirmationPolicy:
    min_stable_frames: int = APRILTAG_MIN_STABLE_FRAMES_DEFAULT
    min_detection_confidence: float = 0.70
    max_detection_distance_m: float = APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT
    max_detection_age_s: float = 0.50

    require_tf_available: bool = True
    require_robot_stationary: bool = True
    max_robot_linear_speed_mps: float = 0.03
    max_robot_angular_speed_radps: float = 0.05

    require_localization_ok: bool = True
    max_pose_covariance_xy: float = 0.25
    max_yaw_covariance: float = 0.20

    require_lidar_map_pose: bool = True
    require_semantic_label: bool = True

    def rejection_reasons(
        self,
        observation: AprilTagObservation,
        robot_pose: RobotPoseSnapshot,
        registration: Optional[TagRegistration],
        now_s: float,
    ) -> list[str]:
        obs = observation.normalized()
        reasons: list[str] = []

        if not obs.is_valid():
            reasons.extend(obs.validation_errors())

        if registration is None:
            reasons.append("tag_not_registered")
        else:
            reg = registration.normalized()
            if not reg.enabled:
                reasons.append("tag_disabled")
            if reg.tag_id != obs.tag_id:
                reasons.append("tag_id_mismatch")
            if self.require_semantic_label and not reg.label:
                reasons.append("missing_semantic_label")
            if not reg.is_valid():
                reasons.extend(reg.validation_errors())

        if obs.stable_frames < int(self.min_stable_frames):
            reasons.append("not_enough_stable_frames")

        if obs.confidence < float(self.min_detection_confidence):
            reasons.append("low_detection_confidence")

        if obs.distance_m > float(self.max_detection_distance_m):
            reasons.append("detection_too_far")

        if obs.age_s(now_s) > float(self.max_detection_age_s):
            reasons.append("detection_stale")

        if self.require_tf_available and not robot_pose.tf_ok:
            reasons.append("tf_unavailable")

        if self.require_robot_stationary and not robot_pose.stationary(
            self.max_robot_linear_speed_mps,
            self.max_robot_angular_speed_radps,
        ):
            reasons.append("robot_not_stationary")

        if self.require_localization_ok:
            if not robot_pose.localization_ok:
                reasons.append("localization_not_ok")
            if not robot_pose.covariance_ok(self.max_pose_covariance_xy, self.max_yaw_covariance):
                reasons.append("localization_covariance_high")

        if self.require_lidar_map_pose and not robot_pose.lidar_map_pose_ok:
            reasons.append("lidar_map_pose_not_ok")

        return reasons

    def can_confirm(
        self,
        observation: AprilTagObservation,
        robot_pose: RobotPoseSnapshot,
        registration: Optional[TagRegistration],
        now_s: float,
    ) -> bool:
        return not self.rejection_reasons(observation, robot_pose, registration, now_s)


@dataclass(frozen=True)
class SemanticConfirmation:
    tag_id: int
    label: str
    confirmation_type: str = SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION
    state: str = CONFIRMATION_CONFIRMED

    source: str = SEMANTIC_CONFIRMATION_SOURCE
    stamp_s: float = 0.0

    map_frame: str = "map"
    base_frame: str = FRAME_BASE
    camera_optical_frame: str = FRAME_PI_CAMERA_OPTICAL

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    confidence: float = 0.0
    distance_m: float = 0.0
    stable_frames: int = 0

    save_as_summon_point: bool = False
    reason: str = ""
    aliases: Tuple[str, ...] = ()

    def normalized(self) -> "SemanticConfirmation":
        confirmation_type = (
            self.confirmation_type
            if self.confirmation_type in VALID_CONFIRMATION_TYPES
            else SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION
        )

        state = self.state if self.state in VALID_CONFIRMATION_STATES else CONFIRMATION_REJECTED

        return SemanticConfirmation(
            tag_id=int(self.tag_id),
            label=str(self.label).strip(),
            confirmation_type=confirmation_type,
            state=state,
            source=str(self.source).strip() or SEMANTIC_CONFIRMATION_SOURCE,
            stamp_s=float(self.stamp_s),
            map_frame=str(self.map_frame).strip() or "map",
            base_frame=str(self.base_frame).strip() or FRAME_BASE,
            camera_optical_frame=str(self.camera_optical_frame).strip() or FRAME_PI_CAMERA_OPTICAL,
            x_m=float(self.x_m),
            y_m=float(self.y_m),
            yaw_rad=float(self.yaw_rad),
            confidence=max(0.0, min(1.0, float(self.confidence))),
            distance_m=max(0.0, float(self.distance_m)),
            stable_frames=max(0, int(self.stable_frames)),
            save_as_summon_point=bool(self.save_as_summon_point),
            reason=str(self.reason),
            aliases=tuple(str(v).strip() for v in self.aliases if str(v).strip()),
        )

    def validation_errors(self) -> list[str]:
        item = self.normalized()
        errors: list[str] = []

        if item.tag_id < 0:
            errors.append("tag_id must be non-negative")

        if not item.label:
            errors.append("label must not be empty")

        if item.confirmation_type not in VALID_CONFIRMATION_TYPES:
            errors.append(f"invalid confirmation_type: {item.confirmation_type!r}")

        if item.state not in VALID_CONFIRMATION_STATES:
            errors.append(f"invalid state: {item.state!r}")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()

    def to_dict(self) -> dict:
        item = self.normalized()
        return {
            "tag_id": item.tag_id,
            "label": item.label,
            "confirmation_type": item.confirmation_type,
            "state": item.state,
            "source": item.source,
            "stamp_s": item.stamp_s,
            "frames": {
                "map": item.map_frame,
                "base": item.base_frame,
                "camera_optical": item.camera_optical_frame,
            },
            "pose": {
                "x_m": item.x_m,
                "y_m": item.y_m,
                "yaw_rad": item.yaw_rad,
            },
            "observation": {
                "confidence": item.confidence,
                "distance_m": item.distance_m,
                "stable_frames": item.stable_frames,
            },
            "save_as_summon_point": item.save_as_summon_point,
            "reason": item.reason,
            "aliases": list(item.aliases),
        }


def make_confirmation(
    observation: AprilTagObservation,
    robot_pose: RobotPoseSnapshot,
    registration: TagRegistration,
    *,
    stamp_s: float,
    reason: str = "apriltag_confirmed",
) -> SemanticConfirmation:
    obs = observation.normalized()
    reg = registration.normalized()

    return SemanticConfirmation(
        tag_id=obs.tag_id,
        label=reg.label,
        confirmation_type=reg.confirmation_type,
        state=CONFIRMATION_CONFIRMED,
        source=SEMANTIC_CONFIRMATION_SOURCE,
        stamp_s=float(stamp_s),
        map_frame=robot_pose.frame_id,
        base_frame=robot_pose.base_frame,
        camera_optical_frame=obs.frame_id,
        x_m=robot_pose.x_m,
        y_m=robot_pose.y_m,
        yaw_rad=robot_pose.yaw_rad,
        confidence=obs.confidence,
        distance_m=obs.distance_m,
        stable_frames=obs.stable_frames,
        save_as_summon_point=reg.save_as_summon_point,
        reason=reason,
        aliases=reg.aliases,
    )


def make_rejection(
    observation: AprilTagObservation,
    *,
    stamp_s: float,
    reasons: list[str],
) -> SemanticConfirmation:
    obs = observation.normalized()

    return SemanticConfirmation(
        tag_id=obs.tag_id,
        label="",
        confirmation_type=SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION,
        state=CONFIRMATION_REJECTED,
        source=SEMANTIC_CONFIRMATION_SOURCE,
        stamp_s=float(stamp_s),
        camera_optical_frame=obs.frame_id,
        confidence=obs.confidence,
        distance_m=obs.distance_m,
        stable_frames=obs.stable_frames,
        reason=",".join(str(v) for v in reasons),
    )


__all__ = [
    "CONFIRMATION_PENDING",
    "CONFIRMATION_CONFIRMED",
    "CONFIRMATION_REJECTED",
    "VALID_CONFIRMATION_STATES",
    "VALID_CONFIRMATION_TYPES",
    "TagRegistration",
    "AprilTagObservation",
    "RobotPoseSnapshot",
    "SemanticConfirmationPolicy",
    "SemanticConfirmation",
    "make_confirmation",
    "make_rejection",
]
