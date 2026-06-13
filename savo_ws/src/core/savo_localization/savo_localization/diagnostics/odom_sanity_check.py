#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Odometry sanity checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)
from savo_localization.math.angle_math import shortest_angle_delta_rad
from savo_localization.math.odom_math import (
    Pose2D,
    Twist2D,
    distance_2d_m,
    pose_is_finite,
    twist_is_finite,
)
from savo_localization.models.wheel_odom_state import (
    WheelOdomSample,
    WheelOdomState,
)
from savo_localization.utils.frames import require_frame_pair


@dataclass(frozen=True)
class OdomSanityCheckResult:
    status: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)

    linear_speed_mps: float = 0.0
    angular_speed_rad_s: float = 0.0
    pose_finite: bool = True
    twist_finite: bool = True
    frame_ok: bool = True
    dt_ok: bool = True

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class OdomJumpCheckResult:
    status: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)

    distance_jump_m: float = 0.0
    yaw_jump_rad: float = 0.0
    dt_s: float = 0.0

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def check_pose_twist(
    pose: Pose2D,
    twist: Twist2D,
    *,
    dt_s: float,
    odom_frame_id: str = FRAME_ODOM,
    base_frame_id: str = FRAME_BASE_LINK,
    require_frame_ids: bool = True,
    require_frames: bool | None = None,
    max_linear_mps: float = DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    max_angular_rad_s: float = DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    max_linear_speed_mps: float | None = None,
    max_angular_speed_rad_s: float | None = None,
) -> OdomSanityCheckResult:
    if require_frames is not None:
        require_frame_ids = require_frames

    if max_linear_speed_mps is not None:
        max_linear_mps = max_linear_speed_mps

    if max_angular_speed_rad_s is not None:
        max_angular_rad_s = max_angular_speed_rad_s

    if max_linear_mps <= 0.0:
        raise ValueError("max_linear_mps must be > 0.0")

    if max_angular_rad_s <= 0.0:
        raise ValueError("max_angular_rad_s must be > 0.0")

    linear_speed = math.hypot(float(twist.vx_mps), float(twist.vy_mps))
    angular_speed = abs(float(twist.omega_rad_s))

    if dt_s <= 0.0:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry dt invalid",
            reasons=["dt_s must be > 0.0"],
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
            pose_finite=pose_is_finite(pose),
            twist_finite=twist_is_finite(twist),
            frame_ok=True,
            dt_ok=False,
        )

    reasons: list[str] = []
    pose_finite = pose_is_finite(pose)
    twist_finite = twist_is_finite(twist)

    if not pose_finite:
        reasons.append("pose contains non-finite values")

    if not twist_finite:
        reasons.append("twist contains non-finite values")

    frame_ok = True
    if require_frame_ids:
        try:
            require_frame_pair(odom_frame_id, base_frame_id)
        except ValueError:
            frame_ok = False
            reasons.append("odom/base frame ids are invalid")

    if linear_speed > max_linear_mps:
        reasons.append(
            f"linear speed too high: {linear_speed:.3f} > {max_linear_mps:.3f}"
        )

    if angular_speed > max_angular_rad_s:
        reasons.append(
            f"angular speed too high: {angular_speed:.3f} > {max_angular_rad_s:.3f}"
        )

    hard_error = (
        not pose_finite
        or not twist_finite
    )

    if hard_error:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry pose/twist invalid",
            reasons=_unique_preserve_order(reasons),
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
            pose_finite=pose_finite,
            twist_finite=twist_finite,
            frame_ok=frame_ok,
            dt_ok=True,
        )

    if reasons:
        return OdomSanityCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry pose/twist usable with notes",
            reasons=_unique_preserve_order(reasons),
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
            pose_finite=pose_finite,
            twist_finite=twist_finite,
            frame_ok=frame_ok,
            dt_ok=True,
        )

    return OdomSanityCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry pose/twist healthy",
        reasons=[],
        linear_speed_mps=linear_speed,
        angular_speed_rad_s=angular_speed,
        pose_finite=pose_finite,
        twist_finite=twist_finite,
        frame_ok=frame_ok,
        dt_ok=True,
    )


def check_wheel_odom_sample(
    sample: WheelOdomSample,
    **kwargs,
) -> OdomSanityCheckResult:
    pose_kwargs = _pose_twist_kwargs(kwargs)

    return check_pose_twist(
        sample.pose,
        sample.twist,
        dt_s=sample.dt_s,
        odom_frame_id=sample.odom_frame_id,
        base_frame_id=sample.base_frame_id,
        **pose_kwargs,
    )


def check_wheel_odom_state(
    state: WheelOdomState,
    *,
    stale_timeout_s: float = 0.5,
    **kwargs,
) -> OdomSanityCheckResult:
    if state.last_sample is None:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="wheel odometry state has no sample",
            reasons=["no sample available"],
        )

    if state.last_sample_age_s is not None and state.last_sample_age_s > stale_timeout_s:
        return OdomSanityCheckResult(
            status=STATUS_STALE,
            ok=False,
            message="wheel odometry sample stale",
            reasons=[
                f"sample stale: age_s={state.last_sample_age_s:.3f} "
                f"> timeout_s={stale_timeout_s:.3f}"
            ],
        )

    return check_wheel_odom_sample(state.last_sample, **kwargs)


def check_odom_jump(
    previous: WheelOdomSample | None = None,
    current: WheelOdomSample | None = None,
    *,
    previous_pose: Pose2D | None = None,
    current_pose: Pose2D | None = None,
    dt_s: float | None = None,
    max_distance_jump_m: float = 0.50,
    max_yaw_jump_rad: float = 1.00,
) -> OdomJumpCheckResult:
    if max_distance_jump_m <= 0.0:
        raise ValueError("max_distance_jump_m must be > 0.0")

    if max_yaw_jump_rad <= 0.0:
        raise ValueError("max_yaw_jump_rad must be > 0.0")

    if previous is not None:
        previous_pose = previous.pose

    if current is not None:
        current_pose = current.pose

    if previous is not None and current is not None and dt_s is None:
        dt_s = float(current.stamp_s) - float(previous.stamp_s)

    if previous_pose is None or current_pose is None:
        raise ValueError("previous/current sample or pose must be provided")

    if dt_s is None:
        raise ValueError("dt_s must be provided")

    dt_s = float(dt_s)

    if dt_s <= 0.0:
        return OdomJumpCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry jump dt invalid",
            reasons=["invalid dt_s: dt_s must be > 0.0"],
            dt_s=dt_s,
        )

    if not pose_is_finite(previous_pose) or not pose_is_finite(current_pose):
        return OdomJumpCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry jump pose invalid",
            reasons=["pose contains non-finite values"],
            dt_s=dt_s,
        )

    distance_jump = distance_2d_m(previous_pose, current_pose)
    yaw_jump = abs(shortest_angle_delta_rad(previous_pose.yaw_rad, current_pose.yaw_rad))

    reasons: list[str] = []

    if distance_jump > max_distance_jump_m:
        reasons.append(
            f"distance jump too large: {distance_jump:.3f} > {max_distance_jump_m:.3f}"
        )

    if yaw_jump > max_yaw_jump_rad:
        reasons.append(
            f"yaw jump too large: {yaw_jump:.3f} > {max_yaw_jump_rad:.3f}"
        )

    if reasons:
        return OdomJumpCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry jump usable with notes",
            reasons=_unique_preserve_order(reasons),
            distance_jump_m=distance_jump,
            yaw_jump_rad=yaw_jump,
            dt_s=dt_s,
        )

    return OdomJumpCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry jump healthy",
        reasons=[],
        distance_jump_m=distance_jump,
        yaw_jump_rad=yaw_jump,
        dt_s=dt_s,
    )


def check_odom_sample_window(
    samples: Iterable[WheelOdomSample],
    **kwargs,
) -> OdomSanityCheckResult:
    sample_list = list(samples)

    if not sample_list:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no odometry samples available",
            reasons=["sample list is empty"],
        )

    pose_kwargs = _pose_twist_kwargs(kwargs)
    jump_kwargs = _jump_kwargs(kwargs)

    latest = check_wheel_odom_sample(sample_list[-1], **pose_kwargs)

    if not latest.ok:
        return latest

    reasons: list[str] = []

    for sample in sample_list:
        result = check_wheel_odom_sample(sample, **pose_kwargs)
        if not result.ok:
            return result
        reasons.extend(result.reasons)

    for previous, current in zip(sample_list, sample_list[1:]):
        jump = check_odom_jump(previous, current, **jump_kwargs)
        reasons.extend(jump.reasons)

    unique_reasons = _unique_preserve_order(reasons)

    if unique_reasons:
        return OdomSanityCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry sample window usable with notes",
            reasons=unique_reasons,
            linear_speed_mps=latest.linear_speed_mps,
            angular_speed_rad_s=latest.angular_speed_rad_s,
            pose_finite=latest.pose_finite,
            twist_finite=latest.twist_finite,
            frame_ok=latest.frame_ok,
            dt_ok=latest.dt_ok,
        )

    return OdomSanityCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry sample window healthy",
        reasons=[],
        linear_speed_mps=latest.linear_speed_mps,
        angular_speed_rad_s=latest.angular_speed_rad_s,
        pose_finite=latest.pose_finite,
        twist_finite=latest.twist_finite,
        frame_ok=latest.frame_ok,
        dt_ok=latest.dt_ok,
    )


def odom_pose_summary(pose: Pose2D) -> dict[str, object]:
    return {
        "x_m": float(pose.x_m),
        "y_m": float(pose.y_m),
        "yaw_rad": float(pose.yaw_rad),
        "finite": pose_is_finite(pose),
    }


def odom_twist_summary(twist: Twist2D) -> dict[str, object]:
    return {
        "vx_mps": float(twist.vx_mps),
        "vy_mps": float(twist.vy_mps),
        "omega_rad_s": float(twist.omega_rad_s),
        "linear_speed_mps": math.hypot(float(twist.vx_mps), float(twist.vy_mps)),
        "angular_speed_rad_s": abs(float(twist.omega_rad_s)),
        "finite": twist_is_finite(twist),
    }


def odom_sample_summary(sample: WheelOdomSample) -> dict[str, object]:
    return {
        "stamp_s": float(sample.stamp_s),
        "dt_s": float(sample.dt_s),
        "odom_frame_id": sample.odom_frame_id,
        "base_frame_id": sample.base_frame_id,
        "pose": odom_pose_summary(sample.pose),
        "twist": odom_twist_summary(sample.twist),
        "active_wheel_count": int(sample.active_wheel_count),
        "encoder_sample_count": int(sample.encoder_sample_count),
        "moving": sample.moving,
        "finite": sample.finite,
    }


def _pose_twist_kwargs(kwargs: dict[str, object]) -> dict[str, object]:
    allowed = {
        "require_frame_ids",
        "require_frames",
        "max_linear_mps",
        "max_angular_rad_s",
        "max_linear_speed_mps",
        "max_angular_speed_rad_s",
    }

    return {
        key: value
        for key, value in kwargs.items()
        if key in allowed
    }


def _jump_kwargs(kwargs: dict[str, object]) -> dict[str, object]:
    allowed = {
        "max_distance_jump_m",
        "max_yaw_jump_rad",
    }

    return {
        key: value
        for key, value in kwargs.items()
        if key in allowed
    }


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result


__all__ = [
    "OdomSanityCheckResult",
    "OdomJumpCheckResult",
    "check_pose_twist",
    "check_wheel_odom_sample",
    "check_wheel_odom_state",
    "check_odom_jump",
    "check_odom_sample_window",
    "odom_pose_summary",
    "odom_twist_summary",
    "odom_sample_summary",
]
