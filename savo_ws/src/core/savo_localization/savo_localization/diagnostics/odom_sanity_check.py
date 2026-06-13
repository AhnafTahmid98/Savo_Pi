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
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.math.angle_math import shortest_angle_delta_rad
from savo_localization.math.odom_math import (
    Pose2D,
    Twist2D,
    distance_2d_m,
    pose_is_finite,
    twist_is_finite,
    twist_magnitude,
)
from savo_localization.models.wheel_odom_state import (
    WheelOdomSample,
    WheelOdomState,
)


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

    max_distance_jump_m: float = 0.50
    max_yaw_jump_rad: float = 1.00

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def check_wheel_odom_sample(
    sample: WheelOdomSample,
    *,
    max_linear_mps: float = DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    max_angular_rad_s: float = DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    require_frame_ids: bool = True,
) -> OdomSanityCheckResult:
    return check_pose_twist(
        pose=sample.pose,
        twist=sample.twist,
        dt_s=sample.dt_s,
        odom_frame_id=sample.odom_frame_id,
        base_frame_id=sample.base_frame_id,
        max_linear_mps=max_linear_mps,
        max_angular_rad_s=max_angular_rad_s,
        require_frame_ids=require_frame_ids,
    )


def check_wheel_odom_state(
    state: WheelOdomState,
    *,
    max_linear_mps: float = DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    max_angular_rad_s: float = DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    require_frame_ids: bool = True,
) -> OdomSanityCheckResult:
    if state.last_sample is None:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="wheel odometry has no sample",
            reasons=["WheelOdomState.last_sample is None"],
        )

    return check_wheel_odom_sample(
        state.last_sample,
        max_linear_mps=max_linear_mps,
        max_angular_rad_s=max_angular_rad_s,
        require_frame_ids=require_frame_ids,
    )


def check_pose_twist(
    *,
    pose: Pose2D,
    twist: Twist2D,
    dt_s: float,
    odom_frame_id: str,
    base_frame_id: str,
    max_linear_mps: float = DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    max_angular_rad_s: float = DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    require_frame_ids: bool = True,
) -> OdomSanityCheckResult:
    if max_linear_mps <= 0.0:
        raise ValueError(f"max_linear_mps must be > 0.0, got {max_linear_mps}")

    if max_angular_rad_s <= 0.0:
        raise ValueError(
            f"max_angular_rad_s must be > 0.0, got {max_angular_rad_s}"
        )

    reasons: list[str] = []

    pose_finite = pose_is_finite(pose)
    twist_finite = twist_is_finite(twist)

    if not pose_finite:
        reasons.append("pose contains non-finite values")

    if not twist_finite:
        reasons.append("twist contains non-finite values")

    dt_ok = math.isfinite(float(dt_s)) and float(dt_s) >= 0.0
    if not dt_ok:
        reasons.append(f"invalid dt_s={dt_s}")

    frame_ok = True
    if require_frame_ids:
        frame_ok = _frame_ids_valid(odom_frame_id, base_frame_id)
        if not frame_ok:
            reasons.append(
                f"invalid odom frames: odom_frame_id={odom_frame_id!r}, "
                f"base_frame_id={base_frame_id!r}"
            )

    linear_speed, angular_speed = twist_magnitude(twist)

    if linear_speed > max_linear_mps:
        reasons.append(
            f"linear speed too high: {linear_speed:.3f} m/s > {max_linear_mps:.3f} m/s"
        )

    if angular_speed > max_angular_rad_s:
        reasons.append(
            "angular speed too high: "
            f"{angular_speed:.3f} rad/s > {max_angular_rad_s:.3f} rad/s"
        )

    if not pose_finite or not twist_finite or not dt_ok:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry sanity check failed",
            reasons=reasons,
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
            pose_finite=pose_finite,
            twist_finite=twist_finite,
            frame_ok=frame_ok,
            dt_ok=dt_ok,
        )

    if reasons:
        return OdomSanityCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry usable with sanity notes",
            reasons=reasons,
            linear_speed_mps=linear_speed,
            angular_speed_rad_s=angular_speed,
            pose_finite=pose_finite,
            twist_finite=twist_finite,
            frame_ok=frame_ok,
            dt_ok=dt_ok,
        )

    return OdomSanityCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry sanity check healthy",
        reasons=[],
        linear_speed_mps=linear_speed,
        angular_speed_rad_s=angular_speed,
        pose_finite=pose_finite,
        twist_finite=twist_finite,
        frame_ok=frame_ok,
        dt_ok=dt_ok,
    )


def check_odom_jump(
    *,
    previous_pose: Pose2D,
    current_pose: Pose2D,
    dt_s: float,
    max_distance_jump_m: float = 0.50,
    max_yaw_jump_rad: float = 1.00,
) -> OdomJumpCheckResult:
    if max_distance_jump_m < 0.0:
        raise ValueError(
            f"max_distance_jump_m must be >= 0.0, got {max_distance_jump_m}"
        )

    if max_yaw_jump_rad < 0.0:
        raise ValueError(f"max_yaw_jump_rad must be >= 0.0, got {max_yaw_jump_rad}")

    reasons: list[str] = []

    if not pose_is_finite(previous_pose):
        reasons.append("previous pose contains non-finite values")

    if not pose_is_finite(current_pose):
        reasons.append("current pose contains non-finite values")

    dt = float(dt_s)
    if not math.isfinite(dt) or dt < 0.0:
        reasons.append(f"invalid dt_s={dt_s}")

    distance_jump = distance_2d_m(previous_pose, current_pose)
    yaw_jump = abs(
        shortest_angle_delta_rad(
            previous_pose.yaw_rad,
            current_pose.yaw_rad,
        )
    )

    if distance_jump > max_distance_jump_m:
        reasons.append(
            f"distance jump too large: {distance_jump:.3f} m > "
            f"{max_distance_jump_m:.3f} m"
        )

    if yaw_jump > max_yaw_jump_rad:
        reasons.append(
            f"yaw jump too large: {yaw_jump:.3f} rad > {max_yaw_jump_rad:.3f} rad"
        )

    if any("non-finite" in reason or "invalid dt" in reason for reason in reasons):
        return OdomJumpCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry jump check failed",
            reasons=reasons,
            distance_jump_m=distance_jump,
            yaw_jump_rad=yaw_jump,
            dt_s=dt,
            max_distance_jump_m=max_distance_jump_m,
            max_yaw_jump_rad=max_yaw_jump_rad,
        )

    if reasons:
        return OdomJumpCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry jump detected",
            reasons=reasons,
            distance_jump_m=distance_jump,
            yaw_jump_rad=yaw_jump,
            dt_s=dt,
            max_distance_jump_m=max_distance_jump_m,
            max_yaw_jump_rad=max_yaw_jump_rad,
        )

    return OdomJumpCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry jump check healthy",
        reasons=[],
        distance_jump_m=distance_jump,
        yaw_jump_rad=yaw_jump,
        dt_s=dt,
        max_distance_jump_m=max_distance_jump_m,
        max_yaw_jump_rad=max_yaw_jump_rad,
    )


def check_odom_sample_window(
    samples: Iterable[WheelOdomSample],
    *,
    max_linear_mps: float = DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    max_angular_rad_s: float = DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    max_distance_jump_m: float = 0.50,
    max_yaw_jump_rad: float = 1.00,
) -> OdomSanityCheckResult:
    sample_list = list(samples)

    if not sample_list:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no odometry samples available",
            reasons=["sample list is empty"],
        )

    reasons: list[str] = []
    latest_result: OdomSanityCheckResult | None = None

    for sample in sample_list:
        result = check_wheel_odom_sample(
            sample,
            max_linear_mps=max_linear_mps,
            max_angular_rad_s=max_angular_rad_s,
        )
        latest_result = result
        reasons.extend(result.reasons)

    for previous, current in zip(sample_list[:-1], sample_list[1:]):
        jump_result = check_odom_jump(
            previous_pose=previous.pose,
            current_pose=current.pose,
            dt_s=current.dt_s,
            max_distance_jump_m=max_distance_jump_m,
            max_yaw_jump_rad=max_yaw_jump_rad,
        )
        reasons.extend(jump_result.reasons)

    if latest_result is None:
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="no odometry samples evaluated",
            reasons=["internal evaluation produced no result"],
        )

    unique_reasons = _unique_preserve_order(reasons)

    if any(
        "non-finite" in reason or "invalid dt" in reason
        for reason in unique_reasons
    ):
        return OdomSanityCheckResult(
            status=STATUS_ERROR,
            ok=False,
            message="odometry sample window failed",
            reasons=unique_reasons,
            linear_speed_mps=latest_result.linear_speed_mps,
            angular_speed_rad_s=latest_result.angular_speed_rad_s,
            pose_finite=latest_result.pose_finite,
            twist_finite=latest_result.twist_finite,
            frame_ok=latest_result.frame_ok,
            dt_ok=latest_result.dt_ok,
        )

    if unique_reasons:
        return OdomSanityCheckResult(
            status=STATUS_WARN,
            ok=True,
            message="odometry sample window usable with notes",
            reasons=unique_reasons,
            linear_speed_mps=latest_result.linear_speed_mps,
            angular_speed_rad_s=latest_result.angular_speed_rad_s,
            pose_finite=latest_result.pose_finite,
            twist_finite=latest_result.twist_finite,
            frame_ok=latest_result.frame_ok,
            dt_ok=latest_result.dt_ok,
        )

    return OdomSanityCheckResult(
        status=STATUS_OK,
        ok=True,
        message="odometry sample window healthy",
        reasons=[],
        linear_speed_mps=latest_result.linear_speed_mps,
        angular_speed_rad_s=latest_result.angular_speed_rad_s,
        pose_finite=latest_result.pose_finite,
        twist_finite=latest_result.twist_finite,
        frame_ok=latest_result.frame_ok,
        dt_ok=latest_result.dt_ok,
    )


def odom_pose_summary(pose: Pose2D) -> dict[str, float | bool]:
    return {
        "x_m": float(pose.x_m),
        "y_m": float(pose.y_m),
        "yaw_rad": float(pose.yaw_rad),
        "finite": pose_is_finite(pose),
    }


def odom_twist_summary(twist: Twist2D) -> dict[str, float | bool]:
    linear_speed, angular_speed = twist_magnitude(twist)

    return {
        "vx_mps": float(twist.vx_mps),
        "vy_mps": float(twist.vy_mps),
        "omega_rad_s": float(twist.omega_rad_s),
        "linear_speed_mps": linear_speed,
        "angular_speed_rad_s": angular_speed,
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


def _frame_ids_valid(odom_frame_id: str, base_frame_id: str) -> bool:
    odom = str(odom_frame_id).strip()
    base = str(base_frame_id).strip()

    return bool(odom) and bool(base) and odom != base


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result