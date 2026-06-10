"""Motion sanity checks for visual odometry."""

from dataclasses import dataclass
from math import pi


@dataclass(frozen=True)
class MotionDelta:
    translation_m: float
    rotation_rad: float


@dataclass(frozen=True)
class MotionQualityResult:
    is_valid: bool
    delta: MotionDelta
    message: str = ""


def normalize_angle_rad(angle_rad: float) -> float:
    while angle_rad > pi:
        angle_rad -= 2.0 * pi
    while angle_rad < -pi:
        angle_rad += 2.0 * pi
    return angle_rad


def compute_planar_motion_delta(
    previous_x_m: float,
    previous_y_m: float,
    previous_yaw_rad: float,
    current_x_m: float,
    current_y_m: float,
    current_yaw_rad: float,
) -> MotionDelta:
    dx = current_x_m - previous_x_m
    dy = current_y_m - previous_y_m
    dyaw = normalize_angle_rad(current_yaw_rad - previous_yaw_rad)

    translation_m = (dx * dx + dy * dy) ** 0.5
    rotation_rad = abs(dyaw)

    return MotionDelta(
        translation_m=translation_m,
        rotation_rad=rotation_rad,
    )


def is_motion_jump(
    delta: MotionDelta,
    max_translation_jump_m: float,
    max_rotation_jump_rad: float,
) -> bool:
    if max_translation_jump_m < 0.0:
        raise ValueError("max_translation_jump_m must be non-negative")
    if max_rotation_jump_rad < 0.0:
        raise ValueError("max_rotation_jump_rad must be non-negative")

    return (
        delta.translation_m > max_translation_jump_m
        or delta.rotation_rad > max_rotation_jump_rad
    )


def check_motion_quality(
    previous_x_m: float,
    previous_y_m: float,
    previous_yaw_rad: float,
    current_x_m: float,
    current_y_m: float,
    current_yaw_rad: float,
    max_translation_jump_m: float,
    max_rotation_jump_rad: float,
) -> MotionQualityResult:
    delta = compute_planar_motion_delta(
        previous_x_m=previous_x_m,
        previous_y_m=previous_y_m,
        previous_yaw_rad=previous_yaw_rad,
        current_x_m=current_x_m,
        current_y_m=current_y_m,
        current_yaw_rad=current_yaw_rad,
    )

    if is_motion_jump(
        delta=delta,
        max_translation_jump_m=max_translation_jump_m,
        max_rotation_jump_rad=max_rotation_jump_rad,
    ):
        return MotionQualityResult(
            is_valid=False,
            delta=delta,
            message=(
                "visual odometry motion jump rejected: "
                f"translation={delta.translation_m:.3f}m, "
                f"rotation={delta.rotation_rad:.3f}rad"
            ),
        )

    return MotionQualityResult(
        is_valid=True,
        delta=delta,
        message="visual odometry motion delta is valid",
    )