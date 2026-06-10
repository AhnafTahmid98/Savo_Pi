"""Geometry helpers used by visual odometry logic."""

from math import atan2, cos, pi, sin, sqrt


def normalize_angle_rad(angle_rad: float) -> float:
    while angle_rad > pi:
        angle_rad -= 2.0 * pi
    while angle_rad < -pi:
        angle_rad += 2.0 * pi
    return angle_rad


def planar_distance_m(
    first_x_m: float,
    first_y_m: float,
    second_x_m: float,
    second_y_m: float,
) -> float:
    dx = second_x_m - first_x_m
    dy = second_y_m - first_y_m
    return sqrt(dx * dx + dy * dy)


def yaw_delta_rad(first_yaw_rad: float, second_yaw_rad: float) -> float:
    return normalize_angle_rad(second_yaw_rad - first_yaw_rad)


def heading_from_delta_rad(dx_m: float, dy_m: float) -> float:
    return atan2(dy_m, dx_m)


def rotate_point_2d(
    x_m: float,
    y_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    c = cos(yaw_rad)
    s = sin(yaw_rad)

    return (
        c * x_m - s * y_m,
        s * x_m + c * y_m,
    )


def transform_point_2d(
    x_m: float,
    y_m: float,
    translation_x_m: float,
    translation_y_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    rotated_x_m, rotated_y_m = rotate_point_2d(
        x_m=x_m,
        y_m=y_m,
        yaw_rad=yaw_rad,
    )

    return (
        rotated_x_m + translation_x_m,
        rotated_y_m + translation_y_m,
    )