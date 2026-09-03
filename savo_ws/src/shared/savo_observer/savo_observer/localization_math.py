"""Small, ROS-independent helpers for planar localization visualization."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class EllipseGeometry:
    """Principal-axis representation of a clamped XY confidence ellipse."""

    major_radius: float
    minor_radius: float
    angle_rad: float
    clamped: bool


def covariance_ellipse(
    covariance: Sequence[float],
    confidence_sigma: float = 2.0,
    max_radius_m: float = 1.5,
) -> EllipseGeometry | None:
    """
    Return the eigendecomposition of the planar pose covariance.

    ROS pose covariance is a row-major 6x6 matrix.  The XY block uses indices
    0, 1, 6, and 7.  Slight publisher asymmetry is handled by averaging the two
    off-diagonal entries before decomposing the symmetric matrix.
    """
    if len(covariance) < 8:
        return None
    values = (
        float(covariance[0]),
        float(covariance[1]),
        float(covariance[6]),
        float(covariance[7]),
        float(confidence_sigma),
        float(max_radius_m),
    )
    if not all(math.isfinite(value) for value in values):
        return None

    cov_xx, cov_xy, cov_yx, cov_yy, sigma, radius_limit = values
    if sigma <= 0.0 or radius_limit <= 0.0:
        return None

    symmetric_xy = 0.5 * (cov_xy + cov_yx)
    half_trace = 0.5 * (cov_xx + cov_yy)
    half_difference = 0.5 * (cov_xx - cov_yy)
    discriminant = math.hypot(half_difference, symmetric_xy)
    major_eigenvalue = half_trace + discriminant
    minor_eigenvalue = half_trace - discriminant

    # Permit tiny negative round-off, but reject a covariance that is not
    # positive semidefinite rather than drawing misleading geometry.
    tolerance = 1.0e-12 * max(1.0, abs(half_trace), discriminant)
    if minor_eigenvalue < -tolerance or major_eigenvalue < -tolerance:
        return None
    major_eigenvalue = max(0.0, major_eigenvalue)
    minor_eigenvalue = max(0.0, minor_eigenvalue)

    raw_major = sigma * math.sqrt(major_eigenvalue)
    raw_minor = sigma * math.sqrt(minor_eigenvalue)
    major_radius = min(raw_major, radius_limit)
    minor_radius = min(raw_minor, radius_limit)
    angle = 0.5 * math.atan2(2.0 * symmetric_xy, cov_xx - cov_yy)
    return EllipseGeometry(
        major_radius=major_radius,
        minor_radius=minor_radius,
        angle_rad=angle,
        clamped=raw_major > radius_limit or raw_minor > radius_limit,
    )


def ellipse_points(
    center_x: float,
    center_y: float,
    geometry: EllipseGeometry,
    point_count: int = 49,
) -> tuple[tuple[float, float], ...]:
    """Sample a closed ellipse line strip in its covariance frame."""
    if point_count < 4:
        raise ValueError('point_count must be at least 4')
    cosine = math.cos(geometry.angle_rad)
    sine = math.sin(geometry.angle_rad)
    points = []
    for index in range(point_count):
        parameter = 2.0 * math.pi * index / (point_count - 1)
        major = geometry.major_radius * math.cos(parameter)
        minor = geometry.minor_radius * math.sin(parameter)
        points.append(
            (
                center_x + major * cosine - minor * sine,
                center_y + major * sine + minor * cosine,
            )
        )
    return tuple(points)


def yaw_uncertainty_half_span(
    yaw_variance: float,
    confidence_sigma: float = 2.0,
    max_half_span_rad: float = math.pi,
) -> tuple[float, bool] | None:
    """Convert yaw variance to a clamped confidence half-span."""
    values = (
        float(yaw_variance),
        float(confidence_sigma),
        float(max_half_span_rad),
    )
    if not all(math.isfinite(value) for value in values):
        return None
    variance, sigma, limit = values
    if variance < 0.0 or sigma <= 0.0 or limit <= 0.0:
        return None
    raw_half_span = sigma * math.sqrt(variance)
    return min(raw_half_span, limit), raw_half_span > limit


def yaw_arc_points(
    center_x: float,
    center_y: float,
    heading_rad: float,
    half_span_rad: float,
    radius_m: float = 0.30,
    point_count: int = 33,
) -> tuple[tuple[float, float], ...]:
    """Sample a floor-level arc centered on the reported robot heading."""
    values = (center_x, center_y, heading_rad, half_span_rad, radius_m)
    if not all(math.isfinite(float(value)) for value in values):
        return ()
    if half_span_rad < 0.0 or radius_m <= 0.0 or point_count < 2:
        return ()
    points = []
    for index in range(point_count):
        fraction = index / (point_count - 1)
        angle = heading_rad - half_span_rad + 2.0 * half_span_rad * fraction
        points.append(
            (
                center_x + radius_m * math.cos(angle),
                center_y + radius_m * math.sin(angle),
            )
        )
    return tuple(points)
