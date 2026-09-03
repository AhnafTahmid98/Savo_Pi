"""Test planar covariance geometry without requiring ROS installation."""

import math

import pytest

from savo_observer.localization_math import (
    covariance_ellipse,
    ellipse_points,
    yaw_arc_points,
    yaw_uncertainty_half_span,
)


def _covariance(xx=0.0, xy=0.0, yx=0.0, yy=0.0, yaw=0.0):
    values = [0.0] * 36
    values[0] = xx
    values[1] = xy
    values[6] = yx
    values[7] = yy
    values[35] = yaw
    return values


def test_positive_diagonal_covariance_becomes_two_sigma_ellipse():
    geometry = covariance_ellipse(_covariance(xx=0.04, yy=0.01))
    assert geometry is not None
    assert geometry.major_radius == pytest.approx(0.4)
    assert geometry.minor_radius == pytest.approx(0.2)
    assert geometry.angle_rad == pytest.approx(0.0)
    assert geometry.clamped is False


def test_rotated_covariance_uses_correct_eigenvectors():
    # R(45 deg) diag(0.09, 0.01) R^T.
    geometry = covariance_ellipse(
        _covariance(xx=0.05, xy=0.04, yx=0.04, yy=0.05)
    )
    assert geometry is not None
    assert geometry.major_radius == pytest.approx(0.6)
    assert geometry.minor_radius == pytest.approx(0.2)
    assert geometry.angle_rad == pytest.approx(math.pi / 4.0)


def test_zero_covariance_is_valid_degenerate_geometry():
    geometry = covariance_ellipse(_covariance())
    assert geometry is not None
    assert geometry.major_radius == 0.0
    assert geometry.minor_radius == 0.0
    points = ellipse_points(1.0, -2.0, geometry)
    assert len(points) == 49
    assert set(points) == {(1.0, -2.0)}


@pytest.mark.parametrize(
    'covariance',
    [
        _covariance(xx=math.nan, yy=0.1),
        _covariance(xx=math.inf, yy=0.1),
        _covariance(xx=0.1, yy=-0.5),
        [0.0] * 7,
    ],
)
def test_invalid_covariance_is_not_visualized(covariance):
    assert covariance_ellipse(covariance) is None


def test_extreme_covariance_is_clamped_for_visualization_only():
    source = _covariance(xx=1.0e12, yy=4.0e12)
    geometry = covariance_ellipse(source, max_radius_m=1.25)
    assert geometry is not None
    assert geometry.major_radius == 1.25
    assert geometry.minor_radius == 1.25
    assert geometry.clamped is True
    assert source[0] == 1.0e12
    assert source[7] == 4.0e12


def test_yaw_variance_produces_progressively_wider_bounded_arcs():
    small = yaw_uncertainty_half_span(0.0025)
    larger = yaw_uncertainty_half_span(0.25)
    extreme = yaw_uncertainty_half_span(1.0e12, max_half_span_rad=1.2)
    assert small == pytest.approx((0.1, False))
    assert larger == pytest.approx((1.0, False))
    assert extreme == pytest.approx((1.2, True))


@pytest.mark.parametrize('variance', [math.nan, math.inf, -0.1])
def test_invalid_yaw_variance_is_not_visualized(variance):
    assert yaw_uncertainty_half_span(variance) is None


def test_yaw_arc_is_planar_and_bounded():
    points = yaw_arc_points(1.0, 2.0, 0.5, 0.2)
    assert len(points) == 33
    assert all(len(point) == 2 for point in points)
    assert all(
        math.hypot(x - 1.0, y - 2.0) == pytest.approx(0.30)
        for x, y in points
    )
