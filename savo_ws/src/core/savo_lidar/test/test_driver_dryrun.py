# -*- coding: utf-8 -*-

import math

import pytest

from savo_lidar.constants import BACKEND_DRYRUN
from savo_lidar.drivers import DryrunLidarDriver, DryrunScan, create_lidar_driver
from savo_lidar.models import make_driver_config


def test_dryrun_driver_starts_and_stops():
    driver = DryrunLidarDriver()

    assert not driver.running
    assert driver.scan_count == 0

    driver.start()

    assert driver.running

    driver.stop()

    assert not driver.running


def test_dryrun_driver_rejects_read_when_not_running():
    driver = DryrunLidarDriver()

    with pytest.raises(RuntimeError):
        driver.read_scan()


def test_dryrun_driver_returns_scan_after_start():
    driver = DryrunLidarDriver()
    driver.start()

    scan = driver.read_scan()

    assert isinstance(scan, DryrunScan)
    assert driver.scan_count == 1
    assert len(scan.ranges) == 360
    assert len(scan.intensities) == 360
    assert scan.range_min_m == 0.15
    assert scan.range_max_m == 12.0
    assert scan.angle_min_rad == pytest.approx(-math.pi)
    assert scan.angle_max_rad == pytest.approx(math.pi)
    assert scan.angle_increment_rad > 0.0

    driver.stop()


def test_dryrun_scan_contains_front_obstacle():
    driver = DryrunLidarDriver(
        point_count=360,
        obstacle_center_deg=0.0,
        obstacle_width_deg=30.0,
        obstacle_distance_m=0.7,
    )
    driver.start()

    scan = driver.read_scan()

    front_index = 180

    assert scan.ranges[front_index] == pytest.approx(0.7)
    assert min(scan.ranges) == pytest.approx(0.7)

    driver.stop()


def test_dryrun_scan_clamps_obstacle_to_min_range():
    driver = DryrunLidarDriver(
        min_range_m=0.15,
        max_range_m=12.0,
        obstacle_distance_m=0.01,
    )
    driver.start()

    scan = driver.read_scan()

    assert min(scan.ranges) == pytest.approx(0.15)

    driver.stop()


def test_dryrun_scan_clamps_base_distance_to_max_range():
    driver = DryrunLidarDriver(
        min_range_m=0.15,
        max_range_m=1.0,
        base_distance_m=5.0,
        obstacle_distance_m=5.0,
    )
    driver.start()

    scan = driver.read_scan()

    assert max(scan.ranges) == pytest.approx(1.0)
    assert min(scan.ranges) == pytest.approx(1.0)

    driver.stop()


def test_dryrun_driver_increments_scan_count():
    driver = DryrunLidarDriver()
    driver.start()

    driver.read_scan()
    driver.read_scan()
    driver.read_scan()

    assert driver.scan_count == 3

    driver.stop()


def test_dryrun_driver_supports_custom_point_count():
    driver = DryrunLidarDriver(point_count=720)
    driver.start()

    scan = driver.read_scan()

    assert len(scan.ranges) == 720
    assert len(scan.intensities) == 720
    assert scan.angle_increment_rad == pytest.approx((2.0 * math.pi) / 720.0)

    driver.stop()


def test_dryrun_driver_rejects_too_few_points():
    with pytest.raises(ValueError):
        DryrunLidarDriver(point_count=3)


def test_dryrun_driver_rejects_invalid_min_range():
    with pytest.raises(ValueError):
        DryrunLidarDriver(min_range_m=0.0)


def test_dryrun_driver_rejects_invalid_max_range():
    with pytest.raises(ValueError):
        DryrunLidarDriver(min_range_m=1.0, max_range_m=1.0)


def test_driver_factory_creates_dryrun_driver():
    config = make_driver_config(backend=BACKEND_DRYRUN)
    driver = create_lidar_driver(config)

    assert isinstance(driver, DryrunLidarDriver)

    driver.start()
    scan = driver.read_scan()

    assert driver.running
    assert driver.scan_count == 1
    assert len(scan.ranges) == 360

    driver.stop()

    assert not driver.running
