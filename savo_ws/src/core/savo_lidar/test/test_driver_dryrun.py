from savo_lidar.drivers.dryrun_lidar_driver import DryrunLidarDriver


def test_dryrun_lidar_driver_starts_and_stops():
    driver = DryrunLidarDriver()

    assert not driver.running

    driver.start()
    assert driver.running

    driver.stop()
    assert not driver.running


def test_dryrun_lidar_driver_rejects_read_before_start():
    driver = DryrunLidarDriver()

    try:
        driver.read_scan()
    except RuntimeError as exc:
        assert "not running" in str(exc)
    else:
        raise AssertionError("Expected RuntimeError when reading before start")


def test_dryrun_lidar_driver_returns_scan_after_start():
    driver = DryrunLidarDriver(point_count=360)
    driver.start()

    scan = driver.read_scan()

    assert driver.scan_count == 1
    assert scan.angle_min_rad < scan.angle_max_rad
    assert scan.angle_increment_rad > 0.0
    assert scan.range_min_m == driver.min_range_m
    assert scan.range_max_m == driver.max_range_m
    assert len(scan.ranges) == 360
    assert len(scan.intensities) == 360

    driver.stop()


def test_dryrun_lidar_driver_generates_front_obstacle():
    driver = DryrunLidarDriver(
        point_count=360,
        obstacle_center_deg=0.0,
        obstacle_width_deg=30.0,
        obstacle_distance_m=0.7,
    )
    driver.start()

    scan = driver.read_scan()

    front_index = 180
    assert scan.ranges[front_index] == 0.7

    driver.stop()


def test_dryrun_lidar_driver_increments_scan_count():
    driver = DryrunLidarDriver()
    driver.start()

    driver.read_scan()
    driver.read_scan()

    assert driver.scan_count == 2

    driver.stop()


def test_dryrun_lidar_driver_rejects_invalid_point_count():
    try:
        DryrunLidarDriver(point_count=3)
    except ValueError as exc:
        assert "point_count" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid point_count")


def test_dryrun_lidar_driver_rejects_invalid_range_limits():
    try:
        DryrunLidarDriver(min_range_m=1.0, max_range_m=0.5)
    except ValueError as exc:
        assert "max_range_m" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid range limits")