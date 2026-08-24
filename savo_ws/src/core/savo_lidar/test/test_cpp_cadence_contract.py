from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _driver_params(profile: str, node: str = "lidar_driver_node") -> dict:
    data = yaml.safe_load((ROOT / "config/profiles" / profile).read_text())
    return data[node]["ros__parameters"]


def test_cpp_driver_is_hardware_acquisition_driven() -> None:
    node = (ROOT / "src/nodes/lidar_driver_node.cpp").read_text()
    assert "ScanAcquisitionWorker" in node
    assert "scan_timer_" not in node
    assert 'declare_parameter<double>("publish_rate_hz"' not in node
    assert "create_wall_timer" in node  # heartbeat remains timer-driven


def test_real_cpp_profiles_do_not_configure_a_publication_timer() -> None:
    for profile in (
        "bench_test.yaml",
        "real_rplidar_a1.yaml",
        "mapping_rplidar_a1.yaml",
        "nav_rplidar_a1.yaml",
    ):
        params = _driver_params(profile)
        assert "publish_rate_hz" not in params
        assert params["serial_port"] == "/dev/ttyUSB0"
        assert params["baudrate"] == 115200
        assert params["inverted"] is False
        assert params["angle_offset_rad"] == 0.0


def test_python_dryrun_keeps_its_timer_rate() -> None:
    params = _driver_params("dryrun_sim.yaml", "lidar_py_driver_node")
    assert params["publish_rate_hz"] == 10.0


def test_cpp_scan_framing_carries_boundaries_and_physical_timing() -> None:
    driver = (ROOT / "src/drivers/rplidar_driver.cpp").read_text()
    assembler = (ROOT / "src/drivers/scan_frame_assembler.cpp").read_text()
    assert "scan_assembler_.add_measurement" in driver
    assert "apply_scan_timing(scan, completed->scan_time_s)" in driver
    assert "scan.ros_start_time_ns = completed->ros_start_time_ns" in driver
    assert driver.count("scan_assembler_.reset()") >= 3
    assert "current_samples_.push_back(to_sample(measurement))" in assembler
    assert "received_at - scan_start_time_" in assembler
    assert "completed->ros_start_time_ns = *ros_scan_start_time_ns_" in assembler
    assert "ros_scan_start_time_ns_.reset()" in assembler


def test_cpp_publisher_uses_preserved_first_ray_ros_timestamp() -> None:
    node = (ROOT / "src/nodes/lidar_driver_node.cpp").read_text()
    publisher = (ROOT / "src/ros/scan_publisher.cpp").read_text()
    assert "[this]() {return now().nanoseconds();}" in node
    assert "scan.ros_start_time_ns" in publisher
    assert "first_ray_stamp" in publisher
    assert "node_->now()" not in publisher
    assert "LaserScan has no first-ray ROS timestamp" in publisher


def test_worker_is_joined_before_node_resources_are_destroyed() -> None:
    node = (ROOT / "src/nodes/lidar_driver_node.cpp").read_text()
    worker = (ROOT / "src/drivers/scan_acquisition_worker.cpp").read_text()
    serial = (ROOT / "src/drivers/serial_port.cpp").read_text()
    driver = (ROOT / "src/drivers/rplidar_driver.cpp").read_text()
    destructor = node[node.index("~LidarDriverNode()") : node.index("private:")]
    assert destructor.index("request_stop()") < destructor.index("join()")
    assert destructor.index("join()") < destructor.index("driver_->stop()")
    assert "driver_->cancel_pending_operation()" in node
    assert "cancel_acquire_();" in worker
    assert "CANCELLATION_POLL_INTERVAL" in serial
    assert "io_cancellation_requested()" in serial
    assert "config_.motor_stop_timeout_s" in driver
    assert "RPLIDAR startup cancelled" in driver
