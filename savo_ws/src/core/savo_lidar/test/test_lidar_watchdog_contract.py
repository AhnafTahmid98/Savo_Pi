from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_watchdog_consumes_lightweight_driver_state_only():
    source = read("savo_lidar/nodes/lidar_watchdog_node.py")

    assert "DriverStateScanEvidence" in source
    assert "DEFAULT_STATE_TOPIC" in source
    assert "driver_state_topic" in source
    assert "create_subscription(\n            String" in source
    assert "LaserScan" not in source
    assert "scan_qos" not in source
    assert "DEFAULT_SCAN_TOPIC" not in source


def test_every_watchdog_config_uses_driver_state_evidence():
    paths = [
        PACKAGE_ROOT / "config/lidar_health.yaml",
        *sorted((PACKAGE_ROOT / "config/profiles").glob("*.yaml")),
    ]

    for path in paths:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        params = data["lidar_watchdog_node"]["ros__parameters"]
        assert params["driver_state_topic"] == "/savo_lidar/state"
        assert "scan_topic" not in params


def test_watchdog_output_contract_and_threshold_parameters_remain():
    source = read("savo_lidar/nodes/lidar_watchdog_node.py")

    for fragment in (
        "DEFAULT_WATCHDOG_STATE_TOPIC",
        '"stale_timeout_s"',
        '"warn_before_stale_ratio"',
        "StaleScanPolicy",
        "stale=decision.stale",
        "scan_ok=decision.scan_ok",
    ):
        assert fragment in source
