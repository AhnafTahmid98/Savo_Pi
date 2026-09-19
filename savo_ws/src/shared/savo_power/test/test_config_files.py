from pathlib import Path

import pytest

from savo_power import constants as c


yaml = pytest.importorskip("yaml")


REQUIRED_CONFIG_FILES = [
    "config/power_common.yaml",
    "config/topics.yaml",
    "config/diagnostic.yaml",
    "config/core/core_ups.yaml",
    "config/core/kit_battery.yaml",
    "config/core/power_aggregator.yaml",
    "config/core/power_health.yaml",
    "config/core/dashboard.yaml",
    "config/edge/edge_ups.yaml",
    "config/edge/power_health.yaml",
    "config/edge/dashboard.yaml",
    "config/profiles/core_real_robot_v1.yaml",
    "config/profiles/edge_real_robot_v1.yaml",
    "config/profiles/dryrun_sim.yaml",
]


def load_yaml(path):
    data = yaml.safe_load(Path(path).read_text())

    assert isinstance(data, dict), path

    return data


def global_params(path):
    data = load_yaml(path)

    assert "/**" in data
    assert "ros__parameters" in data["/**"]

    return data["/**"]["ros__parameters"]


def node_params(path, node_name):
    data = load_yaml(path)

    assert node_name in data
    assert "ros__parameters" in data[node_name]

    return data[node_name]["ros__parameters"]


def test_required_config_files_exist():
    for path in REQUIRED_CONFIG_FILES:
        assert Path(path).is_file(), path


@pytest.mark.parametrize("path", REQUIRED_CONFIG_FILES)
def test_config_yaml_files_parse(path):
    load_yaml(path)


def test_power_common_hardware_defaults():
    params = global_params("config/power_common.yaml")

    assert params["i2c_bus"] == 1
    assert params["ups_address"] == 54
    assert params["ads7830_address"] == 72
    assert params["ads7830_channel"] == 2
    assert params["ads7830_pcb_version"] == "v2"
    assert set(params) == {
        "i2c_bus",
        "ups_address",
        "ads7830_address",
        "ads7830_channel",
        "ads7830_pcb_version",
        "publish_rate_hz",
        "stale_timeout_s",
        "ups_low_voltage_v",
        "ups_critical_voltage_v",
        "base_empty_voltage_v",
        "base_low_voltage_v",
        "base_full_voltage_v",
        "base_low_soc_pct",
        "base_full_soc_pct",
        "full_capacity_pct",
        "automatic_shutdown_enabled",
        "core_ups_expected",
        "edge_ups_expected",
        "base_battery_expected",
    }


def test_power_common_thresholds():
    params = global_params("config/power_common.yaml")

    assert params["ups_low_voltage_v"] == 3.40
    assert params["ups_critical_voltage_v"] == 3.20

    assert params["base_empty_voltage_v"] == 6.40
    assert params["base_low_voltage_v"] == 7.20
    assert params["base_full_voltage_v"] == 8.40
    assert params["base_low_soc_pct"] == 20.0
    assert params["base_full_soc_pct"] == 95.0


def test_topics_match_constants():
    params = global_params("config/topics.yaml")

    assert params["core_ups_topic"] == c.CORE_UPS_TOPIC
    assert params["edge_ups_topic"] == c.EDGE_UPS_TOPIC
    assert params["base_battery_topic"] == c.BASE_BATTERY_TOPIC
    assert params["status_topic"] == c.STATUS_TOPIC
    assert params["health_topic"] == c.HEALTH_TOPIC
    assert params["dashboard_topic"] == c.DASHBOARD_TOPIC
    assert params["dashboard_text_topic"] == c.DASHBOARD_TEXT_TOPIC
    assert params["shutdown_request_topic"] == c.SHUTDOWN_REQUEST_TOPIC


def test_core_ups_config_targets_cpp_and_python_fallback():
    cpp = node_params("config/core/core_ups.yaml", "/core_ups_node")
    py = node_params("config/core/core_ups.yaml", "/core_ups_node_py")

    for params in (cpp, py):
        assert params["i2c_bus"] == 1
        assert params["ups_address"] == 54
        assert set(params) == {
            "i2c_bus",
            "ups_address",
            "publish_rate_hz",
        }


def test_edge_ups_config_targets_cpp_and_python_fallback():
    cpp = node_params("config/edge/edge_ups.yaml", "/edge_ups_node")
    py = node_params("config/edge/edge_ups.yaml", "/edge_ups_node_py")

    for params in (cpp, py):
        assert params["i2c_bus"] == 1
        assert params["ups_address"] == 54
        assert set(params) == {
            "i2c_bus",
            "ups_address",
            "publish_rate_hz",
        }


def test_kit_battery_config_targets_base_battery_topic():
    cpp = node_params("config/core/kit_battery.yaml", "/base_battery_node")
    py = node_params("config/core/kit_battery.yaml", "/base_battery_node_py")

    for params in (cpp, py):
        assert params["ads7830_address"] == 72
        assert params["ads7830_channel"] == 2
        assert params["ads7830_pcb_version"] == "v2"
        assert set(params) == {
            "i2c_bus",
            "ads7830_address",
            "ads7830_channel",
            "ads7830_pcb_version",
            "publish_rate_hz",
        }


def test_core_aggregator_requires_core_and_base_but_not_edge_by_default():
    for node_name in ("/power_aggregator_node", "/power_aggregator_node_py"):
        params = node_params(
            "config/core/power_aggregator.yaml",
            node_name,
        )

        assert params["core_ups_expected"] is True
        assert params["edge_ups_expected"] is False
        assert params["base_battery_expected"] is True
        assert set(params) == {
            "publish_rate_hz",
            "stale_timeout_s",
            "core_ups_expected",
            "edge_ups_expected",
            "base_battery_expected",
        }


def test_health_configs_keep_shutdown_disabled():
    files = [
        "config/core/power_health.yaml",
        "config/edge/power_health.yaml",
    ]

    for path in files:
        for node_name in ("/power_health_node", "/power_health_node_py"):
            params = node_params(path, node_name)

            assert params["automatic_shutdown_enabled"] is False
            assert set(params) == {
                "publish_rate_hz",
                "stale_timeout_s",
                "automatic_shutdown_enabled",
            }


def test_dashboard_configs_use_same_topics():
    files = [
        "config/core/dashboard.yaml",
        "config/edge/dashboard.yaml",
    ]

    for path in files:
        for node_name in ("/power_dashboard_node", "/power_dashboard_node_py"):
            params = node_params(path, node_name)

            expected = {"publish_rate_hz"}
            if node_name == "/power_dashboard_node":
                expected.add("stale_timeout_s")
            assert set(params) == expected


def test_core_profile_uses_cpp_default():
    params = global_params("config/profiles/core_real_robot_v1.yaml")

    assert params["core_ups_expected"] is True
    assert params["edge_ups_expected"] is False
    assert params["base_battery_expected"] is True


def test_edge_profile_only_runs_edge_ups_by_default():
    params = global_params("config/profiles/edge_real_robot_v1.yaml")

    assert params["core_ups_expected"] is False
    assert params["edge_ups_expected"] is True
    assert params["base_battery_expected"] is False


def test_production_power_yaml_has_no_dead_or_legacy_producer_keys():
    forbidden = {
        "source",
        "topic",
        "i2c_address",
        "sample_rate_hz",
        "stale_timeout_s",
        "low_voltage_v",
        "critical_voltage_v",
        "empty_voltage_v",
        "full_voltage_v",
        "low_soc_pct",
        "full_soc_pct",
    }
    producer_files = (
        "config/core/core_ups.yaml",
        "config/core/kit_battery.yaml",
        "config/edge/edge_ups.yaml",
    )

    for path in producer_files:
        data = load_yaml(path)
        for node in data.values():
            assert forbidden.isdisjoint(node["ros__parameters"]), path


def test_shared_power_yaml_uses_only_canonical_threshold_names():
    params = global_params("config/power_common.yaml")

    assert params["ups_low_voltage_v"] == 3.40
    assert params["ups_critical_voltage_v"] == 3.20
    assert params["base_empty_voltage_v"] == 6.40
    assert params["base_low_voltage_v"] == 7.20
    assert params["base_full_voltage_v"] == 8.40
    assert params["base_low_soc_pct"] == 20.0
    assert params["base_full_soc_pct"] == 95.0
    assert "sample_rate_hz" not in params


def test_profiles_do_not_claim_dead_enable_controls():
    expected = {
        "core_ups_expected",
        "edge_ups_expected",
        "base_battery_expected",
        "publish_rate_hz",
        "stale_timeout_s",
        "automatic_shutdown_enabled",
    }
    for path in Path("config/profiles").glob("*.yaml"):
        params = global_params(path)
        assert set(params) == expected, path
        assert not any(key.startswith("enable_") for key in params), path


def test_dryrun_profile_has_fake_values_and_no_shutdown():
    params = global_params("config/profiles/dryrun_sim.yaml")

    assert params["core_ups_expected"] is True
    assert params["edge_ups_expected"] is True
    assert params["base_battery_expected"] is True
    assert params["automatic_shutdown_enabled"] is False
    assert not any(key.startswith("fake_") for key in params)


def test_diagnostic_config_targets_known_i2c_devices():
    params = global_params("config/diagnostic.yaml")

    assert params["i2c_bus"] == 1
    assert params["ups_address"] == 54
    assert params["ads7830_address"] == 72
    assert params["expected_i2c_addresses"] == [54, 72]
    assert params["automatic_shutdown_enabled"] is False


def test_shutdown_disabled_everywhere_it_is_declared():
    for path in Path("config").rglob("*.yaml"):
        text = path.read_text()

        if "automatic_shutdown_enabled" in text:
            assert "automatic_shutdown_enabled: false" in text, str(path)
