import re
from pathlib import Path


CPP_SOURCES = [
    "src/state/power_state.cpp",
    "src/policy/power_policy.cpp",
    "src/drivers/linux_i2c_bus.cpp",
    "src/drivers/ups_hat_driver.cpp",
    "src/drivers/ads7830_driver.cpp",
    "src/aggregation/power_aggregator.cpp",
    "src/health/power_health.cpp",
    "src/nodes/ups_hat_node.cpp",
    "src/nodes/kit_battery_node.cpp",
    "src/nodes/core_ups_node.cpp",
    "src/nodes/edge_ups_node.cpp",
    "src/nodes/base_battery_node.cpp",
    "src/nodes/power_aggregator_node.cpp",
    "src/nodes/power_health_node.cpp",
    "src/nodes/power_dashboard_node.cpp",
]


EXPECTED_HEADER_NAMES = [
    "power_state.hpp",
    "power_policy.hpp",
    "i2c_bus.hpp",
    "linux_i2c_bus.hpp",
    "ups_hat_driver.hpp",
    "ads7830_driver.hpp",
    "power_aggregator.hpp",
    "power_health.hpp",
    "ups_hat_node.hpp",
    "kit_battery_node.hpp",
]


CPP_NODE_SOURCES = [
    "src/nodes/core_ups_node.cpp",
    "src/nodes/edge_ups_node.cpp",
    "src/nodes/base_battery_node.cpp",
    "src/nodes/power_aggregator_node.cpp",
    "src/nodes/power_health_node.cpp",
    "src/nodes/power_dashboard_node.cpp",
]


def read(path):
    return Path(path).read_text()


def include_root():
    return Path("include/savo_power")


def header_path(name):
    matches = sorted(include_root().rglob(name))

    assert matches, name

    return matches[0]


def header_text(name):
    return header_path(name).read_text()


def combined_text(*items):
    parts = []

    for item in items:
        path = Path(item)

        if path.is_file():
            parts.append(path.read_text())
        else:
            parts.append(header_text(item))

    return "\n".join(parts)


def all_cpp_text():
    parts = []

    for path in CPP_SOURCES:
        if Path(path).is_file():
            parts.append(read(path))

    if include_root().is_dir():
        for path in sorted(include_root().rglob("*.hpp")):
            parts.append(path.read_text())

    return "\n".join(parts)


def test_expected_cpp_sources_exist():
    for path in CPP_SOURCES:
        assert Path(path).is_file(), path


def test_expected_public_headers_exist():
    for name in EXPECTED_HEADER_NAMES:
        assert header_path(name).is_file(), name


def test_public_headers_have_include_protection():
    for name in EXPECTED_HEADER_NAMES:
        text = header_text(name)

        assert "#pragma once" in text or "#ifndef" in text, name


def test_public_headers_use_savo_power_namespace():
    for name in EXPECTED_HEADER_NAMES:
        text = header_text(name)

        assert "namespace savo_power" in text, name


def test_cpp_sources_are_not_empty_placeholders():
    for path in CPP_SOURCES:
        text = read(path).strip()

        assert len(text) > 80, path
        assert text != "int main() { return 0; }", path


def test_cpp_sources_reference_savo_power_headers_or_namespace():
    for path in CPP_SOURCES:
        text = read(path)

        assert (
            '#include "savo_power/' in text
            or "#include <savo_power/" in text
            or "namespace savo_power" in text
        ), path


def test_cpp_node_sources_have_ros_runtime_shape():
    for path in CPP_NODE_SOURCES:
        text = read(path)

        assert "rclcpp::init" in text, path
        assert "rclcpp::spin" in text, path
        assert "rclcpp::shutdown" in text, path


def test_ups_driver_keeps_ups_hat_address_contract():
    text = combined_text(
        "src/drivers/ups_hat_driver.cpp",
        "ups_hat_driver.hpp",
    )
    package_text = all_cpp_text()

    assert (
        "kUpsHatAddrDefault" in text
        or "0x36" in package_text
        or "54" in package_text
    )


def test_ups_driver_contains_register_or_read_word_contract():
    text = combined_text(
        "src/drivers/ups_hat_driver.cpp",
        "ups_hat_driver.hpp",
        "src/policy/power_policy.cpp",
        "power_policy.hpp",
    )

    assert "read_word" in text or "readWord" in text or "word" in text.lower()


def test_ups_conversion_contract_exists_in_cpp_or_policy():
    text = all_cpp_text()

    assert "capacity" in text.lower()
    assert "voltage" in text.lower()


def test_ads7830_driver_keeps_base_battery_address_channel_contract():
    text = combined_text(
        "src/drivers/ads7830_driver.cpp",
        "ads7830_driver.hpp",
    )
    package_text = all_cpp_text()

    assert (
        "kAds7830AddrDefault" in text
        or "0x48" in package_text
        or "72" in package_text
    )

    assert (
        "channel" in package_text.lower()
        or "kAds7830ChannelDefault" in package_text
        or "ads7830_channel" in package_text
    )


def test_ads7830_driver_contains_adc_or_byte_contract():
    text = combined_text(
        "src/drivers/ads7830_driver.cpp",
        "ads7830_driver.hpp",
        "src/policy/power_policy.cpp",
        "power_policy.hpp",
    )

    assert "adc" in text.lower() or "raw" in text.lower() or "byte" in text.lower()


def test_ads7830_voltage_contract_mentions_pcb_or_multiplier():
    text = all_cpp_text().lower()

    assert "pcb" in text or "multiplier" in text or "base" in text


def test_power_policy_contains_base_battery_threshold_terms():
    text = combined_text(
        "src/policy/power_policy.cpp",
        "power_policy.hpp",
    )

    assert "low" in text.lower()
    assert "critical" in text.lower()
    assert "voltage" in text.lower()
    assert "capacity" in text.lower() or "soc" in text.lower()


def test_power_state_contains_expected_state_names():
    text = combined_text(
        "src/state/power_state.cpp",
        "power_state.hpp",
    )

    for state in ["ok", "low", "critical", "stale", "unknown"]:
        assert state in text.lower()


def test_aggregator_source_contract_mentions_three_sources():
    text = combined_text(
        "src/aggregation/power_aggregator.cpp",
        "power_aggregator.hpp",
    )

    assert "core_ups" in text
    assert "edge_ups" in text
    assert "base_battery" in text


def test_health_contract_contains_power_health_terms():
    text = combined_text(
        "src/health/power_health.cpp",
        "power_health.hpp",
    ).lower()

    assert "health" in text
    assert "critical" in text
    assert "stale" in text
    assert "ok" in text


def test_node_sources_publish_or_subscribe_power_topics():
    text = all_cpp_text()

    for topic_part in [
        "core/ups",
        "edge/ups",
        "base/battery",
        "status",
        "health",
        "dashboard",
        "shutdown_request",
    ]:
        assert topic_part in text


def test_cpp_sources_do_not_directly_drive_motor_or_cmd_vel():
    text = all_cpp_text().lower()

    forbidden = [
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
        "emergency_recovery",
    ]

    for item in forbidden:
        assert item not in text


def test_cpp_sources_do_not_enable_shutdown_by_default():
    text = all_cpp_text()

    forbidden = [
        "automatic_shutdown_enabled(true)",
        "automatic_shutdown_enabled = true",
        "automatic_shutdown_enabled{true}",
        "shutdown_requested = true",
        "request_shutdown(true)",
    ]

    for item in forbidden:
        assert item not in text


def test_cpp_sources_keep_python_fallback_separate():
    text = all_cpp_text()

    assert "savo_power.nodes." not in text
    assert "python3" not in text.lower()


def test_cmake_references_every_static_contract_source():
    cmake = Path("CMakeLists.txt").read_text()

    for path in CPP_SOURCES:
        assert path in cmake, path
