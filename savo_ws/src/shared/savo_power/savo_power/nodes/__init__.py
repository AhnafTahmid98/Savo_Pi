"""Python fallback nodes for Robot Savo power monitoring.

Production runtime remains C++.

These Python nodes are for diagnostics, development, and fallback operation.
They are safe to import without ROS 2 installed. Instantiating/spinning the
actual nodes still requires rclpy.
"""

from __future__ import annotations

from savo_power.nodes import kit_battery_node_py as kit_battery
from savo_power.nodes import power_aggregator_node_py as power_aggregator
from savo_power.nodes import power_dashboard_node_py as power_dashboard
from savo_power.nodes import power_health_node_py as power_health
from savo_power.nodes import ups_hat_node_py as ups_hat


# UPS HAT fallback node.
UpsHatNodePy = ups_hat.UpsHatNodePy
UpsHatNodeState = ups_hat.UpsHatNodeState
ups_hat_build_startup_summary = ups_hat.build_startup_summary
ups_hat_create_timer_period_s = ups_hat.create_timer_period_s
ups_hat_create_driver_from_params = ups_hat.create_ups_driver_from_params
ups_hat_main = ups_hat.main
ups_hat_main_core = ups_hat.main_core
ups_hat_main_edge = ups_hat.main_edge
ups_hat_make_error_reading = ups_hat.make_error_reading
ups_hat_read_from_driver = ups_hat.read_from_driver
ups_hat_reading_to_publish_text = ups_hat.reading_to_publish_text
ups_hat_source_to_python_node_name = ups_hat.source_to_python_node_name
ups_hat_source_to_topic = ups_hat.source_to_topic
ups_hat_spin_node = ups_hat.spin_ups_node
ups_hat_validate_source = ups_hat.validate_ups_source


# ADS7830/base battery fallback node.
KitBatteryNodePy = kit_battery.KitBatteryNodePy
KitBatteryNodeState = kit_battery.KitBatteryNodeState
kit_battery_base_topic = kit_battery.base_battery_topic
kit_battery_build_startup_summary = kit_battery.build_startup_summary
kit_battery_create_driver_from_params = kit_battery.create_ads7830_driver_from_params
kit_battery_create_timer_period_s = kit_battery.create_timer_period_s
kit_battery_main = kit_battery.main
kit_battery_make_error_reading = kit_battery.make_error_reading
kit_battery_python_node_name = kit_battery.base_battery_python_node_name
kit_battery_read_from_driver = kit_battery.read_from_driver
kit_battery_reading_to_publish_text = kit_battery.reading_to_publish_text
kit_battery_spin_node = kit_battery.spin_kit_battery_node


# Power aggregator fallback node.
AggregatedPowerSourceStatus = getattr(
    power_aggregator,
    "AggregatedPowerSourceStatus",
    None,
)
AggregatedPowerStatusSummary = getattr(
    power_aggregator,
    "AggregatedPowerStatusSummary",
    None,
)
PowerAggregatorMemory = power_aggregator.PowerAggregatorMemory
PowerAggregatorNodePy = power_aggregator.PowerAggregatorNodePy
PowerAggregatorNodeState = power_aggregator.PowerAggregatorNodeState
power_aggregator_aggregate_memory = power_aggregator.aggregate_memory
power_aggregator_build_power_status_summary = power_aggregator.build_power_status_summary
power_aggregator_build_source_statuses = power_aggregator.build_source_statuses
power_aggregator_build_startup_summary = power_aggregator.build_startup_summary
power_aggregator_compute_overall_state = power_aggregator.compute_overall_state
power_aggregator_create_timer_period_s = power_aggregator.create_timer_period_s
power_aggregator_expected_sources_from_params = power_aggregator.expected_sources_from_params
power_aggregator_main = power_aggregator.main
power_aggregator_memory_class = power_aggregator.PowerAggregatorMemory
power_aggregator_node_name = power_aggregator.aggregator_python_node_name
power_aggregator_parse_reading_payload = power_aggregator.parse_reading_payload
power_aggregator_source_from_reading_dict = power_aggregator.source_from_reading_dict
power_aggregator_spin_node = power_aggregator.spin_power_aggregator_node
power_aggregator_state_from_reading_dict = power_aggregator.state_from_reading_dict
power_aggregator_status_topic = power_aggregator.status_topic
power_aggregator_summary_source_statuses = power_aggregator.summary_source_statuses
power_aggregator_summary_status_ok = getattr(
    power_aggregator,
    "summary_status_ok",
    None,
)


# Power health fallback node.
AggregatedPowerHealthResult = power_health.AggregatedPowerHealthResult
PowerHealthMemory = power_health.PowerHealthMemory
PowerHealthNodePy = power_health.PowerHealthNodePy
PowerHealthNodeState = power_health.PowerHealthNodeState
power_health_build_startup_summary = power_health.build_startup_summary
power_health_create_timer_period_s = power_health.create_timer_period_s
power_health_evaluate_memory = power_health.evaluate_memory
power_health_evaluate_status_dict = power_health.evaluate_status_dict
power_health_health_to_publish_text = power_health.health_to_publish_text
power_health_main = power_health.main
power_health_node_name = power_health.health_python_node_name
power_health_parse_status_payload = power_health.parse_status_payload
power_health_shutdown_request_topic = power_health.shutdown_request_topic
power_health_should_request_shutdown = power_health.should_request_shutdown
power_health_spin_node = power_health.spin_power_health_node
power_health_topic = power_health.health_topic


# Power dashboard fallback node.
PowerDashboardMemory = power_dashboard.PowerDashboardMemory
PowerDashboardNodePy = power_dashboard.PowerDashboardNodePy
PowerDashboardNodeState = power_dashboard.PowerDashboardNodeState
PowerDashboardSnapshot = power_dashboard.PowerDashboardSnapshot
power_dashboard_build_snapshot = power_dashboard.build_dashboard_snapshot
power_dashboard_build_startup_summary = power_dashboard.build_startup_summary
power_dashboard_build_text = power_dashboard.build_dashboard_text
power_dashboard_create_timer_period_s = power_dashboard.create_timer_period_s
power_dashboard_main = power_dashboard.main
power_dashboard_node_name = power_dashboard.dashboard_python_node_name
power_dashboard_parse_json_payload = power_dashboard.parse_json_payload
power_dashboard_spin_node = power_dashboard.spin_power_dashboard_node
power_dashboard_text_topic = power_dashboard.dashboard_text_topic
power_dashboard_topic = power_dashboard.dashboard_topic


RCLPY_AVAILABLE = (
    ups_hat.RCLPY_AVAILABLE
    and kit_battery.RCLPY_AVAILABLE
    and power_aggregator.RCLPY_AVAILABLE
    and power_health.RCLPY_AVAILABLE
    and power_dashboard.RCLPY_AVAILABLE
)


def python_fallback_node_names() -> tuple[str, ...]:
    """Return Python fallback node names."""

    return (
        ups_hat_source_to_python_node_name("core_ups"),
        ups_hat_source_to_python_node_name("edge_ups"),
        kit_battery_python_node_name(),
        power_aggregator_node_name(),
        power_health_node_name(),
        power_dashboard_node_name(),
    )


def python_fallback_topic_names() -> tuple[str, ...]:
    """Return primary Python fallback topic names."""

    return (
        ups_hat_source_to_topic("core_ups"),
        ups_hat_source_to_topic("edge_ups"),
        kit_battery_base_topic(),
        power_aggregator_status_topic(),
        power_health_topic(),
        power_health_shutdown_request_topic(),
        power_dashboard_topic(),
        power_dashboard_text_topic(),
    )


__all__ = [
    "AggregatedPowerHealthResult",
    "AggregatedPowerSourceStatus",
    "AggregatedPowerStatusSummary",
    "KitBatteryNodePy",
    "KitBatteryNodeState",
    "PowerAggregatorMemory",
    "PowerAggregatorNodePy",
    "PowerAggregatorNodeState",
    "PowerDashboardMemory",
    "PowerDashboardNodePy",
    "PowerDashboardNodeState",
    "PowerDashboardSnapshot",
    "PowerHealthMemory",
    "PowerHealthNodePy",
    "PowerHealthNodeState",
    "RCLPY_AVAILABLE",
    "UpsHatNodePy",
    "UpsHatNodeState",
    "kit_battery",
    "kit_battery_base_topic",
    "kit_battery_build_startup_summary",
    "kit_battery_create_driver_from_params",
    "kit_battery_create_timer_period_s",
    "kit_battery_main",
    "kit_battery_make_error_reading",
    "kit_battery_python_node_name",
    "kit_battery_read_from_driver",
    "kit_battery_reading_to_publish_text",
    "kit_battery_spin_node",
    "power_aggregator",
    "power_aggregator_aggregate_memory",
    "power_aggregator_build_power_status_summary",
    "power_aggregator_build_source_statuses",
    "power_aggregator_build_startup_summary",
    "power_aggregator_compute_overall_state",
    "power_aggregator_create_timer_period_s",
    "power_aggregator_expected_sources_from_params",
    "power_aggregator_main",
    "power_aggregator_memory_class",
    "power_aggregator_node_name",
    "power_aggregator_parse_reading_payload",
    "power_aggregator_source_from_reading_dict",
    "power_aggregator_spin_node",
    "power_aggregator_state_from_reading_dict",
    "power_aggregator_status_topic",
    "power_aggregator_summary_source_statuses",
    "power_aggregator_summary_status_ok",
    "power_dashboard",
    "power_dashboard_build_snapshot",
    "power_dashboard_build_startup_summary",
    "power_dashboard_build_text",
    "power_dashboard_create_timer_period_s",
    "power_dashboard_main",
    "power_dashboard_node_name",
    "power_dashboard_parse_json_payload",
    "power_dashboard_spin_node",
    "power_dashboard_text_topic",
    "power_dashboard_topic",
    "power_health",
    "power_health_build_startup_summary",
    "power_health_create_timer_period_s",
    "power_health_evaluate_memory",
    "power_health_evaluate_status_dict",
    "power_health_health_to_publish_text",
    "power_health_main",
    "power_health_node_name",
    "power_health_parse_status_payload",
    "power_health_shutdown_request_topic",
    "power_health_should_request_shutdown",
    "power_health_spin_node",
    "power_health_topic",
    "python_fallback_node_names",
    "python_fallback_topic_names",
    "ups_hat",
    "ups_hat_build_startup_summary",
    "ups_hat_create_driver_from_params",
    "ups_hat_create_timer_period_s",
    "ups_hat_main",
    "ups_hat_main_core",
    "ups_hat_main_edge",
    "ups_hat_make_error_reading",
    "ups_hat_read_from_driver",
    "ups_hat_reading_to_publish_text",
    "ups_hat_source_to_python_node_name",
    "ups_hat_source_to_topic",
    "ups_hat_spin_node",
    "ups_hat_validate_source",
]
