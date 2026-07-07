"""Shared constants for Robot Savo power diagnostics and Python fallbacks."""

from __future__ import annotations


PACKAGE_NAME = "savo_power"
ROBOT_NAME = "Robot Savo"


# ---------------------------------------------------------------------------
# Node names
# ---------------------------------------------------------------------------

CORE_UPS_NODE_NAME = "core_ups_node"
EDGE_UPS_NODE_NAME = "edge_ups_node"
BASE_BATTERY_NODE_NAME = "base_battery_node"
POWER_AGGREGATOR_NODE_NAME = "power_aggregator_node"
POWER_HEALTH_NODE_NAME = "power_health_node"
POWER_DASHBOARD_NODE_NAME = "power_dashboard_node"


# ---------------------------------------------------------------------------
# Source names
# ---------------------------------------------------------------------------

CORE_UPS_SOURCE = "core_ups"
EDGE_UPS_SOURCE = "edge_ups"
BASE_BATTERY_SOURCE = "base_battery"
UNKNOWN_SOURCE = "unknown"

BATTERY_SOURCES = (
    CORE_UPS_SOURCE,
    EDGE_UPS_SOURCE,
    BASE_BATTERY_SOURCE,
)


# ---------------------------------------------------------------------------
# ROS topics
# ---------------------------------------------------------------------------

CORE_UPS_TOPIC = "/savo_power/core/ups"
EDGE_UPS_TOPIC = "/savo_power/edge/ups"
BASE_BATTERY_TOPIC = "/savo_power/base/battery"

STATUS_TOPIC = "/savo_power/status"
HEALTH_TOPIC = "/savo_power/health"

DASHBOARD_TOPIC = "/savo_power/dashboard"
DASHBOARD_TEXT_TOPIC = "/savo_power/dashboard_text"

SHUTDOWN_REQUEST_TOPIC = "/savo_power/shutdown_request"


# ---------------------------------------------------------------------------
# I2C defaults
# ---------------------------------------------------------------------------

DEFAULT_I2C_BUS = 1

UPS_HAT_DEFAULT_ADDRESS = 0x36
UPS_HAT_VOLTAGE_REGISTER = 0x02
UPS_HAT_CAPACITY_REGISTER = 0x04

ADS7830_DEFAULT_ADDRESS = 0x48
ADS7830_CMD_BASE = 0x84
ADS7830_DEFAULT_CHANNEL = 2

ADS7830_PCB_V1 = "v1"
ADS7830_PCB_V2 = "v2"
ADS7830_DEFAULT_PCB_VERSION = ADS7830_PCB_V2


# ---------------------------------------------------------------------------
# Power thresholds
# ---------------------------------------------------------------------------

UPS_LOW_VOLTAGE = 3.40
UPS_CRITICAL_VOLTAGE = 3.20

BASE_BATTERY_EMPTY_VOLTAGE = 6.40
BASE_BATTERY_LOW_VOLTAGE = 7.20
BASE_BATTERY_FULL_VOLTAGE = 8.40

BASE_BATTERY_LOW_SOC = 20.0
BASE_BATTERY_FULL_SOC = 95.0

FULL_CAPACITY_PERCENT = 95.0


# ---------------------------------------------------------------------------
# Rates and timeouts
# ---------------------------------------------------------------------------

DEFAULT_SAMPLE_RATE_HZ = 1.0
DEFAULT_PUBLISH_RATE_HZ = 1.0
DEFAULT_HEALTH_PUBLISH_HZ = 1.0
DEFAULT_DASHBOARD_PUBLISH_HZ = 1.0

DEFAULT_STALE_TIMEOUT_S = 5.0

MIN_PUBLISH_RATE_HZ = 0.1
MAX_PUBLISH_RATE_HZ = 10.0


# ---------------------------------------------------------------------------
# Safety policy
# ---------------------------------------------------------------------------

AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT = False


# ---------------------------------------------------------------------------
# Parameter names
# ---------------------------------------------------------------------------

PARAM_I2C_BUS = "i2c_bus"
PARAM_UPS_ADDRESS = "ups_address"

PARAM_ADS7830_ADDRESS = "ads7830_address"
PARAM_ADS7830_CHANNEL = "ads7830_channel"
PARAM_ADS7830_PCB_VERSION = "ads7830_pcb_version"

PARAM_SAMPLE_RATE_HZ = "sample_rate_hz"
PARAM_PUBLISH_RATE_HZ = "publish_rate_hz"
PARAM_STALE_TIMEOUT_S = "stale_timeout_s"

PARAM_UPS_LOW_VOLTAGE = "ups_low_voltage_v"
PARAM_UPS_CRITICAL_VOLTAGE = "ups_critical_voltage_v"

PARAM_BASE_EMPTY_VOLTAGE = "base_empty_voltage_v"
PARAM_BASE_LOW_VOLTAGE = "base_low_voltage_v"
PARAM_BASE_FULL_VOLTAGE = "base_full_voltage_v"
PARAM_BASE_LOW_SOC = "base_low_soc_pct"
PARAM_BASE_FULL_SOC = "base_full_soc_pct"

PARAM_FULL_CAPACITY = "full_capacity_pct"
PARAM_AUTOMATIC_SHUTDOWN_ENABLED = "automatic_shutdown_enabled"

PARAM_CORE_UPS_EXPECTED = "core_ups_expected"
PARAM_EDGE_UPS_EXPECTED = "edge_ups_expected"
PARAM_BASE_BATTERY_EXPECTED = "base_battery_expected"


# ---------------------------------------------------------------------------
# Text states
# ---------------------------------------------------------------------------

STATE_OK = "ok"
STATE_LOW = "low"
STATE_CRITICAL = "critical"
STATE_CHARGING = "charging"
STATE_FULL = "full"
STATE_ERROR = "error"
STATE_STALE = "stale"
STATE_UNKNOWN = "unknown"

POWER_STATES = (
    STATE_OK,
    STATE_LOW,
    STATE_CRITICAL,
    STATE_CHARGING,
    STATE_FULL,
    STATE_ERROR,
    STATE_STALE,
    STATE_UNKNOWN,
)


# ---------------------------------------------------------------------------
# Health levels
# ---------------------------------------------------------------------------

HEALTH_OK = "OK"
HEALTH_WARN = "WARN"
HEALTH_ERROR = "ERROR"
HEALTH_UNKNOWN = "UNKNOWN"


__all__ = [
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "CORE_UPS_NODE_NAME",
    "EDGE_UPS_NODE_NAME",
    "BASE_BATTERY_NODE_NAME",
    "POWER_AGGREGATOR_NODE_NAME",
    "POWER_HEALTH_NODE_NAME",
    "POWER_DASHBOARD_NODE_NAME",
    "CORE_UPS_SOURCE",
    "EDGE_UPS_SOURCE",
    "BASE_BATTERY_SOURCE",
    "UNKNOWN_SOURCE",
    "BATTERY_SOURCES",
    "CORE_UPS_TOPIC",
    "EDGE_UPS_TOPIC",
    "BASE_BATTERY_TOPIC",
    "STATUS_TOPIC",
    "HEALTH_TOPIC",
    "DASHBOARD_TOPIC",
    "DASHBOARD_TEXT_TOPIC",
    "SHUTDOWN_REQUEST_TOPIC",
    "DEFAULT_I2C_BUS",
    "UPS_HAT_DEFAULT_ADDRESS",
    "UPS_HAT_VOLTAGE_REGISTER",
    "UPS_HAT_CAPACITY_REGISTER",
    "ADS7830_DEFAULT_ADDRESS",
    "ADS7830_CMD_BASE",
    "ADS7830_DEFAULT_CHANNEL",
    "ADS7830_PCB_V1",
    "ADS7830_PCB_V2",
    "ADS7830_DEFAULT_PCB_VERSION",
    "UPS_LOW_VOLTAGE",
    "UPS_CRITICAL_VOLTAGE",
    "BASE_BATTERY_EMPTY_VOLTAGE",
    "BASE_BATTERY_LOW_VOLTAGE",
    "BASE_BATTERY_FULL_VOLTAGE",
    "BASE_BATTERY_LOW_SOC",
    "BASE_BATTERY_FULL_SOC",
    "FULL_CAPACITY_PERCENT",
    "DEFAULT_SAMPLE_RATE_HZ",
    "DEFAULT_PUBLISH_RATE_HZ",
    "DEFAULT_HEALTH_PUBLISH_HZ",
    "DEFAULT_DASHBOARD_PUBLISH_HZ",
    "DEFAULT_STALE_TIMEOUT_S",
    "MIN_PUBLISH_RATE_HZ",
    "MAX_PUBLISH_RATE_HZ",
    "AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT",
    "PARAM_I2C_BUS",
    "PARAM_UPS_ADDRESS",
    "PARAM_ADS7830_ADDRESS",
    "PARAM_ADS7830_CHANNEL",
    "PARAM_ADS7830_PCB_VERSION",
    "PARAM_SAMPLE_RATE_HZ",
    "PARAM_PUBLISH_RATE_HZ",
    "PARAM_STALE_TIMEOUT_S",
    "PARAM_UPS_LOW_VOLTAGE",
    "PARAM_UPS_CRITICAL_VOLTAGE",
    "PARAM_BASE_EMPTY_VOLTAGE",
    "PARAM_BASE_LOW_VOLTAGE",
    "PARAM_BASE_FULL_VOLTAGE",
    "PARAM_BASE_LOW_SOC",
    "PARAM_BASE_FULL_SOC",
    "PARAM_FULL_CAPACITY",
    "PARAM_AUTOMATIC_SHUTDOWN_ENABLED",
    "PARAM_CORE_UPS_EXPECTED",
    "PARAM_EDGE_UPS_EXPECTED",
    "PARAM_BASE_BATTERY_EXPECTED",
    "STATE_OK",
    "STATE_LOW",
    "STATE_CRITICAL",
    "STATE_CHARGING",
    "STATE_FULL",
    "STATE_ERROR",
    "STATE_STALE",
    "STATE_UNKNOWN",
    "POWER_STATES",
    "HEALTH_OK",
    "HEALTH_WARN",
    "HEALTH_ERROR",
    "HEALTH_UNKNOWN",
]
