"""Python diagnostics and fallback helpers for Robot Savo power monitoring.

The production runtime for this package is C++.
Python code is kept for diagnostics, quick hardware checks, and safe fallback tools.
"""

__package_name__ = "savo_power"
__version__ = "0.1.0"

ROBOT_NAME = "Robot Savo"

CORE_UPS_SOURCE = "core_ups"
EDGE_UPS_SOURCE = "edge_ups"
BASE_BATTERY_SOURCE = "base_battery"

CORE_UPS_TOPIC = "/savo_power/core/ups"
EDGE_UPS_TOPIC = "/savo_power/edge/ups"
BASE_BATTERY_TOPIC = "/savo_power/base/battery"

STATUS_TOPIC = "/savo_power/status"
HEALTH_TOPIC = "/savo_power/health"
DASHBOARD_TOPIC = "/savo_power/dashboard"
DASHBOARD_TEXT_TOPIC = "/savo_power/dashboard_text"

DEFAULT_I2C_BUS = 1

DEFAULT_UPS_HAT_ADDRESS = 0x36
DEFAULT_ADS7830_ADDRESS = 0x48
DEFAULT_ADS7830_CHANNEL = 2

DEFAULT_UPS_LOW_VOLTAGE = 3.40
DEFAULT_UPS_CRITICAL_VOLTAGE = 3.20

DEFAULT_BASE_EMPTY_VOLTAGE = 6.40
DEFAULT_BASE_LOW_VOLTAGE = 7.20
DEFAULT_BASE_FULL_VOLTAGE = 8.40

DEFAULT_BASE_LOW_SOC = 20.0
DEFAULT_BASE_FULL_SOC = 95.0

AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT = False

__all__ = [
    "__package_name__",
    "__version__",
    "ROBOT_NAME",
    "CORE_UPS_SOURCE",
    "EDGE_UPS_SOURCE",
    "BASE_BATTERY_SOURCE",
    "CORE_UPS_TOPIC",
    "EDGE_UPS_TOPIC",
    "BASE_BATTERY_TOPIC",
    "STATUS_TOPIC",
    "HEALTH_TOPIC",
    "DASHBOARD_TOPIC",
    "DASHBOARD_TEXT_TOPIC",
    "DEFAULT_I2C_BUS",
    "DEFAULT_UPS_HAT_ADDRESS",
    "DEFAULT_ADS7830_ADDRESS",
    "DEFAULT_ADS7830_CHANNEL",
    "DEFAULT_UPS_LOW_VOLTAGE",
    "DEFAULT_UPS_CRITICAL_VOLTAGE",
    "DEFAULT_BASE_EMPTY_VOLTAGE",
    "DEFAULT_BASE_LOW_VOLTAGE",
    "DEFAULT_BASE_FULL_VOLTAGE",
    "DEFAULT_BASE_LOW_SOC",
    "DEFAULT_BASE_FULL_SOC",
    "AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT",
]
