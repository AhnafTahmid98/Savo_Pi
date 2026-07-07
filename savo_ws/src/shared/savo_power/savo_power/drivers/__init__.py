"""Python diagnostic drivers for Robot Savo power monitoring.

These drivers are for diagnostics, quick hardware checks, and fallback tools.
The production runtime for Robot Savo power monitoring remains C++.
"""

from __future__ import annotations

from savo_power.drivers.smbus_adapter import (
    I2cBusError,
    SmbusAdapter,
    checked_byte,
    checked_i2c_address,
    checked_register,
    format_i2c_address,
    is_reserved_7bit_i2c_address,
    is_usable_7bit_i2c_address,
    is_valid_7bit_i2c_address,
    load_smbus_backend_class,
    make_linux_i2c_device_path,
)

from savo_power.drivers.ups_hat import (
    UpsHatConfig,
    UpsHatDriver,
    WordReadableBus,
    make_core_ups_driver,
    make_edge_ups_driver,
    make_ups_hat_driver,
    read_ups_once,
)

from savo_power.drivers.ads7830 import (
    Ads7830Config,
    Ads7830Driver,
    ByteReadableWritableBus,
    make_ads7830_driver,
    read_kit_battery_once,
)


__all__ = [
    "I2cBusError",
    "SmbusAdapter",
    "checked_byte",
    "checked_i2c_address",
    "checked_register",
    "format_i2c_address",
    "is_reserved_7bit_i2c_address",
    "is_usable_7bit_i2c_address",
    "is_valid_7bit_i2c_address",
    "load_smbus_backend_class",
    "make_linux_i2c_device_path",
    "UpsHatConfig",
    "UpsHatDriver",
    "WordReadableBus",
    "make_core_ups_driver",
    "make_edge_ups_driver",
    "make_ups_hat_driver",
    "read_ups_once",
    "Ads7830Config",
    "Ads7830Driver",
    "ByteReadableWritableBus",
    "make_ads7830_driver",
    "read_kit_battery_once",
]
