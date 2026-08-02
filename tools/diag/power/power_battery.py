#!/usr/bin/env python3
"""Observe one normalized Robot SAVO base-battery sample without touching I2C."""

from tools.diag.infra.diag_utils import main_topic


if __name__ == "__main__":
    raise SystemExit(
        main_topic(
            "power_battery",
            "Observe the normalized core base-battery diagnostic topic.",
            "/savo_power/base/battery",
            "sensor_msgs/msg/BatteryState",
        )
    )
