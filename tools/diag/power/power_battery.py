#!/usr/bin/env python3
"""Observe one normalized Robot SAVO base-battery sample without touching I2C."""


import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

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
