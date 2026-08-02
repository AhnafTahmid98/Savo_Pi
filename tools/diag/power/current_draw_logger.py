#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("current_draw", "Capture one bounded power status sample", "/savo_power/status", "std_msgs/msg/String"))
