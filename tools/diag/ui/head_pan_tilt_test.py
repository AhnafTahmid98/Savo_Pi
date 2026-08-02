#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("head_pan_tilt", "Observe head state; actuation is never issued by this tool", "/savo_head/state", "std_msgs/msg/String", motion=True))
