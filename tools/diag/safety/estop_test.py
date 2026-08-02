#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("estop", "Observe the external STOP latch without changing it", "/savo_control/external_stop", "std_msgs/msg/Bool"))
