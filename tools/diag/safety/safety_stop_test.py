#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("safety_stop", "Observe the authoritative safety stop", "/safety/stop", "std_msgs/msg/Bool"))
