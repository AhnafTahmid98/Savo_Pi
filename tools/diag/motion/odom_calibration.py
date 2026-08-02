#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("odom_calibration", "Collect one odometry sample; motion requires a physical operator", "/wheel/odom", "nav_msgs/msg/Odometry", motion=True))
