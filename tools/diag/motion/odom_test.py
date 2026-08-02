#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("odometry", "Observe wheel odometry while the base remains stopped", "/wheel/odom", "nav_msgs/msg/Odometry"))
