#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("cmd_vel_gate", "Observe safety-gated velocity; never publishes velocity", "/cmd_vel_safe", "geometry_msgs/msg/Twist"))
