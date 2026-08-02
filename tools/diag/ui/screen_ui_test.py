#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("screen_ui", "Observe shared diagnostics without opening the framebuffer", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"))
