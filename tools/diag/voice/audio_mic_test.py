#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("audio_mic", "Observe speech audio diagnostics; raw audio is not published", "/savo_speech/status", "std_msgs/msg/String"))
