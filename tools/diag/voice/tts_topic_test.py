#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("tts_topic", "Observe robot response text without requesting synthesis", "/savo_speech/response", "std_msgs/msg/String"))
