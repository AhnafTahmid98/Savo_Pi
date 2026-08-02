#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("asr_topic", "Observe privacy-filtered speech transcript", "/savo_speech/transcript", "std_msgs/msg/String"))
