#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("audio_speaker", "Observe playback state; speaker actuation needs operator approval", "/savo_speech/status", "std_msgs/msg/String", motion=True))
