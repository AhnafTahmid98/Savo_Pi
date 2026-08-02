#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("apriltag", "Observe AprilTag observations from the head detector", "/savo_head/apriltag_detections", "savo_msgs/msg/AprilTagObservation"))
