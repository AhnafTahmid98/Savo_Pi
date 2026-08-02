#!/usr/bin/env python3
from tools.diag.infra.diag_utils import main_topic

raise SystemExit(main_topic("tf_tree", "Observe one TF sample without changing transforms", "/tf", "tf2_msgs/msg/TFMessage"))
