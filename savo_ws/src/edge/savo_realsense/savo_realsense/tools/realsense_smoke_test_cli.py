#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid

import subprocess
import sys

from savo_realsense.diagnostics import (
    format_missing_topics_report,
    format_usb_report,
    list_usb_devices,
    missing_topics,
)
from savo_realsense.ros.topic_contract import REQUIRED_IMAGE_TOPICS


def ros2_topic_list() -> list[str]:
    result = subprocess.run(
        ["ros2", "topic", "list"],
        check=False,
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        return []

    return [
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    ]


def main() -> int:
    usb_lines = list_usb_devices()
    available_topics = ros2_topic_list()
    missing = missing_topics(available_topics, REQUIRED_IMAGE_TOPICS)

    print(format_usb_report(usb_lines))
    print()
    print(format_missing_topics_report(missing))

    return 0 if usb_lines and not missing else 1


if __name__ == "__main__":
    sys.exit(main())
