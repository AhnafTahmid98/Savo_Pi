#!/usr/bin/env bash

set -Ee
set -o pipefail

cd ~/Savo_Pi/savo_ws || exit 1

source /opt/ros/jazzy/setup.bash

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

set -u

python3 - <<'PY'
import os
import re
import socket
import subprocess
from datetime import datetime, timezone


def run(command, timeout=12):
    try:
        completed = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )

        return completed.returncode, completed.stdout.rstrip()

    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""

        if isinstance(output, bytes):
            output = output.decode(
                errors="replace",
            )

        return (
            124,
            output.rstrip() + "\n[TIMEOUT]",
        )


def heading(title):
    print()
    print("=" * 80)
    print(title)
    print("=" * 80)


print("Robot Savo Phase 2C-1 live interface inventory")
print(
    "UTC time:",
    datetime.now(timezone.utc).isoformat(),
)
print("Local hostname:", socket.gethostname())
print(
    "ROS_DOMAIN_ID:",
    os.environ.get("ROS_DOMAIN_ID", "<unset>"),
)
print(
    "RMW_IMPLEMENTATION:",
    os.environ.get(
        "RMW_IMPLEMENTATION",
        "<default>",
    ),
)

heading("ROS nodes")

node_code, node_output = run(
    ["ros2", "node", "list"],
    timeout=15,
)

print(node_output or "<none>")

nodes = sorted({
    line.strip()
    for line in node_output.splitlines()
    if line.strip().startswith("/")
})

heading("ROS topics and message types")

topic_code, topic_output = run(
    ["ros2", "topic", "list", "-t"],
    timeout=15,
)

print(topic_output or "<none>")

topic_pattern = re.compile(
    r"^(?P<name>/\S+)\s+\[(?P<types>.*)\]$"
)

topics = []

for line in topic_output.splitlines():
    match = topic_pattern.match(line.strip())

    if match is None:
        continue

    topic_name = match.group("name")

    topic_types = [
        item.strip()
        for item in match.group("types").split(",")
        if item.strip()
    ]

    topics.append(
        (topic_name, topic_types)
    )

ignored_topics = {
    "/parameter_events",
    "/rosout",
}

candidate_pattern = re.compile(
    r"(heartbeat|health|status|state|readiness|"
    r"safety|power|battery|voltage|current|"
    r"temperature|diagnostic|odom|mode|"
    r"localization|emergency|estop|fault|"
    r"watchdog|ready)",
    re.IGNORECASE,
)

candidates = [
    (topic_name, topic_types)
    for topic_name, topic_types in topics
    if topic_name not in ignored_topics
    and not topic_name.startswith("/savo_bridge/")
    and candidate_pattern.search(topic_name)
]

heading("Candidate read-only observation topics")

if not candidates:
    print("<none matched>")
else:
    for topic_name, topic_types in candidates:
        print(
            f"{topic_name}: "
            f"{', '.join(topic_types)}"
        )

heading("Candidate topic endpoint details")

for topic_name, topic_types in candidates:
    print()
    print("-" * 80)
    print("TOPIC:", topic_name)
    print("TYPES:", ", ".join(topic_types))
    print("-" * 80)

    code, output = run(
        [
            "ros2",
            "topic",
            "info",
            "-v",
            topic_name,
        ],
        timeout=15,
    )

    print(
        output
        or f"<no output; exit={code}>"
    )

heading("Candidate topic rates")

for topic_name, _ in candidates:
    print()
    print("-" * 80)
    print("RATE:", topic_name)
    print("-" * 80)

    code, output = run(
        [
            "timeout",
            "--signal=INT",
            "4s",
            "ros2",
            "topic",
            "hz",
            topic_name,
            "--window",
            "20",
        ],
        timeout=7,
    )

    filtered_lines = [
        line
        for line in output.splitlines()
        if (
            "average rate:" in line
            or "min:" in line
            or "max:" in line
            or "std dev:" in line
            or "no new messages" in line.lower()
            or "unknown topic" in line.lower()
            or "could not determine" in line.lower()
        )
    ]

    if filtered_lines:
        print("\n".join(filtered_lines))
    elif output:
        print(output)
    else:
        print(
            f"<no rate evidence; exit={code}>"
        )

heading("Robot Savo node interface details")

robot_nodes = [
    node_name
    for node_name in nodes
    if (
        "savo" in node_name.lower()
        or "slam_toolbox" in node_name
        or "nav2" in node_name.lower()
        or "robot_state_publisher" in node_name
    )
    and not node_name.startswith(
        "/savo_bridge_test/"
    )
    and not node_name.startswith("/phase2")
]

if not robot_nodes:
    print("<no Robot Savo nodes matched>")
else:
    for node_name in robot_nodes:
        print()
        print("-" * 80)
        print("NODE:", node_name)
        print("-" * 80)

        code, output = run(
            [
                "ros2",
                "node",
                "info",
                node_name,
            ],
            timeout=15,
        )

        print(
            output
            or f"<no output; exit={code}>"
        )

heading("Local ROS-related processes")

code, output = run(
    [
        "bash",
        "-lc",
        (
            "ps -eo pid=,comm=,args= | "
            "grep -E "
            "'savo_|ros2|component_container|"
            "slam_toolbox|nav2|robot_state_publisher' | "
            "grep -v grep || true"
        ),
    ],
    timeout=10,
)

print(output or "<none>")

heading("Inventory summary")

print("Node count:", len(nodes))
print("Topic count:", len(topics))
print(
    "Candidate observation topics:",
    len(candidates),
)
print(
    "Matched Robot Savo nodes:",
    len(robot_nodes),
)

if node_code != 0:
    print(
        "WARNING: ros2 node list exit code:",
        node_code,
    )

if topic_code != 0:
    print(
        "WARNING: ros2 topic list exit code:",
        topic_code,
    )

print()
print("Phase 2C-1 inventory complete.")
print("No package files were changed.")
PY
