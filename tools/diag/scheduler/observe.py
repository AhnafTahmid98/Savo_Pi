#!/usr/bin/env python3
"""Common A/B observer: best-effort subscriptions, bounded RAM, write after capture."""
import argparse
import json
from pathlib import Path
import signal
import time

TOPICS = ("/savo_localization/health", "/savo_bringup/core/startup_trace",
          "/savo_localization/imu_state", "/savo_localization/wheel_odom_state",
          "/savo_control/mode_state", "/savo_base/base_state")
MAX_BYTES = 32 * 1024 * 1024


def safety_error(topic, data):
    if topic == "/savo_control/mode_state" and data.strip() != "STOP":
        return "Control mode left STOP; experiment invalid"
    if topic == "/savo_base/base_state":
        try:
            value = json.loads(data)
            if (value["backend"]["board_backend"] != "dryrun" or
                    value["limits"]["max_duty"] != 0 or
                    any(value["last_duty"][wheel] != 0 for wheel in ("fl", "fr", "rl", "rr"))):
                return "Base motor-off invariant failed; experiment invalid"
        except (ValueError, KeyError, TypeError):
            return "Cannot verify base motor-off state; experiment invalid"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=int, required=True)
    args = parser.parse_args()
    if not 10 <= args.duration <= 200:
        parser.error("duration must be 10–200 seconds")
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import get_rmw_implementation_identifier
    from std_msgs.msg import String

    # Shutdown is handled here so SIGINT flushes the finite RAM buffer.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("scheduler_experiment_observer")
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    records, subscriptions = [], []
    size, stopped, error = 0, False, None

    def stop_requested(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)

    def receive(topic, message, info):
        nonlocal size, stopped, error
        mono_before = time.monotonic_ns()
        realtime = time.time_ns()
        mono_after = time.monotonic_ns()
        size += len(message.data.encode("utf-8")) + 512
        if size > MAX_BYTES:
            error, stopped = "observer RAM limit exceeded; capture incomplete", True
            return
        records.append({"topic": topic, "observer_monotonic_ns": (mono_before + mono_after) // 2,
                        "observer_realtime_ns": realtime,
                        "clock_pair_uncertainty_ns": (mono_after - mono_before + 1) // 2,
                        "dds_source_timestamp_ns": info["source_timestamp"],
                        "dds_received_timestamp_ns": info["received_timestamp"],
                        "data": message.data})
        problem = safety_error(topic, message.data)
        if problem:
            error, stopped = problem, True

    for topic in TOPICS:
        # rclpy selects the MessageInfo callback signature by argument count.
        def callback(message, info, *, topic=topic):
            receive(topic, message, info)
        subscriptions.append(node.create_subscription(String, topic, callback, qos))
    args.output.joinpath("observer-ready").write_text(str(time.monotonic_ns()))
    start = time.monotonic()
    endpoints = {}
    try:
        while not stopped and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        for topic in TOPICS:
            endpoints[topic] = {kind: [{"node": item.node_namespace + "/" + item.node_name,
                                       "qos": str(item.qos_profile),
                                       "gid": list(item.endpoint_gid)} for item in getter(topic)]
                for kind, getter in (("publishers", node.get_publishers_info_by_topic),
                                     ("subscriptions", node.get_subscriptions_info_by_topic))}
        with (args.output / "observer.jsonl").open("w") as stream:
            for item in records:
                stream.write(json.dumps(item) + "\n")
        (args.output / "observer-summary.json").write_text(json.dumps({
            "messages": len(records), "estimated_buffer_bytes": size, "error": error,
            "rmw": get_rmw_implementation_identifier(), "qos": str(qos),
            "endpoints_at_shutdown": endpoints}, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    if error:
        raise SystemExit(error)


if __name__ == "__main__":
    main()
