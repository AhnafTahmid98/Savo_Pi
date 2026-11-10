"""Installed-node ROS contracts. Run on Jazzy; no motor nodes are started."""
import ast
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import warnings

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 runtime is not installed")
from ament_index_python.packages import get_package_prefix  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import parameter_value_to_python  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]


def assert_discovered_history_depth(qos, endpoint):
    """Validate known metadata and explicitly report discovery's blind spot."""
    if qos.history == HistoryPolicy.UNKNOWN:
        warnings.warn(
            f"QoS discovery limitation for {endpoint}: history=UNKNOWN "
            f"(reported depth={qos.depth}); history/depth could not be verified "
            "using discovery. KEEP_LAST/depth 10 are not confirmed.",
            pytest.PytestWarning,
            stacklevel=2,
        )
    else:
        assert qos.history == HistoryPolicy.KEEP_LAST, f"{endpoint}: expected KEEP_LAST history, got {qos.history}"
        assert qos.depth == 10, f"{endpoint}: expected depth 10, got {qos.depth}"


def defaults():
    tree = ast.parse((PACKAGE / "savo_base/nodes/base_state_publisher_node.py").read_text())
    return {ast.literal_eval(n.args[0]): ast.literal_eval(n.args[1]) for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "declare_parameter"}


@pytest.fixture
def ros_probe(monkeypatch):
    # Own every ROS resource per scenario, including cleanup after setup failures.
    domain = 170 + os.getpid() % 30
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    with ExitStack() as cleanup:
        context = Context()
        rclpy.init(context=context, domain_id=domain)
        cleanup.callback(context.shutdown)
        node = rclpy.create_node("state_probe", context=context)
        cleanup.callback(node.destroy_node)
        executor = SingleThreadedExecutor(context=context)
        cleanup.callback(executor.shutdown)
        executor.add_node(node)
        # LIFO cleanup stops the executor before destroying the node and context.
        yield node, executor, domain


@pytest.mark.parametrize("executable", ["base_state_publisher_node.py", "base_state_publisher_node_cpp"])
@pytest.mark.parametrize("mode", ["normal", "disabled", "simulated", "board"])
def test_installed_node_contract(executable, mode, tmp_path, ros_probe):
    node, executor, domain = ros_probe
    prefix = "/state_test_" + uuid.uuid4().hex[:10]
    settings = defaults()
    settings.update({name: prefix + "/" + name for name in settings if name.endswith("_topic")})
    if mode == "board":
        settings["subscribe_motor_board_status"] = True
    if mode == "disabled":
        settings.update({name: False for name in settings if name.startswith("subscribe_")})
        settings.update(pretty_json=True, robot_name="SAVO ä 中", publish_hz=8.0, heartbeat_hz=2.0)
    # Local, isolated test transport. These tests never publish robot topic names.
    process = None
    summaries, heartbeats = [], []

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def until(predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            if process is not None and process.poll() is not None:
                log.flush()
                pytest.fail("node exited: " + (tmp_path / "node.log").read_text())
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "Timed out waiting for ROS contract"

    reliable = QoSProfile(depth=10)
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String, settings["summary_topic"],
                             lambda msg: summaries.append((time.monotonic(), msg.data)), reliable)
    node.create_subscription(Bool, settings["heartbeat_topic"],
                             lambda msg: heartbeats.append((time.monotonic(), msg.data)), reliable)
    publishers = {
        "cmd": node.create_publisher(Twist, settings["cmd_topic"], reliable),
        "stop": node.create_publisher(Bool, settings["safety_stop_topic"], sensor),
        "slowdown": node.create_publisher(Float32, settings["slowdown_topic"], sensor),
        "watchdog": node.create_publisher(String, settings["watchdog_topic"], reliable),
        "base": node.create_publisher(String, settings["base_state_topic"], reliable),
    }
    if mode == "board":
        publishers["board"] = node.create_publisher(String, settings["motor_board_status_topic"], reliable)
    clock = node.create_publisher(Clock, prefix + "/clock", reliable)
    command = [str(Path(get_package_prefix("savo_base")) / "lib/savo_base" / executable),
               "--ros-args", "-r", "__ns:=" + prefix, "-r", "/clock:=" + prefix + "/clock"]
    for name, value in settings.items():
        # JSON literals are valid YAML parameter values, including Unicode strings.
        command += ["-p", name + ":=" + json.dumps(value, ensure_ascii=False)]
    command += ["-p", "use_sim_time:=" + str(mode == "simulated").lower()]
    log = (tmp_path / "node.log").open("w+")
    try:
        process = subprocess.Popen(command, stdout=log, stderr=log,
                                   env={**os.environ, "ROS_DOMAIN_ID": str(domain)})
        client = node.create_client(GetParameters, prefix + "/base_state_publisher_node/get_parameters")
        until(lambda: client.service_is_ready())
        future = client.call_async(GetParameters.Request(names=list(settings)))
        until(future.done)
        actual = dict(zip(settings, map(parameter_value_to_python, future.result().values)))
        assert actual == settings  # Every custom parameter exists with matching type/value.

        contracts = [("cmd_topic", "subscribe_cmd", "geometry_msgs/msg/Twist", ReliabilityPolicy.RELIABLE),
                     ("safety_stop_topic", "subscribe_safety_stop", "std_msgs/msg/Bool", ReliabilityPolicy.BEST_EFFORT),
                     ("slowdown_topic", "subscribe_slowdown", "std_msgs/msg/Float32", ReliabilityPolicy.BEST_EFFORT),
                     ("watchdog_topic", "subscribe_watchdog", "std_msgs/msg/String", ReliabilityPolicy.RELIABLE),
                     ("base_state_topic", "subscribe_base_state", "std_msgs/msg/String", ReliabilityPolicy.RELIABLE),
                     ("motor_board_status_topic", "subscribe_motor_board_status", "std_msgs/msg/String",
                      ReliabilityPolicy.RELIABLE)]
        for key, enabled, msg_type, reliability in contracts:
            def endpoints():
                return [e for e in node.get_subscriptions_info_by_topic(settings[key])
                        if e.node_name == "base_state_publisher_node"]
            if settings[enabled]:
                until(lambda: len(endpoints()) == 1)
                endpoint = endpoints()[0]
                assert endpoint.topic_type == msg_type
                assert endpoint.qos_profile.reliability == reliability
                assert_discovered_history_depth(endpoint.qos_profile, f"subscription {settings[key]}")
                assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
            else:
                assert endpoints() == []
        for key, msg_type in [("summary_topic", "std_msgs/msg/String"), ("heartbeat_topic", "std_msgs/msg/Bool")]:
            endpoints = node.get_publishers_info_by_topic(settings[key])
            assert len(endpoints) == 1
            assert endpoints[0].topic_type == msg_type
            qos = endpoints[0].qos_profile
            assert qos.reliability == ReliabilityPolicy.RELIABLE
            assert_discovered_history_depth(qos, f"publisher {settings[key]}")
            assert qos.durability == DurabilityPolicy.VOLATILE

        if mode == "simulated":
            until(lambda: clock.get_subscription_count() == 1)
            pump(0.3)
            assert not summaries and not heartbeats

            def tick(sec, nanosec=0):
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = sec, nanosec
                clock.publish(msg)
                pump(0.15)
            tick(10)
            tick(11)
            until(lambda: summaries and heartbeats)
            pump(0.2)
            before = len(summaries), len(heartbeats)
            pump(0.4)
            assert before == (len(summaries), len(heartbeats))
            publishers["cmd"].publish(Twist())
            pump(0.4)  # freshness elapses while ROS time is frozen
            tick(12)
            until(lambda: len(summaries) > before[0])
            sample = json.loads(summaries[-1][1])
            assert sample["command_path"]["cmd_vel_safe"]["seen"]
            assert sample["command_path"]["cmd_vel_safe"]["stale"]
            assert sample["status_level"] == "CMD_STALE"
            assert all(value is True for _, value in heartbeats)
            return

        until(lambda: len(summaries) >= 3 and len(heartbeats) >= 2)
        assert all(value is True for _, value in heartbeats)
        sample = json.loads(summaries[-1][1])
        assert sample["node"] == "base_state_publisher_node"
        assert sample["robot_name"] == settings["robot_name"]
        assert sample["status_level"] == ("OK" if mode == "disabled" else "CMD_STALE")
        assert sample["timestamp_monotonic_s"] == pytest.approx(time.monotonic(), abs=1.0)
        counts = [json.loads(raw)["summary_pub_count"] for _, raw in summaries]
        assert counts == list(range(counts[0], counts[0] + len(counts)))
        assert (summaries[-1][0] - summaries[-2][0]) == pytest.approx(1 / settings["publish_hz"], abs=0.12)
        assert (heartbeats[-1][0] - heartbeats[-2][0]) == pytest.approx(1 / settings["heartbeat_hz"], abs=0.2)
        if mode == "disabled":
            assert all(value is None for value in sample["topics"]["inputs"].values())
            assert '\n  "node":' in summaries[-1][1]
            return

        for pub in publishers.values():
            until(lambda: pub.get_subscription_count() == 1)
        if mode == "board":
            publishers["board"].publish(String(data='{"status_level":"FAULT","board":7}'))
            until(lambda: json.loads(summaries[-1][1])["driver_state_topics"]["motor_board_status"]["seen"])
            sample = json.loads(summaries[-1][1])
            assert sample["driver_state_topics"]["motor_board_status"]["data"]["board"] == 7
            assert sample["status_level"] == "CMD_STALE"  # board does not affect priority
        publishers["watchdog"].publish(String(data='{"status_level":"ERROR","kept":42}'))
        until(lambda: json.loads(summaries[-1][1])["status_level"] == "ERROR")
        publishers["stop"].publish(Bool(data=True))
        until(lambda: json.loads(summaries[-1][1])["status_level"] == "SAFETY_STOP")
        pump(1.7)
        assert json.loads(summaries[-1][1])["status_level"] == "SAFETY_STOP"
        publishers["stop"].publish(Bool(data=False))
        publishers["watchdog"].publish(String(data="malformed"))
        until(lambda: json.loads(summaries[-1][1])["driver_state_topics"]["watchdog_state"]["parse_error_count"] == 1)
        sample = json.loads(summaries[-1][1])
        assert sample["driver_state_topics"]["watchdog_state"]["data"]["kept"] == 42
        assert sample["status_level"] == "ERROR"
    finally:
        try:
            if process is not None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        finally:
            log.close()
