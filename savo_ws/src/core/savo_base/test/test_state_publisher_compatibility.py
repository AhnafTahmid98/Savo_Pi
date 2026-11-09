"""Differential tests execute the unmodified Python logic and compiled C++ core."""
import importlib.util
import json
import math
import os
import shlex
from pathlib import Path
import subprocess
import sys
import types

import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def cpp_runner(tmp_path_factory):
    source = PACKAGE / "src/state/state_summary.cpp"
    assert source.exists(), "C++ state summary core is not implemented"
    binary = tmp_path_factory.mktemp("state-summary") / "runner"
    command = [os.environ.get("CXX", "g++"), "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
               "-I", str(PACKAGE / "include")]
    command += shlex.split(os.environ.get("CXXFLAGS", ""))
    if os.environ.get("RAPIDJSON_INCLUDE_DIR"):
        command += ["-isystem", os.environ["RAPIDJSON_INCLUDE_DIR"]]
    command += [str(PACKAGE / "test/state_summary_runner.cpp"), str(source), "-o", str(binary)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    return binary


@pytest.fixture
def reference(monkeypatch):
    # Only transport is replaced: all parameters, callbacks, aggregation and JSON
    # encoding below execute from the actual, unchanged Python production source.
    class Message:
        def __init__(self, data=None):
            self.data = data
            self.linear = types.SimpleNamespace(x=0.0, y=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    class Node:
        overrides = {}

        def __init__(self, name):
            self.params = {}
            self.published = {}

        def declare_parameter(self, name, value):
            self.params[name] = self.overrides.get(name, value)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.params[name])

        def create_publisher(self, kind, topic, qos):
            return types.SimpleNamespace(publish=lambda msg: self.published.__setitem__(topic, msg.data))

        def create_subscription(self, *args):
            return None

        def create_timer(self, *args):
            return None

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *args: None)

    modules = {
        "rclpy": {}, "rclpy.node": {"Node": Node},
        "rclpy.qos": {"QoSProfile": lambda **kw: kw,
                      "ReliabilityPolicy": types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
                      "HistoryPolicy": types.SimpleNamespace(KEEP_LAST=1)},
        "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": Message},
        "std_msgs": {}, "std_msgs.msg": {"Bool": Message, "Float32": Message, "String": Message},
    }
    for name, attrs in modules.items():
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
    spec = importlib.util.spec_from_file_location(
        "state_reference", PACKAGE / "savo_base/nodes/base_state_publisher_node.py")
    mod = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, mod)
    spec.loader.exec_module(mod)
    return mod, Node, Message


def assert_equivalent(expected, actual):
    if isinstance(expected, dict):
        assert expected.keys() == actual.keys()
        for key in expected:
            assert_equivalent(expected[key], actual[key])
    elif isinstance(expected, list):
        assert len(expected) == len(actual)
        for left, right in zip(expected, actual):
            assert_equivalent(left, right)
    elif isinstance(expected, float) and math.isnan(expected):
        assert math.isnan(actual)
    else:
        assert expected == actual


def compare(cpp_runner, reference, events, params=None):
    mod, node_type, message = reference
    node_type.overrides = params or {}
    mod._now_mono = lambda: 100.0
    node = mod.BaseStatePublisherNode()
    expected = []
    for event in events:
        mod._now_mono = lambda: event["t"]
        op = event["op"]
        if op == "summary":
            node._publish_summary()
            expected.append(json.loads(node.published[node.summary_topic]))
        elif op == "cmd":
            msg = message()
            msg.linear.x, msg.linear.y, msg.angular.z = event["value"]
            node._on_cmd_vel(msg)
        else:
            callback = {"stop": "_on_safety_stop", "slowdown": "_on_slowdown",
                        "watchdog": "_on_watchdog_json", "base": "_on_base_state_json",
                        "board": "_on_motor_board_status_json"}[op]
            getattr(node, callback)(message(event["value"]))
    result = subprocess.run([str(cpp_runner)], input=json.dumps({"params": params or {}, "events": events}),
                            capture_output=True, text=True, check=True)
    actual = [json.loads(json.loads(line)) for line in result.stdout.splitlines()]
    assert len(actual) == len(expected)
    for left, right in zip(expected, actual):
        assert_equivalent(left, right)
    return actual


def test_startup_and_clamps(cpp_runner, reference):
    compare(cpp_runner, reference, [{"op": "summary", "t": 100.0}, {"op": "summary", "t": 101.0}])
    compare(cpp_runner, reference, [{"op": "summary", "t": 100.0}], {
        "subscribe_cmd": False, "subscribe_watchdog": False, "subscribe_base_state": False,
        "subscribe_slowdown": False, "subscribe_safety_stop": False, "subscribe_motor_board_status": True,
        "robot_name": "SAVO — 测试", "cmd_stale_s": -1.0, "safety_stale_s": 0.0,
        "watchdog_stale_s": 0.0, "base_state_stale_s": 0.0, "motor_board_status_stale_s": 0.0,
        "publish_hz": 0.0, "heartbeat_hz": 0.0, "pretty_json": True,
        "summary_topic": "/test/summary", "heartbeat_topic": "/test/heartbeat"})


def test_priority_freshness_and_retained_data(cpp_runner, reference):
    events = []

    def add(op, t, value=None):
        events.append(dict(op=op, t=t, value=value))
        events.append(dict(op="summary", t=t))
    add("cmd", 100.0, [0.5, -0.2, 0.3])
    add("slowdown", 100.0, 0.25)
    add("base", 100.0, '{"status_level":"warning","nested":[1,true,null]}')
    for level in ["error", "stale", "tripped", "fault", "warn", "OK"]:
        add("watchdog", 100.0, json.dumps({"status_level": level}))
    add("watchdog", 100.0, '{"status_level":"ERROR"}')
    add("stop", 100.0, True)
    add("summary", 102.0)
    add("stop", 102.0, False)
    add("watchdog", 102.0, "bad")
    add("watchdog", 102.0, "bad")
    add("summary", 103.5)
    add("cmd", 104.0, [0, 0, 0])
    add("summary", 104.25)
    add("summary", 104.5)
    add("summary", 103.0)  # backward test clock clamps age and uptime
    compare(cpp_runner, reference, events, {"cmd_stale_s": 0.5})


@pytest.mark.parametrize("raw", [
    "{}", "[]", "null", "true", '"hello 🌍"', "42", "-1.25e3",
    '{"status_level":"FAULT","status_level":"warning"}',
    '{"status_level":null}', '{"nested":{"status_level":"ERROR"}}',
    '{"x":NaN,"y":Infinity,"z":-Infinity}',
    '{"big":123456789012345678901234567890}',
    "", "{", '{"trailing":1,}', "{} garbage", "{}\u0000garbage", "Inf", "-NaN",
    '{"quoted":"Infinity \\" NaN","unicode":"äö中"}',
])
def test_json_compatibility(cpp_runner, reference, raw):
    events = [{"op": "board", "t": 100.0, "value": '{"old":1}'}]
    for op in ["watchdog", "base", "board"]:
        events += [{"op": op, "t": 101.0, "value": raw}] * 2
    events.append({"op": "summary", "t": 101.0})
    compare(cpp_runner, reference, events)


@pytest.mark.parametrize("pretty", [False, True])
def test_nonfinite_motion_unicode_status_and_extreme_numbers(cpp_runner, reference, pretty):
    events = [
        {"op": "cmd", "t": 100.0, "value": [float("nan"), float("inf"), -float("inf")]},
        {"op": "slowdown", "t": 100.0, "value": float("nan")},
        {"op": "watchdog", "t": 100.0, "value": '{"status_level":"trıpped"}'},
        {"op": "summary", "t": 100.0},
        {"op": "watchdog", "t": 100.0, "value": '{"status_level":"ſtale"}'},
        {"op": "summary", "t": 100.0},
        {"op": "base", "t": 100.0, "value": '{"large_float":1e999,"large_int":' + "9" * 400 + '}'},
        {"op": "summary", "t": 100.0},
    ]
    compare(cpp_runner, reference, events, {"pretty_json": pretty})


@pytest.mark.parametrize("number", ["9" * 50000, "9" * 50000 + ".0", "1e" + "9" * 50000,
                                    "1e-" + "9" * 50000, "1e999oops"],
                         ids=["long_int", "long_float", "long_exponent", "long_negative_exponent", "invalid"])
def test_oversized_numbers_do_not_crash_and_match_python(cpp_runner, reference, number):
    compare(cpp_runner, reference, [
        {"op": "watchdog", "t": 100.0, "value": '{"status_level":"FAULT"}'},
        {"op": "watchdog", "t": 101.0, "value": '{"number":' + number + '}'},
        {"op": "summary", "t": 101.0},
    ])


@pytest.mark.parametrize("level", ["ﬅale", "ﬆale"])
def test_unicode_status_ligatures(cpp_runner, reference, level):
    compare(cpp_runner, reference, [
        {"op": "watchdog", "t": 100.0, "value": json.dumps({"status_level": level})},
        {"op": "summary", "t": 100.0},
    ])
