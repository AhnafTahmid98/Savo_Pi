import json
import py_compile
from pathlib import Path

from savo_power.nodes import power_health_node_py


class FakeString:
    def __init__(self, data):
        self.data = data


def test_power_health_python_node_module_compiles():
    py_compile.compile(
        "savo_power/nodes/power_health_node_py.py",
        doraise=True,
    )


def test_power_health_python_node_exports_runtime_helpers():
    for name in [
        "PowerHealthMemory",
        "PowerHealthNodeState",
        "health_python_node_name",
        "health_topic",
        "shutdown_request_topic",
        "create_timer_period_s",
        "parse_status_payload",
        "status_overall_state",
        "status_ok",
        "evaluate_status_dict",
        "evaluate_memory",
        "health_to_publish_text",
        "build_startup_summary",
        "main",
    ]:
        assert hasattr(power_health_node_py, name), name


def test_power_health_node_identity_is_locked():
    assert power_health_node_py.health_python_node_name() == "power_health_node_py"
    assert power_health_node_py.health_topic() == "/savo_power/health"
    assert (
        power_health_node_py.shutdown_request_topic()
        == "/savo_power/shutdown_request"
    )


def test_power_health_timer_period_helper_is_safe():
    assert power_health_node_py.create_timer_period_s(1.0) == 1.0
    assert power_health_node_py.create_timer_period_s(2.0) == 0.5
    assert power_health_node_py.create_timer_period_s(0.0) > 0.0


def test_power_health_node_state_defaults_are_safe():
    state = power_health_node_py.PowerHealthNodeState()

    for field_name in [
        "publish_count",
        "callback_count",
        "error_count",
        "shutdown_request_count",
    ]:
        if hasattr(state, field_name):
            assert getattr(state, field_name) == 0

    if hasattr(state, "last_error"):
        assert state.last_error == ""


def test_power_health_memory_defaults_are_safe():
    memory = power_health_node_py.PowerHealthMemory()

    if hasattr(memory, "parse_error_count"):
        assert memory.parse_error_count == 0

    if hasattr(memory, "last_parse_error"):
        assert memory.last_parse_error == ""


def test_parse_status_payload_accepts_dict_json_and_string_message():
    payload = {
        "overall_state": "ok",
        "state": "ok",
        "ok": True,
        "sources": [
            {
                "source": "core_ups",
                "state": "ok",
                "ok": True,
                "expected": True,
                "seen": True,
                "age_s": 1.0,
            }
        ],
    }

    parsed_from_dict = power_health_node_py.parse_status_payload(payload)
    parsed_from_json = power_health_node_py.parse_status_payload(json.dumps(payload))
    parsed_from_msg = power_health_node_py.parse_status_payload(
        FakeString(json.dumps(payload))
    )

    assert parsed_from_dict["overall_state"] == "ok"
    assert parsed_from_json["overall_state"] == "ok"
    assert parsed_from_msg["overall_state"] == "ok"


def test_status_helpers_read_ok_status_payload():
    payload = {
        "overall_state": "ok",
        "state": "ok",
        "ok": True,
        "sources": [
            {
                "source": "core_ups",
                "state": "ok",
                "ok": True,
                "expected": True,
                "seen": True,
                "age_s": 1.0,
            }
        ],
    }

    assert str(power_health_node_py.status_overall_state(payload)).lower().endswith("ok")
    assert power_health_node_py.status_ok(payload) is True

    items = tuple(power_health_node_py.status_source_items(payload))

    assert items


def test_health_evaluation_does_not_request_shutdown_when_disabled():
    payload = {
        "overall_state": "ok",
        "state": "ok",
        "ok": True,
        "sources": [],
    }

    result = power_health_node_py.evaluate_status_dict(
        payload,
        automatic_shutdown_enabled=False,
    )

    text = power_health_node_py.health_to_publish_text(result)

    assert "shutdown" in text.lower() or "ok" in text.lower()

    if hasattr(result, "shutdown_requested"):
        assert result.shutdown_requested is False


def test_power_health_cpp_sources_exist():
    for path in [
        "src/nodes/power_health_node.cpp",
        "src/health/power_health.cpp",
        "include/savo_power/power_health.hpp",
    ]:
        assert Path(path).is_file(), path


def test_power_health_cpp_node_has_ros_runtime_shape():
    text = Path("src/nodes/power_health_node.cpp").read_text()

    assert "rclcpp::init" in text
    assert "rclcpp::spin" in text
    assert "rclcpp::shutdown" in text
    assert "PowerHealthNode" in text


def test_power_health_cpp_contract_mentions_health_states():
    text = (
        Path("src/health/power_health.cpp").read_text()
        + "\n"
        + Path("include/savo_power/power_health.hpp").read_text()
    ).lower()

    assert "health" in text
    assert "ok" in text
    assert "critical" in text
    assert "stale" in text


def test_power_health_script_wrapper_exists():
    path = Path("scripts/power_health_node_py")

    assert path.is_file()

    text = path.read_text()

    assert "savo_power.nodes.power_health_node_py" in text
    assert "main" in text


def test_cmake_references_power_health_sources_and_node():
    text = Path("CMakeLists.txt").read_text()

    assert "src/health/power_health.cpp" in text
    assert "src/nodes/power_health_node.cpp" in text
    assert "power_health_node" in text


def test_power_health_node_does_not_enable_shutdown_or_motor_control():
    files = [
        "savo_power/nodes/power_health_node_py.py",
        "src/health/power_health.cpp",
        "include/savo_power/power_health.hpp",
        "src/nodes/power_health_node.cpp",
        "scripts/power_health_node_py",
    ]

    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in files
        if Path(path).is_file()
    ).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
    ]:
        assert forbidden not in combined
