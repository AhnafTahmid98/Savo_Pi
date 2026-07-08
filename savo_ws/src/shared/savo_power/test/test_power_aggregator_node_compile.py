import py_compile
from pathlib import Path

from savo_power.nodes import power_aggregator_node_py


def test_power_aggregator_python_node_module_compiles():
    py_compile.compile(
        "savo_power/nodes/power_aggregator_node_py.py",
        doraise=True,
    )


def test_power_aggregator_python_node_exports_runtime_helpers():
    for name in [
        "PowerAggregatorMemory",
        "PowerAggregatorNodeState",
        "aggregator_python_node_name",
        "status_topic",
        "create_timer_period_s",
        "parse_reading_payload",
        "build_source_statuses",
        "compute_overall_state",
        "aggregate_memory",
        "build_startup_summary",
        "main",
    ]:
        assert hasattr(power_aggregator_node_py, name), name


def test_power_aggregator_node_identity_is_locked():
    assert (
        power_aggregator_node_py.aggregator_python_node_name()
        == "power_aggregator_node_py"
    )
    assert power_aggregator_node_py.status_topic() == "/savo_power/status"


def test_power_aggregator_timer_period_helper_is_safe():
    assert power_aggregator_node_py.create_timer_period_s(1.0) == 1.0
    assert power_aggregator_node_py.create_timer_period_s(2.0) == 0.5
    assert power_aggregator_node_py.create_timer_period_s(0.0) > 0.0


def test_power_aggregator_node_state_defaults_are_safe():
    state = power_aggregator_node_py.PowerAggregatorNodeState()

    assert state.publish_count == 0
    assert state.callback_count == 0
    assert state.error_count == 0
    assert state.last_error == ""


def test_power_aggregator_memory_defaults_are_safe():
    memory = power_aggregator_node_py.PowerAggregatorMemory()

    assert memory.parse_error_count == 0
    assert memory.last_parse_error == ""
    assert memory.to_dict()["readings_by_source"] == {}


def test_power_aggregator_cpp_sources_exist():
    for path in [
        "src/nodes/power_aggregator_node.cpp",
        "src/aggregation/power_aggregator.cpp",
        "include/savo_power/power_aggregator.hpp",
    ]:
        assert Path(path).is_file(), path


def test_power_aggregator_cpp_node_has_ros_runtime_shape():
    text = Path("src/nodes/power_aggregator_node.cpp").read_text()

    assert "rclcpp::init" in text
    assert "rclcpp::spin" in text
    assert "rclcpp::shutdown" in text
    assert "PowerAggregatorNode" in text


def test_power_aggregator_cpp_contract_mentions_expected_sources():
    text = (
        Path("src/aggregation/power_aggregator.cpp").read_text()
        + "\n"
        + Path("include/savo_power/power_aggregator.hpp").read_text()
    )

    assert "core_ups" in text
    assert "edge_ups" in text
    assert "base_battery" in text


def test_power_aggregator_script_wrapper_exists():
    path = Path("scripts/power_aggregator_node_py")

    assert path.is_file()

    text = path.read_text()

    assert "savo_power.nodes.power_aggregator_node_py" in text
    assert "main" in text


def test_cmake_references_power_aggregator_sources_and_node():
    text = Path("CMakeLists.txt").read_text()

    assert "src/aggregation/power_aggregator.cpp" in text
    assert "src/nodes/power_aggregator_node.cpp" in text
    assert "power_aggregator_node" in text


def test_power_aggregator_node_does_not_enable_shutdown_or_motor_control():
    files = [
        "savo_power/nodes/power_aggregator_node_py.py",
        "src/aggregation/power_aggregator.cpp",
        "include/savo_power/power_aggregator.hpp",
        "src/nodes/power_aggregator_node.cpp",
        "scripts/power_aggregator_node_py",
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
