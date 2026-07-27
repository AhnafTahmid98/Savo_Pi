from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]

TOPICS_PATH = ROOT / "include/savo_control/topic_names.hpp"
NODE_PATH = ROOT / "src/nodes/rotate_to_heading_node.cpp"
CONFIG_PATH = ROOT / "config/rotate_to_heading.yaml"
CMAKE_PATH = ROOT / "CMakeLists.txt"


EXPECTED_TOPICS = {
    "ROTATE_TARGET_RAD": "/savo_control/rotate_target_rad",
    "ROTATE_ENABLE": "/savo_control/rotate_enable",
    "ROTATE_CANCEL": "/savo_control/rotate_cancel",
    "ROTATE_STATE": "/savo_control/rotate_state",
    "ROTATE_STATUS": "/savo_control/rotate_status",
}


EXPECTED_PARAMETERS = {
    "publish_hz",
    "input_timeout_s",
    "enabled",
    "start_on_target",
    "auto_disable_when_goal_reached",
    "safety_stop_blocks_motion",
    "publish_zero_when_inactive",
    "target_tolerance_rad",
    "max_duration_s",
    "kp",
    "ki",
    "kd",
    "max_wz_rad_s",
    "min_wz_when_active",
    "disable_min_wz_below_error_rad",
    "output_deadband_rad_s",
    "min_dt_sec",
    "max_dt_sec",
    "odom_topic",
    "target_topic",
    "enable_topic",
    "cancel_topic",
    "safety_stop_topic",
    "output_topic",
    "state_topic",
    "status_topic",
}


LEGACY_PARAMETERS = {
    "rotate_target_topic",
    "publish_rate_hz",
    "odom_timeout_sec",
    "zero_on_stale_odom",
    "auto_enable_on_target",
    "stop_and_hold_zero_after_done",
    "reset_pid_on_new_target",
    "settle_cycles_required",
    "max_rotate_time_sec",
    "min_error_to_command_rad",
    "output",
    "pid",
    "heading_tolerance_rad",
    "min_effective_wz_rad_s",
    "reset_pid_on_hold_capture",
    "reset_pid_on_target_change",
    "target_change_reset_threshold_rad",
    "ctrl",
}


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def rotation_parameters() -> dict:
    document = yaml.safe_load(read_text(CONFIG_PATH))

    assert isinstance(document, dict)
    assert set(document) == {"rotate_to_heading_node"}

    node = document["rotate_to_heading_node"]
    assert isinstance(node, dict)
    assert set(node) == {"ros__parameters"}

    parameters = node["ros__parameters"]
    assert isinstance(parameters, dict)

    return parameters


def test_canonical_rotation_topic_constants() -> None:
    header = read_text(TOPICS_PATH)

    for name, topic in EXPECTED_TOPICS.items():
        declaration = (
            f'inline constexpr const char * {name} = "{topic}";'
        )
        assert declaration in header


def test_rotation_topics_are_status_topics() -> None:
    header = read_text(TOPICS_PATH)

    assert "topic == ROTATE_STATE" in header
    assert "topic == ROTATE_STATUS" in header


def test_yaml_uses_exact_runtime_parameter_schema() -> None:
    parameters = rotation_parameters()

    assert set(parameters) == EXPECTED_PARAMETERS
    assert LEGACY_PARAMETERS.isdisjoint(parameters)


def test_yaml_uses_canonical_rotation_topics() -> None:
    parameters = rotation_parameters()

    assert parameters["target_topic"] == EXPECTED_TOPICS[
        "ROTATE_TARGET_RAD"
    ]
    assert parameters["enable_topic"] == EXPECTED_TOPICS[
        "ROTATE_ENABLE"
    ]
    assert parameters["cancel_topic"] == EXPECTED_TOPICS[
        "ROTATE_CANCEL"
    ]
    assert parameters["state_topic"] == EXPECTED_TOPICS[
        "ROTATE_STATE"
    ]
    assert parameters["status_topic"] == EXPECTED_TOPICS[
        "ROTATE_STATUS"
    ]


def test_rotation_uses_auto_command_lane() -> None:
    parameters = rotation_parameters()
    source = read_text(NODE_PATH)

    assert parameters["output_topic"] == "/cmd_vel_auto"

    assert (
        'declare_parameter<std::string>('
        '"output_topic", topics::CMD_VEL_AUTO);'
    ) in source

    assert (
        "std::string output_topic_{topics::CMD_VEL_AUTO};"
    ) in source

    assert "topics::CMD_VEL_RECOVERY" not in source


def test_node_defaults_use_rotation_constants() -> None:
    source = read_text(NODE_PATH)

    expected_defaults = (
        "topics::ROTATE_TARGET_RAD",
        "topics::ROTATE_ENABLE",
        "topics::ROTATE_CANCEL",
        "topics::ROTATE_STATE",
        "topics::ROTATE_STATUS",
    )

    for marker in expected_defaults:
        assert source.count(marker) == 2


def test_rotation_contract_has_no_mapping_ownership() -> None:
    combined = "\n".join(
        (
            read_text(TOPICS_PATH),
            read_text(NODE_PATH),
            read_text(CONFIG_PATH),
        )
    )

    assert "savo_mapping" not in combined
    assert "/savo_mapping/" not in combined


def test_rotation_contract_test_is_registered() -> None:
    cmake = read_text(CMAKE_PATH)

    assert (
        "add_pytest_if_exists("
        "test_rotate_to_heading_contract "
        "test/test_rotate_to_heading_contract.py)"
    ) in cmake
