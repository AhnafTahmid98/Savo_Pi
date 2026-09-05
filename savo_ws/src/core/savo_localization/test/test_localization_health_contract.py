"""Lock health profile selection to the localization VO mode."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PROFILE = "robot_savo_4enc_imu_ekf.yaml"
VO_PROFILE = "robot_savo_4enc_imu_vo_ekf.yaml"


def _launch_nodes(filename: str, executable: str) -> list[ast.Call]:
    source = (PACKAGE_ROOT / "launch" / filename).read_text(encoding="utf-8")
    tree = ast.parse(source)
    matches = []
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        if ast.literal_eval(keywords["executable"]) == executable:
            matches.append(call)
    return matches


def _launch_argument_defaults(filename: str) -> dict[str, str]:
    source = (PACKAGE_ROOT / "launch" / filename).read_text(encoding="utf-8")
    tree = ast.parse(source)
    defaults = {}
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != "DeclareLaunchArgument":
            continue
        argument_name = ast.literal_eval(call.args[0])
        default = next(item.value for item in call.keywords if item.arg == "default_value")
        defaults[argument_name] = ast.unparse(default)
    return defaults


def _keyword_source(call: ast.Call, keyword_name: str) -> str:
    keyword = next(item for item in call.keywords if item.arg == keyword_name)
    return ast.unparse(keyword.value)


def _parameters(call: ast.Call) -> tuple[str, ...]:
    keyword = next(item for item in call.keywords if item.arg == "parameters")
    assert isinstance(keyword.value, ast.List)
    return tuple(ast.unparse(item) for item in keyword.value.elts)


def _vo_mode(call: ast.Call) -> bool:
    condition = _keyword_source(call, "condition")
    if condition == "UnlessCondition(use_vo)" or "' == 'false'" in condition:
        return False
    if condition == "IfCondition(use_vo)" or "' == 'true'" in condition:
        return True
    raise AssertionError(f"Node condition does not select a VO mode: {condition}")


def _load_profile(filename: str) -> dict:
    path = PACKAGE_ROOT / "config" / "profiles" / filename
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_profiles_select_baseline_and_vo_health_modes() -> None:
    baseline_health = _load_profile(BASELINE_PROFILE)["localization_health_node"][
        "ros__parameters"
    ]
    vo_health = _load_profile(VO_PROFILE)["localization_health_node"][
        "ros__parameters"
    ]

    assert baseline_health["use_vo"] is False
    assert vo_health["use_vo"] is True
    assert vo_health["expected_vo_rate_hz"] == 15.0
    assert vo_health["max_vo_odom_age_s"] == 0.5
    assert "vo_required" not in baseline_health
    assert "vo_required" not in vo_health


def test_localization_bringup_supports_both_vo_modes_with_consistent_profiles() -> None:
    ekf_nodes = _launch_nodes("localization_bringup.launch.py", "ekf_node")
    health_nodes = _launch_nodes(
        "localization_bringup.launch.py", "localization_health_node"
    )

    assert len(ekf_nodes) == 2
    assert len(health_nodes) == 2
    assert {_vo_mode(node): _parameters(node) for node in ekf_nodes} == {
        False: ("topics_config", "frames_config", "ekf_config", "baseline_profile_config"),
        True: (
            "topics_config",
            "frames_config",
            "ekf_config",
            "vo_config",
            "vo_profile_config",
        ),
    }
    assert {_vo_mode(node): _parameters(node) for node in health_nodes} == {
        False: (
            "topics_config",
            "frames_config",
            "diagnostics_config",
            "baseline_profile_config",
        ),
        True: (
            "topics_config",
            "frames_config",
            "diagnostics_config",
            "vo_profile_config",
        ),
    }


def test_ekf_launch_keeps_ekf_and_health_profiles_consistent() -> None:
    ekf_nodes = _launch_nodes("ekf.launch.py", "ekf_node")
    health_nodes = _launch_nodes("ekf.launch.py", "localization_health_node")

    assert len(ekf_nodes) == 2
    assert len(health_nodes) == 2
    assert {_vo_mode(node): _parameters(node) for node in ekf_nodes} == {
        False: ("topics_config", "frames_config", "ekf_config", "profile_config"),
        True: (
            "topics_config",
            "frames_config",
            "ekf_config",
            "vo_config",
            "vo_profile_config",
        ),
    }
    assert {_vo_mode(node): _parameters(node) for node in health_nodes} == {
        False: (
            "topics_config",
            "frames_config",
            "diagnostics_config",
            "profile_config",
        ),
        True: (
            "topics_config",
            "frames_config",
            "diagnostics_config",
            "vo_profile_config",
        ),
    }


def test_vo_health_remains_optional_with_existing_rate_contract() -> None:
    diagnostics = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "diagnostics.yaml").read_text(encoding="utf-8")
    )["localization_health_node"]["ros__parameters"]
    source = (PACKAGE_ROOT / "src" / "localization_health_node.cpp").read_text(
        encoding="utf-8"
    )

    assert diagnostics["expected_vo_rate_hz"] == 15.0
    assert diagnostics["rate_tolerance_ratio"] == 0.50
    assert (
        diagnostics["expected_vo_rate_hz"] * diagnostics["rate_tolerance_ratio"]
        == 7.5
    )
    assert 'declare_parameter<bool>("vo_required", false)' in source


def test_producer_health_rate_accounting_preserves_existing_floor_and_freshness() -> None:
    diagnostics = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "diagnostics.yaml").read_text(encoding="utf-8")
    )["localization_health_node"]["ros__parameters"]
    standalone = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "localization_health.yaml").read_text(
            encoding="utf-8"
        )
    )["localization_health_node"]["ros__parameters"]
    source = (PACKAGE_ROOT / "src" / "localization_health_node.cpp").read_text(
        encoding="utf-8"
    )
    imu = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "imu.yaml").read_text(encoding="utf-8")
    )["imu_node"]["ros__parameters"]
    wheel = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "wheel_odom.yaml").read_text(encoding="utf-8")
    )["wheel_odom_node"]["ros__parameters"]

    assert diagnostics["expected_imu_rate_hz"] == 25.0
    assert diagnostics["rate_tolerance_ratio"] == 0.50
    assert (
        diagnostics["expected_imu_rate_hz"] * diagnostics["rate_tolerance_ratio"]
        == 12.5
    )
    assert diagnostics["rate_transition_debounce_s"] == 1.0
    assert standalone["expected_imu_rate_hz"] == 25.0
    assert standalone["rate_tolerance_ratio"] == 0.50
    assert standalone["rate_transition_debounce_s"] == 1.0
    assert diagnostics["max_imu_age_s"] == 0.5
    assert diagnostics["max_wheel_odom_age_s"] == 0.5
    assert diagnostics["max_filtered_odom_age_s"] == 0.5
    assert diagnostics["max_tf_age_s"] == 0.5
    assert imu["publish_rate_hz"] == 25.0
    assert imu["health_publish_rate_hz"] == 5.0
    assert imu["diagnostics_publish_rate_hz"] == 2.0
    assert wheel["publish_rate_hz"] == 30.0
    assert wheel["health_publish_rate_hz"] == 5.0
    assert wheel["publish_debug_state"] is False
    for token in (
        "source_rate_hz",
        "receive_rate_hz",
        "source_rate_available",
        "rate_basis",
        "rate_quality",
        "producer_successful_publication",
        "ProducerHealthConsumer",
    ):
        assert token in source


def test_health_aggregator_does_not_observe_required_raw_topics_or_global_diagnostics() -> None:
    source = (PACKAGE_ROOT / "src" / "localization_health_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "sensor_msgs/msg/imu.hpp" not in source
    assert "imu_subscription_" not in source
    assert "wheel_odom_subscription_" not in source
    assert "diagnostics_subscription_" not in source
    assert "on_diagnostics" not in source
    assert "std::regex" not in source
    assert "rclcpp::KeepLast(1)).reliable().transient_local()" in source
    assert "rclcpp::SensorDataQoS().keep_last(1)" in source


def test_large_wheel_debug_state_is_disabled_in_production() -> None:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "wheel_odom.yaml").read_text(encoding="utf-8")
    )["wheel_odom_node"]["ros__parameters"]
    source = (PACKAGE_ROOT / "src" / "wheel_odom_node.cpp").read_text(
        encoding="utf-8"
    )

    assert config["publish_debug_state"] is False
    assert config["debug_publish_rate_hz"] == 5.0
    assert "debug_state_pub_->publish(msg);" in source
    assert "state_pub_->publish(msg);" in source
    assert "SerializeProducerHealth(snapshot)" in source


def test_imu_state_is_compact_and_diagnostics_are_decimated() -> None:
    source = (PACKAGE_ROOT / "src" / "imu_node.cpp").read_text(encoding="utf-8")

    assert "SerializeProducerHealth(snapshot)" in source
    assert "make_state_json" not in source
    assert 'declare_parameter<double>("health_publish_rate_hz"' in source
    assert '"diagnostics_publish_rate_hz", diagnostics_publish_rate_hz_' in source


def test_rate_quality_does_not_expand_top_level_state_vocabulary() -> None:
    core = (PACKAGE_ROOT / "src" / "localization_health_core.cpp").read_text(
        encoding="utf-8"
    )
    node = (PACKAGE_ROOT / "src" / "localization_health_node.cpp").read_text(
        encoding="utf-8"
    )
    producer = (PACKAGE_ROOT / "src" / "producer_health.cpp").read_text(
        encoding="utf-8"
    )

    for state in ("INITIALIZING", "OK", "DEGRADED", "STALE", "ERROR"):
        assert f'return "{state}";' in core
    assert '\\"state\\":\\"' in node
    assert "LocalizationHealthCore::ToString(result.state)" in node
    assert 'return "GOOD";' in producer
    assert 'return "EXCELLENT";' in producer
    assert 'LocalizationHealthState::kGood' not in core
    assert 'LocalizationHealthState::kExcellent' not in core


def test_launch_defaults_reference_the_intended_profiles() -> None:
    bringup = _launch_argument_defaults("localization_bringup.launch.py")
    ekf = _launch_argument_defaults("ekf.launch.py")

    assert bringup["use_vo"] == "'true'"
    assert BASELINE_PROFILE in bringup["baseline_profile_config"]
    assert VO_PROFILE in bringup["vo_profile_config"]
    assert BASELINE_PROFILE in ekf["profile_config"]
    assert VO_PROFILE in ekf["vo_profile_config"]
