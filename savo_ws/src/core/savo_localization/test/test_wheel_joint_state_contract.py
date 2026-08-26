"""Contract tests for passive encoder-derived wheel JointState output."""

from pathlib import Path

import pytest
import yaml

from savo_localization.constants import DEFAULT_JOINT_STATES_TOPIC
from savo_localization.models.wheel_odom_config import (
    wheel_odom_config_from_ros_params,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_JOINT_NAMES = (
    "wheel_fl_link_joint",
    "wheel_fr_link_joint",
    "wheel_rl_link_joint",
    "wheel_rr_link_joint",
)


def test_joint_state_topic_and_default_are_explicit() -> None:
    """Keep production publication enabled on the conventional topic."""
    topics = yaml.safe_load(
        (PACKAGE_ROOT / "config/topics.yaml").read_text(encoding="utf-8")
    )
    wheel_odom = yaml.safe_load(
        (PACKAGE_ROOT / "config/wheel_odom.yaml").read_text(encoding="utf-8")
    )

    assert topics["wheel_odom_node"]["ros__parameters"][
        "joint_states_topic"
    ] == "/joint_states"
    assert wheel_odom["wheel_odom_node"]["ros__parameters"][
        "publish_joint_states"
    ] is True
    assert DEFAULT_JOINT_STATES_TOPIC == "/joint_states"


def test_wheel_odom_model_mirrors_joint_state_parameters() -> None:
    """Keep the no-ROS configuration model aligned with the C++ node."""
    config = wheel_odom_config_from_ros_params(
        {
            "joint_states_topic": "/measured_wheels",
            "publish_joint_states": False,
        }
    )

    assert config.joint_states_topic == "/measured_wheels"
    assert config.publish_joint_states is False


def test_empty_joint_state_topic_is_conditional_and_does_not_gate_odometry() -> None:
    """Reject an enabled empty topic but accept it when output is disabled."""
    with pytest.raises(ValueError, match="Topic name cannot be empty"):
        wheel_odom_config_from_ros_params(
            {
                "joint_states_topic": "",
                "publish_joint_states": True,
            }
        )

    disabled = wheel_odom_config_from_ros_params(
        {
            "joint_states_topic": "",
            "publish_joint_states": False,
        }
    )
    assert disabled.publish_joint_states is False
    assert disabled.joint_states_topic == ""

    source = (PACKAGE_ROOT / "src/wheel_odom_node.cpp").read_text(
        encoding="utf-8"
    )
    assert "if (publish_joint_states_ && joint_states_topic_.empty())" in source
    assert (
        "joint_states_topic cannot be empty when publish_joint_states is true"
        in source
    )
    assert source.index("publish_odometry(odom_sample, encoder_sample);") < (
        source.index("publish_joint_state(encoder_sample, now_time);")
    )


def test_python_topic_contract_marks_joint_states_optional() -> None:
    """Ensure visualization state cannot become a localization prerequisite."""
    source = (
        PACKAGE_ROOT / "savo_localization/ros/topic_contract.py"
    ).read_text(encoding="utf-8")
    start = source.index('"joint_states": TopicSpec(')
    end = source.index('"filtered_odom": TopicSpec(', start)
    joint_states_spec = source[start:end]

    assert 'message_type="sensor_msgs/msg/JointState"' in joint_states_spec
    assert 'owner="wheel_odom_node"' in joint_states_spec
    assert "required=False" in joint_states_spec


def test_converter_uses_exact_four_urdf_joint_names() -> None:
    """Match the existing continuous wheel joints without head joints."""
    header = (
        PACKAGE_ROOT / "include/savo_localization/wheel_joint_state.hpp"
    ).read_text(encoding="utf-8")

    assert "std::array<const char *, 4> WHEEL_JOINT_NAMES" in header
    for joint_name in EXPECTED_JOINT_NAMES:
        assert f'"{joint_name}"' in header
    assert "head_pan_joint" not in header
    assert "head_tilt_joint" not in header


def test_converter_preserves_signed_encoder_units() -> None:
    """Require cumulative radians and rad/s from the effective resolution."""
    source = (PACKAGE_ROOT / "src/wheel_joint_state.cpp").read_text(
        encoding="utf-8"
    )

    assert "TWO_PI / static_cast<double>(counts_per_wheel_rev)" in source
    assert "wheels[index].count) * radians_per_count" in source
    assert "wheels[index].counts_per_second * radians_per_count" in source
    assert "std::fmod" not in source
    assert "std::abs" not in source


def test_node_uses_existing_sample_and_measurement_timestamp_passively() -> None:
    """Keep JointState downstream of encoder sampling and isolated from odom."""
    source = (PACKAGE_ROOT / "src/wheel_odom_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "publish_joint_state(encoder_sample, now_time);" in source
    assert "msg.header.stamp = measurement_time;" in source
    assert "msg.position.assign(" in source
    assert "msg.velocity.assign(" in source
    assert "msg.effort" not in source
    assert source.index("publish_odometry(odom_sample, encoder_sample);") < (
        source.index("publish_joint_state(encoder_sample, now_time);")
    )

    function_start = source.index("void WheelOdomNode::publish_joint_state(")
    function_end = source.index(
        "void WheelOdomNode::publish_transform(", function_start
    )
    function = source[function_start:function_end]
    assert "try {" in function
    assert "catch (const std::exception & exc)" in function
    assert "without affecting odometry" in function
    assert "create_publisher<sensor_msgs::msg::JointState>" in source
    assert "create_subscription" not in function


def test_joint_state_path_introduces_no_tf_or_motor_command_authority() -> None:
    """Keep the new converter and publisher strictly state-only."""
    paths = (
        PACKAGE_ROOT / "include/savo_localization/wheel_joint_state.hpp",
        PACKAGE_ROOT / "src/wheel_joint_state.cpp",
    )
    source = "\n".join(path.read_text(encoding="utf-8") for path in paths)

    for forbidden in (
        "TransformBroadcaster",
        "sendTransform",
        "/cmd_vel",
        "/cmd_vel_nav",
        "/cmd_vel_safe",
        "create_subscription",
    ):
        assert forbidden not in source


def test_existing_hardware_resolution_and_direction_contract_is_unchanged() -> None:
    """Pin the effective 80-count resolution and real encoder wiring."""
    params = yaml.safe_load(
        (PACKAGE_ROOT / "config/encoders.yaml").read_text(encoding="utf-8")
    )["wheel_odom_node"]["ros__parameters"]

    assert params["cpr"] * params["decoding"] * params["gear_ratio"] == 80
    assert {
        key: params[key]
        for key in ("invert_fl", "invert_fr", "invert_rl", "invert_rr")
    } == {
        "invert_fl": False,
        "invert_fr": False,
        "invert_rl": False,
        "invert_rr": False,
    }
    assert {
        key: params[key]
        for key in (
            "fl_a_gpio",
            "fl_b_gpio",
            "fr_a_gpio",
            "fr_b_gpio",
            "rl_a_gpio",
            "rl_b_gpio",
            "rr_a_gpio",
            "rr_b_gpio",
        )
    } == {
        "fl_a_gpio": 20,
        "fl_b_gpio": 21,
        "fr_a_gpio": 13,
        "fr_b_gpio": 25,
        "rl_a_gpio": 24,
        "rl_b_gpio": 23,
        "rr_a_gpio": 12,
        "rr_b_gpio": 26,
    }
