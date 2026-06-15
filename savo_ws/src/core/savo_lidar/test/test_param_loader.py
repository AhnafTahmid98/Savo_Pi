# -*- coding: utf-8 -*-

import pytest
import rclpy
from rclpy.node import Node

from savo_lidar.ros import (
    declare_if_missing,
    get_bool_param,
    get_float_param,
    get_int_param,
    get_positive_float_param,
    get_range_param,
    get_string_param,
    init_rclpy_if_needed,
)


@pytest.fixture(scope="module", autouse=True)
def rclpy_context():
    if not rclpy.ok():
        rclpy.init()

    yield

    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node():
    test_node = Node("test_savo_lidar_param_loader")
    yield test_node
    test_node.destroy_node()


def test_declare_if_missing_declares_default_value(node):
    value = declare_if_missing(node, "scan_topic", "/scan")

    assert value == "/scan"
    assert node.has_parameter("scan_topic")


def test_declare_if_missing_returns_existing_value(node):
    node.declare_parameter("backend", "real")

    value = declare_if_missing(node, "backend", "dryrun")

    assert value == "real"


def test_get_string_param_returns_stripped_string(node):
    value = get_string_param(node, "frame_id", " laser ")

    assert value == "laser"


def test_get_string_param_converts_non_string_value(node):
    node.declare_parameter("serial_port_number", 123)

    value = get_string_param(node, "serial_port_number", "0")

    assert value == "123"


def test_get_bool_param_returns_bool_value(node):
    node.declare_parameter("angle_compensate", True)

    assert get_bool_param(node, "angle_compensate", False) is True


@pytest.mark.parametrize(
    "raw_value",
    ["1", "true", "TRUE", "yes", "on"],
)
def test_get_bool_param_accepts_true_strings(raw_value):
    test_node = Node(f"test_bool_true_{raw_value.lower()}")
    try:
        test_node.declare_parameter("enabled", raw_value)

        assert get_bool_param(test_node, "enabled", False) is True
    finally:
        test_node.destroy_node()


@pytest.mark.parametrize(
    "raw_value",
    ["0", "false", "FALSE", "no", "off", "anything_else"],
)
def test_get_bool_param_rejects_false_strings(raw_value):
    test_node = Node(f"test_bool_false_{raw_value.lower()}")
    try:
        test_node.declare_parameter("enabled", raw_value)

        assert get_bool_param(test_node, "enabled", True) is False
    finally:
        test_node.destroy_node()


def test_get_bool_param_uses_default_when_missing(node):
    assert get_bool_param(node, "publish_driver_state", True) is True


def test_get_int_param_returns_integer(node):
    node.declare_parameter("baudrate", 115200)

    assert get_int_param(node, "baudrate", 0) == 115200


def test_get_int_param_converts_string_integer(node):
    node.declare_parameter("samples", "5")

    assert get_int_param(node, "samples", 1) == 5


def test_get_float_param_returns_float(node):
    node.declare_parameter("publish_rate_hz", 10.0)

    assert get_float_param(node, "publish_rate_hz", 1.0) == 10.0


def test_get_float_param_converts_integer(node):
    node.declare_parameter("publish_rate_hz_int", 10)

    assert get_float_param(node, "publish_rate_hz_int", 1.0) == 10.0


def test_get_positive_float_param_accepts_positive_value(node):
    node.declare_parameter("stale_timeout_s", 0.5)

    assert get_positive_float_param(node, "stale_timeout_s", 1.0) == 0.5


def test_get_positive_float_param_uses_default_for_zero(node):
    node.declare_parameter("stale_timeout_s", 0.0)

    assert get_positive_float_param(node, "stale_timeout_s", 1.0) == 1.0


def test_get_positive_float_param_uses_default_for_negative(node):
    node.declare_parameter("stale_timeout_s", -0.5)

    assert get_positive_float_param(node, "stale_timeout_s", 1.0) == 1.0


def test_get_range_param_accepts_valid_range(node):
    node.declare_parameter("min_range_m", 0.15)
    node.declare_parameter("max_range_m", 12.0)

    min_value, max_value = get_range_param(
        node,
        "min_range_m",
        "max_range_m",
        0.1,
        10.0,
    )

    assert min_value == 0.15
    assert max_value == 12.0


def test_get_range_param_uses_defaults_for_invalid_range(node):
    node.declare_parameter("min_range_m", 12.0)
    node.declare_parameter("max_range_m", 0.15)

    min_value, max_value = get_range_param(
        node,
        "min_range_m",
        "max_range_m",
        0.1,
        10.0,
    )

    assert min_value == 0.1
    assert max_value == 10.0


def test_get_range_param_uses_defaults_when_equal(node):
    node.declare_parameter("min_range_m", 1.0)
    node.declare_parameter("max_range_m", 1.0)

    min_value, max_value = get_range_param(
        node,
        "min_range_m",
        "max_range_m",
        0.1,
        10.0,
    )

    assert min_value == 0.1
    assert max_value == 10.0


def test_init_rclpy_if_needed_returns_false_when_already_initialized():
    assert init_rclpy_if_needed() is False
