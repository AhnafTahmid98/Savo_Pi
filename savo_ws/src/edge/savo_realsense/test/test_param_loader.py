from __future__ import annotations

from dataclasses import dataclass

from savo_realsense.utils.param_loader import (
    get_bool_param,
    get_float_param,
    get_int_param,
    get_param_value,
    get_str_param,
)


@dataclass
class FakeParameter:
    value: object


class FakeNode:
    def __init__(self, values: dict[str, object] | None = None) -> None:
        self._values = values or {}
        self.declared: dict[str, object] = {}

    def declare_parameter(self, name: str, default: object) -> None:
        self.declared[name] = default
        self._values.setdefault(name, default)

    def get_parameter(self, name: str) -> FakeParameter:
        return FakeParameter(self._values[name])


def test_get_param_value_declares_and_reads_value() -> None:
    node = FakeNode({"rate_hz": 15.0})

    value = get_param_value(node, "rate_hz", 30.0)

    assert value == 15.0
    assert node.declared["rate_hz"] == 30.0


def test_get_float_param_returns_float_value() -> None:
    node = FakeNode({"timeout_s": 0.75})

    assert get_float_param(node, "timeout_s", 0.50) == 0.75


def test_get_float_param_falls_back_below_minimum() -> None:
    node = FakeNode({"timeout_s": -1.0})

    assert get_float_param(node, "timeout_s", 0.50, min_value=0.0) == 0.50


def test_get_int_param_returns_int_value() -> None:
    node = FakeNode({"window_size": 20})

    assert get_int_param(node, "window_size", 10) == 20


def test_get_int_param_falls_back_below_minimum() -> None:
    node = FakeNode({"window_size": 0})

    assert get_int_param(node, "window_size", 10, min_value=1) == 10


def test_get_bool_param_returns_bool_value() -> None:
    node = FakeNode({"require_pointcloud": True})

    assert get_bool_param(node, "require_pointcloud", False) is True


def test_get_str_param_returns_clean_string() -> None:
    node = FakeNode({"camera_name": "  realsense_d435  "})

    assert get_str_param(node, "camera_name", "camera") == "realsense_d435"


def test_get_str_param_falls_back_for_empty_string() -> None:
    node = FakeNode({"camera_name": "   "})

    assert get_str_param(node, "camera_name", "camera") == "camera"


def test_get_str_param_allows_empty_when_requested() -> None:
    node = FakeNode({"camera_name": "   "})

    assert get_str_param(node, "camera_name", "camera", allow_empty=True) == ""