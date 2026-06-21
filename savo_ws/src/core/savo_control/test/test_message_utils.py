#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for ROS message helper utilities."""

from __future__ import annotations

from dataclasses import dataclass

from savo_control.ros import (
    bool_msg_value,
    float_msg_value,
    has_header,
    header_stamp_to_sec,
    make_bool_msg,
    make_float32_msg,
    make_float64_msg,
    make_string_msg,
    odom_twist_tuple,
    safe_frame_id,
    stamp_to_sec,
    string_msg_value,
    twist_tuple,
)


@dataclass
class DataMsg:
    data: object = None


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp
    frame_id: str = ""


@dataclass
class HeaderMsg:
    header: Header


@dataclass
class NoHeaderMsg:
    value: int = 0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeTwist:
    linear: Vector3
    angular: Vector3


@dataclass
class FakeOdomTwist:
    twist: FakeTwist


@dataclass
class FakeOdomTwistOuter:
    twist: FakeOdomTwist


class BoolMsg:
    data = False


class FloatMsg:
    data = 0.0


class StringMsg:
    data = ""


def make_twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> FakeTwist:
    return FakeTwist(
        linear=Vector3(x=vx, y=vy, z=99.0),
        angular=Vector3(x=11.0, y=12.0, z=wz),
    )


def test_bool_msg_value():
    assert bool_msg_value(DataMsg(True)) is True
    assert bool_msg_value(DataMsg(False)) is False
    assert bool_msg_value(True) is True
    assert bool_msg_value(False) is False
    assert bool_msg_value(None, default=True) is True


def test_float_msg_value():
    assert float_msg_value(DataMsg("0.25")) == 0.25
    assert float_msg_value(DataMsg(1.5)) == 1.5
    assert float_msg_value(2.5) == 2.5
    assert float_msg_value(DataMsg("bad"), default=1.0) == 1.0
    assert float_msg_value(None, default=2.0) == 2.0


def test_string_msg_value():
    assert string_msg_value(DataMsg("AUTO")) == "AUTO"
    assert string_msg_value(DataMsg(123)) == "123"
    assert string_msg_value("MANUAL") == "MANUAL"
    assert string_msg_value(DataMsg(None), default="STOP") == "STOP"
    assert string_msg_value(None, default="STOP") == "STOP"


def test_make_bool_msg_with_fake_type():
    msg = make_bool_msg(True, msg_type=BoolMsg)

    assert isinstance(msg, BoolMsg)
    assert msg.data is True


def test_make_float32_msg_with_fake_type():
    msg = make_float32_msg("0.25", msg_type=FloatMsg)

    assert isinstance(msg, FloatMsg)
    assert msg.data == 0.25


def test_make_float64_msg_with_fake_type():
    msg = make_float64_msg("0.50", msg_type=FloatMsg)

    assert isinstance(msg, FloatMsg)
    assert msg.data == 0.50


def test_make_string_msg_with_fake_type():
    msg = make_string_msg("AUTO", msg_type=StringMsg)

    assert isinstance(msg, StringMsg)
    assert msg.data == "AUTO"


def test_stamp_to_sec():
    stamp = Stamp(sec=2, nanosec=500_000_000)

    assert stamp_to_sec(stamp) == 2.5
    assert stamp_to_sec(None, default=-1.0) == -1.0
    assert stamp_to_sec(object(), default=-2.0) == -2.0


def test_header_stamp_to_sec():
    msg = HeaderMsg(header=Header(stamp=Stamp(sec=3, nanosec=250_000_000)))

    assert header_stamp_to_sec(msg) == 3.25
    assert header_stamp_to_sec(NoHeaderMsg(), default=-1.0) == -1.0


def test_has_header():
    assert has_header(HeaderMsg(header=Header(stamp=Stamp()))) is True
    assert has_header(NoHeaderMsg()) is False


def test_safe_frame_id():
    msg = HeaderMsg(header=Header(stamp=Stamp(), frame_id="base_link"))

    assert safe_frame_id(msg) == "base_link"
    assert safe_frame_id(NoHeaderMsg(), default="map") == "map"

    empty = HeaderMsg(header=Header(stamp=Stamp(), frame_id=""))
    assert safe_frame_id(empty, default="odom") == "odom"


def test_twist_tuple():
    twist = make_twist(vx=0.1, vy=-0.2, wz=0.3)

    assert twist_tuple(twist) == (0.1, -0.2, 0.3)


def test_odom_twist_tuple():
    twist = make_twist(vx=0.12, vy=-0.04, wz=0.50)
    odom = FakeOdomTwistOuter(twist=FakeOdomTwist(twist=twist))

    assert odom_twist_tuple(odom) == (0.12, -0.04, 0.50)
