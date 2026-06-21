#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for package identity constants and version metadata."""

from savo_control import VERSION, __version__, get_package_version_info
from savo_control.constants import (
    DEFAULTS,
    MODE_AUTO,
    MODE_MANUAL,
    MODE_NAV,
    MODE_RECOVERY,
    MODE_STOP,
    PACKAGE_NAME,
    ROBOT_NAME,
    TOPIC_CMD_VEL,
    TOPIC_CMD_VEL_AUTO,
    TOPIC_CMD_VEL_MANUAL,
    TOPIC_CMD_VEL_MUX,
    TOPIC_CMD_VEL_NAV,
    TOPIC_CMD_VEL_RECOVERY,
    TOPIC_CMD_VEL_SAFE,
    TOPIC_CONTROL_MODE_CMD,
    TOPIC_CONTROL_MODE_STATE,
    TOPIC_SAFETY_STOP,
)


def test_version_metadata_matches_package_identity():
    info = get_package_version_info()

    assert PACKAGE_NAME == "savo_control"
    assert ROBOT_NAME == "Robot Savo"
    assert VERSION == __version__
    assert info.package_name == PACKAGE_NAME
    assert info.robot_name == ROBOT_NAME
    assert info.ros_distro == "jazzy"
    assert info.banner() == "Robot Savo | savo_control 0.1.0 (ROS 2 jazzy)"


def test_core_topic_constants_match_runtime_contract():
    assert TOPIC_CMD_VEL_MANUAL == "/cmd_vel_manual"
    assert TOPIC_CMD_VEL_AUTO == "/cmd_vel_auto"
    assert TOPIC_CMD_VEL_NAV == "/cmd_vel_nav"
    assert TOPIC_CMD_VEL_RECOVERY == "/cmd_vel_recovery"
    assert TOPIC_CMD_VEL_MUX == "/cmd_vel_mux"
    assert TOPIC_CMD_VEL == "/cmd_vel"
    assert TOPIC_CMD_VEL_SAFE == "/cmd_vel_safe"
    assert TOPIC_SAFETY_STOP == "/safety/stop"
    assert TOPIC_CONTROL_MODE_CMD == "/savo_control/mode_cmd"
    assert TOPIC_CONTROL_MODE_STATE == "/savo_control/mode_state"


def test_control_modes_are_safe_and_canonical():
    assert (MODE_STOP, MODE_MANUAL, MODE_AUTO, MODE_NAV, MODE_RECOVERY) == (
        "STOP",
        "MANUAL",
        "AUTO",
        "NAV",
        "RECOVERY",
    )
    assert DEFAULTS.default_mode == MODE_STOP


def test_defaults_export_practical_control_contract():
    data = DEFAULTS.to_dict()

    assert data["topics"]["cmd_vel"] == "/cmd_vel"
    assert data["topics"]["cmd_vel_safe"] == "/cmd_vel_safe"
    assert data["modes"]["default"] == "STOP"
    assert "MANUAL" in data["modes"]["allowed"]
    assert data["limits"]["max_vx"] > 0.0
    assert data["limits"]["max_vy"] > 0.0
    assert data["limits"]["max_wz"] > 0.0
