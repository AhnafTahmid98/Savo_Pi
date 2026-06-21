#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for Python topic-name contracts."""

from __future__ import annotations

from savo_control import constants as c
from savo_control.ros import (
    CMD_VEL,
    CMD_VEL_AUTO,
    CMD_VEL_MANUAL,
    CMD_VEL_MUX,
    CMD_VEL_NAV,
    CMD_VEL_RECOVERY,
    CMD_VEL_SAFE,
    CONTROL_MODE_CMD,
    CONTROL_MODE_STATE,
    DEPTH_MIN_FRONT,
    ODOM_FILTERED,
    RECOVERY_ACTIVE,
    RECOVERY_REQUEST,
    SAFETY_STOP,
    TOPICS,
    get_control_topics,
    get_control_topics_dict,
    is_control_output_topic,
    outputs_directly_to_base,
)


def test_command_chain_topics_are_canonical():
    assert CMD_VEL_MANUAL == "/cmd_vel_manual"
    assert CMD_VEL_AUTO == "/cmd_vel_auto"
    assert CMD_VEL_NAV == "/cmd_vel_nav"
    assert CMD_VEL_RECOVERY == "/cmd_vel_recovery"
    assert CMD_VEL_MUX == "/cmd_vel_mux"
    assert CMD_VEL == "/cmd_vel"
    assert CMD_VEL_SAFE == "/cmd_vel_safe"


def test_control_topics_match_constants():
    assert CMD_VEL_MANUAL == c.TOPIC_CMD_VEL_MANUAL
    assert CMD_VEL_AUTO == c.TOPIC_CMD_VEL_AUTO
    assert CMD_VEL_NAV == c.TOPIC_CMD_VEL_NAV
    assert CMD_VEL_RECOVERY == c.TOPIC_CMD_VEL_RECOVERY
    assert CMD_VEL_MUX == c.TOPIC_CMD_VEL_MUX
    assert CMD_VEL == c.TOPIC_CMD_VEL
    assert CMD_VEL_SAFE == c.TOPIC_CMD_VEL_SAFE
    assert SAFETY_STOP == c.TOPIC_SAFETY_STOP
    assert CONTROL_MODE_CMD == c.TOPIC_CONTROL_MODE_CMD
    assert CONTROL_MODE_STATE == c.TOPIC_CONTROL_MODE_STATE


def test_runtime_support_topics_are_canonical():
    assert ODOM_FILTERED == "/odometry/filtered"
    assert DEPTH_MIN_FRONT == "/depth/min_front_m"
    assert SAFETY_STOP == "/safety/stop"
    assert CONTROL_MODE_CMD == "/savo_control/mode_cmd"
    assert CONTROL_MODE_STATE == "/savo_control/mode_state"
    assert RECOVERY_REQUEST == "/savo_control/recovery_request"
    assert RECOVERY_ACTIVE == "/savo_control/recovery_active"


def test_control_topics_dataclass_exports_expected_values():
    assert TOPICS.cmd_vel_manual == "/cmd_vel_manual"
    assert TOPICS.cmd_vel_auto == "/cmd_vel_auto"
    assert TOPICS.cmd_vel_nav == "/cmd_vel_nav"
    assert TOPICS.cmd_vel_recovery == "/cmd_vel_recovery"
    assert TOPICS.cmd_vel_mux == "/cmd_vel_mux"
    assert TOPICS.cmd_vel == "/cmd_vel"
    assert TOPICS.cmd_vel_safe == "/cmd_vel_safe"
    assert TOPICS.safety_stop == "/safety/stop"
    assert TOPICS.mode_cmd == "/savo_control/mode_cmd"


def test_get_control_topics_returns_single_contract_object():
    topics = get_control_topics()

    assert topics is TOPICS
    assert topics.cmd_vel == "/cmd_vel"
    assert topics.cmd_vel_safe == "/cmd_vel_safe"
    assert topics.odom_filtered == "/odometry/filtered"


def test_get_control_topics_dict_contains_core_topics():
    data = get_control_topics_dict()

    assert data["cmd_vel_manual"] == "/cmd_vel_manual"
    assert data["cmd_vel_auto"] == "/cmd_vel_auto"
    assert data["cmd_vel_nav"] == "/cmd_vel_nav"
    assert data["cmd_vel_recovery"] == "/cmd_vel_recovery"
    assert data["cmd_vel_mux"] == "/cmd_vel_mux"
    assert data["cmd_vel"] == "/cmd_vel"
    assert data["cmd_vel_safe"] == "/cmd_vel_safe"
    assert data["safety_stop"] == "/safety/stop"
    assert data["depth_min_front"] == "/depth/min_front_m"


def test_control_output_topic_helper():
    assert is_control_output_topic("/cmd_vel_manual") is True
    assert is_control_output_topic("/cmd_vel_auto") is True
    assert is_control_output_topic("/cmd_vel_nav") is True
    assert is_control_output_topic("/cmd_vel_recovery") is True
    assert is_control_output_topic("/cmd_vel_mux") is True
    assert is_control_output_topic("/cmd_vel") is True

    assert is_control_output_topic("/cmd_vel_safe") is False
    assert is_control_output_topic("/scan") is False


def test_direct_base_output_helper():
    assert outputs_directly_to_base("/cmd_vel_safe") is True
    assert outputs_directly_to_base("/cmd_vel") is False
    assert outputs_directly_to_base("/cmd_vel_auto") is False
    assert outputs_directly_to_base("/cmd_vel_nav") is False
    assert outputs_directly_to_base("/cmd_vel_recovery") is False


def test_safety_chain_is_not_collapsed():
    assert CMD_VEL != CMD_VEL_SAFE
    assert CMD_VEL_MUX != CMD_VEL_SAFE
    assert CMD_VEL_AUTO != CMD_VEL_SAFE
    assert CMD_VEL_NAV != CMD_VEL_SAFE
    assert CMD_VEL_RECOVERY != CMD_VEL_SAFE
