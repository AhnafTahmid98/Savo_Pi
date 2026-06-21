#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for Python QoS profile helpers."""

from __future__ import annotations

from savo_control.ros import (
    QOS_COMMAND,
    QOS_DEFAULT,
    QOS_LATCHED,
    QOS_SENSOR,
    QOS_SPECS,
    QOS_STATUS,
    QosProfileSpec,
    get_qos_spec,
    get_qos_spec_dict,
    qos_name_for_topic,
)


def test_qos_profile_spec_to_dict():
    spec = QosProfileSpec(
        name="test",
        depth=7,
        reliability="reliable",
        durability="volatile",
        history="keep_last",
    )

    assert spec.to_dict() == {
        "name": "test",
        "depth": 7,
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
    }


def test_qos_specs_are_canonical():
    assert set(QOS_SPECS) == {
        QOS_COMMAND,
        QOS_STATUS,
        QOS_SENSOR,
        QOS_LATCHED,
        QOS_DEFAULT,
    }

    assert QOS_SPECS[QOS_COMMAND].depth == 10
    assert QOS_SPECS[QOS_COMMAND].reliability == "reliable"
    assert QOS_SPECS[QOS_COMMAND].durability == "volatile"

    assert QOS_SPECS[QOS_SENSOR].depth == 5
    assert QOS_SPECS[QOS_SENSOR].reliability == "best_effort"
    assert QOS_SPECS[QOS_SENSOR].durability == "volatile"

    assert QOS_SPECS[QOS_LATCHED].depth == 1
    assert QOS_SPECS[QOS_LATCHED].reliability == "reliable"
    assert QOS_SPECS[QOS_LATCHED].durability == "transient_local"


def test_get_qos_spec_accepts_known_names():
    assert get_qos_spec(QOS_COMMAND).name == QOS_COMMAND
    assert get_qos_spec(QOS_STATUS).name == QOS_STATUS
    assert get_qos_spec(QOS_SENSOR).name == QOS_SENSOR
    assert get_qos_spec(QOS_LATCHED).name == QOS_LATCHED
    assert get_qos_spec(QOS_DEFAULT).name == QOS_DEFAULT


def test_get_qos_spec_normalizes_text_and_falls_back():
    assert get_qos_spec(" command ").name == QOS_COMMAND
    assert get_qos_spec("SENSOR").name == QOS_SENSOR
    assert get_qos_spec("bad").name == QOS_DEFAULT
    assert get_qos_spec("").name == QOS_DEFAULT


def test_get_qos_spec_dict():
    data = get_qos_spec_dict(QOS_STATUS)

    assert data == {
        "name": QOS_STATUS,
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
    }


def test_qos_name_for_command_topics():
    for topic in [
        "/cmd_vel_manual",
        "/cmd_vel_auto",
        "/cmd_vel_nav",
        "/cmd_vel_recovery",
        "/cmd_vel_mux",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/savo_control/mode_cmd",
    ]:
        assert qos_name_for_topic(topic) == QOS_COMMAND


def test_qos_name_for_sensor_topics():
    for topic in [
        "/scan",
        "/depth/min_front_m",
        "/savo_perception/range/front_ultrasonic_m",
        "/savo_perception/range/left_m",
        "/savo_perception/range/right_m",
        "/odometry/filtered",
        "/wheel/odom",
        "/imu/data",
    ]:
        assert qos_name_for_topic(topic) == QOS_SENSOR


def test_qos_name_for_status_topics():
    assert qos_name_for_topic("/savo_control/control_status") == QOS_STATUS
    assert qos_name_for_topic("/savo_control/recovery_status") == QOS_STATUS
    assert qos_name_for_topic("/savo_control/mode_state") == QOS_STATUS
    assert qos_name_for_topic("/custom/status") == QOS_STATUS


def test_qos_name_for_unknown_topic_uses_default():
    assert qos_name_for_topic("/unknown") == QOS_DEFAULT
    assert qos_name_for_topic("") == QOS_DEFAULT
