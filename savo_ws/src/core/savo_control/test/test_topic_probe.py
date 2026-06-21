#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for topic freshness probe helpers."""

from __future__ import annotations

from math import isclose

from savo_control.diagnostics import (
    TopicProbe,
    TopicProbeSet,
    control_topic_probe_set,
    make_topic_probe_set,
)


def test_topic_probe_create_sets_defaults_and_qos():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    )

    assert probe.name == "cmd_vel"
    assert probe.topic == "/cmd_vel"
    assert probe.required is True
    assert probe.timeout_s == 0.30
    assert probe.last_seen_s is None
    assert probe.count == 0
    assert probe.last_value == ""
    assert probe.qos_name == "command"


def test_topic_probe_create_accepts_optional_topic_and_custom_qos():
    probe = TopicProbe.create(
        name="debug",
        topic="/debug/topic",
        required=False,
        timeout_s=-1.0,
        qos_name="latched",
    )

    assert probe.required is False
    assert probe.timeout_s == 0.0
    assert probe.qos_name == "latched"


def test_topic_probe_observed_returns_updated_copy():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    )

    updated = probe.observed(now_s=1.0, value="vx=0.1")

    assert updated is not probe
    assert updated.name == probe.name
    assert updated.topic == probe.topic
    assert updated.last_seen_s == 1.0
    assert updated.count == 1
    assert updated.last_value == "vx=0.1"

    assert probe.last_seen_s is None
    assert probe.count == 0


def test_topic_probe_age_fresh_and_stale():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0)

    assert isclose(probe.age_s(now_s=1.2), 0.20, abs_tol=1e-9)
    assert probe.fresh(now_s=1.2) is True
    assert probe.stale(now_s=1.2) is False

    assert probe.fresh(now_s=1.31) is False
    assert probe.stale(now_s=1.31) is True


def test_topic_probe_age_never_seen():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    )

    assert probe.age_s(now_s=1.0) is None
    assert probe.fresh(now_s=1.0) is False
    assert probe.stale(now_s=1.0) is True


def test_topic_probe_age_does_not_go_negative():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=2.0)

    assert probe.age_s(now_s=1.0) == 0.0
    assert probe.fresh(now_s=1.0) is True


def test_required_topic_health_check_never_seen_is_stale():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        required=True,
        timeout_s=0.30,
    )

    check = probe.health_check(now_s=1.0)

    assert check.name == "cmd_vel"
    assert check.ok is False
    assert check.stale is True
    assert check.status() == "STALE"
    assert check.value == "never"
    assert check.note == "/cmd_vel"


def test_optional_topic_health_check_never_seen_is_ok():
    probe = TopicProbe.create(
        name="recovery_status",
        topic="/savo_control/recovery_status",
        required=False,
        timeout_s=1.0,
    )

    check = probe.health_check(now_s=1.0)

    assert check.ok is True
    assert check.stale is False
    assert check.status() == "OK"
    assert check.value == "never"


def test_topic_health_check_fresh():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0)

    check = probe.health_check(now_s=1.2)

    assert check.ok is True
    assert check.stale is False
    assert check.status() == "OK"
    assert check.value == "0.200s"
    assert check.note == "/cmd_vel"


def test_topic_health_check_stale():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0)

    check = probe.health_check(now_s=1.5)

    assert check.ok is False
    assert check.stale is True
    assert check.status() == "STALE"
    assert check.value == "0.500s"
    assert check.note == "/cmd_vel"


def test_topic_probe_to_dict_without_now():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0, value="ok")

    assert probe.to_dict() == {
        "name": "cmd_vel",
        "topic": "/cmd_vel",
        "required": True,
        "timeout_s": 0.30,
        "last_seen_s": 1.0,
        "age_s": None,
        "count": 1,
        "last_value": "ok",
        "qos_name": "command",
    }


def test_topic_probe_to_dict_with_now():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0, value="ok")

    data = probe.to_dict(now_s=1.2)

    assert isclose(data["age_s"], 0.20, abs_tol=1e-9)
    assert data["count"] == 1
    assert data["last_value"] == "ok"


def test_topic_probe_status_text_fresh():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    ).observed(now_s=1.0)

    assert probe.status_text(now_s=1.2) == (
        "cmd_vel: fresh; topic=/cmd_vel; age=0.200s; "
        "count=1; qos=command"
    )


def test_topic_probe_status_text_never_seen():
    probe = TopicProbe.create(
        name="cmd_vel",
        topic="/cmd_vel",
        timeout_s=0.30,
    )

    assert probe.status_text(now_s=1.0) == (
        "cmd_vel: stale; topic=/cmd_vel; age=never; "
        "count=0; qos=command"
    )


def test_make_topic_probe_set():
    probe = TopicProbe.create(name="cmd_vel", topic="/cmd_vel")
    probe_set = make_topic_probe_set("test", [probe])

    assert isinstance(probe_set, TopicProbeSet)
    assert probe_set.name == "test"
    assert probe_set.probes == (probe,)


def test_topic_probe_set_update_matching_topic():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel"),
            TopicProbe.create(name="odom", topic="/odometry/filtered"),
        ],
    )

    updated = probe_set.update("/cmd_vel", now_s=1.0, value="vx=0.1")

    assert updated is not probe_set
    assert updated.probes[0].count == 1
    assert updated.probes[0].last_seen_s == 1.0
    assert updated.probes[0].last_value == "vx=0.1"

    assert updated.probes[1].count == 0
    assert updated.probes[1].last_seen_s is None


def test_topic_probe_set_update_non_matching_topic():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel"),
        ],
    )

    updated = probe_set.update("/unknown", now_s=1.0, value="ignored")

    assert updated.probes[0].count == 0
    assert updated.probes[0].last_seen_s is None


def test_topic_probe_set_health_checks_and_report():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel", timeout_s=0.30),
            TopicProbe.create(
                name="recovery_status",
                topic="/savo_control/recovery_status",
                required=False,
                timeout_s=1.0,
            ),
        ],
    ).update("/cmd_vel", now_s=1.0, value="ok")

    checks = probe_set.health_checks(now_s=1.2)
    report = probe_set.health_report(now_s=1.2)

    assert len(checks) == 2
    assert checks[0].status() == "OK"
    assert checks[1].status() == "OK"
    assert report.overall == "OK"


def test_topic_probe_set_missing_required():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel", timeout_s=0.30),
            TopicProbe.create(
                name="recovery_status",
                topic="/savo_control/recovery_status",
                required=False,
                timeout_s=1.0,
            ),
        ],
    )

    missing = probe_set.missing_required(now_s=1.0)

    assert len(missing) == 1
    assert missing[0].topic == "/cmd_vel"


def test_topic_probe_set_to_dict():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel", timeout_s=0.30),
        ],
    ).update("/cmd_vel", now_s=1.0, value="ok")

    data = probe_set.to_dict(now_s=1.2)

    assert data["name"] == "test"
    assert data["overall"] == "OK"
    assert data["probes"][0]["topic"] == "/cmd_vel"
    assert isclose(data["probes"][0]["age_s"], 0.20, abs_tol=1e-9)


def test_topic_probe_set_status_text():
    probe_set = make_topic_probe_set(
        "test",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel", timeout_s=0.30),
        ],
    ).update("/cmd_vel", now_s=1.0, value="ok")

    text = probe_set.status_text(now_s=1.2)

    assert "test: OK" in text
    assert "cmd_vel: OK" in text
    assert "cmd_vel: fresh; topic=/cmd_vel" in text


def test_control_topic_probe_set_contains_expected_topics():
    probe_set = control_topic_probe_set()
    topics = [probe.topic for probe in probe_set.probes]

    assert probe_set.name == "savo_control_topics"
    assert "/cmd_vel" in topics
    assert "/cmd_vel_safe" in topics
    assert "/savo_control/mode_state" in topics
    assert "/safety/stop" in topics
    assert "/safety/slowdown_factor" in topics
    assert "/odometry/filtered" in topics
    assert "/savo_control/recovery_status" in topics


def test_control_topic_probe_set_qos_names_are_assigned():
    probe_set = control_topic_probe_set()
    qos_by_topic = {probe.topic: probe.qos_name for probe in probe_set.probes}

    assert qos_by_topic["/cmd_vel"] == "command"
    assert qos_by_topic["/cmd_vel_safe"] == "command"
    assert qos_by_topic["/savo_control/mode_state"] == "status"
    assert qos_by_topic["/safety/stop"] == "default"
    assert qos_by_topic["/odometry/filtered"] == "sensor"
