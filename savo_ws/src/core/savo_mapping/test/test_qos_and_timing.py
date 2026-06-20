#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping QoS and timing helpers."""

from __future__ import annotations

import time

import pytest

from savo_mapping.ros.qos_profiles import (
    default_qos,
    get_qos_profile,
    get_topic_qos_profile,
    map_metadata_qos,
    map_qos,
    pointcloud_qos,
    qos_config_to_profile,
    reliable_qos,
    scan_qos,
    sensor_data_qos,
    status_qos,
    tf_static_qos,
    transient_local_qos,
)
from savo_mapping.utils.qos import (
    DEFAULT_QOS,
    QOS_DEFAULT,
    QOS_RELIABLE,
    QOS_SENSOR_DATA,
    QOS_TRANSIENT_LOCAL,
    RELIABLE_QOS,
    SENSOR_DATA_QOS,
    TRANSIENT_LOCAL_QOS,
    get_qos_config,
    list_qos_profiles,
    normalize_qos_name,
    qos_name_for_topic,
    to_rclpy_qos,
)
from savo_mapping.utils.ratekeeper import LoopBudget, Ratekeeper
from savo_mapping.utils.timing import (
    RateTracker,
    SimpleTimer,
    StaleMonitor,
    age_s,
    is_stale,
    monotonic_s,
    now_s,
)


# =============================================================================
# QoS config helpers
# =============================================================================
def test_normalize_qos_name() -> None:
    assert normalize_qos_name("sensor-data") == "sensor_data"
    assert normalize_qos_name("TRANSIENT LOCAL") == "transient_local"
    assert normalize_qos_name("reliable") == "reliable"


def test_normalize_qos_name_rejects_empty_name() -> None:
    with pytest.raises(ValueError):
        normalize_qos_name("")


def test_list_qos_profiles_contains_expected_profiles() -> None:
    profiles = list_qos_profiles()

    assert QOS_DEFAULT in profiles
    assert QOS_SENSOR_DATA in profiles
    assert QOS_RELIABLE in profiles
    assert QOS_TRANSIENT_LOCAL in profiles


def test_get_qos_config_default() -> None:
    config = get_qos_config("default")

    assert config == DEFAULT_QOS
    assert config.depth == 10
    assert config.reliability == "reliable"
    assert config.durability == "volatile"


def test_get_qos_config_sensor_data() -> None:
    config = get_qos_config("sensor-data")

    assert config == SENSOR_DATA_QOS
    assert config.depth == 10
    assert config.reliability == "best_effort"
    assert config.durability == "volatile"


def test_get_qos_config_reliable() -> None:
    config = get_qos_config("reliable")

    assert config == RELIABLE_QOS
    assert config.reliability == "reliable"
    assert config.durability == "volatile"


def test_get_qos_config_transient_local() -> None:
    config = get_qos_config("transient local")

    assert config == TRANSIENT_LOCAL_QOS
    assert config.depth == 1
    assert config.reliability == "reliable"
    assert config.durability == "transient_local"


def test_get_qos_config_rejects_unknown_profile() -> None:
    with pytest.raises(KeyError):
        get_qos_config("unknown_qos")


def test_qos_config_to_dict() -> None:
    data = SENSOR_DATA_QOS.to_dict()

    assert data["name"] == "sensor_data"
    assert data["depth"] == 10
    assert data["reliability"] == "best_effort"
    assert data["durability"] == "volatile"
    assert data["history"] == "keep_last"


# =============================================================================
# Topic-specific QoS names
# =============================================================================
def test_qos_name_for_scan_topic() -> None:
    assert qos_name_for_topic("/scan") == QOS_SENSOR_DATA


def test_qos_name_for_pointcloud_topic() -> None:
    assert qos_name_for_topic("/savo_edge/realsense/points") == QOS_SENSOR_DATA


def test_qos_name_for_map_topic() -> None:
    assert qos_name_for_topic("/map") == QOS_TRANSIENT_LOCAL


def test_qos_name_for_map_metadata_topic() -> None:
    assert qos_name_for_topic("/map_metadata") == QOS_TRANSIENT_LOCAL


def test_qos_name_for_tf_static_topic() -> None:
    assert qos_name_for_topic("/tf_static") == QOS_TRANSIENT_LOCAL


def test_qos_name_for_status_topic() -> None:
    assert qos_name_for_topic("/savo_mapping/status") == QOS_RELIABLE


# =============================================================================
# rclpy QoS adapters
# =============================================================================
def test_to_rclpy_qos_sensor_data() -> None:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

    profile = to_rclpy_qos(SENSOR_DATA_QOS)

    assert profile.history == HistoryPolicy.KEEP_LAST
    assert profile.depth == 10
    assert profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert profile.durability == DurabilityPolicy.VOLATILE


def test_to_rclpy_qos_transient_local() -> None:
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    profile = to_rclpy_qos(TRANSIENT_LOCAL_QOS)

    assert profile.depth == 1
    assert profile.reliability == ReliabilityPolicy.RELIABLE
    assert profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_qos_config_to_profile_matches_utils_adapter() -> None:
    profile_a = to_rclpy_qos(SENSOR_DATA_QOS)
    profile_b = qos_config_to_profile(SENSOR_DATA_QOS)

    assert profile_a.depth == profile_b.depth
    assert profile_a.reliability == profile_b.reliability
    assert profile_a.durability == profile_b.durability


def test_get_qos_profile() -> None:
    profile = get_qos_profile("sensor_data")

    assert profile.depth == SENSOR_DATA_QOS.depth


def test_get_topic_qos_profile() -> None:
    profile = get_topic_qos_profile("/scan")

    assert profile.depth == SENSOR_DATA_QOS.depth


def test_standard_ros_qos_helpers() -> None:
    assert sensor_data_qos().depth == SENSOR_DATA_QOS.depth
    assert scan_qos().depth == SENSOR_DATA_QOS.depth
    assert pointcloud_qos().depth == SENSOR_DATA_QOS.depth

    assert reliable_qos().depth == RELIABLE_QOS.depth
    assert status_qos().depth == RELIABLE_QOS.depth

    assert transient_local_qos().depth == TRANSIENT_LOCAL_QOS.depth
    assert map_qos().depth == TRANSIENT_LOCAL_QOS.depth
    assert map_metadata_qos().depth == TRANSIENT_LOCAL_QOS.depth
    assert tf_static_qos().depth == TRANSIENT_LOCAL_QOS.depth

    assert default_qos().depth == DEFAULT_QOS.depth


# =============================================================================
# Clock helpers
# =============================================================================
def test_now_s_returns_wall_time() -> None:
    before = time.time()
    value = now_s()
    after = time.time()

    assert before <= value <= after


def test_monotonic_s_increases() -> None:
    first = monotonic_s()
    second = monotonic_s()

    assert second >= first


def test_age_s_with_timestamp() -> None:
    assert age_s(10.0, now=12.5) == pytest.approx(2.5)


def test_age_s_none_returns_none() -> None:
    assert age_s(None) is None


def test_age_s_never_negative() -> None:
    assert age_s(20.0, now=10.0) == pytest.approx(0.0)


def test_is_stale_true_for_none_timestamp() -> None:
    assert is_stale(None, timeout_s=1.0) is True


def test_is_stale_false_when_fresh() -> None:
    assert is_stale(10.0, timeout_s=1.0, now=10.5) is False


def test_is_stale_true_when_old() -> None:
    assert is_stale(10.0, timeout_s=1.0, now=11.5) is True


# =============================================================================
# RateTracker
# =============================================================================
def test_rate_tracker_initial_rate_is_zero() -> None:
    tracker = RateTracker()

    assert tracker.count == 0
    assert tracker.rate_hz == 0.0
    assert tracker.last_stamp_s is None


def test_rate_tracker_computes_rate() -> None:
    tracker = RateTracker()

    tracker.tick(0.0)
    tracker.tick(1.0)
    tracker.tick(2.0)

    assert tracker.count == 3
    assert tracker.last_stamp_s == pytest.approx(2.0)
    assert tracker.rate_hz == pytest.approx(1.0)


def test_rate_tracker_reset() -> None:
    tracker = RateTracker()

    tracker.tick(0.0)
    tracker.tick(1.0)
    tracker.reset()

    assert tracker.count == 0
    assert tracker.rate_hz == 0.0
    assert tracker.last_stamp_s is None


def test_rate_tracker_window_trim() -> None:
    tracker = RateTracker(window_s=1.0)

    tracker.tick(0.0)
    tracker.tick(0.5)
    tracker.tick(2.0)

    assert tracker.count == 1
    assert tracker.last_stamp_s == pytest.approx(2.0)
    assert tracker.rate_hz == 0.0


# =============================================================================
# StaleMonitor
# =============================================================================
def test_stale_monitor_starts_stale() -> None:
    monitor = StaleMonitor(timeout_s=0.5)

    assert monitor.age_s() is None
    assert monitor.stale() is True


def test_stale_monitor_update_and_age() -> None:
    monitor = StaleMonitor(timeout_s=0.5)

    monitor.update(stamp_s=10.0)

    assert monitor.age_s(now=10.25) == pytest.approx(0.25)
    assert monitor.stale(now=10.25) is False
    assert monitor.stale(now=11.0) is True


def test_stale_monitor_reset() -> None:
    monitor = StaleMonitor(timeout_s=0.5)

    monitor.update(stamp_s=10.0)
    monitor.reset()

    assert monitor.last_update_s is None
    assert monitor.stale() is True


# =============================================================================
# SimpleTimer
# =============================================================================
def test_simple_timer_elapsed_non_negative() -> None:
    timer = SimpleTimer()

    assert timer.elapsed_s() >= 0.0


def test_simple_timer_expired() -> None:
    timer = SimpleTimer(started_s=monotonic_s() - 2.0)

    assert timer.expired(1.0) is True
    assert timer.expired(3.0) is False


def test_simple_timer_reset() -> None:
    timer = SimpleTimer(started_s=monotonic_s() - 2.0)

    timer.reset()

    assert timer.elapsed_s() < 1.0


# =============================================================================
# Ratekeeper
# =============================================================================
def test_ratekeeper_period() -> None:
    ratekeeper = Ratekeeper(5.0)

    assert ratekeeper.period_s == pytest.approx(0.2)


def test_ratekeeper_rejects_invalid_frequency() -> None:
    with pytest.raises(ValueError):
        Ratekeeper(0.0)

    with pytest.raises(ValueError):
        Ratekeeper(-1.0)


def test_ratekeeper_should_run_and_mark_run() -> None:
    ratekeeper = Ratekeeper(10.0)

    assert ratekeeper.should_run() is False

    ratekeeper._next_tick_s = time.monotonic() - 0.01

    assert ratekeeper.should_run() is True

    ratekeeper.mark_run()

    assert ratekeeper.should_run() is False


def test_ratekeeper_reset_moves_next_tick_forward() -> None:
    ratekeeper = Ratekeeper(10.0)

    ratekeeper._next_tick_s = 0.0
    ratekeeper.reset()

    assert ratekeeper._next_tick_s > time.monotonic()


def test_ratekeeper_sleep_does_not_crash() -> None:
    ratekeeper = Ratekeeper(1000.0)

    ratekeeper.sleep()

    assert ratekeeper.period_s == pytest.approx(0.001)


# =============================================================================
# LoopBudget
# =============================================================================
def test_loop_budget_period() -> None:
    budget = LoopBudget(5.0)

    assert budget.period_s == pytest.approx(0.2)


def test_loop_budget_rejects_invalid_frequency() -> None:
    budget = LoopBudget(0.0)

    with pytest.raises(ValueError):
        _ = budget.period_s


def test_loop_budget_elapsed_remaining_and_overrun() -> None:
    budget = LoopBudget(10.0)

    assert budget.elapsed_s() >= 0.0
    assert 0.0 <= budget.remaining_s() <= 0.1
    assert budget.overrun_s() == pytest.approx(0.0)


def test_loop_budget_overrun() -> None:
    budget = LoopBudget(10.0, started_s=time.monotonic() - 0.2)

    assert budget.overrun_s() > 0.0
    assert budget.remaining_s() == pytest.approx(0.0)


def test_loop_budget_reset() -> None:
    budget = LoopBudget(10.0, started_s=time.monotonic() - 0.2)

    budget.reset()

    assert budget.elapsed_s() < 0.1