# Copyright 2026 Ahnaf Tahmid
from savo_realsense.utils.timing import RateTracker, is_stale


def test_rate_tracker_has_no_data_initially() -> None:
    tracker = RateTracker()

    assert not tracker.seen
    assert tracker.rate_hz == 0.0
    assert tracker.last_age_s == float("inf")


def test_rate_tracker_marks_stream_as_seen_after_tick() -> None:
    tracker = RateTracker()

    tracker.tick(10.0)

    assert tracker.seen


def test_rate_tracker_estimates_rate_from_timestamps() -> None:
    tracker = RateTracker()

    tracker.tick(10.0)
    tracker.tick(10.5)
    tracker.tick(11.0)

    assert tracker.rate_hz == 2.0


def test_rate_tracker_returns_zero_rate_for_single_sample() -> None:
    tracker = RateTracker()

    tracker.tick(10.0)

    assert tracker.rate_hz == 0.0


def test_rate_tracker_keeps_window_size() -> None:
    tracker = RateTracker(window_size=2)

    tracker.tick(10.0)
    tracker.tick(11.0)
    tracker.tick(12.0)

    assert tracker.rate_hz == 1.0


def test_is_stale_detects_old_stream() -> None:
    assert is_stale(last_age_s=0.80, stale_timeout_s=0.50)


def test_is_stale_accepts_fresh_stream() -> None:
    assert not is_stale(last_age_s=0.20, stale_timeout_s=0.50)
