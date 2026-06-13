# -*- coding: utf-8 -*-

import math

from savo_lidar.filters import (
    count_valid_ranges,
    filter_range_value,
    filter_ranges,
    is_valid_range,
    range_stats,
    valid_range_ratio,
)


def test_is_valid_range_accepts_finite_value_inside_limits():
    assert is_valid_range(1.0, 0.15, 12.0)


def test_is_valid_range_rejects_value_below_minimum():
    assert not is_valid_range(0.10, 0.15, 12.0)


def test_is_valid_range_rejects_value_above_maximum():
    assert not is_valid_range(13.0, 0.15, 12.0)


def test_is_valid_range_rejects_nan_and_inf():
    assert not is_valid_range(float("nan"), 0.15, 12.0)
    assert not is_valid_range(float("inf"), 0.15, 12.0)
    assert not is_valid_range(float("-inf"), 0.15, 12.0)


def test_is_valid_range_rejects_non_numeric_value():
    assert not is_valid_range("bad", 0.15, 12.0)
    assert not is_valid_range(None, 0.15, 12.0)


def test_filter_range_value_keeps_valid_value():
    assert filter_range_value(
        2.0,
        min_range_m=0.15,
        max_range_m=12.0,
    ) == 2.0


def test_filter_range_value_replaces_invalid_value_with_inf_by_default():
    result = filter_range_value(
        0.10,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert math.isinf(result)


def test_filter_range_value_supports_custom_invalid_value():
    result = filter_range_value(
        float("nan"),
        min_range_m=0.15,
        max_range_m=12.0,
        invalid_value=-1.0,
    )

    assert result == -1.0


def test_filter_ranges_replaces_invalid_values():
    ranges = [0.10, 0.20, 2.0, 13.0, float("inf"), float("nan"), "bad"]

    filtered = filter_ranges(
        ranges,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert math.isinf(filtered[0])
    assert filtered[1] == 0.20
    assert filtered[2] == 2.0
    assert math.isinf(filtered[3])
    assert math.isinf(filtered[4])
    assert math.isinf(filtered[5])
    assert math.isinf(filtered[6])


def test_filter_ranges_supports_custom_invalid_value():
    ranges = [0.10, 0.20, 13.0]

    filtered = filter_ranges(
        ranges,
        min_range_m=0.15,
        max_range_m=12.0,
        invalid_value=-1.0,
    )

    assert filtered == [-1.0, 0.20, -1.0]


def test_count_valid_ranges_counts_only_valid_values():
    ranges = [0.10, 0.20, 2.0, 13.0, float("inf"), float("nan"), "bad"]

    assert count_valid_ranges(
        ranges,
        min_range_m=0.15,
        max_range_m=12.0,
    ) == 2


def test_valid_range_ratio_returns_zero_for_empty_input():
    assert valid_range_ratio(
        [],
        min_range_m=0.15,
        max_range_m=12.0,
    ) == 0.0


def test_valid_range_ratio_returns_fraction_of_valid_values():
    ranges = [0.10, 0.20, 2.0, 13.0]

    ratio = valid_range_ratio(
        ranges,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert ratio == 0.5


def test_range_stats_returns_empty_stats_when_no_ranges():
    stats = range_stats(
        [],
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert stats == (0, 0, 0.0, None, None, None)


def test_range_stats_returns_none_values_when_no_valid_ranges():
    stats = range_stats(
        [0.10, 13.0, float("inf"), float("nan"), "bad"],
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert stats == (5, 0, 0.0, None, None, None)


def test_range_stats_returns_summary_for_valid_ranges():
    stats = range_stats(
        [0.10, 0.20, 2.0, 13.0, float("inf"), float("nan"), "bad"],
        min_range_m=0.15,
        max_range_m=12.0,
    )

    total_points, valid_points, ratio, min_seen, max_seen, mean_seen = stats

    assert total_points == 7
    assert valid_points == 2
    assert ratio == 2 / 7
    assert min_seen == 0.20
    assert max_seen == 2.0
    assert mean_seen == 1.10


def test_range_stats_accepts_tuple_input():
    stats = range_stats(
        (0.20, 1.0, 2.0),
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert stats == (3, 3, 1.0, 0.20, 2.0, 1.0666666666666667)
