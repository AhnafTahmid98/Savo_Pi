from savo_lidar.filters.range_filter import (
    count_valid_ranges,
    filter_range_value,
    filter_ranges,
    is_valid_range,
    range_stats,
    valid_range_ratio,
)


def test_is_valid_range_accepts_finite_value_inside_limits():
    assert is_valid_range(1.0, 0.15, 12.0)


def test_is_valid_range_rejects_values_outside_limits():
    assert not is_valid_range(0.10, 0.15, 12.0)
    assert not is_valid_range(13.0, 0.15, 12.0)
    assert not is_valid_range(float("inf"), 0.15, 12.0)


def test_filter_range_value_returns_inf_for_invalid_value():
    assert filter_range_value(0.10, min_range_m=0.15, max_range_m=12.0) == float("inf")


def test_filter_ranges_keeps_valid_values_and_replaces_invalid_values():
    ranges = [0.10, 0.20, 2.0, 13.0, float("nan")]
    filtered = filter_ranges(ranges, min_range_m=0.15, max_range_m=12.0)

    assert filtered[0] == float("inf")
    assert filtered[1] == 0.20
    assert filtered[2] == 2.0
    assert filtered[3] == float("inf")
    assert filtered[4] == float("inf")


def test_count_valid_ranges_counts_only_ranges_inside_limits():
    ranges = [0.10, 0.20, 2.0, 13.0, float("inf")]

    assert count_valid_ranges(ranges, min_range_m=0.15, max_range_m=12.0) == 2


def test_valid_range_ratio_returns_valid_fraction():
    ranges = [0.10, 0.20, 2.0, 13.0]

    assert valid_range_ratio(ranges, min_range_m=0.15, max_range_m=12.0) == 0.5


def test_valid_range_ratio_returns_zero_for_empty_ranges():
    assert valid_range_ratio([], min_range_m=0.15, max_range_m=12.0) == 0.0


def test_range_stats_returns_summary_for_valid_ranges():
    ranges = [0.10, 1.0, 2.0, 13.0]

    total, valid, ratio, min_seen, max_seen, mean_seen = range_stats(
        ranges,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert total == 4
    assert valid == 2
    assert ratio == 0.5
    assert min_seen == 1.0
    assert max_seen == 2.0
    assert mean_seen == 1.5


def test_range_stats_handles_no_valid_ranges():
    total, valid, ratio, min_seen, max_seen, mean_seen = range_stats(
        [0.01, 99.0],
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert total == 2
    assert valid == 0
    assert ratio == 0.0
    assert min_seen is None
    assert max_seen is None
    assert mean_seen is None