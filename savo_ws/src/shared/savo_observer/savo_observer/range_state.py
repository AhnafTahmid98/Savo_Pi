"""ROS-independent range freshness classification for observer markers."""

import math


def range_sample_state(value, received_monotonic, now_monotonic, stale_s):
    """Classify only data quality/freshness, never robot safety."""
    if received_monotonic is None or value is None:
        return 'MISSING'
    if now_monotonic - received_monotonic > stale_s:
        return 'STALE'
    if not math.isfinite(value) or value <= 0.0:
        return 'INVALID'
    return 'VALID'
