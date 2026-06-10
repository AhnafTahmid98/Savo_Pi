"""Timestamp checks for synchronized RGB-D visual odometry inputs."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TimestampSyncResult:
    is_synced: bool
    delta_s: float
    message: str = ""


def timestamp_delta_s(first_timestamp_s: float, second_timestamp_s: float) -> float:
    return abs(first_timestamp_s - second_timestamp_s)


def is_timestamp_synced(
    first_timestamp_s: float,
    second_timestamp_s: float,
    max_delta_s: float,
) -> bool:
    if max_delta_s < 0.0:
        raise ValueError("max_delta_s must be non-negative")

    return timestamp_delta_s(first_timestamp_s, second_timestamp_s) <= max_delta_s


def check_timestamp_sync(
    first_timestamp_s: float,
    second_timestamp_s: float,
    max_delta_s: float,
) -> TimestampSyncResult:
    delta_s = timestamp_delta_s(first_timestamp_s, second_timestamp_s)
    synced = is_timestamp_synced(first_timestamp_s, second_timestamp_s, max_delta_s)

    if synced:
        return TimestampSyncResult(
            is_synced=True,
            delta_s=delta_s,
            message="timestamps are synchronized",
        )

    return TimestampSyncResult(
        is_synced=False,
        delta_s=delta_s,
        message=f"timestamp delta {delta_s:.3f}s exceeds limit {max_delta_s:.3f}s",
    )


def is_stale(
    timestamp_s: float,
    now_s: float,
    stale_timeout_s: float,
) -> bool:
    if stale_timeout_s < 0.0:
        raise ValueError("stale_timeout_s must be non-negative")

    return age_s(timestamp_s=timestamp_s, now_s=now_s) > stale_timeout_s


def age_s(timestamp_s: float, now_s: float) -> float:
    return max(0.0, now_s - timestamp_s)