from savo_realsense.models.stream_status import StreamStatus
from savo_realsense.utils.timing import is_stale


def build_stream_status(
    topic: str,
    seen: bool,
    rate_hz: float,
    expected_hz: float,
    last_age_s: float,
    stale_timeout_s: float,
) -> StreamStatus:
    return StreamStatus(
        topic=topic,
        seen=seen,
        stale=is_stale(last_age_s, stale_timeout_s),
        rate_hz=max(0.0, rate_hz),
        expected_hz=max(0.0, expected_hz),
        last_age_s=max(0.0, last_age_s),
    )


def stream_is_healthy(status: StreamStatus) -> bool:
    return status.ok and not status.below_expected_rate


def missing_or_stale_topics(statuses: list[StreamStatus]) -> list[str]:
    return [
        status.topic
        for status in statuses
        if not status.seen or status.stale
    ]


def low_rate_topics(statuses: list[StreamStatus]) -> list[str]:
    return [
        status.topic
        for status in statuses
        if status.seen and not status.stale and status.below_expected_rate
    ]


def all_required_streams_healthy(statuses: list[StreamStatus]) -> bool:
    return all(stream_is_healthy(status) for status in statuses)