"""ROS time conversion helpers for savo_vo."""


def stamp_to_seconds(stamp: object) -> float:
    sec = getattr(stamp, "sec")
    nanosec = getattr(stamp, "nanosec")
    return float(sec) + float(nanosec) * 1e-9


def seconds_to_stamp_parts(timestamp_s: float) -> tuple[int, int]:
    if timestamp_s < 0.0:
        raise ValueError("timestamp_s must be non-negative")

    sec = int(timestamp_s)
    nanosec = int(round((timestamp_s - sec) * 1e9))

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return sec, nanosec


def now_seconds_from_clock(clock: object) -> float:
    now_msg = clock.now().to_msg()
    return stamp_to_seconds(now_msg)