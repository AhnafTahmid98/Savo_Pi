def normalize_frame_name(frame: str) -> str:
    cleaned = frame.strip()
    if not cleaned:
        raise ValueError("Frame name cannot be empty")

    return cleaned.lstrip("/")


def has_optical_frame_suffix(frame: str) -> bool:
    return normalize_frame_name(frame).endswith("_optical_frame")


def is_camera_frame(frame: str) -> bool:
    cleaned = normalize_frame_name(frame)
    return cleaned.startswith("camera_") or cleaned == "camera_link"


def unique_frame_names(*frames: str) -> tuple[str, ...]:
    seen = set()
    unique = []

    for frame in frames:
        cleaned = normalize_frame_name(frame)
        if cleaned not in seen:
            seen.add(cleaned)
            unique.append(cleaned)

    return tuple(unique)