from collections.abc import Iterable

from savo_realsense.ros.frame_contract import REQUIRED_TF_FRAMES


def missing_frames(
    available_frames: Iterable[str],
    required_frames: Iterable[str] = REQUIRED_TF_FRAMES,
) -> list[str]:
    available = {frame.lstrip("/") for frame in available_frames}

    return [
        frame
        for frame in required_frames
        if frame.lstrip("/") not in available
    ]


def frames_available(
    available_frames: Iterable[str],
    required_frames: Iterable[str] = REQUIRED_TF_FRAMES,
) -> bool:
    return not missing_frames(available_frames, required_frames)


def optical_frames_present(available_frames: Iterable[str]) -> list[str]:
    return [
        frame
        for frame in available_frames
        if frame.lstrip("/").endswith("_optical_frame")
    ]


def format_missing_frames_report(missing: list[str]) -> str:
    if not missing:
        return "All required RealSense frames are available"

    lines = ["Missing RealSense frames:"]
    lines.extend(f"- {frame}" for frame in missing)

    return "\n".join(lines)