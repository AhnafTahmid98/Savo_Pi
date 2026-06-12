"""Frame-id diagnostics for LiDAR TF integration."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class FrameIdCheckResult:
    frame_id: str
    expected_frame_id: str
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def normalize_frame_id(frame_id: str) -> str:
    return str(frame_id).strip().lstrip("/")


def check_frame_id(
    *,
    frame_id: str,
    expected_frame_id: str = "laser",
) -> FrameIdCheckResult:
    normalized = normalize_frame_id(frame_id)
    expected = normalize_frame_id(expected_frame_id)

    if not normalized:
        return FrameIdCheckResult(
            frame_id=normalized,
            expected_frame_id=expected,
            ok=False,
            message="LiDAR frame_id is empty",
        )

    if normalized != expected:
        return FrameIdCheckResult(
            frame_id=normalized,
            expected_frame_id=expected,
            ok=False,
            message="LiDAR frame_id does not match expected TF frame",
        )

    return FrameIdCheckResult(
        frame_id=normalized,
        expected_frame_id=expected,
        ok=True,
        message="LiDAR frame_id matches expected TF frame",
    )