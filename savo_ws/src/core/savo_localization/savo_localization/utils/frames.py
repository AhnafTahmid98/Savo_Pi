#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frame-name helpers for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_ODOM,
    FRAME_WHEEL_ODOM,
)


@dataclass(frozen=True)
class LocalizationFrames:
    odom: str = FRAME_ODOM
    base_link: str = FRAME_BASE_LINK
    imu: str = FRAME_IMU
    wheel_odom: str = FRAME_WHEEL_ODOM

    def to_dict(self) -> dict[str, str]:
        return {
            "odom": self.odom,
            "base_link": self.base_link,
            "imu": self.imu,
            "wheel_odom": self.wheel_odom,
        }


def normalize_frame_id(frame_id: str) -> str:
    frame_id = str(frame_id).strip().lstrip("/")

    if not frame_id:
        raise ValueError("Frame id cannot be empty.")

    if " " in frame_id:
        raise ValueError(f"Frame id cannot contain spaces: {frame_id!r}")

    return frame_id


def make_localization_frames(
    *,
    odom: str = FRAME_ODOM,
    base_link: str = FRAME_BASE_LINK,
    imu: str = FRAME_IMU,
    wheel_odom: str = FRAME_WHEEL_ODOM,
) -> LocalizationFrames:
    frames = LocalizationFrames(
        odom=normalize_frame_id(odom),
        base_link=normalize_frame_id(base_link),
        imu=normalize_frame_id(imu),
        wheel_odom=normalize_frame_id(wheel_odom),
    )

    validate_localization_frames(frames)
    return frames


def validate_localization_frames(frames: LocalizationFrames) -> None:
    values = [
        frames.odom,
        frames.base_link,
        frames.imu,
        frames.wheel_odom,
    ]

    normalized = [normalize_frame_id(value) for value in values]

    if len(set(normalized)) != len(normalized):
        raise ValueError("Localization frame contract contains duplicate frame ids.")


def is_expected_imu_frame(frame_id: str, *, expected: str = FRAME_IMU) -> bool:
    return normalize_frame_id(frame_id) == normalize_frame_id(expected)


def is_expected_odom_frame(frame_id: str, *, expected: str = FRAME_ODOM) -> bool:
    return normalize_frame_id(frame_id) == normalize_frame_id(expected)


def is_expected_base_frame(frame_id: str, *, expected: str = FRAME_BASE_LINK) -> bool:
    return normalize_frame_id(frame_id) == normalize_frame_id(expected)


def frame_pair(parent: str, child: str) -> tuple[str, str]:
    return normalize_frame_id(parent), normalize_frame_id(child)


def odom_to_base_pair(
    *,
    odom: str = FRAME_ODOM,
    base_link: str = FRAME_BASE_LINK,
) -> tuple[str, str]:
    return frame_pair(odom, base_link)


def base_to_imu_pair(
    *,
    base_link: str = FRAME_BASE_LINK,
    imu: str = FRAME_IMU,
) -> tuple[str, str]:
    return frame_pair(base_link, imu)


def require_expected_frame(
    actual: str,
    expected: str,
    *,
    label: str = "frame_id",
) -> None:
    actual_norm = normalize_frame_id(actual)
    expected_norm = normalize_frame_id(expected)

    if actual_norm != expected_norm:
        raise ValueError(
            f"{label} mismatch: expected {expected_norm!r}, got {actual_norm!r}"
        )