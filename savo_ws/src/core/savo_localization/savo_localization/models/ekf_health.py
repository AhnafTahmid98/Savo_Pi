#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EKF health models for Robot Savo localization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from savo_localization.constants import (
    DEFAULT_FILTERED_ODOM_TOPIC,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    STATUS_UNKNOWN,
)


@dataclass
class EkfInputHealth:
    name: str = ""
    topic: str = ""
    enabled: bool = True

    seen: bool = False
    fresh: bool = False
    last_age_s: float | None = None
    message_count: int = 0
    rate_hz: float = 0.0

    status: str = STATUS_UNKNOWN
    message: str = ""
    reasons: list[str] = field(default_factory=list)

    @property
    def ready(self) -> bool:
        return self.enabled and self.seen and self.fresh and self.message_count > 0

    @property
    def ok(self) -> bool:
        return self.ready

    @property
    def usable(self) -> bool:
        return (not self.enabled) or self.ready

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "topic": self.topic,
            "enabled": self.enabled,
            "seen": self.seen,
            "fresh": self.fresh,
            "last_age_s": self.last_age_s,
            "message_count": self.message_count,
            "rate_hz": self.rate_hz,
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "ready": self.ready,
            "ok": self.ok,
            "usable": self.usable,
        }


@dataclass
class EkfOutputHealth:
    topic: str = DEFAULT_FILTERED_ODOM_TOPIC

    seen: bool = False
    fresh: bool = False
    last_age_s: float | None = None
    message_count: int = 0
    rate_hz: float = 0.0

    frame_ok: bool = False
    finite_ok: bool = False
    covariance_ok: bool = False

    status: str = STATUS_UNKNOWN
    message: str = ""
    reasons: list[str] = field(default_factory=list)

    @property
    def ready(self) -> bool:
        return (
            self.seen
            and self.fresh
            and self.message_count > 0
            and self.frame_ok
            and self.finite_ok
            and self.covariance_ok
        )

    @property
    def ok(self) -> bool:
        return self.ready

    @property
    def usable(self) -> bool:
        return self.ready

    def to_dict(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "seen": self.seen,
            "fresh": self.fresh,
            "last_age_s": self.last_age_s,
            "message_count": self.message_count,
            "rate_hz": self.rate_hz,
            "frame_ok": self.frame_ok,
            "finite_ok": self.finite_ok,
            "covariance_ok": self.covariance_ok,
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "ready": self.ready,
            "ok": self.ok,
            "usable": self.usable,
        }


@dataclass
class EkfTfHealth:
    parent_frame: str = FRAME_ODOM
    child_frame: str = FRAME_BASE_LINK

    available: bool = False
    fresh: bool = False
    last_age_s: float | None = None

    status: str = STATUS_UNKNOWN
    message: str = ""
    reasons: list[str] = field(default_factory=list)

    @property
    def label(self) -> str:
        return f"{self.parent_frame} -> {self.child_frame}"

    @property
    def ready(self) -> bool:
        return self.available and self.fresh

    @property
    def ok(self) -> bool:
        return self.ready

    @property
    def usable(self) -> bool:
        return self.ready

    def to_dict(self) -> dict[str, Any]:
        return {
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "label": self.label,
            "available": self.available,
            "fresh": self.fresh,
            "last_age_s": self.last_age_s,
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "ready": self.ready,
            "ok": self.ok,
            "usable": self.usable,
        }


@dataclass
class EkfHealth:
    inputs: list[EkfInputHealth] = field(default_factory=list)
    output: EkfOutputHealth = field(default_factory=EkfOutputHealth)
    tf: EkfTfHealth = field(default_factory=EkfTfHealth)

    status: str = STATUS_UNKNOWN
    message: str = ""
    reasons: list[str] = field(default_factory=list)

    @property
    def active_inputs(self) -> list[EkfInputHealth]:
        return [sensor for sensor in self.inputs if sensor.enabled]

    @property
    def healthy_inputs(self) -> list[EkfInputHealth]:
        return [sensor for sensor in self.active_inputs if sensor.ok]

    @property
    def active_input_count(self) -> int:
        return len(self.active_inputs)

    @property
    def healthy_input_count(self) -> int:
        return len(self.healthy_inputs)

    @property
    def ready(self) -> bool:
        return self.output.ok and self.tf.ok and self.healthy_input_count > 0

    @property
    def ok(self) -> bool:
        return self.ready

    @property
    def usable(self) -> bool:
        return self.ready

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "ready": self.ready,
            "ok": self.ok,
            "usable": self.usable,
            "active_input_count": self.active_input_count,
            "healthy_input_count": self.healthy_input_count,
            "inputs": [sensor.to_dict() for sensor in self.inputs],
            "output": self.output.to_dict(),
            "tf": self.tf.to_dict(),
        }


def make_ekf_input_health(
    *,
    name: str = "",
    topic: str = "",
    enabled: bool = True,
    seen: bool = False,
    fresh: bool = False,
    last_age_s: float | None = None,
    message_count: int = 0,
    rate_hz: float = 0.0,
    status: str = STATUS_UNKNOWN,
    message: str = "",
    reasons: list[str] | None = None,
) -> EkfInputHealth:
    return EkfInputHealth(
        name=name,
        topic=topic,
        enabled=enabled,
        seen=seen,
        fresh=fresh,
        last_age_s=last_age_s,
        message_count=message_count,
        rate_hz=rate_hz,
        status=status,
        message=message,
        reasons=list(reasons or []),
    )


def make_ekf_output_health(
    *,
    topic: str = DEFAULT_FILTERED_ODOM_TOPIC,
    seen: bool = False,
    fresh: bool = False,
    last_age_s: float | None = None,
    message_count: int = 0,
    rate_hz: float = 0.0,
    frame_ok: bool = False,
    finite_ok: bool = False,
    covariance_ok: bool = False,
    status: str = STATUS_UNKNOWN,
    message: str = "",
    reasons: list[str] | None = None,
) -> EkfOutputHealth:
    return EkfOutputHealth(
        topic=topic,
        seen=seen,
        fresh=fresh,
        last_age_s=last_age_s,
        message_count=message_count,
        rate_hz=rate_hz,
        frame_ok=frame_ok,
        finite_ok=finite_ok,
        covariance_ok=covariance_ok,
        status=status,
        message=message,
        reasons=list(reasons or []),
    )


def make_ekf_tf_health(
    *,
    parent_frame: str = FRAME_ODOM,
    child_frame: str = FRAME_BASE_LINK,
    available: bool = False,
    fresh: bool = False,
    last_age_s: float | None = None,
    status: str = STATUS_UNKNOWN,
    message: str = "",
    reasons: list[str] | None = None,
) -> EkfTfHealth:
    return EkfTfHealth(
        parent_frame=parent_frame,
        child_frame=child_frame,
        available=available,
        fresh=fresh,
        last_age_s=last_age_s,
        status=status,
        message=message,
        reasons=list(reasons or []),
    )


def make_ekf_health(
    *,
    inputs: list[EkfInputHealth] | None = None,
    output: EkfOutputHealth | None = None,
    tf: EkfTfHealth | None = None,
    status: str = STATUS_UNKNOWN,
    message: str = "",
    reasons: list[str] | None = None,
) -> EkfHealth:
    return EkfHealth(
        inputs=list(inputs or []),
        output=output or EkfOutputHealth(),
        tf=tf or EkfTfHealth(),
        status=status,
        message=message,
        reasons=list(reasons or []),
    )


__all__ = [
    "EkfInputHealth",
    "EkfOutputHealth",
    "EkfTfHealth",
    "EkfHealth",
    "make_ekf_input_health",
    "make_ekf_output_health",
    "make_ekf_tf_health",
    "make_ekf_health",
]
