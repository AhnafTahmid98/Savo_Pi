#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EKF health models for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    DEFAULT_EKF_HEALTH_TOPIC,
    DEFAULT_EKF_RATE_HZ,
    DEFAULT_EKF_SENSOR_TIMEOUT_S,
    DEFAULT_FILTERED_ODOM_TOPIC,
    DEFAULT_IMU_TOPIC,
    DEFAULT_WHEEL_ODOM_TOPIC,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)


@dataclass
class EkfInputHealth:
    name: str
    topic: str
    enabled: bool = True

    seen: bool = False
    fresh: bool = False
    last_age_s: float | None = None
    message_count: int = 0
    rate_hz: float = 0.0

    status: str = STATUS_UNKNOWN
    message: str = "input state unknown"
    reasons: list[str] = field(default_factory=list)

    def mark_seen(
        self,
        *,
        age_s: float | None = 0.0,
        rate_hz: float = 0.0,
        message_count: int | None = None,
    ) -> None:
        self.seen = True
        self.fresh = True
        self.last_age_s = age_s
        self.rate_hz = float(rate_hz)

        if message_count is not None:
            self.message_count = int(message_count)
        else:
            self.message_count += 1

        self.status = STATUS_OK
        self.message = "input healthy"
        self.reasons.clear()

    def mark_stale(self, age_s: float | None = None) -> None:
        self.seen = self.message_count > 0
        self.fresh = False
        self.last_age_s = age_s

        self.status = STATUS_STALE
        self.message = "input stale"
        self.reasons = [
            "no input age available" if age_s is None else f"age_s={age_s:.3f}"
        ]

    def mark_missing(self) -> None:
        self.seen = False
        self.fresh = False
        self.last_age_s = None

        self.status = STATUS_STALE
        self.message = "input not received"
        self.reasons = [f"no messages received on {self.topic}"]

    def mark_disabled(self) -> None:
        self.enabled = False
        self.status = STATUS_UNKNOWN
        self.message = "input disabled"
        self.reasons.clear()

    @property
    def ok(self) -> bool:
        return bool(self.enabled and self.seen and self.fresh)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self) | {"ok": self.ok}


@dataclass
class EkfOutputHealth:
    topic: str = DEFAULT_FILTERED_ODOM_TOPIC
    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BASE_LINK

    seen: bool = False
    fresh: bool = False
    last_age_s: float | None = None
    message_count: int = 0
    rate_hz: float = 0.0

    frame_ok: bool = False
    finite_ok: bool = False
    covariance_ok: bool = True

    status: str = STATUS_UNKNOWN
    message: str = "EKF output state unknown"
    reasons: list[str] = field(default_factory=list)

    def mark_output(
        self,
        *,
        age_s: float | None = 0.0,
        rate_hz: float = 0.0,
        message_count: int | None = None,
        frame_ok: bool = True,
        finite_ok: bool = True,
        covariance_ok: bool = True,
    ) -> None:
        self.seen = True
        self.fresh = True
        self.last_age_s = age_s
        self.rate_hz = float(rate_hz)

        if message_count is not None:
            self.message_count = int(message_count)
        else:
            self.message_count += 1

        self.frame_ok = bool(frame_ok)
        self.finite_ok = bool(finite_ok)
        self.covariance_ok = bool(covariance_ok)

        reasons: list[str] = []

        if not self.frame_ok:
            reasons.append("EKF odometry frame ids are not valid")

        if not self.finite_ok:
            reasons.append("EKF output contains non-finite pose or twist values")

        if not self.covariance_ok:
            reasons.append("EKF output covariance is not valid")

        if reasons:
            self.status = STATUS_WARN
            self.message = "EKF output usable with notes"
            self.reasons = reasons
            return

        self.status = STATUS_OK
        self.message = "EKF output healthy"
        self.reasons.clear()

    def mark_stale(self, age_s: float | None = None) -> None:
        self.fresh = False
        self.last_age_s = age_s

        self.status = STATUS_STALE
        self.message = "EKF output stale"
        self.reasons = [
            "no output age available" if age_s is None else f"age_s={age_s:.3f}"
        ]

    def mark_missing(self) -> None:
        self.seen = False
        self.fresh = False
        self.last_age_s = None

        self.status = STATUS_STALE
        self.message = "EKF output not received"
        self.reasons = [f"no messages received on {self.topic}"]

    @property
    def ok(self) -> bool:
        return (
            self.seen
            and self.fresh
            and self.frame_ok
            and self.finite_ok
            and self.covariance_ok
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self) | {"ok": self.ok}


@dataclass
class EkfTfHealth:
    parent_frame: str = FRAME_ODOM
    child_frame: str = FRAME_BASE_LINK

    available: bool = False
    fresh: bool = False
    last_age_s: float | None = None

    status: str = STATUS_UNKNOWN
    message: str = "EKF TF state unknown"
    reasons: list[str] = field(default_factory=list)

    def mark_available(self, *, age_s: float | None = 0.0) -> None:
        self.available = True
        self.fresh = True
        self.last_age_s = age_s

        self.status = STATUS_OK
        self.message = "EKF TF healthy"
        self.reasons.clear()

    def mark_missing(self, reason: str = "transform not available") -> None:
        self.available = False
        self.fresh = False

        self.status = STATUS_STALE
        self.message = "EKF TF missing"
        self.reasons = [reason]

    def mark_stale(self, age_s: float | None = None) -> None:
        self.available = True
        self.fresh = False
        self.last_age_s = age_s

        self.status = STATUS_STALE
        self.message = "EKF TF stale"
        self.reasons = [
            "no TF age available" if age_s is None else f"age_s={age_s:.3f}"
        ]

    @property
    def ok(self) -> bool:
        return self.available and self.fresh

    def to_dict(self) -> dict[str, Any]:
        return asdict(self) | {"ok": self.ok}


@dataclass
class EkfHealth:
    health_topic: str = DEFAULT_EKF_HEALTH_TOPIC
    output_topic: str = DEFAULT_FILTERED_ODOM_TOPIC

    expected_rate_hz: float = DEFAULT_EKF_RATE_HZ
    sensor_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S

    wheel_odom: EkfInputHealth = field(
        default_factory=lambda: EkfInputHealth(
            name="wheel_odom",
            topic=DEFAULT_WHEEL_ODOM_TOPIC,
            enabled=True,
        )
    )
    imu: EkfInputHealth = field(
        default_factory=lambda: EkfInputHealth(
            name="imu",
            topic=DEFAULT_IMU_TOPIC,
            enabled=True,
        )
    )
    vo_odom: EkfInputHealth = field(
        default_factory=lambda: EkfInputHealth(
            name="vo_odom",
            topic="/vo/odom",
            enabled=False,
        )
    )

    output: EkfOutputHealth = field(default_factory=EkfOutputHealth)
    tf: EkfTfHealth = field(default_factory=EkfTfHealth)

    status: str = STATUS_UNKNOWN
    message: str = "EKF health unknown"
    reasons: list[str] = field(default_factory=list)

    def evaluate(self) -> None:
        reasons: list[str] = []

        active_inputs = [sensor for sensor in self.inputs if sensor.enabled]
        healthy_inputs = [sensor for sensor in active_inputs if sensor.ok]

        if not active_inputs:
            reasons.append("no EKF inputs are enabled")

        for sensor in active_inputs:
            if not sensor.seen:
                reasons.append(f"{sensor.name} input not received")
            elif not sensor.fresh:
                reasons.append(f"{sensor.name} input stale")

        if not healthy_inputs:
            reasons.append("no healthy EKF inputs available")

        if not self.output.seen:
            reasons.append("EKF output not received")
        elif not self.output.fresh:
            reasons.append("EKF output stale")
        elif not self.output.ok:
            reasons.extend(self.output.reasons)

        if not self.tf.ok:
            reasons.append("odom to base_link TF is not healthy")

        self.reasons = reasons

        if not reasons:
            self.status = STATUS_OK
            self.message = "EKF healthy"
            return

        if self.output.seen and healthy_inputs:
            self.status = STATUS_WARN
            self.message = "EKF usable with notes"
            return

        self.status = STATUS_ERROR
        self.message = "EKF not ready"

    @property
    def inputs(self) -> tuple[EkfInputHealth, EkfInputHealth, EkfInputHealth]:
        return (self.wheel_odom, self.imu, self.vo_odom)

    @property
    def active_inputs(self) -> tuple[EkfInputHealth, ...]:
        return tuple(sensor for sensor in self.inputs if sensor.enabled)

    @property
    def ready(self) -> bool:
        return self.status == STATUS_OK

    @property
    def usable(self) -> bool:
        return self.status in (STATUS_OK, STATUS_WARN)

    def mark_all_stale(self) -> None:
        for sensor in self.active_inputs:
            sensor.mark_stale()

        self.output.mark_stale()
        self.tf.mark_stale()
        self.evaluate()

    def to_dict(self) -> dict[str, Any]:
        return {
            "health_topic": self.health_topic,
            "output_topic": self.output_topic,
            "expected_rate_hz": float(self.expected_rate_hz),
            "sensor_timeout_s": float(self.sensor_timeout_s),
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "ready": self.ready,
            "usable": self.usable,
            "active_inputs": [sensor.name for sensor in self.active_inputs],
            "wheel_odom": self.wheel_odom.to_dict(),
            "imu": self.imu.to_dict(),
            "vo_odom": self.vo_odom.to_dict(),
            "output": self.output.to_dict(),
            "tf": self.tf.to_dict(),
        }


def make_ekf_health(
    *,
    use_wheel_odom: bool = True,
    use_imu: bool = True,
    use_vo: bool = False,
    health_topic: str = DEFAULT_EKF_HEALTH_TOPIC,
    output_topic: str = DEFAULT_FILTERED_ODOM_TOPIC,
) -> EkfHealth:
    health = EkfHealth(
        health_topic=health_topic,
        output_topic=output_topic,
    )
    health.wheel_odom.enabled = bool(use_wheel_odom)
    health.imu.enabled = bool(use_imu)
    health.vo_odom.enabled = bool(use_vo)
    health.evaluate()
    return health