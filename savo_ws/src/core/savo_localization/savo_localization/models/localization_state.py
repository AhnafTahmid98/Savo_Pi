#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Combined localization state model for Robot Savo. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    DEFAULT_STATE_SUMMARY_TOPIC,
    PACKAGE_NAME,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)
from savo_localization.models.ekf_health import EkfHealth, make_ekf_health
from savo_localization.models.encoder_state import EncoderState
from savo_localization.models.imu_state import ImuState
from savo_localization.models.wheel_odom_state import WheelOdomState


@dataclass
class LocalizationComponentStatus:
    name: str
    ready: bool = False
    usable: bool = False
    status: str = STATUS_UNKNOWN
    message: str = ""
    sample_count: int = 0
    last_age_s: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class LocalizationState:
    package_name: str = PACKAGE_NAME
    state_summary_topic: str = DEFAULT_STATE_SUMMARY_TOPIC

    imu: ImuState = field(default_factory=ImuState)
    encoders: EncoderState = field(default_factory=EncoderState)
    wheel_odom: WheelOdomState = field(default_factory=WheelOdomState)
    ekf: EkfHealth = field(default_factory=EkfHealth)

    status: str = STATUS_UNKNOWN
    message: str = "localization state unknown"
    reasons: list[str] = field(default_factory=list)

    evaluation_count: int = 0

    def evaluate(self) -> None:
        self.evaluation_count += 1
        self.ekf.evaluate()

        reasons: list[str] = []

        imu_ready = self.imu.ready
        encoders_ready = self.encoders.ready
        wheel_odom_ready = self.wheel_odom.ready
        ekf_ready = self.ekf.ready
        ekf_usable = self.ekf.usable

        if not imu_ready:
            reasons.append("IMU is not ready")

        if not encoders_ready:
            reasons.append("encoders are not ready")

        if not wheel_odom_ready:
            reasons.append("wheel odometry is not ready")

        if not ekf_usable:
            reasons.append("EKF is not usable")
        elif not ekf_ready:
            reasons.append("EKF usable with notes")

        self.reasons = reasons

        if not reasons:
            self.status = STATUS_OK
            self.message = "localization healthy"
            return

        if imu_ready and encoders_ready and wheel_odom_ready and ekf_usable:
            self.status = STATUS_WARN
            self.message = "localization usable with notes"
            return

        self.status = STATUS_ERROR
        self.message = "localization not ready"

    @property
    def ready(self) -> bool:
        return self.status == STATUS_OK

    @property
    def usable(self) -> bool:
        return self.status in (STATUS_OK, STATUS_WARN)

    @property
    def core_sensor_ready(self) -> bool:
        return self.imu.ready and self.encoders.ready

    @property
    def odometry_ready(self) -> bool:
        return self.wheel_odom.ready and self.ekf.usable

    def mark_all_stale(self) -> None:
        self.imu.mark_stale(None)
        self.encoders.mark_stale(None)
        self.wheel_odom.mark_stale(None)
        self.ekf.mark_all_stale()

        self.status = STATUS_STALE
        self.message = "localization data stale"
        self.reasons = [
            "IMU stale",
            "encoder data stale",
            "wheel odometry stale",
            "EKF stale",
        ]

    def component_statuses(self) -> dict[str, LocalizationComponentStatus]:
        return {
            "imu": LocalizationComponentStatus(
                name="imu",
                ready=self.imu.ready,
                usable=self.imu.health.status in (STATUS_OK, STATUS_WARN),
                status=self.imu.health.status,
                message=self.imu.health.message,
                sample_count=self.imu.sample_count,
                last_age_s=self.imu.last_sample_age_s,
            ),
            "encoders": LocalizationComponentStatus(
                name="encoders",
                ready=self.encoders.ready,
                usable=self.encoders.health.status in (STATUS_OK, STATUS_WARN),
                status=self.encoders.health.status,
                message=self.encoders.health.message,
                sample_count=self.encoders.sample_count,
                last_age_s=self.encoders.last_sample_age_s,
            ),
            "wheel_odom": LocalizationComponentStatus(
                name="wheel_odom",
                ready=self.wheel_odom.ready,
                usable=self.wheel_odom.health.status in (STATUS_OK, STATUS_WARN),
                status=self.wheel_odom.health.status,
                message=self.wheel_odom.health.message,
                sample_count=self.wheel_odom.sample_count,
                last_age_s=self.wheel_odom.last_sample_age_s,
            ),
            "ekf": LocalizationComponentStatus(
                name="ekf",
                ready=self.ekf.ready,
                usable=self.ekf.usable,
                status=self.ekf.status,
                message=self.ekf.message,
                sample_count=self.ekf.output.message_count,
                last_age_s=self.ekf.output.last_age_s,
            ),
        }

    def summary_dict(self) -> dict[str, Any]:
        components = self.component_statuses()

        return {
            "package_name": self.package_name,
            "status": self.status,
            "message": self.message,
            "ready": self.ready,
            "usable": self.usable,
            "core_sensor_ready": self.core_sensor_ready,
            "odometry_ready": self.odometry_ready,
            "reasons": list(self.reasons),
            "evaluation_count": int(self.evaluation_count),
            "components": {
                name: component.to_dict()
                for name, component in components.items()
            },
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            **self.summary_dict(),
            "state_summary_topic": self.state_summary_topic,
            "imu": self.imu.to_dict(),
            "encoders": self.encoders.to_dict(),
            "wheel_odom": self.wheel_odom.to_dict(),
            "ekf": self.ekf.to_dict(),
        }


def make_localization_state(
    *,
    use_wheel_odom: bool = True,
    use_imu: bool = True,
    use_vo: bool = False,
    state_summary_topic: str = DEFAULT_STATE_SUMMARY_TOPIC,
) -> LocalizationState:
    state = LocalizationState(
        state_summary_topic=state_summary_topic,
        ekf=make_ekf_health(
            use_wheel_odom=use_wheel_odom,
            use_imu=use_imu,
            use_vo=use_vo,
        ),
    )
    state.evaluate()
    return state