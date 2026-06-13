#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EKF health checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_EKF_RATE_HZ,
    DEFAULT_EKF_SENSOR_TIMEOUT_S,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)
from savo_localization.models.ekf_health import (
    EkfHealth,
    EkfInputHealth,
    EkfOutputHealth,
    EkfTfHealth,
)


@dataclass(frozen=True)
class EkfHealthCheckResult:
    status: str
    ok: bool
    usable: bool
    message: str
    reasons: list[str] = field(default_factory=list)

    active_input_count: int = 0
    healthy_input_count: int = 0
    output_ok: bool = False
    tf_ok: bool = False

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def check_ekf_health(
    health: EkfHealth,
    *,
    require_tf: bool = True,
    require_output: bool = True,
    min_healthy_inputs: int = 1,
) -> EkfHealthCheckResult:
    if min_healthy_inputs <= 0:
        raise ValueError(f"min_healthy_inputs must be > 0, got {min_healthy_inputs}")

    active_inputs = list(health.active_inputs)
    healthy_inputs = [sensor for sensor in active_inputs if sensor.ok]

    reasons: list[str] = []

    if not active_inputs:
        reasons.append("no EKF inputs are enabled")

    if len(healthy_inputs) < min_healthy_inputs:
        reasons.append(
            "not enough healthy EKF inputs: "
            f"{len(healthy_inputs)}/{min_healthy_inputs}"
        )

    for sensor in active_inputs:
        input_result = check_ekf_input(sensor)
        if not input_result.ok:
            reasons.extend(f"{sensor.name}: {reason}" for reason in input_result.reasons)

    output_result = check_ekf_output(health.output)
    if require_output and not output_result.ok:
        reasons.extend(f"output: {reason}" for reason in output_result.reasons)

    tf_result = check_ekf_tf(health.tf)
    if require_tf and not tf_result.ok:
        reasons.extend(f"tf: {reason}" for reason in tf_result.reasons)

    if not reasons:
        return EkfHealthCheckResult(
            status=STATUS_OK,
            ok=True,
            usable=True,
            message="EKF healthy",
            reasons=[],
            active_input_count=len(active_inputs),
            healthy_input_count=len(healthy_inputs),
            output_ok=output_result.ok,
            tf_ok=tf_result.ok,
        )

    if healthy_inputs and (not require_output or output_result.ok):
        return EkfHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            usable=True,
            message="EKF usable with health notes",
            reasons=_unique_preserve_order(reasons),
            active_input_count=len(active_inputs),
            healthy_input_count=len(healthy_inputs),
            output_ok=output_result.ok,
            tf_ok=tf_result.ok,
        )

    return EkfHealthCheckResult(
        status=STATUS_ERROR,
        ok=False,
        usable=False,
        message="EKF health check failed",
        reasons=_unique_preserve_order(reasons),
        active_input_count=len(active_inputs),
        healthy_input_count=len(healthy_inputs),
        output_ok=output_result.ok,
        tf_ok=tf_result.ok,
    )


def check_ekf_input(
    sensor: EkfInputHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    reasons: list[str] = []

    if not sensor.enabled:
        return EkfHealthCheckResult(
            status=STATUS_OK,
            ok=True,
            usable=True,
            message=f"{sensor.name} input disabled",
            reasons=[],
            active_input_count=0,
            healthy_input_count=0,
        )

    if not sensor.seen:
        reasons.append(f"no messages received on {sensor.topic}")

    if sensor.last_age_s is None:
        reasons.append("input age is not available")
    elif sensor.last_age_s > stale_timeout_s:
        reasons.append(
            f"input stale: age_s={sensor.last_age_s:.3f} "
            f"> timeout_s={stale_timeout_s:.3f}"
        )

    if not sensor.fresh:
        reasons.append("input marked stale")

    if sensor.message_count <= 0:
        reasons.append("input message_count is zero")

    if reasons:
        return EkfHealthCheckResult(
            status=STATUS_STALE if sensor.seen else STATUS_ERROR,
            ok=False,
            usable=False,
            message=f"{sensor.name} input not healthy",
            reasons=_unique_preserve_order(reasons),
            active_input_count=1,
            healthy_input_count=0,
        )

    return EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message=f"{sensor.name} input healthy",
        reasons=[],
        active_input_count=1,
        healthy_input_count=1,
    )


def check_ekf_output(
    output: EkfOutputHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    reasons: list[str] = []

    if not output.seen:
        reasons.append(f"no messages received on {output.topic}")

    if output.last_age_s is None:
        reasons.append("output age is not available")
    elif output.last_age_s > stale_timeout_s:
        reasons.append(
            f"output stale: age_s={output.last_age_s:.3f} "
            f"> timeout_s={stale_timeout_s:.3f}"
        )

    if not output.fresh:
        reasons.append("output marked stale")

    if output.message_count <= 0:
        reasons.append("output message_count is zero")

    if not output.frame_ok:
        reasons.append("output frame ids are not valid")

    if not output.finite_ok:
        reasons.append("output pose or twist contains non-finite values")

    if not output.covariance_ok:
        reasons.append("output covariance is not valid")

    if reasons:
        return EkfHealthCheckResult(
            status=STATUS_STALE if output.seen else STATUS_ERROR,
            ok=False,
            usable=False,
            message="EKF output not healthy",
            reasons=_unique_preserve_order(reasons),
            output_ok=False,
        )

    return EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message="EKF output healthy",
        reasons=[],
        output_ok=True,
    )


def check_ekf_tf(
    tf: EkfTfHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    reasons: list[str] = []

    if not tf.available:
        reasons.append(f"transform not available: {tf.parent_frame} -> {tf.child_frame}")

    if tf.last_age_s is None:
        reasons.append("TF age is not available")
    elif tf.last_age_s > stale_timeout_s:
        reasons.append(
            f"TF stale: age_s={tf.last_age_s:.3f} "
            f"> timeout_s={stale_timeout_s:.3f}"
        )

    if not tf.fresh:
        reasons.append("TF marked stale")

    if reasons:
        return EkfHealthCheckResult(
            status=STATUS_STALE,
            ok=False,
            usable=False,
            message="EKF TF not healthy",
            reasons=_unique_preserve_order(reasons),
            tf_ok=False,
        )

    return EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message="EKF TF healthy",
        reasons=[],
        tf_ok=True,
    )


def check_ekf_rate_health(
    *,
    measured_hz: float,
    expected_hz: float = DEFAULT_EKF_RATE_HZ,
    tolerance_ratio: float = 0.50,
) -> EkfHealthCheckResult:
    if expected_hz <= 0.0:
        raise ValueError(f"expected_hz must be > 0.0, got {expected_hz}")

    if not 0.0 <= tolerance_ratio < 1.0:
        raise ValueError(
            f"tolerance_ratio must be in range [0.0, 1.0), got {tolerance_ratio}"
        )

    measured = max(0.0, float(measured_hz))
    expected = float(expected_hz)
    min_allowed = expected * (1.0 - float(tolerance_ratio))

    if measured <= 0.0:
        return EkfHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            usable=False,
            message="EKF output rate not measured",
            reasons=["measured_hz is zero"],
        )

    if measured < min_allowed:
        return EkfHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            usable=True,
            message="EKF output rate lower than expected",
            reasons=[
                f"measured_hz={measured:.3f} < min_allowed_hz={min_allowed:.3f}"
            ],
        )

    return EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message="EKF output rate healthy",
        reasons=[],
    )


def summarize_ekf_inputs(inputs: Iterable[EkfInputHealth]) -> dict[str, object]:
    input_list = list(inputs)

    return {
        "active_input_count": sum(1 for sensor in input_list if sensor.enabled),
        "healthy_input_count": sum(1 for sensor in input_list if sensor.ok),
        "inputs": {
            sensor.name: {
                "topic": sensor.topic,
                "enabled": sensor.enabled,
                "seen": sensor.seen,
                "fresh": sensor.fresh,
                "ok": sensor.ok,
                "last_age_s": sensor.last_age_s,
                "message_count": sensor.message_count,
                "rate_hz": sensor.rate_hz,
                "status": sensor.status,
                "message": sensor.message,
                "reasons": list(sensor.reasons),
            }
            for sensor in input_list
        },
    }


def ekf_health_summary(health: EkfHealth) -> dict[str, object]:
    health.evaluate()

    return {
        "status": health.status,
        "message": health.message,
        "ready": health.ready,
        "usable": health.usable,
        "reasons": list(health.reasons),
        "inputs": summarize_ekf_inputs(health.inputs),
        "output": health.output.to_dict(),
        "tf": health.tf.to_dict(),
    }


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result