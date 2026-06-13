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


def check_ekf_input(
    sensor: EkfInputHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    if stale_timeout_s <= 0.0:
        raise ValueError("stale_timeout_s must be > 0.0")

    if not sensor.enabled:
        return EkfHealthCheckResult(
            status=STATUS_OK,
            ok=True,
            usable=True,
            message=f"{sensor.name} input disabled",
            active_input_count=0,
            healthy_input_count=0,
        )

    reasons: list[str] = []

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
        active_input_count=1,
        healthy_input_count=1,
    )


def check_ekf_output(
    output: EkfOutputHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    if stale_timeout_s <= 0.0:
        raise ValueError("stale_timeout_s must be > 0.0")

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
        output_ok=True,
    )


def check_ekf_tf(
    tf: EkfTfHealth,
    *,
    stale_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S,
) -> EkfHealthCheckResult:
    if stale_timeout_s <= 0.0:
        raise ValueError("stale_timeout_s must be > 0.0")

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
        tf_ok=True,
    )


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
    input_results = [check_ekf_input(sensor) for sensor in active_inputs]
    healthy_input_count = sum(1 for result in input_results if result.ok)

    output_result = check_ekf_output(health.output)
    tf_result = check_ekf_tf(health.tf)

    reasons: list[str] = []

    if not active_inputs:
        reasons.append("no EKF inputs are enabled")

    if healthy_input_count < min_healthy_inputs:
        reasons.append(
            "not enough healthy EKF inputs: "
            f"{healthy_input_count}/{min_healthy_inputs}"
        )

    for sensor, result in zip(active_inputs, input_results):
        if not result.ok:
            reasons.extend(f"{sensor.name}: {reason}" for reason in result.reasons)

    if require_output and not output_result.ok:
        reasons.extend(f"output: {reason}" for reason in output_result.reasons)

    if require_tf and not tf_result.ok:
        reasons.extend(f"tf: {reason}" for reason in tf_result.reasons)

    hard_failed = (
        healthy_input_count < min_healthy_inputs
        or (require_output and not output_result.ok)
        or (require_tf and not tf_result.ok)
    )

    if not reasons:
        return EkfHealthCheckResult(
            status=STATUS_OK,
            ok=True,
            usable=True,
            message="EKF healthy",
            active_input_count=len(active_inputs),
            healthy_input_count=healthy_input_count,
            output_ok=output_result.ok,
            tf_ok=tf_result.ok,
        )

    if not hard_failed and healthy_input_count >= min_healthy_inputs:
        return EkfHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            usable=True,
            message="EKF usable with health notes",
            reasons=_unique_preserve_order(reasons),
            active_input_count=len(active_inputs),
            healthy_input_count=healthy_input_count,
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
        healthy_input_count=healthy_input_count,
        output_ok=output_result.ok,
        tf_ok=tf_result.ok,
    )


def check_ekf_rate_health(
    *,
    measured_hz: float,
    expected_hz: float = DEFAULT_EKF_RATE_HZ,
    tolerance_ratio: float = 0.50,
) -> EkfHealthCheckResult:
    if expected_hz <= 0.0:
        raise ValueError("expected_hz must be > 0.0")

    if tolerance_ratio < 0.0 or tolerance_ratio >= 1.0:
        raise ValueError("tolerance_ratio must be >= 0.0 and < 1.0")

    if measured_hz <= 0.0:
        return EkfHealthCheckResult(
            status=STATUS_ERROR,
            ok=False,
            usable=False,
            message="EKF rate is zero",
            reasons=["measured_hz <= 0.0"],
        )

    minimum_hz = expected_hz * (1.0 - tolerance_ratio)

    if measured_hz < minimum_hz:
        return EkfHealthCheckResult(
            status=STATUS_WARN,
            ok=True,
            usable=True,
            message="EKF rate below expected range",
            reasons=[
                f"measured_hz={measured_hz:.3f} < minimum_hz={minimum_hz:.3f}"
            ],
        )

    return EkfHealthCheckResult(
        status=STATUS_OK,
        ok=True,
        usable=True,
        message="EKF rate healthy",
    )


def summarize_ekf_inputs(inputs: Iterable[EkfInputHealth]) -> dict[str, object]:
    sensors = list(inputs)
    active = [sensor for sensor in sensors if sensor.enabled]
    healthy = [sensor for sensor in active if sensor.ok]

    return {
        "active_input_count": len(active),
        "healthy_input_count": len(healthy),
        "inputs": {
            sensor.name: sensor.to_dict()
            for sensor in sensors
        },
    }


def ekf_health_summary(health: EkfHealth) -> dict[str, object]:
    result = check_ekf_health(health)

    return {
        "status": result.status,
        "message": result.message,
        "ready": result.ok,
        "usable": result.usable,
        "reasons": list(result.reasons),
        "active_input_count": result.active_input_count,
        "healthy_input_count": result.healthy_input_count,
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


__all__ = [
    "EkfHealthCheckResult",
    "check_ekf_health",
    "check_ekf_input",
    "check_ekf_output",
    "check_ekf_tf",
    "check_ekf_rate_health",
    "summarize_ekf_inputs",
    "ekf_health_summary",
]
