"""Power policy helpers for Robot Savo Python diagnostics and fallbacks."""

from __future__ import annotations

from savo_power.policy.power_policy import (
    PowerPolicyThresholds,
    apply_power_policy,
    evaluate_base_battery_state,
    evaluate_power_state,
    evaluate_ups_state,
    has_driver_error,
    is_valid_percentage,
    reading_allows_normal_operation,
    should_request_shutdown,
)


__all__ = [
    "PowerPolicyThresholds",
    "apply_power_policy",
    "evaluate_base_battery_state",
    "evaluate_power_state",
    "evaluate_ups_state",
    "has_driver_error",
    "is_valid_percentage",
    "reading_allows_normal_operation",
    "should_request_shutdown",
]
