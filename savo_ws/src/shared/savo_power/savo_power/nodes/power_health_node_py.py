"""Python fallback power health node for Robot Savo."""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable, Mapping

from savo_power import constants as c
from savo_power.models.power_health import PowerHealthLevel, normalize_health_level
from savo_power.models.power_status import PowerState, normalize_power_state
from savo_power.ros.adapters import (
    bool_message_class,
    make_bool_message,
    make_string_message,
    string_message_class,
)
from savo_power.ros.params import (
    PowerHealthNodeParams,
    read_power_health_node_params,
)
from savo_power.ros.qos_profiles import (
    latched_status_qos,
    power_health_qos,
    power_status_qos,
)
from savo_power.utils.logging import (
    get_node_logger,
    log_exception,
    log_info,
    log_startup,
    log_throttled,
)
from savo_power.utils.timing import (
    age_s,
    is_stale,
    monotonic_time_s,
    rate_to_period_s,
)


try:
    import rclpy
    from rclpy.node import Node

    RCLPY_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on ROS environment
    rclpy = None
    Node = object  # type: ignore[assignment]
    RCLPY_AVAILABLE = False


SAFE_STATES = {
    PowerState.OK,
    PowerState.CHARGING,
    PowerState.FULL,
}

WARNING_STATES = {
    PowerState.LOW,
    PowerState.UNKNOWN,
}

ERROR_STATES = {
    PowerState.CRITICAL,
    PowerState.ERROR,
    PowerState.STALE,
}


@dataclass(frozen=True)
class AggregatedPowerHealthResult:
    """Fallback-compatible health result for Python power health node."""

    level: PowerHealthLevel
    ok: bool
    state: PowerState
    shutdown_requested: bool = False
    reason: str = ""
    status_age_s: float = 0.0
    automatic_shutdown_enabled: bool = False

    @property
    def text(self) -> str:
        return self.format_line()

    @property
    def message(self) -> str:
        return self.format_line()

    def to_dict(self) -> dict[str, object]:
        return {
            "level": self.level.value,
            "ok": self.ok,
            "state": self.state.value,
            "shutdown_requested": self.shutdown_requested,
            "reason": self.reason,
            "status_age_s": self.status_age_s,
            "automatic_shutdown_enabled": self.automatic_shutdown_enabled,
        }

    def format_line(self) -> str:
        shutdown_text = "shutdown_requested=true" if self.shutdown_requested else "shutdown_requested=false"

        if self.reason:
            return (
                f"power_health={self.level.value} "
                f"state={self.state.value} "
                f"ok={str(self.ok).lower()} "
                f"{shutdown_text} "
                f"reason={self.reason}"
            )

        return (
            f"power_health={self.level.value} "
            f"state={self.state.value} "
            f"ok={str(self.ok).lower()} "
            f"{shutdown_text}"
        )


@dataclass
class PowerHealthMemory:
    """In-memory cache for latest power status."""

    last_status: dict[str, object] | None = None
    last_status_time_s: float | None = None
    received_count: int = 0
    parse_error_count: int = 0
    last_parse_error: str = ""

    @property
    def seen(self) -> bool:
        return self.last_status is not None

    def update(
        self,
        status: Mapping[str, object],
        *,
        now_s: float | None = None,
    ) -> None:
        self.last_status = {
            str(key): value
            for key, value in status.items()
        }
        self.last_status_time_s = monotonic_time_s() if now_s is None else float(now_s)
        self.received_count += 1
        self.last_parse_error = ""

    def update_from_payload(
        self,
        payload: object,
        *,
        now_s: float | None = None,
    ) -> None:
        self.update(
            parse_status_payload(payload),
            now_s=now_s,
        )

    def mark_parse_error(self, error: object) -> None:
        self.parse_error_count += 1
        self.last_parse_error = f"{type(error).__name__}: {error}"

    def age_s(self, *, now_s: float | None = None) -> float:
        return age_s(self.last_status_time_s, now_s=now_s)

    def stale(
        self,
        *,
        timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
        now_s: float | None = None,
    ) -> bool:
        return is_stale(
            self.last_status_time_s,
            timeout_s=timeout_s,
            now_s=now_s,
        )

    def clear(self) -> None:
        self.last_status = None
        self.last_status_time_s = None
        self.received_count = 0
        self.parse_error_count = 0
        self.last_parse_error = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "last_status": self.last_status,
            "last_status_time_s": self.last_status_time_s,
            "received_count": self.received_count,
            "parse_error_count": self.parse_error_count,
            "last_parse_error": self.last_parse_error,
            "seen": self.seen,
        }


@dataclass
class PowerHealthNodeState:
    """Runtime state for Python power health node."""

    publish_count: int = 0
    shutdown_publish_count: int = 0
    callback_count: int = 0
    error_count: int = 0
    last_health: AggregatedPowerHealthResult | None = None
    last_error: str = ""

    @property
    def ok(self) -> bool:
        return self.error_count == 0

    def to_dict(self) -> dict[str, object]:
        return {
            "publish_count": self.publish_count,
            "shutdown_publish_count": self.shutdown_publish_count,
            "callback_count": self.callback_count,
            "error_count": self.error_count,
            "last_error": self.last_error,
            "has_last_health": self.last_health is not None,
        }


def health_python_node_name() -> str:
    """Return Python fallback power health node name."""

    return f"{c.POWER_HEALTH_NODE_NAME}_py"


def health_topic() -> str:
    """Return health topic."""

    return c.HEALTH_TOPIC


def shutdown_request_topic() -> str:
    """Return shutdown request topic."""

    return c.SHUTDOWN_REQUEST_TOPIC


def create_timer_period_s(rate_hz: float) -> float:
    """Create timer period from publish rate."""

    return rate_to_period_s(rate_hz)


def parse_status_payload(payload: object) -> dict[str, object]:
    """Parse status payload from String message, raw text, or mapping."""

    if isinstance(payload, Mapping):
        return {
            str(key): value
            for key, value in payload.items()
        }

    data = getattr(payload, "data", payload)

    if isinstance(data, bytes):
        data = data.decode("utf-8")

    if not isinstance(data, str):
        raise TypeError(f"Unsupported status payload type: {type(payload).__name__}")

    text = data.strip()

    if not text:
        raise ValueError("Status payload is empty")

    loaded = json.loads(text)

    if not isinstance(loaded, Mapping):
        raise ValueError("Status payload JSON must be an object")

    return {
        str(key): value
        for key, value in loaded.items()
    }


def _enum_value(value: object) -> object:
    if isinstance(value, Enum):
        return value.value

    return value


def status_overall_state(status: Mapping[str, object]) -> PowerState:
    """Extract overall power state from status dictionary."""

    for key in ("overall_state", "state"):
        if key in status:
            return normalize_power_state(_enum_value(status[key]))

    return PowerState.UNKNOWN


def status_ok(status: Mapping[str, object]) -> bool:
    """Extract status OK flag with fallback from state."""

    ok_value = status.get("ok")

    if isinstance(ok_value, bool):
        return ok_value

    if isinstance(ok_value, str):
        normalized = ok_value.strip().lower()

        if normalized in {"true", "1", "yes", "ok"}:
            return True

        if normalized in {"false", "0", "no", "fail"}:
            return False

    return status_overall_state(status) in SAFE_STATES


def status_source_items(status: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    """Return source status objects from status dictionary."""

    for key in ("sources", "source_statuses", "statuses"):
        value = status.get(key)

        if isinstance(value, Iterable) and not isinstance(value, (str, bytes, Mapping)):
            items: list[Mapping[str, object]] = []

            for item in value:
                if isinstance(item, Mapping):
                    items.append(item)

            return tuple(items)

    return ()


def unhealthy_source_reasons(status: Mapping[str, object]) -> tuple[str, ...]:
    """Return compact reasons for unhealthy source statuses."""

    reasons: list[str] = []

    for item in status_source_items(status):
        expected = item.get("expected", True)
        seen = item.get("seen", True)
        ok = item.get("ok", None)
        source = str(_enum_value(item.get("source", c.UNKNOWN_SOURCE)))
        state = normalize_power_state(_enum_value(item.get("state", c.STATE_UNKNOWN)))

        if expected is False:
            continue

        if ok is True and state in SAFE_STATES:
            continue

        if seen is False:
            reasons.append(f"{source} missing")
            continue

        if state not in SAFE_STATES:
            reasons.append(f"{source} {state.value}")
            continue

        if ok is False:
            reasons.append(f"{source} not_ok")

    return tuple(reasons)


def health_level_from_state_and_ok(
    state: str | PowerState | None,
    *,
    ok: bool,
) -> PowerHealthLevel:
    """Evaluate health level from overall state and ok flag."""

    normalized_state = normalize_power_state(state)

    if not ok:
        if normalized_state in ERROR_STATES:
            return PowerHealthLevel.ERROR

        return PowerHealthLevel.WARN

    if normalized_state in ERROR_STATES:
        return PowerHealthLevel.ERROR

    if normalized_state in WARNING_STATES:
        return PowerHealthLevel.WARN

    if normalized_state in SAFE_STATES:
        return PowerHealthLevel.OK

    return PowerHealthLevel.UNKNOWN


def should_request_shutdown(
    *,
    level: str | PowerHealthLevel,
    state: str | PowerState | None,
    automatic_shutdown_enabled: bool,
) -> bool:
    """Return shutdown request decision.

    Automatic shutdown is intentionally disabled by default.
    Even when enabled, this fallback only requests shutdown for critical state.
    """

    if not automatic_shutdown_enabled:
        return False

    normalized_level = normalize_health_level(level)
    normalized_state = normalize_power_state(state)

    return (
        normalized_level == PowerHealthLevel.ERROR
        and normalized_state == PowerState.CRITICAL
    )


def evaluate_status_dict(
    status: Mapping[str, object] | None,
    *,
    status_age_s: float = 0.0,
    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
    automatic_shutdown_enabled: bool = c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
) -> AggregatedPowerHealthResult:
    """Evaluate power health from aggregate status dictionary."""

    if status is None:
        return AggregatedPowerHealthResult(
            level=PowerHealthLevel.UNKNOWN,
            ok=False,
            state=PowerState.UNKNOWN,
            shutdown_requested=False,
            reason="no power status received",
            status_age_s=max(0.0, float(status_age_s)),
            automatic_shutdown_enabled=bool(automatic_shutdown_enabled),
        )

    state = status_overall_state(status)
    ok = status_ok(status)
    age_value = max(0.0, float(status_age_s))

    if age_value > max(0.0, float(stale_timeout_s)):
        shutdown = should_request_shutdown(
            level=PowerHealthLevel.ERROR,
            state=PowerState.STALE,
            automatic_shutdown_enabled=automatic_shutdown_enabled,
        )
        return AggregatedPowerHealthResult(
            level=PowerHealthLevel.ERROR,
            ok=False,
            state=PowerState.STALE,
            shutdown_requested=shutdown,
            reason=f"power status stale age_s={age_value:.2f}",
            status_age_s=age_value,
            automatic_shutdown_enabled=bool(automatic_shutdown_enabled),
        )

    level = health_level_from_state_and_ok(state, ok=ok)
    reasons = unhealthy_source_reasons(status)

    if reasons:
        reason = "; ".join(reasons)
    elif ok:
        reason = "power status healthy"
    else:
        reason = f"power status state={state.value}"

    shutdown = should_request_shutdown(
        level=level,
        state=state,
        automatic_shutdown_enabled=automatic_shutdown_enabled,
    )

    return AggregatedPowerHealthResult(
        level=level,
        ok=(level == PowerHealthLevel.OK),
        state=state,
        shutdown_requested=shutdown,
        reason=reason,
        status_age_s=age_value,
        automatic_shutdown_enabled=bool(automatic_shutdown_enabled),
    )


def evaluate_memory(
    memory: PowerHealthMemory,
    *,
    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
    automatic_shutdown_enabled: bool = c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
    now_s: float | None = None,
) -> AggregatedPowerHealthResult:
    """Evaluate health from memory cache."""

    age_value = memory.age_s(now_s=now_s) if memory.seen else 0.0

    return evaluate_status_dict(
        memory.last_status,
        status_age_s=age_value,
        stale_timeout_s=stale_timeout_s,
        automatic_shutdown_enabled=automatic_shutdown_enabled,
    )


def health_to_publish_text(result: AggregatedPowerHealthResult) -> str:
    """Convert health result to JSON publish text."""

    return json.dumps(
        result.to_dict(),
        sort_keys=True,
    )


def build_startup_summary(params: PowerHealthNodeParams, topic: str) -> str:
    """Create stable startup summary."""

    automatic_shutdown_enabled = bool(
        getattr(
            params,
            "automatic_shutdown_enabled",
            c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
        )
    )

    return (
        f"topic={topic} "
        f"shutdown_topic={c.SHUTDOWN_REQUEST_TOPIC} "
        f"stale_timeout_s={params.stale_timeout_s:.2f} "
        f"publish_rate_hz={params.publish_rate_hz:.2f} "
        f"automatic_shutdown_enabled={str(automatic_shutdown_enabled).lower()}"
    )


if RCLPY_AVAILABLE:

    class PowerHealthNodePy(Node):  # type: ignore[misc]
        """ROS 2 Python fallback power health node."""

        def __init__(self) -> None:
            node_name = health_python_node_name()

            super().__init__(node_name)

            self._logger = get_node_logger(self)
            self._params = read_power_health_node_params(self)
            self._memory = PowerHealthMemory()
            self._state = PowerHealthNodeState()

            self._health_publisher = self.create_publisher(
                string_message_class(),
                c.HEALTH_TOPIC,
                power_health_qos(),
            )

            self._shutdown_publisher = self.create_publisher(
                bool_message_class(),
                c.SHUTDOWN_REQUEST_TOPIC,
                latched_status_qos(),
            )

            self._subscription = self.create_subscription(
                string_message_class(),
                c.STATUS_TOPIC,
                self._on_status_msg,
                power_status_qos(),
            )

            self._timer = self.create_timer(
                create_timer_period_s(self._params.publish_rate_hz),
                self._on_timer,
            )

            log_startup(
                self._logger,
                node_name,
                backend="python_fallback",
            )
            log_info(
                self._logger,
                build_startup_summary(self._params, c.HEALTH_TOPIC),
            )

        @property
        def memory(self) -> PowerHealthMemory:
            return self._memory

        @property
        def state(self) -> PowerHealthNodeState:
            return self._state

        def _automatic_shutdown_enabled(self) -> bool:
            return bool(
                getattr(
                    self._params,
                    "automatic_shutdown_enabled",
                    c.AUTOMATIC_SHUTDOWN_ENABLED_DEFAULT,
                )
            )

        def _on_status_msg(self, msg: object) -> None:
            try:
                self._memory.update_from_payload(msg)
                self._state.callback_count += 1
            except Exception as exc:  # noqa: BLE001 - malformed message must not kill node
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"
                self._memory.mark_parse_error(exc)

                log_exception(
                    self._logger,
                    "Power status parse failed",
                    exc,
                )

        def build_health(self) -> AggregatedPowerHealthResult:
            """Build current power health."""

            return evaluate_memory(
                self._memory,
                stale_timeout_s=self._params.stale_timeout_s,
                automatic_shutdown_enabled=self._automatic_shutdown_enabled(),
            )

        def publish_health(self, result: AggregatedPowerHealthResult) -> None:
            """Publish health and shutdown request."""

            self._health_publisher.publish(
                make_string_message(
                    health_to_publish_text(result)
                )
            )
            self._state.publish_count += 1
            self._state.last_health = result

            self._shutdown_publisher.publish(
                make_bool_message(result.shutdown_requested)
            )
            self._state.shutdown_publish_count += 1

            log_throttled(
                self._logger,
                "power_health",
                "info",
                result.format_line(),
                period_s=5.0,
            )

        def _on_timer(self) -> None:
            result = self.build_health()
            self.publish_health(result)

else:

    class PowerHealthNodePy:  # pragma: no cover - only used when ROS is missing
        """Placeholder when rclpy is unavailable."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError(
                "rclpy is not available. Source ROS 2 first, or use the "
                "helper functions in this module for offline checks."
            )


def spin_power_health_node() -> None:
    """Spin power health Python fallback node."""

    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")

    rclpy.init()
    node = PowerHealthNodePy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        logger = get_node_logger(node)
        log_info(logger, f"{health_python_node_name()} stopped")
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Entry point for power health Python fallback node."""

    spin_power_health_node()


__all__ = [
    "ERROR_STATES",
    "RCLPY_AVAILABLE",
    "SAFE_STATES",
    "WARNING_STATES",
    "AggregatedPowerHealthResult",
    "PowerHealthMemory",
    "PowerHealthNodePy",
    "PowerHealthNodeState",
    "build_startup_summary",
    "create_timer_period_s",
    "evaluate_memory",
    "evaluate_status_dict",
    "health_level_from_state_and_ok",
    "health_python_node_name",
    "health_to_publish_text",
    "health_topic",
    "main",
    "parse_status_payload",
    "should_request_shutdown",
    "shutdown_request_topic",
    "spin_power_health_node",
    "status_ok",
    "status_overall_state",
    "status_source_items",
    "unhealthy_source_reasons",
]
