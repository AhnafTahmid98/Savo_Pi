"""Python fallback power aggregator node for Robot Savo."""

from __future__ import annotations

import json
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Any, Iterable, Mapping

from savo_power import constants as c
from savo_power.models.power_status import (
    BatterySource,
    PowerSourceStatus,
    PowerState,
    PowerStatusSummary,
    aggregate_overall_state,
    missing_source_status,
    normalize_battery_source,
    normalize_power_state,
    not_expected_source_status,
)
from savo_power.ros.adapters import (
    make_status_summary_json_message,
    status_summary_to_text,
    string_message_class,
)
from savo_power.ros.params import (
    PowerAggregatorNodeParams,
    read_power_aggregator_node_params,
)
from savo_power.ros.qos_profiles import (
    power_sensor_qos,
    power_status_qos,
)
from savo_power.utils.formatting import battery_source_label
from savo_power.utils.logging import (
    get_node_logger,
    log_exception,
    log_info,
    log_startup,
    log_throttled,
)
from savo_power.utils.timing import (
    age_s,
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


POWER_SOURCE_ORDER = (
    BatterySource.CORE_UPS,
    BatterySource.EDGE_UPS,
    BatterySource.BASE_BATTERY,
)

SAFE_POWER_STATES = {
    PowerState.OK,
    PowerState.CHARGING,
    PowerState.FULL,
}


@dataclass
class PowerAggregatorMemory:
    """In-memory reading cache for power aggregation."""

    readings_by_source: dict[BatterySource, dict[str, object]] = field(
        default_factory=dict
    )
    updated_time_by_source: dict[BatterySource, float] = field(
        default_factory=dict
    )
    received_count_by_source: dict[BatterySource, int] = field(
        default_factory=dict
    )
    parse_error_count: int = 0
    last_parse_error: str = ""

    def update(
        self,
        source: str | BatterySource,
        reading: Mapping[str, object],
        *,
        now_s: float | None = None,
    ) -> BatterySource:
        """Update cached reading for one source."""

        normalized = normalize_battery_source(source)
        timestamp = monotonic_time_s() if now_s is None else float(now_s)

        self.readings_by_source[normalized] = {
            str(key): value
            for key, value in reading.items()
        }
        self.updated_time_by_source[normalized] = timestamp
        self.received_count_by_source[normalized] = (
            self.received_count_by_source.get(normalized, 0) + 1
        )

        return normalized

    def update_from_payload(
        self,
        payload: object,
        *,
        now_s: float | None = None,
    ) -> BatterySource:
        """Parse and update from message payload."""

        reading = parse_reading_payload(payload)
        source = source_from_reading_dict(reading)
        return self.update(source, reading, now_s=now_s)

    def mark_parse_error(self, error: object) -> None:
        """Record one parse error."""

        self.parse_error_count += 1
        self.last_parse_error = f"{type(error).__name__}: {error}"

    def reading_for_source(
        self,
        source: str | BatterySource,
    ) -> dict[str, object] | None:
        """Return cached reading for source."""

        normalized = normalize_battery_source(source)
        return self.readings_by_source.get(normalized)

    def source_seen(self, source: str | BatterySource) -> bool:
        """Return True if source has been received."""

        normalized = normalize_battery_source(source)
        return normalized in self.readings_by_source

    def source_age_s(
        self,
        source: str | BatterySource,
        *,
        now_s: float | None = None,
    ) -> float:
        """Return age of cached reading."""

        normalized = normalize_battery_source(source)
        return age_s(
            self.updated_time_by_source.get(normalized),
            now_s=now_s,
        )

    def clear(self) -> None:
        """Clear cached readings."""

        self.readings_by_source.clear()
        self.updated_time_by_source.clear()
        self.received_count_by_source.clear()
        self.parse_error_count = 0
        self.last_parse_error = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "readings_by_source": {
                source.value: dict(reading)
                for source, reading in self.readings_by_source.items()
            },
            "updated_time_by_source": {
                source.value: timestamp
                for source, timestamp in self.updated_time_by_source.items()
            },
            "received_count_by_source": {
                source.value: count
                for source, count in self.received_count_by_source.items()
            },
            "parse_error_count": self.parse_error_count,
            "last_parse_error": self.last_parse_error,
        }


@dataclass(frozen=True)
class AggregatedPowerSourceStatus:
    """Fallback-compatible source status used by the Python aggregator.

    The shared model may evolve, so this node keeps the fields it needs stable:
    source, state, ok, expected, seen, age_s, and detail.
    """

    source: BatterySource
    state: PowerState
    ok: bool
    expected: bool
    seen: bool
    age_s: float = 0.0
    detail: str = ""

    @property
    def text(self) -> str:
        return self.detail

    @property
    def message(self) -> str:
        return self.detail

    def to_dict(self) -> dict[str, object]:
        return {
            "source": self.source.value,
            "state": self.state.value,
            "ok": self.ok,
            "expected": self.expected,
            "seen": self.seen,
            "age_s": self.age_s,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class AggregatedPowerStatusSummary:
    """Fallback-compatible power summary used by the Python aggregator."""

    overall_state: PowerState
    state: PowerState
    ok: bool
    sources: tuple[object, ...]
    source_statuses: tuple[object, ...]
    statuses: tuple[object, ...]
    stale_timeout_s: float = 0.0
    detail: str = ""

    @property
    def text(self) -> str:
        return self.detail

    @property
    def message(self) -> str:
        return self.detail

    def to_dict(self) -> dict[str, object]:
        return {
            "overall_state": self.overall_state.value,
            "state": self.state.value,
            "ok": self.ok,
            "stale_timeout_s": self.stale_timeout_s,
            "detail": self.detail,
            "sources": [
                status.to_dict() if hasattr(status, "to_dict") else str(status)
                for status in self.sources
            ],
        }


@dataclass
class PowerAggregatorNodeState:
    """Runtime state for Python aggregator node."""

    publish_count: int = 0
    callback_count: int = 0
    error_count: int = 0
    last_status: PowerStatusSummary | None = None
    last_error: str = ""

    @property
    def ok(self) -> bool:
        return self.error_count == 0

    def to_dict(self) -> dict[str, object]:
        return {
            "publish_count": self.publish_count,
            "callback_count": self.callback_count,
            "error_count": self.error_count,
            "last_error": self.last_error,
            "has_last_status": self.last_status is not None,
        }


def aggregator_python_node_name() -> str:
    """Return Python fallback aggregator node name."""

    return f"{c.POWER_AGGREGATOR_NODE_NAME}_py"


def status_topic() -> str:
    """Return aggregated status topic."""

    return c.STATUS_TOPIC


def create_timer_period_s(rate_hz: float) -> float:
    """Create timer period from publish rate."""

    return rate_to_period_s(rate_hz)


def parse_reading_payload(payload: object) -> dict[str, object]:
    """Parse reading payload from String message, raw text, or dictionary."""

    if isinstance(payload, Mapping):
        return {
            str(key): value
            for key, value in payload.items()
        }

    data = getattr(payload, "data", payload)

    if isinstance(data, bytes):
        data = data.decode("utf-8")

    if not isinstance(data, str):
        raise TypeError(f"Unsupported reading payload type: {type(payload).__name__}")

    text = data.strip()

    if not text:
        raise ValueError("Reading payload is empty")

    loaded = json.loads(text)

    if not isinstance(loaded, Mapping):
        raise ValueError("Reading payload JSON must be an object")

    return {
        str(key): value
        for key, value in loaded.items()
    }


def source_from_reading_dict(reading: Mapping[str, object]) -> BatterySource:
    """Extract battery source from reading dictionary."""

    source = reading.get("source", c.UNKNOWN_SOURCE)
    return normalize_battery_source(source)


def state_from_reading_dict(reading: Mapping[str, object]) -> PowerState:
    """Extract power state from reading dictionary."""

    return normalize_power_state(reading.get("state", c.STATE_UNKNOWN))


def expected_sources_from_params(
    params: PowerAggregatorNodeParams,
) -> tuple[BatterySource, ...]:
    """Return expected power sources from node params."""

    expected: list[BatterySource] = []

    if getattr(params, "core_ups_expected", True):
        expected.append(BatterySource.CORE_UPS)

    if getattr(params, "edge_ups_expected", True):
        expected.append(BatterySource.EDGE_UPS)

    if getattr(params, "base_battery_expected", True):
        expected.append(BatterySource.BASE_BATTERY)

    return tuple(expected)


def state_is_ok(state: str | PowerState | None) -> bool:
    """Return True if state allows normal operation."""

    return normalize_power_state(state) in SAFE_POWER_STATES


def make_power_source_status(
    source: str | BatterySource,
    *,
    state: str | PowerState | None,
    expected: bool,
    seen: bool,
    age_value_s: float = 0.0,
    detail: str = "",
) -> AggregatedPowerSourceStatus:
    """Create fallback-compatible source status."""

    normalized_source = normalize_battery_source(source)
    normalized_state = normalize_power_state(state)
    ok = (not bool(expected)) or (bool(seen) and state_is_ok(normalized_state))

    return AggregatedPowerSourceStatus(
        source=normalized_source,
        state=normalized_state,
        ok=ok,
        expected=bool(expected),
        seen=bool(seen),
        age_s=max(0.0, float(age_value_s)),
        detail=str(detail),
    )


def make_missing_power_source_status(
    source: str | BatterySource,
    *,
    age_value_s: float = 0.0,
) -> AggregatedPowerSourceStatus:
    """Create missing-source status."""

    normalized = normalize_battery_source(source)

    return make_power_source_status(
        normalized,
        state=PowerState.STALE,
        expected=True,
        seen=False,
        age_value_s=age_value_s,
        detail=f"{battery_source_label(normalized)} missing",
    )


def make_not_expected_power_source_status(
    source: str | BatterySource,
) -> AggregatedPowerSourceStatus:
    """Create not-expected source status."""

    normalized = normalize_battery_source(source)

    return make_power_source_status(
        normalized,
        state=PowerState.OK,
        expected=False,
        seen=False,
        detail=f"{battery_source_label(normalized)} not expected",
    )


def source_status_ok(status: object) -> bool:
    """Return source status ok flag with compatibility fallback."""

    ok = getattr(status, "ok", None)

    if isinstance(ok, bool):
        return ok

    state = normalize_power_state(getattr(status, "state", PowerState.UNKNOWN))
    expected = getattr(status, "expected", True)
    seen = getattr(status, "seen", True)

    if expected is False:
        return True

    if seen is False:
        return False

    return state_is_ok(state)


def summary_status_ok(summary: object) -> bool:
    """Return summary ok flag with compatibility fallback."""

    ok = getattr(summary, "ok", None)

    if isinstance(ok, bool):
        return ok

    statuses = summary_source_statuses(summary)

    if not statuses:
        return False

    return all(source_status_ok(status) for status in statuses)


def reading_dict_to_source_status(
    source: str | BatterySource,
    reading: Mapping[str, object],
    *,
    expected: bool,
    age_value_s: float,
    stale_timeout_s: float,
) -> PowerSourceStatus:
    """Convert one cached reading dictionary into source status."""

    normalized = normalize_battery_source(source)

    if not expected:
        return make_not_expected_power_source_status(normalized)

    if age_value_s > max(0.0, float(stale_timeout_s)):
        return make_power_source_status(
            normalized,
            state=PowerState.STALE,
            expected=True,
            seen=True,
            age_value_s=age_value_s,
            detail=(
                f"{battery_source_label(normalized)} stale "
                f"age_s={age_value_s:.2f}"
            ),
        )

    state = state_from_reading_dict(reading)

    return make_power_source_status(
        normalized,
        state=state,
        expected=True,
        seen=True,
        age_value_s=age_value_s,
        detail=str(reading.get("detail", "")),
    )


def build_source_statuses(
    memory: PowerAggregatorMemory,
    *,
    expected_sources: Iterable[BatterySource],
    stale_timeout_s: float,
    now_s: float | None = None,
) -> tuple[PowerSourceStatus, ...]:
    """Build source statuses in stable source order."""

    now = monotonic_time_s() if now_s is None else float(now_s)
    expected_set = set(expected_sources)
    statuses: list[PowerSourceStatus] = []

    for source in POWER_SOURCE_ORDER:
        expected = source in expected_set

        if not expected:
            statuses.append(make_not_expected_power_source_status(source))
            continue

        reading = memory.reading_for_source(source)

        if reading is None:
            statuses.append(make_missing_power_source_status(source))
            continue

        age_value = memory.source_age_s(source, now_s=now)

        statuses.append(
            reading_dict_to_source_status(
                source,
                reading,
                expected=True,
                age_value_s=age_value,
                stale_timeout_s=stale_timeout_s,
            )
        )

    return tuple(statuses)


def compute_overall_state(
    statuses: Iterable[PowerSourceStatus],
) -> PowerState:
    """Compute overall power state.

    Not-expected sources must not degrade the overall state.
    Example: if only core_ups is expected, edge_ups/base_battery are still listed
    for visibility, but they are ignored for overall severity.
    """

    status_tuple = tuple(statuses)

    expected_statuses = tuple(
        status
        for status in status_tuple
        if getattr(status, "expected", True) is not False
    )

    if not expected_statuses:
        return PowerState.OK

    try:
        return normalize_power_state(aggregate_overall_state(expected_statuses))
    except Exception:
        pass

    try:
        return normalize_power_state(
            aggregate_overall_state(
                tuple(status.state for status in expected_statuses)
            )
        )
    except Exception:
        pass

    severity_order = (
        PowerState.CRITICAL,
        PowerState.ERROR,
        PowerState.STALE,
        PowerState.LOW,
        PowerState.UNKNOWN,
        PowerState.CHARGING,
        PowerState.FULL,
        PowerState.OK,
    )

    states = {
        normalize_power_state(getattr(status, "state", PowerState.UNKNOWN))
        for status in expected_statuses
    }

    for state in severity_order:
        if state in states:
            return state

    return PowerState.UNKNOWN


def summary_source_statuses(
    summary: object,
) -> tuple[object, ...]:
    """Return source statuses from a summary, tolerating field-name changes."""

    for name in ("sources", "source_statuses", "statuses"):
        value = getattr(summary, name, None)

        if value is not None:
            return tuple(value)

    if isinstance(summary, Mapping):
        for name in ("sources", "source_statuses", "statuses"):
            value = summary.get(name)

            if value is not None:
                return tuple(value)

    return ()


def build_power_status_summary(
    statuses: Iterable[PowerSourceStatus],
    *,
    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
) -> AggregatedPowerStatusSummary:
    """Create fallback-compatible aggregate power summary."""

    status_tuple = tuple(statuses)
    overall_state = compute_overall_state(status_tuple)
    ok = all(source_status_ok(status) for status in status_tuple)

    return AggregatedPowerStatusSummary(
        overall_state=overall_state,
        state=overall_state,
        ok=ok,
        sources=status_tuple,
        source_statuses=status_tuple,
        statuses=status_tuple,
        stale_timeout_s=max(0.0, float(stale_timeout_s)),
        detail="",
    )


def aggregate_memory(
    memory: PowerAggregatorMemory,
    *,
    expected_sources: Iterable[BatterySource],
    stale_timeout_s: float = c.DEFAULT_STALE_TIMEOUT_S,
    now_s: float | None = None,
) -> PowerStatusSummary:
    """Aggregate current memory into PowerStatusSummary."""

    statuses = build_source_statuses(
        memory,
        expected_sources=expected_sources,
        stale_timeout_s=stale_timeout_s,
        now_s=now_s,
    )

    return build_power_status_summary(
        statuses,
        stale_timeout_s=stale_timeout_s,
    )


def build_startup_summary(params: PowerAggregatorNodeParams, topic: str) -> str:
    """Create stable startup summary."""

    expected = expected_sources_from_params(params)
    expected_text = ",".join(source.value for source in expected) or "none"

    return (
        f"topic={topic} "
        f"expected_sources={expected_text} "
        f"stale_timeout_s={params.stale_timeout_s:.2f} "
        f"publish_rate_hz={params.publish_rate_hz:.2f}"
    )


if RCLPY_AVAILABLE:

    class PowerAggregatorNodePy(Node):  # type: ignore[misc]
        """ROS 2 Python fallback power aggregator node."""

        def __init__(self) -> None:
            node_name = aggregator_python_node_name()

            super().__init__(node_name)

            self._logger = get_node_logger(self)
            self._params = read_power_aggregator_node_params(self)
            self._state = PowerAggregatorNodeState()
            self._memory = PowerAggregatorMemory()

            self._expected_sources = expected_sources_from_params(self._params)

            self._publisher = self.create_publisher(
                string_message_class(),
                c.STATUS_TOPIC,
                power_status_qos(),
            )

            self._subscriptions = [
                self.create_subscription(
                    string_message_class(),
                    c.CORE_UPS_TOPIC,
                    lambda msg: self._on_reading_msg(BatterySource.CORE_UPS, msg),
                    power_sensor_qos(),
                ),
                self.create_subscription(
                    string_message_class(),
                    c.EDGE_UPS_TOPIC,
                    lambda msg: self._on_reading_msg(BatterySource.EDGE_UPS, msg),
                    power_sensor_qos(),
                ),
                self.create_subscription(
                    string_message_class(),
                    c.BASE_BATTERY_TOPIC,
                    lambda msg: self._on_reading_msg(BatterySource.BASE_BATTERY, msg),
                    power_sensor_qos(),
                ),
            ]

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
                build_startup_summary(self._params, c.STATUS_TOPIC),
            )

        @property
        def state(self) -> PowerAggregatorNodeState:
            return self._state

        @property
        def memory(self) -> PowerAggregatorMemory:
            return self._memory

        def _on_reading_msg(
            self,
            source_hint: BatterySource,
            msg: object,
        ) -> None:
            try:
                reading = parse_reading_payload(msg)

                if "source" not in reading:
                    reading["source"] = source_hint.value

                self._memory.update(source_hint, reading)
                self._state.callback_count += 1
            except Exception as exc:  # noqa: BLE001 - malformed messages must not kill node
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"
                self._memory.mark_parse_error(exc)

                log_exception(
                    self._logger,
                    "Power reading parse failed",
                    exc,
                )

        def build_status(self) -> PowerStatusSummary:
            """Build current aggregate status."""

            return aggregate_memory(
                self._memory,
                expected_sources=self._expected_sources,
                stale_timeout_s=self._params.stale_timeout_s,
            )

        def publish_status(self, summary: PowerStatusSummary) -> None:
            """Publish aggregate status."""

            message = make_status_summary_json_message(summary)
            self._publisher.publish(message)

            self._state.publish_count += 1
            self._state.last_status = summary

            log_throttled(
                self._logger,
                "power_status_summary",
                "info",
                status_summary_to_text(summary),
                period_s=5.0,
            )

        def _on_timer(self) -> None:
            summary = self.build_status()
            self.publish_status(summary)

else:

    class PowerAggregatorNodePy:  # pragma: no cover - only used when ROS is missing
        """Placeholder when rclpy is unavailable."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError(
                "rclpy is not available. Source ROS 2 first, or use the "
                "helper functions in this module for offline checks."
            )


def spin_power_aggregator_node() -> None:
    """Spin power aggregator Python fallback node."""

    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")

    rclpy.init()
    node = PowerAggregatorNodePy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        logger = get_node_logger(node)
        log_info(logger, f"{aggregator_python_node_name()} stopped")
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Entry point for power aggregator Python fallback node."""

    spin_power_aggregator_node()


__all__ = [
    "POWER_SOURCE_ORDER",
    "RCLPY_AVAILABLE",
    "SAFE_POWER_STATES",
    "AggregatedPowerSourceStatus",
    "AggregatedPowerStatusSummary",
    "PowerAggregatorMemory",
    "PowerAggregatorNodePy",
    "PowerAggregatorNodeState",
    "aggregate_memory",
    "aggregator_python_node_name",
    "build_power_status_summary",
    "build_source_statuses",
    "build_startup_summary",
    "compute_overall_state",
    "create_timer_period_s",
    "expected_sources_from_params",
    "main",
    "make_missing_power_source_status",
    "make_not_expected_power_source_status",
    "make_power_source_status",
    "parse_reading_payload",
    "reading_dict_to_source_status",
    "source_from_reading_dict",
    "spin_power_aggregator_node",
    "state_from_reading_dict",
    "status_topic",
    "source_status_ok",
    "summary_status_ok",
    "state_is_ok",
    "summary_source_statuses",
]
