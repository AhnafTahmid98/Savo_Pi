"""Python fallback power dashboard node for Robot Savo."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

from savo_power import constants as c
from savo_power.models.power_health import PowerHealthLevel, normalize_health_level
from savo_power.models.power_status import (
    BatterySource,
    PowerState,
    normalize_battery_source,
    normalize_power_state,
)
from savo_power.ros.adapters import (
    make_string_message,
    string_message_class,
)
from savo_power.ros.params import (
    PowerDashboardNodeParams,
    read_power_dashboard_node_params,
)
from savo_power.ros.qos_profiles import (
    power_dashboard_qos,
    power_health_qos,
    power_sensor_qos,
    power_status_qos,
)
from savo_power.utils.formatting import (
    battery_source_label,
    format_percentage,
    format_voltage,
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


@dataclass(frozen=True)
class PowerDashboardSnapshot:
    """Stable dashboard snapshot."""

    readings: dict[str, dict[str, object]]
    status: dict[str, object] | None
    health: dict[str, object] | None
    reading_age_s: dict[str, float]
    status_age_s: float
    health_age_s: float
    text: str

    def to_dict(self) -> dict[str, object]:
        return {
            "readings": self.readings,
            "status": self.status,
            "health": self.health,
            "reading_age_s": self.reading_age_s,
            "status_age_s": self.status_age_s,
            "health_age_s": self.health_age_s,
            "text": self.text,
        }

    def to_json_text(self) -> str:
        return json.dumps(
            self.to_dict(),
            sort_keys=True,
        )


@dataclass
class PowerDashboardMemory:
    """In-memory cache for dashboard node."""

    readings_by_source: dict[BatterySource, dict[str, object]] = field(
        default_factory=dict
    )
    reading_time_by_source: dict[BatterySource, float] = field(
        default_factory=dict
    )
    status: dict[str, object] | None = None
    status_time_s: float | None = None
    health: dict[str, object] | None = None
    health_time_s: float | None = None
    parse_error_count: int = 0
    last_parse_error: str = ""

    def update_reading(
        self,
        source: str | BatterySource,
        payload: object,
        *,
        now_s: float | None = None,
    ) -> BatterySource:
        """Update one source reading from payload."""

        normalized = normalize_battery_source(source)
        reading = parse_json_payload(payload)

        if "source" not in reading:
            reading["source"] = normalized.value

        timestamp = monotonic_time_s() if now_s is None else float(now_s)

        self.readings_by_source[normalized] = reading
        self.reading_time_by_source[normalized] = timestamp
        self.last_parse_error = ""

        return normalized

    def update_status(
        self,
        payload: object,
        *,
        now_s: float | None = None,
    ) -> None:
        """Update aggregate status from payload."""

        self.status = parse_json_payload(payload)
        self.status_time_s = monotonic_time_s() if now_s is None else float(now_s)
        self.last_parse_error = ""

    def update_health(
        self,
        payload: object,
        *,
        now_s: float | None = None,
    ) -> None:
        """Update health from payload."""

        self.health = parse_json_payload(payload)
        self.health_time_s = monotonic_time_s() if now_s is None else float(now_s)
        self.last_parse_error = ""

    def mark_parse_error(self, error: object) -> None:
        """Record one parse error."""

        self.parse_error_count += 1
        self.last_parse_error = f"{type(error).__name__}: {error}"

    def reading_age_s(
        self,
        source: str | BatterySource,
        *,
        now_s: float | None = None,
    ) -> float:
        """Return age of one source reading."""

        normalized = normalize_battery_source(source)

        return age_s(
            self.reading_time_by_source.get(normalized),
            now_s=now_s,
        )

    def status_age_s(self, *, now_s: float | None = None) -> float:
        """Return aggregate status age."""

        return age_s(self.status_time_s, now_s=now_s)

    def health_age_s(self, *, now_s: float | None = None) -> float:
        """Return health age."""

        return age_s(self.health_time_s, now_s=now_s)

    def clear(self) -> None:
        """Clear dashboard memory."""

        self.readings_by_source.clear()
        self.reading_time_by_source.clear()
        self.status = None
        self.status_time_s = None
        self.health = None
        self.health_time_s = None
        self.parse_error_count = 0
        self.last_parse_error = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "readings_by_source": {
                source.value: reading
                for source, reading in self.readings_by_source.items()
            },
            "reading_time_by_source": {
                source.value: timestamp
                for source, timestamp in self.reading_time_by_source.items()
            },
            "status": self.status,
            "status_time_s": self.status_time_s,
            "health": self.health,
            "health_time_s": self.health_time_s,
            "parse_error_count": self.parse_error_count,
            "last_parse_error": self.last_parse_error,
        }


@dataclass
class PowerDashboardNodeState:
    """Runtime state for Python dashboard node."""

    publish_count: int = 0
    callback_count: int = 0
    error_count: int = 0
    last_snapshot: PowerDashboardSnapshot | None = None
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
            "has_last_snapshot": self.last_snapshot is not None,
        }


def dashboard_python_node_name() -> str:
    """Return Python fallback dashboard node name."""

    return f"{c.POWER_DASHBOARD_NODE_NAME}_py"


def dashboard_topic() -> str:
    """Return machine-readable dashboard topic."""

    return c.DASHBOARD_TOPIC


def dashboard_text_topic() -> str:
    """Return human-readable dashboard topic."""

    return c.DASHBOARD_TEXT_TOPIC


def create_timer_period_s(rate_hz: float) -> float:
    """Create timer period from publish rate."""

    return rate_to_period_s(rate_hz)


def _plain_value(value: object) -> object:
    if isinstance(value, Enum):
        return value.value

    if isinstance(value, Mapping):
        return {
            str(key): _plain_value(item)
            for key, item in value.items()
        }

    if isinstance(value, (tuple, list)):
        return [
            _plain_value(item)
            for item in value
        ]

    return value


def parse_json_payload(payload: object) -> dict[str, object]:
    """Parse JSON payload from String message, raw text, or mapping."""

    if isinstance(payload, Mapping):
        return {
            str(key): _plain_value(value)
            for key, value in payload.items()
        }

    data = getattr(payload, "data", payload)

    if isinstance(data, bytes):
        data = data.decode("utf-8")

    if not isinstance(data, str):
        raise TypeError(f"Unsupported payload type: {type(payload).__name__}")

    text = data.strip()

    if not text:
        raise ValueError("Payload is empty")

    loaded = json.loads(text)

    if not isinstance(loaded, Mapping):
        raise ValueError("Payload JSON must be an object")

    return {
        str(key): _plain_value(value)
        for key, value in loaded.items()
    }


def source_from_reading(
    reading: Mapping[str, object],
    *,
    fallback: str | BatterySource = BatterySource.UNKNOWN,
) -> BatterySource:
    """Extract source from reading payload."""

    return normalize_battery_source(
        reading.get("source", fallback)
    )


def state_from_payload(
    payload: Mapping[str, object] | None,
    *,
    default: str | PowerState = PowerState.UNKNOWN,
) -> PowerState:
    """Extract state from status/reading payload."""

    if payload is None:
        return normalize_power_state(default)

    for key in ("overall_state", "state"):
        if key in payload:
            return normalize_power_state(_plain_value(payload[key]))

    return normalize_power_state(default)


def health_level_from_payload(
    payload: Mapping[str, object] | None,
) -> PowerHealthLevel:
    """Extract health level from health payload."""

    if payload is None:
        return PowerHealthLevel.UNKNOWN

    for key in ("level", "health", "status"):
        if key in payload:
            return normalize_health_level(_plain_value(payload[key]))

    return PowerHealthLevel.UNKNOWN


def payload_ok(payload: Mapping[str, object] | None) -> bool | None:
    """Extract bool ok from payload if available."""

    if payload is None or "ok" not in payload:
        return None

    value = payload.get("ok")

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        text = value.strip().lower()

        if text in {"true", "1", "yes", "ok"}:
            return True

        if text in {"false", "0", "no", "fail"}:
            return False

    return None


def reading_voltage_text(reading: Mapping[str, object]) -> str:
    """Format reading voltage."""

    value = reading.get("voltage_v")

    if value is None:
        return "voltage unknown"

    return format_voltage(float(value))


def reading_percentage_text(reading: Mapping[str, object]) -> str:
    """Format reading percentage/capacity/SoC."""

    for key, label in (
        ("capacity_pct", "capacity"),
        ("soc_pct", "SoC"),
        ("percentage", "percentage"),
    ):
        if key in reading and reading[key] is not None:
            return f"{label} {format_percentage(float(reading[key]))}"

    return "percentage unknown"


def format_reading_line(
    source: str | BatterySource,
    reading: Mapping[str, object] | None,
    *,
    age_value_s: float = 0.0,
) -> str:
    """Format one source reading for dashboard text."""

    normalized = normalize_battery_source(source)
    label = battery_source_label(normalized)

    if reading is None:
        return f"{label}: missing"

    state = state_from_payload(reading)

    return (
        f"{label}: {state.value}, "
        f"{reading_voltage_text(reading)}, "
        f"{reading_percentage_text(reading)}, "
        f"age {age_value_s:.1f}s"
    )


def format_status_line(
    status: Mapping[str, object] | None,
    *,
    age_value_s: float = 0.0,
) -> str:
    """Format aggregate status line."""

    if status is None:
        return "Status: missing"

    state = state_from_payload(status)
    ok = payload_ok(status)

    if ok is None:
        ok_text = "unknown"
    else:
        ok_text = str(ok).lower()

    return f"Status: {state.value}, ok={ok_text}, age {age_value_s:.1f}s"


def format_health_line(
    health: Mapping[str, object] | None,
    *,
    age_value_s: float = 0.0,
) -> str:
    """Format health line."""

    if health is None:
        return "Health: missing"

    level = health_level_from_payload(health)
    state = state_from_payload(health)
    ok = payload_ok(health)
    reason = str(health.get("reason", ""))

    if ok is None:
        ok_text = "unknown"
    else:
        ok_text = str(ok).lower()

    if reason:
        return (
            f"Health: {level.value}, state={state.value}, "
            f"ok={ok_text}, age {age_value_s:.1f}s, {reason}"
        )

    return (
        f"Health: {level.value}, state={state.value}, "
        f"ok={ok_text}, age {age_value_s:.1f}s"
    )


def build_dashboard_text(
    memory: PowerDashboardMemory,
    *,
    now_s: float | None = None,
) -> str:
    """Build human-readable dashboard text."""

    now = monotonic_time_s() if now_s is None else float(now_s)
    lines = ["Robot Savo power dashboard"]

    for source in POWER_SOURCE_ORDER:
        lines.append(
            format_reading_line(
                source,
                memory.readings_by_source.get(source),
                age_value_s=memory.reading_age_s(source, now_s=now),
            )
        )

    lines.append(
        format_status_line(
            memory.status,
            age_value_s=memory.status_age_s(now_s=now),
        )
    )
    lines.append(
        format_health_line(
            memory.health,
            age_value_s=memory.health_age_s(now_s=now),
        )
    )

    if memory.parse_error_count:
        lines.append(
            f"Parse errors: {memory.parse_error_count}, "
            f"last={memory.last_parse_error}"
        )

    return "\n".join(lines)


def build_dashboard_snapshot(
    memory: PowerDashboardMemory,
    *,
    now_s: float | None = None,
) -> PowerDashboardSnapshot:
    """Build dashboard snapshot from memory."""

    now = monotonic_time_s() if now_s is None else float(now_s)

    readings = {
        source.value: dict(reading)
        for source, reading in memory.readings_by_source.items()
    }

    reading_ages = {
        source.value: memory.reading_age_s(source, now_s=now)
        for source in POWER_SOURCE_ORDER
    }

    text = build_dashboard_text(
        memory,
        now_s=now,
    )

    return PowerDashboardSnapshot(
        readings=readings,
        status=dict(memory.status) if memory.status is not None else None,
        health=dict(memory.health) if memory.health is not None else None,
        reading_age_s=reading_ages,
        status_age_s=memory.status_age_s(now_s=now),
        health_age_s=memory.health_age_s(now_s=now),
        text=text,
    )


def build_startup_summary(params: PowerDashboardNodeParams, topic: str) -> str:
    """Create stable startup summary."""

    return (
        f"topic={topic} "
        f"text_topic={c.DASHBOARD_TEXT_TOPIC} "
        f"publish_rate_hz={params.publish_rate_hz:.2f}"
    )


if RCLPY_AVAILABLE:

    class PowerDashboardNodePy(Node):  # type: ignore[misc]
        """ROS 2 Python fallback dashboard node."""

        def __init__(self) -> None:
            node_name = dashboard_python_node_name()

            super().__init__(node_name)

            self._logger = get_node_logger(self)
            self._params = read_power_dashboard_node_params(self)
            self._memory = PowerDashboardMemory()
            self._state = PowerDashboardNodeState()

            self._dashboard_publisher = self.create_publisher(
                string_message_class(),
                c.DASHBOARD_TOPIC,
                power_dashboard_qos(),
            )

            self._dashboard_text_publisher = self.create_publisher(
                string_message_class(),
                c.DASHBOARD_TEXT_TOPIC,
                power_dashboard_qos(),
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
                self.create_subscription(
                    string_message_class(),
                    c.STATUS_TOPIC,
                    self._on_status_msg,
                    power_status_qos(),
                ),
                self.create_subscription(
                    string_message_class(),
                    c.HEALTH_TOPIC,
                    self._on_health_msg,
                    power_health_qos(),
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
                build_startup_summary(self._params, c.DASHBOARD_TOPIC),
            )

        @property
        def memory(self) -> PowerDashboardMemory:
            return self._memory

        @property
        def state(self) -> PowerDashboardNodeState:
            return self._state

        def _on_reading_msg(
            self,
            source: BatterySource,
            msg: object,
        ) -> None:
            try:
                self._memory.update_reading(source, msg)
                self._state.callback_count += 1
            except Exception as exc:  # noqa: BLE001 - malformed message must not kill node
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"
                self._memory.mark_parse_error(exc)
                log_exception(self._logger, "Dashboard reading parse failed", exc)

        def _on_status_msg(self, msg: object) -> None:
            try:
                self._memory.update_status(msg)
                self._state.callback_count += 1
            except Exception as exc:  # noqa: BLE001
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"
                self._memory.mark_parse_error(exc)
                log_exception(self._logger, "Dashboard status parse failed", exc)

        def _on_health_msg(self, msg: object) -> None:
            try:
                self._memory.update_health(msg)
                self._state.callback_count += 1
            except Exception as exc:  # noqa: BLE001
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"
                self._memory.mark_parse_error(exc)
                log_exception(self._logger, "Dashboard health parse failed", exc)

        def build_snapshot(self) -> PowerDashboardSnapshot:
            """Build current dashboard snapshot."""

            return build_dashboard_snapshot(self._memory)

        def publish_snapshot(self, snapshot: PowerDashboardSnapshot) -> None:
            """Publish dashboard snapshot."""

            self._dashboard_publisher.publish(
                make_string_message(snapshot.to_json_text())
            )
            self._dashboard_text_publisher.publish(
                make_string_message(snapshot.text)
            )

            self._state.publish_count += 1
            self._state.last_snapshot = snapshot

            log_throttled(
                self._logger,
                "power_dashboard",
                "info",
                snapshot.text.replace("\n", " | "),
                period_s=5.0,
            )

        def _on_timer(self) -> None:
            snapshot = self.build_snapshot()
            self.publish_snapshot(snapshot)

else:

    class PowerDashboardNodePy:  # pragma: no cover - only used when ROS is missing
        """Placeholder when rclpy is unavailable."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError(
                "rclpy is not available. Source ROS 2 first, or use the "
                "helper functions in this module for offline checks."
            )


def spin_power_dashboard_node() -> None:
    """Spin power dashboard Python fallback node."""

    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")

    rclpy.init()
    node = PowerDashboardNodePy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        logger = get_node_logger(node)
        log_info(logger, f"{dashboard_python_node_name()} stopped")
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Entry point for power dashboard Python fallback node."""

    spin_power_dashboard_node()


__all__ = [
    "POWER_SOURCE_ORDER",
    "RCLPY_AVAILABLE",
    "PowerDashboardMemory",
    "PowerDashboardNodePy",
    "PowerDashboardNodeState",
    "PowerDashboardSnapshot",
    "build_dashboard_snapshot",
    "build_dashboard_text",
    "build_startup_summary",
    "create_timer_period_s",
    "dashboard_python_node_name",
    "dashboard_text_topic",
    "dashboard_topic",
    "format_health_line",
    "format_reading_line",
    "format_status_line",
    "health_level_from_payload",
    "main",
    "parse_json_payload",
    "payload_ok",
    "reading_percentage_text",
    "reading_voltage_text",
    "source_from_reading",
    "spin_power_dashboard_node",
    "state_from_payload",
]
