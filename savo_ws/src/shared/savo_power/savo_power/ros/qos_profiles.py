"""QoS profile helpers for Robot Savo Python power fallback nodes."""

from __future__ import annotations

import importlib
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class QosReliability(str, Enum):
    """Portable QoS reliability names."""

    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class QosDurability(str, Enum):
    """Portable QoS durability names."""

    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


class QosHistory(str, Enum):
    """Portable QoS history names."""

    KEEP_LAST = "keep_last"
    KEEP_ALL = "keep_all"


@dataclass(frozen=True)
class QosProfileSpec:
    """ROS-independent QoS profile specification."""

    depth: int = 10
    reliability: QosReliability = QosReliability.RELIABLE
    durability: QosDurability = QosDurability.VOLATILE
    history: QosHistory = QosHistory.KEEP_LAST

    def to_dict(self) -> dict[str, object]:
        return {
            "depth": int(self.depth),
            "reliability": self.reliability.value,
            "durability": self.durability.value,
            "history": self.history.value,
        }


@dataclass(frozen=True)
class QosProfileFallback:
    """Fallback replacement for rclpy.qos.QoSProfile."""

    depth: int = 10
    reliability: str = QosReliability.RELIABLE.value
    durability: str = QosDurability.VOLATILE.value
    history: str = QosHistory.KEEP_LAST.value

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _qos_module() -> Any | None:
    try:
        return importlib.import_module("rclpy.qos")
    except ImportError:
        return None


def rclpy_qos_available() -> bool:
    """Return True when rclpy.qos can be imported."""

    return _qos_module() is not None


def normalize_reliability(value: str | QosReliability | None) -> QosReliability:
    """Normalize reliability value."""

    if isinstance(value, QosReliability):
        return value

    if value is None:
        return QosReliability.RELIABLE

    normalized = str(value).strip().lower().replace("-", "_")

    for option in QosReliability:
        if normalized == option.value:
            return option

    return QosReliability.RELIABLE


def normalize_durability(value: str | QosDurability | None) -> QosDurability:
    """Normalize durability value."""

    if isinstance(value, QosDurability):
        return value

    if value is None:
        return QosDurability.VOLATILE

    normalized = str(value).strip().lower().replace("-", "_")

    for option in QosDurability:
        if normalized == option.value:
            return option

    return QosDurability.VOLATILE


def normalize_history(value: str | QosHistory | None) -> QosHistory:
    """Normalize history value."""

    if isinstance(value, QosHistory):
        return value

    if value is None:
        return QosHistory.KEEP_LAST

    normalized = str(value).strip().lower().replace("-", "_")

    for option in QosHistory:
        if normalized == option.value:
            return option

    return QosHistory.KEEP_LAST


def make_qos_spec(
    *,
    depth: int = 10,
    reliability: str | QosReliability = QosReliability.RELIABLE,
    durability: str | QosDurability = QosDurability.VOLATILE,
    history: str | QosHistory = QosHistory.KEEP_LAST,
) -> QosProfileSpec:
    """Create normalized QoS profile specification."""

    safe_depth = max(1, int(depth))

    return QosProfileSpec(
        depth=safe_depth,
        reliability=normalize_reliability(reliability),
        durability=normalize_durability(durability),
        history=normalize_history(history),
    )


def _enum_value(
    enum_class: Any,
    *,
    reliable_name: str,
    fallback_value: str,
) -> Any:
    if enum_class is None:
        return fallback_value

    if hasattr(enum_class, reliable_name):
        return getattr(enum_class, reliable_name)

    if hasattr(enum_class, reliable_name.upper()):
        return getattr(enum_class, reliable_name.upper())

    return fallback_value


def _rclpy_reliability(module: Any, reliability: QosReliability) -> Any:
    enum_class = (
        getattr(module, "ReliabilityPolicy", None)
        or getattr(module, "QoSReliabilityPolicy", None)
    )

    if reliability == QosReliability.BEST_EFFORT:
        return _enum_value(
            enum_class,
            reliable_name="BEST_EFFORT",
            fallback_value=QosReliability.BEST_EFFORT.value,
        )

    return _enum_value(
        enum_class,
        reliable_name="RELIABLE",
        fallback_value=QosReliability.RELIABLE.value,
    )


def _rclpy_durability(module: Any, durability: QosDurability) -> Any:
    enum_class = (
        getattr(module, "DurabilityPolicy", None)
        or getattr(module, "QoSDurabilityPolicy", None)
    )

    if durability == QosDurability.TRANSIENT_LOCAL:
        return _enum_value(
            enum_class,
            reliable_name="TRANSIENT_LOCAL",
            fallback_value=QosDurability.TRANSIENT_LOCAL.value,
        )

    return _enum_value(
        enum_class,
        reliable_name="VOLATILE",
        fallback_value=QosDurability.VOLATILE.value,
    )


def _rclpy_history(module: Any, history: QosHistory) -> Any:
    enum_class = (
        getattr(module, "HistoryPolicy", None)
        or getattr(module, "QoSHistoryPolicy", None)
    )

    if history == QosHistory.KEEP_ALL:
        return _enum_value(
            enum_class,
            reliable_name="KEEP_ALL",
            fallback_value=QosHistory.KEEP_ALL.value,
        )

    return _enum_value(
        enum_class,
        reliable_name="KEEP_LAST",
        fallback_value=QosHistory.KEEP_LAST.value,
    )


def make_qos_profile(spec: QosProfileSpec) -> Any:
    """Create rclpy QoSProfile when available, otherwise fallback."""

    module = _qos_module()

    if module is None:
        return QosProfileFallback(
            depth=spec.depth,
            reliability=spec.reliability.value,
            durability=spec.durability.value,
            history=spec.history.value,
        )

    qos_profile_class = getattr(module, "QoSProfile", None)

    if qos_profile_class is None:
        return QosProfileFallback(
            depth=spec.depth,
            reliability=spec.reliability.value,
            durability=spec.durability.value,
            history=spec.history.value,
        )

    return qos_profile_class(
        depth=spec.depth,
        reliability=_rclpy_reliability(module, spec.reliability),
        durability=_rclpy_durability(module, spec.durability),
        history=_rclpy_history(module, spec.history),
    )


def qos_profile_to_dict(profile: Any) -> dict[str, object]:
    """Convert rclpy or fallback QoS profile into stable dictionary."""

    if isinstance(profile, QosProfileSpec):
        return profile.to_dict()

    if isinstance(profile, QosProfileFallback):
        return profile.to_dict()

    depth = getattr(profile, "depth", 10)
    reliability = getattr(profile, "reliability", QosReliability.RELIABLE.value)
    durability = getattr(profile, "durability", QosDurability.VOLATILE.value)
    history = getattr(profile, "history", QosHistory.KEEP_LAST.value)

    return {
        "depth": int(depth),
        "reliability": _policy_to_text(reliability),
        "durability": _policy_to_text(durability),
        "history": _policy_to_text(history),
    }


def _policy_to_text(value: object) -> str:
    """Convert rclpy enum/fallback policy value to readable text."""

    if isinstance(value, Enum):
        return str(value.name).lower()

    text = str(value).strip()

    if "." in text:
        text = text.split(".")[-1]

    return text.lower()


def power_sensor_qos_spec() -> QosProfileSpec:
    """QoS for direct UPS/base battery sensor readings."""

    return make_qos_spec(
        depth=10,
        reliability=QosReliability.RELIABLE,
        durability=QosDurability.VOLATILE,
        history=QosHistory.KEEP_LAST,
    )


def power_status_qos_spec() -> QosProfileSpec:
    """QoS for aggregated power status."""

    return make_qos_spec(
        depth=10,
        reliability=QosReliability.RELIABLE,
        durability=QosDurability.VOLATILE,
        history=QosHistory.KEEP_LAST,
    )


def power_health_qos_spec() -> QosProfileSpec:
    """QoS for power health messages."""

    return make_qos_spec(
        depth=10,
        reliability=QosReliability.RELIABLE,
        durability=QosDurability.VOLATILE,
        history=QosHistory.KEEP_LAST,
    )


def power_dashboard_qos_spec() -> QosProfileSpec:
    """QoS for dashboard text/status output."""

    return make_qos_spec(
        depth=5,
        reliability=QosReliability.RELIABLE,
        durability=QosDurability.VOLATILE,
        history=QosHistory.KEEP_LAST,
    )


def latched_status_qos_spec() -> QosProfileSpec:
    """QoS for optional late-joiner status/debug topics."""

    return make_qos_spec(
        depth=1,
        reliability=QosReliability.RELIABLE,
        durability=QosDurability.TRANSIENT_LOCAL,
        history=QosHistory.KEEP_LAST,
    )


def power_sensor_qos() -> Any:
    """Create QoS profile for direct sensor readings."""

    return make_qos_profile(power_sensor_qos_spec())


def power_status_qos() -> Any:
    """Create QoS profile for aggregated status."""

    return make_qos_profile(power_status_qos_spec())


def power_health_qos() -> Any:
    """Create QoS profile for power health."""

    return make_qos_profile(power_health_qos_spec())


def power_dashboard_qos() -> Any:
    """Create QoS profile for dashboard output."""

    return make_qos_profile(power_dashboard_qos_spec())


def latched_status_qos() -> Any:
    """Create QoS profile for optional latched status/debug output."""

    return make_qos_profile(latched_status_qos_spec())


def qos_profile_by_name(name: str) -> Any:
    """Create QoS profile by stable profile name."""

    normalized = str(name).strip().lower().replace("-", "_")

    if normalized in {"sensor", "power_sensor", "reading", "readings"}:
        return power_sensor_qos()

    if normalized in {"status", "power_status", "aggregator"}:
        return power_status_qos()

    if normalized in {"health", "power_health"}:
        return power_health_qos()

    if normalized in {"dashboard", "power_dashboard"}:
        return power_dashboard_qos()

    if normalized in {"latched", "latched_status", "transient_local"}:
        return latched_status_qos()

    return power_status_qos()


__all__ = [
    "QosDurability",
    "QosHistory",
    "QosProfileFallback",
    "QosProfileSpec",
    "QosReliability",
    "latched_status_qos",
    "latched_status_qos_spec",
    "make_qos_profile",
    "make_qos_spec",
    "normalize_durability",
    "normalize_history",
    "normalize_reliability",
    "power_dashboard_qos",
    "power_dashboard_qos_spec",
    "power_health_qos",
    "power_health_qos_spec",
    "power_sensor_qos",
    "power_sensor_qos_spec",
    "power_status_qos",
    "power_status_qos_spec",
    "qos_profile_by_name",
    "qos_profile_to_dict",
    "rclpy_qos_available",
]
