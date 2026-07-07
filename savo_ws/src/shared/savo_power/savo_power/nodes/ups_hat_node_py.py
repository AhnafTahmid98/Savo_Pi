"""Python fallback UPS HAT node for Robot Savo power monitoring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_power import constants as c
from savo_power.drivers.ups_hat import (
    UpsHatConfig,
    UpsHatDriver,
)
from savo_power.models.power_status import (
    BatterySource,
    normalize_battery_source,
)
from savo_power.models.ups_reading import (
    UpsReading,
    make_ups_error,
)
from savo_power.ros.adapters import (
    make_reading_json_message,
    reading_to_json_text,
    string_message_class,
)
from savo_power.ros.params import (
    UpsNodeParams,
    read_ups_node_params,
)
from savo_power.ros.qos_profiles import power_sensor_qos
from savo_power.utils.logging import (
    get_node_logger,
    log_error,
    log_exception,
    log_info,
    log_power_reading,
    log_startup,
)
from savo_power.utils.timing import rate_to_period_s


try:
    import rclpy
    from rclpy.node import Node

    RCLPY_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on ROS environment
    rclpy = None
    Node = object  # type: ignore[assignment]
    RCLPY_AVAILABLE = False


@dataclass
class UpsHatNodeState:
    """Runtime state for Python UPS node."""

    read_count: int = 0
    publish_count: int = 0
    error_count: int = 0
    last_reading: UpsReading | None = None
    last_error: str = ""

    @property
    def ok(self) -> bool:
        return self.error_count == 0 or self.last_reading is not None

    def to_dict(self) -> dict[str, object]:
        return {
            "read_count": self.read_count,
            "publish_count": self.publish_count,
            "error_count": self.error_count,
            "last_error": self.last_error,
            "has_last_reading": self.last_reading is not None,
        }


def source_to_topic(source: str | BatterySource) -> str:
    """Return UPS topic for source."""

    normalized = normalize_battery_source(source)

    if normalized == BatterySource.CORE_UPS:
        return c.CORE_UPS_TOPIC

    if normalized == BatterySource.EDGE_UPS:
        return c.EDGE_UPS_TOPIC

    raise ValueError(f"UPS node does not support source: {source}")


def source_to_python_node_name(source: str | BatterySource) -> str:
    """Return Python fallback node name for source."""

    normalized = normalize_battery_source(source)

    if normalized == BatterySource.CORE_UPS:
        return f"{c.CORE_UPS_NODE_NAME}_py"

    if normalized == BatterySource.EDGE_UPS:
        return f"{c.EDGE_UPS_NODE_NAME}_py"

    raise ValueError(f"UPS node does not support source: {source}")


def validate_ups_source(source: str | BatterySource) -> BatterySource:
    """Validate and normalize UPS source."""

    normalized = normalize_battery_source(source)

    if normalized not in {
        BatterySource.CORE_UPS,
        BatterySource.EDGE_UPS,
    }:
        raise ValueError(
            "UPS HAT node source must be core_ups or edge_ups, "
            f"got {source}"
        )

    return normalized


def create_timer_period_s(rate_hz: float) -> float:
    """Create timer period from publish rate."""

    return rate_to_period_s(rate_hz)


def create_ups_driver_from_params(params: UpsNodeParams) -> UpsHatDriver:
    """Create UPS HAT driver from node parameters."""

    config = UpsHatConfig(
        source=params.source,
        bus_id=params.i2c_bus,
        address=params.address,
    )

    return UpsHatDriver(config)


def read_from_driver(driver: object) -> UpsReading:
    """Read one UPS sample from a driver-like object."""

    for method_name in ("read", "read_once", "sample"):
        method = getattr(driver, method_name, None)

        if callable(method):
            reading = method()

            if isinstance(reading, UpsReading):
                return reading

            raise TypeError(
                f"UPS driver method {method_name} returned "
                f"{type(reading).__name__}, expected UpsReading"
            )

    raise AttributeError("UPS driver has no read/read_once/sample method")


def make_error_reading(source: str | BatterySource, error: object) -> UpsReading:
    """Create UPS error reading while tolerating factory signature changes."""

    normalized = validate_ups_source(source)
    error_text = str(error)

    try:
        return make_ups_error(normalized, error_text)
    except TypeError:
        return make_ups_error(source=normalized, error=error_text)


def reading_to_publish_text(reading: UpsReading) -> str:
    """Convert reading to JSON publish text."""

    return reading_to_json_text(reading)


def build_startup_summary(params: UpsNodeParams, topic: str) -> str:
    """Create stable startup summary."""

    return (
        f"source={params.source.value} "
        f"topic={topic} "
        f"bus={params.i2c_bus} "
        f"address=0x{params.address:02X} "
        f"publish_rate_hz={params.publish_rate_hz:.2f}"
    )


if RCLPY_AVAILABLE:

    class UpsHatNodePy(Node):  # type: ignore[misc]
        """ROS 2 Python fallback UPS HAT node."""

        def __init__(
            self,
            source: str | BatterySource = BatterySource.CORE_UPS,
            *,
            driver: object | None = None,
        ) -> None:
            normalized_source = validate_ups_source(source)
            node_name = source_to_python_node_name(normalized_source)

            super().__init__(node_name)

            self._logger = get_node_logger(self)
            self._params = read_ups_node_params(
                self,
                source=normalized_source,
            )

            self._source = validate_ups_source(self._params.source)
            self._topic = source_to_topic(self._source)
            self._state = UpsHatNodeState()

            self._publisher = self.create_publisher(
                string_message_class(),
                self._topic,
                power_sensor_qos(),
            )

            self._driver = driver or create_ups_driver_from_params(self._params)

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
                build_startup_summary(self._params, self._topic),
            )

        @property
        def state(self) -> UpsHatNodeState:
            return self._state

        @property
        def topic(self) -> str:
            return self._topic

        @property
        def source(self) -> BatterySource:
            return self._source

        def read_once(self) -> UpsReading:
            """Read once from UPS HAT driver."""

            try:
                reading = read_from_driver(self._driver)
                self._state.read_count += 1
                self._state.last_reading = reading
                self._state.last_error = ""
                return reading
            except Exception as exc:  # noqa: BLE001 - hardware errors must be published
                self._state.error_count += 1
                self._state.last_error = f"{type(exc).__name__}: {exc}"

                log_exception(
                    self._logger,
                    "UPS HAT read failed",
                    exc,
                )

                reading = make_error_reading(self._source, self._state.last_error)
                self._state.last_reading = reading
                return reading

        def publish_reading(self, reading: UpsReading) -> None:
            """Publish one reading."""

            message = make_reading_json_message(reading)
            self._publisher.publish(message)
            self._state.publish_count += 1
            log_power_reading(self._logger, reading)

        def _on_timer(self) -> None:
            reading = self.read_once()
            self.publish_reading(reading)

else:

    class UpsHatNodePy:  # pragma: no cover - only used when ROS is missing
        """Placeholder when rclpy is unavailable."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError(
                "rclpy is not available. Source ROS 2 first, or use the "
                "diagnostic CLI modules instead."
            )


def spin_ups_node(source: str | BatterySource) -> None:
    """Spin one UPS Python fallback node."""

    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")

    rclpy.init()
    node = UpsHatNodePy(source=source)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        logger = get_node_logger(node)
        log_info(logger, f"{source_to_python_node_name(source)} stopped")
        node.destroy_node()
        rclpy.shutdown()


def main_core() -> None:
    """Entry point for core UPS Python fallback node."""

    spin_ups_node(BatterySource.CORE_UPS)


def main_edge() -> None:
    """Entry point for edge UPS Python fallback node."""

    spin_ups_node(BatterySource.EDGE_UPS)


def main() -> None:
    """Default entry point runs core UPS fallback node."""

    main_core()


__all__ = [
    "RCLPY_AVAILABLE",
    "UpsHatNodePy",
    "UpsHatNodeState",
    "build_startup_summary",
    "create_timer_period_s",
    "create_ups_driver_from_params",
    "main",
    "main_core",
    "main_edge",
    "make_error_reading",
    "read_from_driver",
    "reading_to_publish_text",
    "source_to_python_node_name",
    "source_to_topic",
    "spin_ups_node",
    "validate_ups_source",
]
