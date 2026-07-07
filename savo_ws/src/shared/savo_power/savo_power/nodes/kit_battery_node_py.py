"""Python fallback ADS7830/base battery node for Robot Savo."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_power import constants as c
from savo_power.drivers.ads7830 import (
    Ads7830Config,
    Ads7830Driver,
)
from savo_power.models.kit_battery_reading import (
    KitBatteryReading,
    make_kit_battery_error,
)
from savo_power.models.power_status import BatterySource
from savo_power.ros.adapters import (
    make_reading_json_message,
    reading_to_json_text,
    string_message_class,
)
from savo_power.ros.params import (
    KitBatteryNodeParams,
    read_kit_battery_node_params,
)
from savo_power.ros.qos_profiles import power_sensor_qos
from savo_power.utils.logging import (
    get_node_logger,
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
class KitBatteryNodeState:
    """Runtime state for Python base battery node."""

    read_count: int = 0
    publish_count: int = 0
    error_count: int = 0
    last_reading: KitBatteryReading | None = None
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


def base_battery_topic() -> str:
    """Return base battery topic."""

    return c.BASE_BATTERY_TOPIC


def base_battery_python_node_name() -> str:
    """Return Python fallback base battery node name."""

    return f"{c.BASE_BATTERY_NODE_NAME}_py"


def create_timer_period_s(rate_hz: float) -> float:
    """Create timer period from publish rate."""

    return rate_to_period_s(rate_hz)


def create_ads7830_driver_from_params(params: KitBatteryNodeParams) -> Ads7830Driver:
    """Create ADS7830 driver from node parameters."""

    config = Ads7830Config(
        bus_id=params.i2c_bus,
        address=params.address,
        channel=params.channel,
        pcb_version=params.pcb_version,
    )

    return Ads7830Driver(config)


def read_from_driver(driver: object) -> KitBatteryReading:
    """Read one base battery sample from a driver-like object."""

    for method_name in ("read", "read_once", "sample"):
        method = getattr(driver, method_name, None)

        if callable(method):
            reading = method()

            if isinstance(reading, KitBatteryReading):
                return reading

            raise TypeError(
                f"ADS7830 driver method {method_name} returned "
                f"{type(reading).__name__}, expected KitBatteryReading"
            )

    raise AttributeError("ADS7830 driver has no read/read_once/sample method")


def make_error_reading(error: object) -> KitBatteryReading:
    """Create base battery error reading while tolerating factory changes."""

    error_text = str(error)

    try:
        return make_kit_battery_error(error_text)
    except TypeError:
        try:
            return make_kit_battery_error(error=error_text)
        except TypeError:
            return make_kit_battery_error(
                source=BatterySource.BASE_BATTERY,
                error=error_text,
            )


def reading_to_publish_text(reading: KitBatteryReading) -> str:
    """Convert reading to JSON publish text."""

    return reading_to_json_text(reading)


def build_startup_summary(params: KitBatteryNodeParams, topic: str) -> str:
    """Create stable startup summary."""

    return (
        f"source={BatterySource.BASE_BATTERY.value} "
        f"topic={topic} "
        f"bus={params.i2c_bus} "
        f"address=0x{params.address:02X} "
        f"channel={params.channel} "
        f"pcb_version={params.pcb_version} "
        f"publish_rate_hz={params.publish_rate_hz:.2f}"
    )


if RCLPY_AVAILABLE:

    class KitBatteryNodePy(Node):  # type: ignore[misc]
        """ROS 2 Python fallback ADS7830/base battery node."""

        def __init__(
            self,
            *,
            driver: object | None = None,
        ) -> None:
            node_name = base_battery_python_node_name()

            super().__init__(node_name)

            self._logger = get_node_logger(self)
            self._params = read_kit_battery_node_params(self)

            self._topic = base_battery_topic()
            self._state = KitBatteryNodeState()

            self._publisher = self.create_publisher(
                string_message_class(),
                self._topic,
                power_sensor_qos(),
            )

            self._driver = driver or create_ads7830_driver_from_params(self._params)

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
        def state(self) -> KitBatteryNodeState:
            return self._state

        @property
        def topic(self) -> str:
            return self._topic

        def read_once(self) -> KitBatteryReading:
            """Read once from ADS7830 driver."""

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
                    "ADS7830/base battery read failed",
                    exc,
                )

                reading = make_error_reading(self._state.last_error)
                self._state.last_reading = reading
                return reading

        def publish_reading(self, reading: KitBatteryReading) -> None:
            """Publish one base battery reading."""

            message = make_reading_json_message(reading)
            self._publisher.publish(message)
            self._state.publish_count += 1
            log_power_reading(self._logger, reading)

        def _on_timer(self) -> None:
            reading = self.read_once()
            self.publish_reading(reading)

else:

    class KitBatteryNodePy:  # pragma: no cover - only used when ROS is missing
        """Placeholder when rclpy is unavailable."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError(
                "rclpy is not available. Source ROS 2 first, or use the "
                "diagnostic CLI modules instead."
            )


def spin_kit_battery_node() -> None:
    """Spin base battery Python fallback node."""

    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")

    rclpy.init()
    node = KitBatteryNodePy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        logger = get_node_logger(node)
        log_info(logger, f"{base_battery_python_node_name()} stopped")
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Entry point for base battery Python fallback node."""

    spin_kit_battery_node()


__all__ = [
    "RCLPY_AVAILABLE",
    "KitBatteryNodePy",
    "KitBatteryNodeState",
    "base_battery_python_node_name",
    "base_battery_topic",
    "build_startup_summary",
    "create_ads7830_driver_from_params",
    "create_timer_period_s",
    "main",
    "make_error_reading",
    "read_from_driver",
    "reading_to_publish_text",
    "spin_kit_battery_node",
]
