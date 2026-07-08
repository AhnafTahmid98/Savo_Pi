import inspect
import py_compile
from pathlib import Path

from savo_power.drivers import ads7830, smbus_adapter, ups_hat
from savo_power.models import kit_battery_reading, ups_reading


def test_python_driver_modules_compile():
    for path in [
        "savo_power/drivers/smbus_adapter.py",
        "savo_power/drivers/ups_hat.py",
        "savo_power/drivers/ads7830.py",
    ]:
        py_compile.compile(path, doraise=True)


def test_python_driver_modules_import_without_hardware_access():
    assert smbus_adapter is not None
    assert ups_hat is not None
    assert ads7830 is not None


def test_smbus_adapter_exports_bus_adapter_symbol():
    names = set(dir(smbus_adapter))

    assert (
        "SmbusAdapter" in names
        or "SMBusAdapter" in names
        or "I2cBusAdapter" in names
        or "I2CBusAdapter" in names
        or "LinuxSmbusAdapter" in names
    )


def test_ups_hat_driver_exports_driver_or_factory_symbol():
    names = set(dir(ups_hat))

    assert any("Ups" in name and "Driver" in name for name in names) or any(
        "ups" in name.lower() and "driver" in name.lower()
        for name in names
    )


def test_ads7830_driver_exports_known_dryrun_helpers():
    for name in [
        "Ads7830Config",
        "Ads7830Driver",
        "ads7830_channel_command",
        "ads7830_adc_voltage_from_byte",
        "ads7830_battery_voltage_from_byte",
        "estimate_linear_soc_pct",
        "make_ads7830_driver",
        "read_kit_battery_once",
    ]:
        assert hasattr(ads7830, name), name


def test_ads7830_channel_command_is_locked_for_base_channel_two():
    assert ads7830.ads7830_channel_command(2) == 0x94
    assert kit_battery_reading.ads7830_channel_command(2) == 0x94


def test_ads7830_channel_command_rejects_invalid_channels():
    for channel in [-1, 8]:
        try:
            ads7830.ads7830_channel_command(channel)
        except Exception:
            pass
        else:
            raise AssertionError(f"invalid ADS7830 channel accepted: {channel}")


def test_ads7830_adc_voltage_conversion_is_dryrun_safe():
    value = ads7830.ads7830_adc_voltage_from_byte(128)

    assert isinstance(value, float)
    assert value > 0.0


def test_ads7830_battery_voltage_conversion_is_dryrun_safe():
    signature = inspect.signature(ads7830.ads7830_battery_voltage_from_byte)

    if "pcb_version" in signature.parameters:
        value = ads7830.ads7830_battery_voltage_from_byte(
            200,
            pcb_version="v2",
        )
    elif len(signature.parameters) >= 2:
        value = ads7830.ads7830_battery_voltage_from_byte(200, "v2")
    else:
        value = ads7830.ads7830_battery_voltage_from_byte(200)

    assert isinstance(value, float)
    assert value > 0.0


def test_ads7830_soc_estimation_is_clamped():
    assert ads7830.estimate_linear_soc_pct(0.0) == 0.0
    assert ads7830.estimate_linear_soc_pct(99.0) == 100.0

    middle = ads7830.estimate_linear_soc_pct(
        7.4,
        empty_voltage_v=6.4,
        full_voltage_v=8.4,
    )

    assert 45.0 <= middle <= 55.0


def test_ups_raw_voltage_conversion_is_dryrun_safe():
    assert hasattr(ups_reading, "UpsReading")

    candidates = [
        value
        for name, value in vars(ups_reading).items()
        if callable(value)
        and "voltage" in name.lower()
        and "raw" in name.lower()
    ]

    assert candidates

    converted = None

    for func in candidates:
        try:
            converted = func(0x00D2)
            break
        except TypeError:
            continue

    assert converted is not None
    assert isinstance(converted, float)
    assert converted > 0.0


def test_ups_raw_capacity_conversion_is_dryrun_safe():
    candidates = [
        value
        for name, value in vars(ups_reading).items()
        if callable(value)
        and "capacity" in name.lower()
        and "raw" in name.lower()
    ]

    assert candidates

    converted = None

    for func in candidates:
        try:
            converted = func(0x0064)
            break
        except TypeError:
            continue

    assert converted is not None
    assert isinstance(converted, float)
    assert converted >= 0.0


def test_driver_source_files_do_not_open_hardware_at_import_level():
    for path in [
        "savo_power/drivers/smbus_adapter.py",
        "savo_power/drivers/ups_hat.py",
        "savo_power/drivers/ads7830.py",
    ]:
        text = Path(path).read_text()

        assert "open('/dev/i2c" not in text
        assert 'open("/dev/i2c' not in text


def test_driver_dryrun_layer_does_not_import_ros_runtime():
    for path in [
        "savo_power/drivers/smbus_adapter.py",
        "savo_power/drivers/ups_hat.py",
        "savo_power/drivers/ads7830.py",
    ]:
        text = Path(path).read_text()

        assert "import rclpy" not in text
        assert "from rclpy" not in text


def test_driver_dryrun_layer_does_not_enable_shutdown_or_motor_control():
    combined = "\n".join(
        Path(path).read_text(errors="ignore")
        for path in [
            "savo_power/drivers/smbus_adapter.py",
            "savo_power/drivers/ups_hat.py",
            "savo_power/drivers/ads7830.py",
        ]
    ).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
        "pca9685",
    ]:
        assert forbidden not in combined


def test_python_driver_package_exports_driver_modules():
    import savo_power.drivers as drivers

    assert hasattr(drivers, "ads7830")
    assert hasattr(drivers, "ups_hat")
    assert hasattr(drivers, "smbus_adapter")
