import inspect
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def get_public_values(value, seen=None):
    if seen is None:
        seen = set()

    value_id = id(value)
    if value_id in seen:
        return

    seen.add(value_id)

    if isinstance(value, (str, int, float, bool)):
        yield value
        return

    if isinstance(value, dict):
        for key, item in value.items():
            yield key
            yield from get_public_values(item, seen)
        return

    if isinstance(value, (list, tuple, set, frozenset)):
        for item in value:
            yield from get_public_values(item, seen)
        return

    if hasattr(value, "__dict__"):
        for name, item in vars(value).items():
            if not name.startswith("_"):
                yield name
                yield from get_public_values(item, seen)


def call_angle_to_servo_output(axis: str, angle_deg: int):
    from savo_head.core import calibration

    assert hasattr(calibration, "angle_to_servo_output"), (
        "calibration module missing angle_to_servo_output()"
    )

    func = calibration.angle_to_servo_output
    signature = inspect.signature(func)
    param_count = len(signature.parameters)

    default_config = None
    if hasattr(calibration, "default_servo_calibration"):
        default_config = calibration.default_servo_calibration()

    if param_count == 2:
        return func(axis, angle_deg)

    if param_count == 3:
        return func(axis, angle_deg, default_config)

    raise AssertionError(
        f"Unsupported angle_to_servo_output() signature: {signature}"
    )


def servo_output_parts(output):
    if isinstance(output, dict):
        channel = output.get("pca9685_channel", output.get("channel"))
        pulse_us = output.get("pulse_us")
        ticks = output.get("ticks")
        angle_deg = output.get("angle_deg")
        return channel, pulse_us, ticks, angle_deg

    if isinstance(output, (tuple, list)):
        assert len(output) >= 3, f"Unexpected servo output tuple/list: {output}"
        channel = output[0]
        pulse_us = output[1]
        ticks = output[2]
        angle_deg = output[3] if len(output) > 3 else None
        return channel, pulse_us, ticks, angle_deg

    channel = getattr(output, "pca9685_channel", getattr(output, "channel", None))
    pulse_us = getattr(output, "pulse_us", None)
    ticks = getattr(output, "ticks", None)
    angle_deg = getattr(output, "angle_deg", None)

    return channel, pulse_us, ticks, angle_deg


def test_logical_channel_mapping_locked():
    from savo_head.core import calibration

    assert hasattr(calibration, "logical_channel_to_pca9685_channel")

    assert calibration.logical_channel_to_pca9685_channel("7") == 15
    assert calibration.logical_channel_to_pca9685_channel("6") == 14


def test_default_calibration_contains_robot_savo_values():
    from savo_head.core import calibration

    values = set(get_public_values(calibration))

    if hasattr(calibration, "default_servo_calibration"):
        values.update(get_public_values(calibration.default_servo_calibration()))

    text_values = {str(value) for value in values}

    for required in [
        "7",
        "6",
        "15",
        "14",
        "72",
        "55",
        "130",
    ]:
        assert required in text_values, f"Missing calibration value: {required}"


def test_center_servo_outputs_match_validated_values():
    pan = call_angle_to_servo_output("pan", 72)
    tilt = call_angle_to_servo_output("tilt", 55)

    pan_channel, pan_pulse_us, pan_ticks, pan_angle = servo_output_parts(pan)
    tilt_channel, tilt_pulse_us, tilt_ticks, tilt_angle = servo_output_parts(tilt)

    assert pan_channel == 15
    assert tilt_channel == 14

    if pan_angle is not None:
        assert int(pan_angle) == 72

    if tilt_angle is not None:
        assert int(tilt_angle) == 55

    assert math.isclose(float(pan_pulse_us), 1411.1111111111113, rel_tol=0.0, abs_tol=1e-6)
    assert int(pan_ticks) == 288

    assert math.isclose(float(tilt_pulse_us), 1222.2222222222222, rel_tol=0.0, abs_tol=1e-6)
    assert int(tilt_ticks) == 250


def test_servo_output_clamps_to_safe_limits():
    pan = call_angle_to_servo_output("pan", 999)
    tilt = call_angle_to_servo_output("tilt", -20)

    pan_channel, _, _, pan_angle = servo_output_parts(pan)
    tilt_channel, _, _, tilt_angle = servo_output_parts(tilt)

    assert pan_channel == 15
    assert tilt_channel == 14

    if pan_angle is not None:
        assert int(pan_angle) == 170

    if tilt_angle is not None:
        assert int(tilt_angle) == 45


if __name__ == "__main__":
    test_logical_channel_mapping_locked()
    test_default_calibration_contains_robot_savo_values()
    test_center_servo_outputs_match_validated_values()
    test_servo_output_clamps_to_safe_limits()
    print("PASS: savo_head Python servo calibration locks pan/tilt mapping and pulse outputs.")
