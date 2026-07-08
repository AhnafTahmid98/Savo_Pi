import inspect
import math

import pytest

import savo_power.models.ups_reading as ups_model


def _call_word_function(func, raw_word):
    signature = inspect.signature(func)

    for key in ("raw_word", "word", "value", "raw"):
        if key in signature.parameters:
            return func(**{key: raw_word})

    if len(signature.parameters) == 1:
        return func(raw_word)

    raise TypeError(f"Cannot call {func.__name__} with one raw word")


def _candidate_functions(keyword):
    for name, value in sorted(vars(ups_model).items()):
        if name.startswith("_"):
            continue
        if not callable(value):
            continue
        if inspect.isclass(value):
            continue
        if keyword not in name.lower():
            continue
        yield name, value


def _find_converter(keyword, raw_word, expected, tolerance):
    errors = []

    for name, func in _candidate_functions(keyword):
        try:
            result = _call_word_function(func, raw_word)
        except Exception as exc:  # noqa: BLE001 - candidates may not be converters
            errors.append(f"{name}: {type(exc).__name__}")
            continue

        if isinstance(result, bool):
            continue

        if isinstance(result, (int, float)) and math.isclose(
            float(result),
            expected,
            rel_tol=0.0,
            abs_tol=tolerance,
        ):
            return name, func, float(result)

    raise AssertionError(
        f"No UPS {keyword} converter produced {expected}. "
        f"Checked candidates: {errors}"
    )


def test_ups_voltage_raw_word_formula_is_locked():
    name, _func, voltage_v = _find_converter(
        keyword="voltage",
        raw_word=0x00D2,
        expected=4.20,
        tolerance=0.02,
    )

    assert "voltage" in name.lower()
    assert math.isclose(voltage_v, 4.20, rel_tol=0.0, abs_tol=0.02)


def test_ups_capacity_raw_word_formula_is_locked():
    name, _func, capacity_pct = _find_converter(
        keyword="capacity",
        raw_word=0x0064,
        expected=100.0,
        tolerance=0.01,
    )

    assert "capacity" in name.lower()
    assert math.isclose(capacity_pct, 100.0, rel_tol=0.0, abs_tol=0.01)


def test_ups_reading_model_exists():
    assert hasattr(ups_model, "UpsReading")


def test_ups_error_factory_exists():
    assert hasattr(ups_model, "make_ups_error")


def test_ups_reading_factory_exists():
    factory_names = {
        name
        for name, value in vars(ups_model).items()
        if callable(value) and name.startswith("make_ups")
    }

    assert factory_names
    assert any("reading" in name for name in factory_names)


@pytest.mark.parametrize(
    "raw_word",
    [
        0x0000,
        0x00D2,
        0xFFFF,
    ],
)
def test_ups_voltage_converter_returns_finite_number(raw_word):
    _name, _func, value = _find_converter(
        keyword="voltage",
        raw_word=raw_word,
        expected={
            0x0000: 0.0,
            0x00D2: 4.20,
            0xFFFF: 5.119921875,
        }[raw_word],
        tolerance=0.03,
    )

    assert math.isfinite(value)
