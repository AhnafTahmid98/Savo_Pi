#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HC-SR04 helper built on gpiozero with graceful import fallback."""

from typing import Optional, Tuple
import time
from statistics import mean, pstdev
try:
    from gpiozero import DistanceSensor, Device
    from gpiozero.pins.lgpio import LGPIOFactory
    try:
        from gpiozero.pins.pigpio import PiGPIOFactory  # optional
        _HAS_PIGPIO = True
    except Exception:
        PiGPIOFactory = None
        _HAS_PIGPIO = False
except Exception:
    DistanceSensor = None  # type: ignore
    Device = None          # type: ignore
    LGPIOFactory = None    # type: ignore
    PiGPIOFactory = None   # type: ignore
    _HAS_PIGPIO = False

def _choose_factory(factory: str = "lgpio") -> None:
    """Use pigpio when available, otherwise fall back to lgpio."""
    if Device is None:
        return
    if factory == "pigpio" and _HAS_PIGPIO and PiGPIOFactory is not None:
        try:
            Device.pin_factory = PiGPIOFactory()   # raises if pigpiod not running
            return
        except Exception:
            pass  # fall through to lgpio
    if LGPIOFactory is not None:
        Device.pin_factory = LGPIOFactory()


def _make_sensor(trig_pin: int, echo_pin: int, max_m: float, queue_len: int = 1) -> Optional["DistanceSensor"]:
    """Create a DistanceSensor or return None if unavailable."""
    if DistanceSensor is None:
        return None
    return DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=max_m, queue_len=queue_len)

def read_ultrasonic_cm(
    trig_pin: int = 27,
    echo_pin: int = 22,
    *,
    factory: str = "lgpio",
    max_distance_m: float = 3.0,
    samples: int = 5,
    per_sample_timeout_s: float = 0.08,
    sample_period_s: float = 0.02,
) -> Optional[float]:
    """Return averaged centimeters, or None when no valid echo is available."""
    _choose_factory(factory)
    sen = _make_sensor(trig_pin, echo_pin, max_m=max_distance_m, queue_len=1)
    if sen is None:
        return None

    vals = []
    try:
        t_start = time.time()
        target = max(1, samples)
        while len(vals) < target and (time.time() - t_start) < (target * per_sample_timeout_s):
            try:
                d_m = float(sen.distance)  # meters (0..max_distance_m); inf on no-echo
            except (RuntimeWarning, ValueError):
                d_m = float("inf")

            if d_m != float("inf"):
                vals.append(d_m * 100.0)  # → centimeters

            time.sleep(sample_period_s)

        if not vals:
            return None

        # Clamp to a sane ultrasonic range
        cm = max(2.0, min(400.0, mean(vals)))
        return cm
    finally:
        try:
            sen.close()
        except Exception:
            pass


def read_ultrasonic_stats(
    trig_pin: int = 27,
    echo_pin: int = 22,
    *,
    factory: str = "lgpio",
    max_distance_m: float = 3.0,
    samples: int = 20,
    per_sample_timeout_s: float = 0.08,
    sample_period_s: float = 0.02,
) -> Tuple[bool, Optional[float], Optional[float]]:
    """Return validity, mean distance, and standard deviation."""
    _choose_factory(factory)
    sen = _make_sensor(trig_pin, echo_pin, max_m=max_distance_m, queue_len=1)
    if sen is None:
        return False, None, None

    vals = []
    try:
        t_start = time.time()
        target = max(1, samples)
        while len(vals) < target and (time.time() - t_start) < (target * per_sample_timeout_s):
            try:
                d_m = float(sen.distance)
            except (RuntimeWarning, ValueError):
                d_m = float("inf")

            if d_m != float("inf"):
                vals.append(d_m * 100.0)
            time.sleep(sample_period_s)

        if not vals:
            return False, None, None

        m = max(2.0, min(400.0, mean(vals)))
        s = pstdev(vals) if len(vals) > 1 else 0.0
        return True, m, s
    finally:
        try:
            sen.close()
        except Exception:
            pass


def ultrasonic_available() -> bool:
    """True if gpiozero is present and at least one pin factory is usable."""
    return DistanceSensor is not None and (LGPIOFactory is not None or (_HAS_PIGPIO and PiGPIOFactory is not None))

if __name__ == "__main__":
    import argparse, sys, warnings
    try:
        from gpiozero.exc import DistanceSensorNoEcho, PWMSoftwareFallback  # type: ignore
    except Exception:
        DistanceSensorNoEcho = RuntimeWarning  # fallback
        PWMSoftwareFallback = RuntimeWarning   # fallback

    ap = argparse.ArgumentParser(
        description="Robot Savo — Ultrasonic API quick check (prints cm values using the import-safe API)."
    )
    ap.add_argument("--trig", type=int, default=27, help="BCM TRIG pin (default: 27)")
    ap.add_argument("--echo", type=int, default=22, help="BCM ECHO pin (default: 22)")
    ap.add_argument("--factory", choices=["lgpio", "pigpio"], default="lgpio",
                    help="gpiozero pin factory (default: lgpio)")
    ap.add_argument("--samples", type=int, default=5,
                    help="samples per reading to average (default: 5)")
    ap.add_argument("--repeat", type=int, default=10,
                    help="how many readings to print (default: 10; 0 = run forever)")
    ap.add_argument("--rate", type=float, default=5.0,
                    help="prints per second (default: 5.0)")
    ap.add_argument("--stats", action="store_true",
                    help="print one mean/std burst and exit")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress gpiozero warnings in CLI (no-echo, PWM fallback)")
    args = ap.parse_args()

    if args.quiet:
        warnings.filterwarnings("ignore", category=DistanceSensorNoEcho)
        warnings.filterwarnings("ignore", category=PWMSoftwareFallback)

    if args.stats:
        ok, mean_cm, std_cm = read_ultrasonic_stats(
            trig_pin=args.trig, echo_pin=args.echo, factory=args.factory, samples=max(5, args.samples)
        )
        if not ok:
            print("No echoes (None). Check wiring/power or try swapping pins.", file=sys.stderr)
            sys.exit(2)
        print(f"mean={mean_cm:.1f} cm  std={std_cm:.1f} cm")
        sys.exit(0)
    period = 1.0 / max(0.5, args.rate)
    i = 0
    try:
        while True:
            d = read_ultrasonic_cm(
                trig_pin=args.trig, echo_pin=args.echo,
                factory=args.factory, samples=max(1, args.samples)
            )
            ts = time.strftime("%H:%M:%S")
            print(f"{ts}  {('%.1f cm' % d) if d is not None else 'None'}")
            i += 1
            if args.repeat > 0 and i >= args.repeat:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
