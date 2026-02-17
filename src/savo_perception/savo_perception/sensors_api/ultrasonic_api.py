#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Ultrasonic API (ROS2-ready) (HC-SR04 via gpiozero)
---------------------------------------------------------------
Location (canonical):
  savo_ws/src/savo_perception/savo_perception/sensors_api/ultrasonic_api.py

Use in ROS nodes:
  from savo_perception.sensors_api.ultrasonic_api import UltrasonicReader

  reader = UltrasonicReader(trig_pin=27, echo_pin=22, factory="lgpio")
  d_m = reader.read_m(samples=3)   # meters or None
  reader.close()

Notes:
- Designed for ROS2 loops: keeps the DistanceSensor open (no re-create per read).
- Publishes should be in meters (topic contract uses meters).
- TRIG/ECHO defaults are locked to Robot Savo: TRIG=27, ECHO=22.
"""

from __future__ import annotations

from typing import Optional, Tuple
import time
from statistics import mean, pstdev

# gpiozero + pin factories (graceful fallback)
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
    """
    Select gpiozero pin factory.
    - If 'pigpio' requested but daemon is not running, falls back to lgpio.
    """
    if Device is None:
        return

    if factory == "pigpio" and _HAS_PIGPIO and PiGPIOFactory is not None:
        try:
            Device.pin_factory = PiGPIOFactory()  # raises if pigpiod not running
            return
        except Exception:
            pass

    if LGPIOFactory is not None:
        Device.pin_factory = LGPIOFactory()


def ultrasonic_available() -> bool:
    """True if gpiozero is present and at least one pin factory is usable."""
    return DistanceSensor is not None and (LGPIOFactory is not None or (_HAS_PIGPIO and PiGPIOFactory is not None))


class UltrasonicReader:
    """
    Persistent ultrasonic reader suitable for ROS2 nodes.

    Keeps DistanceSensor open; use read_m() periodically in your node loop.
    """

    def __init__(
        self,
        trig_pin: int = 27,
        echo_pin: int = 22,
        *,
        factory: str = "lgpio",          # "lgpio" | "pigpio"
        max_distance_m: float = 3.0,     # gpiozero max_distance
        queue_len: int = 1,              # internal gpiozero smoothing (keep 1; we handle our own)
        clamp_min_m: float = 0.02,       # reject below this
        clamp_max_m: float = 4.00,       # reject above this (HC-SR04 practical max ~4m)
    ) -> None:
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.factory = factory
        self.max_distance_m = max_distance_m
        self.queue_len = queue_len
        self.clamp_min_m = clamp_min_m
        self.clamp_max_m = clamp_max_m

        self._sensor = None
        self._init_sensor()

    def _init_sensor(self) -> None:
        if DistanceSensor is None:
            self._sensor = None
            return
        _choose_factory(self.factory)
        self._sensor = DistanceSensor(
            echo=self.echo_pin,
            trigger=self.trig_pin,
            max_distance=self.max_distance_m,
            queue_len=self.queue_len,
        )

    def close(self) -> None:
        """Close the underlying gpiozero sensor."""
        try:
            if self._sensor is not None:
                self._sensor.close()
        except Exception:
            pass
        self._sensor = None

    def read_m(
        self,
        *,
        samples: int = 3,
        per_sample_timeout_s: float = 0.06,
        sample_period_s: float = 0.02,
    ) -> Optional[float]:
        """
        Return averaged distance in **meters**, or None on failure/no-echo.

        Args:
          samples: number of quick samples to average (>=1)
          per_sample_timeout_s: time budget per sample window
          sample_period_s: delay between reads

        Returns:
          meters (float) or None
        """
        if self._sensor is None:
            return None

        vals = []
        target = max(1, int(samples))
        t_start = time.time()

        # Keep time bounded so ROS loops stay responsive
        while len(vals) < target and (time.time() - t_start) < (target * per_sample_timeout_s):
            try:
                d_m = float(self._sensor.distance)  # 0..max_distance_m or inf-ish on no-echo
            except (RuntimeWarning, ValueError):
                d_m = float("inf")

            if d_m != float("inf"):
                # Clamp to sane ultrasonic range
                if self.clamp_min_m <= d_m <= self.clamp_max_m:
                    vals.append(d_m)

            time.sleep(sample_period_s)

        if not vals:
            return None

        return float(mean(vals))

    def read_stats_m(
        self,
        *,
        samples: int = 20,
        per_sample_timeout_s: float = 0.06,
        sample_period_s: float = 0.02,
    ) -> Tuple[bool, Optional[float], Optional[float]]:
        """
        Take N samples and return (ok, mean_m, std_m). ok=False if no valid echo.
        """
        if self._sensor is None:
            return False, None, None

        vals = []
        target = max(1, int(samples))
        t_start = time.time()

        while len(vals) < target and (time.time() - t_start) < (target * per_sample_timeout_s):
            try:
                d_m = float(self._sensor.distance)
            except (RuntimeWarning, ValueError):
                d_m = float("inf")

            if d_m != float("inf"):
                if self.clamp_min_m <= d_m <= self.clamp_max_m:
                    vals.append(d_m)
            time.sleep(sample_period_s)

        if not vals:
            return False, None, None

        m = float(mean(vals))
        s = float(pstdev(vals)) if len(vals) > 1 else 0.0
        return True, m, s


# ------------------------------- CLI probe ---------------------------------

if __name__ == "__main__":
    import argparse
    import sys
    import warnings

    # allow suppressing common gpiozero warnings in CLI mode
    try:
        from gpiozero.exc import DistanceSensorNoEcho, PWMSoftwareFallback  # type: ignore
    except Exception:
        DistanceSensorNoEcho = RuntimeWarning  # fallback
        PWMSoftwareFallback = RuntimeWarning   # fallback

    ap = argparse.ArgumentParser(
        description="Robot Savo — Ultrasonic API (ROS2-ready) quick check (prints meters)."
    )
    ap.add_argument("--trig", type=int, default=27, help="BCM TRIG pin (default: 27)")
    ap.add_argument("--echo", type=int, default=22, help="BCM ECHO pin (default: 22)")
    ap.add_argument("--factory", choices=["lgpio", "pigpio"], default="lgpio",
                    help="gpiozero pin factory (default: lgpio)")
    ap.add_argument("--samples", type=int, default=3,
                    help="samples per reading to average (default: 3)")
    ap.add_argument("--repeat", type=int, default=20,
                    help="how many readings to print (default: 20; 0 = run forever)")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="prints per second (default: 10.0)")
    ap.add_argument("--stats", action="store_true",
                    help="print one mean/std burst and exit")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress gpiozero warnings in CLI (no-echo, PWM fallback)")
    args = ap.parse_args()

    if args.quiet:
        warnings.filterwarnings("ignore", category=DistanceSensorNoEcho)
        warnings.filterwarnings("ignore", category=PWMSoftwareFallback)

    if not ultrasonic_available():
        print("Ultrasonic not available: gpiozero or pin factory missing.", file=sys.stderr)
        sys.exit(2)

    reader = UltrasonicReader(trig_pin=args.trig, echo_pin=args.echo, factory=args.factory)
    try:
        if args.stats:
            ok, mean_m, std_m = reader.read_stats_m(samples=max(5, args.samples))
            if not ok:
                print("No echoes (None). Check wiring/power or try swapping pins.", file=sys.stderr)
                sys.exit(2)
            print(f"mean={mean_m:.3f} m  std={std_m:.3f} m")
            sys.exit(0)

        period = 1.0 / max(0.5, args.rate)
        i = 0
        while True:
            d = reader.read_m(samples=max(1, args.samples))
            ts = time.strftime("%H:%M:%S")
            print(f"{ts}  {('%.3f m' % d) if d is not None else 'None'}")
            i += 1
            if args.repeat > 0 and i >= args.repeat:
                break
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
