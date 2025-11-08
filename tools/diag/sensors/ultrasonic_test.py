#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Ultrasonic Tester (gpiozero)
-----------------------------------------
- Works with Freenove HAT (5V sensor, level-shifted to 3.3V).
- Defaults: TRIG=27, ECHO=22 (per your board).
- Uses gpiozero DistanceSensor (robust on this HAT).
- Pin factory selectable: lgpio (default) or pigpio (higher timing accuracy).
- Optional buzzer on BCM17 for near-field alert.

Author: Savo Copilot

"""

import argparse
import csv
import sys
import time
from statistics import mean, pstdev

# gpiozero + pin factories
try:
    from gpiozero import DistanceSensor, Device
    from gpiozero.pins.lgpio import LGPIOFactory
    try:
        from gpiozero.pins.pigpio import PiGPIOFactory
        _HAS_PIGPIO = True
    except Exception:
        _HAS_PIGPIO = False
except Exception as e:
    print("ERROR: gpiozero not available.\n"
          "Install: sudo apt install -y python3-gpiozero python3-lgpio", file=sys.stderr)
    raise

def choose_factory(name: str):
    """Select gpiozero pin factory."""
    if name == "pigpio":
        if not _HAS_PIGPIO:
            print("WARNING: pigpio factory requested but python3-pigpio not found; falling back to lgpio.", file=sys.stderr)
            Device.pin_factory = LGPIOFactory()
        else:
            Device.pin_factory = PiGPIOFactory()
    else:
        Device.pin_factory = LGPIOFactory()

def make_sensor(trig: int, echo: int, max_m: float, queue_len: int = 1) -> DistanceSensor:
    """
    queue_len=1 (no extra smoothing) → better for diagnostics.
    gpiozero returns distance in meters [0..max_distance], or inf for no echo.
    """
    return DistanceSensor(echo=echo, trigger=trig, max_distance=max_m, queue_len=queue_len)

def maybe_beep(buzzer_pin: int | None, near: bool, last_state: list):
    """Edge-triggered double beep when entering 'near' state. last_state is [prev_bool]."""
    if buzzer_pin is None:
        return
    # lazy import to avoid pulling RPi.GPIO/others; pigpio/lgpio already run pigpiod if needed
    try:
        import gpiozero
    except Exception:
        return
    if not hasattr(maybe_beep, "_bz"):
        maybe_beep._bz = gpiozero.LED(buzzer_pin)  # simple on/off is fine for a piezo/buzzer
    bz = maybe_beep._bz
    if near and not last_state[0]:
        # two short beeps
        for _ in range(2):
            bz.on(); time.sleep(0.05); bz.off(); time.sleep(0.06)
    last_state[0] = near

def main():
    ap = argparse.ArgumentParser(description="Robot Savo ultrasonic tester (gpiozero)")
    ap.add_argument("--trig", type=int, default=27, help="TRIG BCM pin (default 27)")
    ap.add_argument("--echo", type=int, default=22, help="ECHO BCM pin (default 22)")
    ap.add_argument("--max",  type=float, default=3.0, help="Max distance in meters (default 3.0)")
    ap.add_argument("--rate", type=float, default=5.0, help="Readings per second (default 5)")
    ap.add_argument("--samples", type=int, default=40, help="Number of samples (0=run forever)")
    ap.add_argument("--thresh", type=float, default=0.28, help="Near-field threshold in meters (default 0.28)")
    ap.add_argument("--buzzer", type=int, default=None, help="Optional buzzer BCM pin (e.g., 17)")
    ap.add_argument("--csv", type=str, default=None, help="CSV output path")
    ap.add_argument("--tag", type=str, default=None, help="Optional tag for CSV rows")
    ap.add_argument("--factory", choices=["lgpio", "pigpio"], default="lgpio", help="Pin factory (default lgpio)")
    args = ap.parse_args()

    # Pin factory
    choose_factory(args.factory)

    # Create sensor
    sensor = make_sensor(args.trig, args.echo, args.max, queue_len=1)
    sensor.threshold_distance = args.thresh  # informational (not used for events here)

    # CSV setup
    writer = None; fcsv = None
    if args.csv:
        try:
            fcsv = open(args.csv, "w", newline="")
            writer = csv.writer(fcsv)
            writer.writerow(["ts_iso", "tag", "distance_m"])
        except Exception as e:
            print(f"WARNING: cannot open CSV '{args.csv}': {e}", file=sys.stderr)
            writer = None

    period = 1.0 / max(1e-3, args.rate)
    values = []
    near_last = [False]

    print(f"[gpiozero:{args.factory}] TRIG={args.trig}  ECHO={args.echo}  max={args.max:.2f} m  "
          f"rate={args.rate:.2f} Hz  thresh={args.thresh:.2f} m"
          + (f"  buzzer={args.buzzer}" if args.buzzer is not None else ""))

    n = 0
    try:
        while True:
            t0 = time.time()
            ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
            try:
                d = float(sensor.distance)  # meters; inf on no-echo
            except RuntimeWarning:
                d = float("inf")

            if d == float("inf"):
                print(f"{ts}  distance: --- (no echo)")
                near = False
                if writer: writer.writerow([ts, args.tag or "", ""])
            else:
                cm = d * 100.0
                print(f"{ts}  distance: {cm:.1f} cm")
                values.append(d)
                near = (d <= args.thresh)
                if writer: writer.writerow([ts, args.tag or "", f"{d:.6f}"])

            maybe_beep(args.buzzer, near, near_last)

            n += 1
            if args.samples > 0 and n >= args.samples:
                break

            # rate control
            dt = time.time() - t0
            left = period - dt
            if left > 0:
                time.sleep(left)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            sensor.close()
        except Exception:
            pass
        if fcsv:
            fcsv.close()
        if values:
            m = mean(values)
            s = pstdev(values) if len(values) > 1 else 0.0
            print(f"\nSummary: N={len(values)}  mean={m*100:.1f} cm  std={s*100:.1f} cm")
        else:
            print("\nSummary: No valid echoes.")

if __name__ == "__main__":
    main()
