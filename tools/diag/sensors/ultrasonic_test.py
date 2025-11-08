#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Ultrasonic tester (gpiozero + lgpio)
- Defaults: TRIG=27, ECHO=22 (Freenove HAT)
- Prints distances (cm), summary stats, optional CSV logging
- Uses gpiozero DistanceSensor which is known to work on your board

Install once:
  sudo apt update
  sudo apt install -y python3-gpiozero python3-lgpio
  sudo usermod -aG gpio $USER && newgrp gpio
"""

import argparse, csv, sys, time
from statistics import mean, pstdev

# Prefer lgpio pin factory on Ubuntu 24.04
try:
    from gpiozero import DistanceSensor, Device
    from gpiozero.pins.lgpio import LGPIOFactory
except Exception as e:
    print("ERROR: need gpiozero + lgpio. Install:\n"
          "  sudo apt install -y python3-gpiozero python3-lgpio", file=sys.stderr)
    raise

def make_sensor(trig:int, echo:int, max_m:float, queue_len:int=1) -> DistanceSensor:
    Device.pin_factory = LGPIOFactory()  # stable on Ubuntu 24.04
    return DistanceSensor(echo=echo, trigger=trig, max_distance=max_m, queue_len=queue_len)

def main():
    ap = argparse.ArgumentParser(description="Robot Savo ultrasonic tester (gpiozero)")
    ap.add_argument("--trig", type=int, default=27, help="TRIG BCM pin (default 27)")
    ap.add_argument("--echo", type=int, default=22, help="ECHO BCM pin (default 22)")
    ap.add_argument("--max",  type=float, default=3.0, help="Max distance in meters (default 3.0)")
    ap.add_argument("--rate", type=float, default=5.0, help="Readings per second (default 5)")
    ap.add_argument("--samples", type=int, default=40, help="Number of samples (0=run forever)")
    ap.add_argument("--csv", type=str, default=None, help="CSV output path")
    ap.add_argument("--tag", type=str, default=None, help="Optional tag for CSV rows")
    args = ap.parse_args()

    period = 1.0 / max(1e-3, args.rate)
    sensor = make_sensor(args.trig, args.echo, args.max, queue_len=1)
    sensor.threshold_distance = 0.28  # m, informational

    writer = None
    fcsv = None
    if args.csv:
        fcsv = open(args.csv, "w", newline="")
        writer = csv.writer(fcsv)
        writer.writerow(["ts_iso", "tag", "distance_m"])

    print(f"[gpiozero] TRIG={args.trig}  ECHO={args.echo}  max={args.max:.2f} m  rate={args.rate:.2f} Hz")
    vals = []
    n = 0
    try:
        while True:
            ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
            try:
                d = float(sensor.distance)  # 0..max (meters)
            except RuntimeWarning:
                d = None

            if d is None or d == float("inf"):
                print(f"{ts}  distance: --- (no echo)")
                if writer: writer.writerow([ts, args.tag or "", ""])
            else:
                print(f"{ts}  distance: {d*100:.1f} cm")
                vals.append(d)
                if writer: writer.writerow([ts, args.tag or "", f"{d:.6f}"])

            n += 1
            if args.samples > 0 and n >= args.samples:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sensor.close()
        if fcsv: fcsv.close()
        if vals:
            m = mean(vals); s = pstdev(vals) if len(vals) > 1 else 0.0
            print(f"\nSummary: N={len(vals)}  mean={m*100:.1f} cm  std={s*100:.1f} cm")
        else:
            print("\nSummary: No valid echoes.")
            
if __name__ == "__main__":
    main()
