#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ultrasonic_test.py
HC-SR04-class ultrasonic tester for Robot Savo (Pi 5, Ubuntu 24.04)

Features:
- Precise echo pulse timing with perf_counter_ns()
- Temperature-aware speed of sound (331.3 + 0.606*T m/s)
- Median-of-N burst per reading to reject outliers
- CSV logging (--csv) with raw burst vectors
- Optional buzzer (GPIO 17) on near-field detections
- Health grading (PASS/CAUTION/FAIL) around ~0.28 m stop rule
- Clean timeouts, rate control

Author: Savo Copilot
"""

import argparse
import csv
import sys
import time
from statistics import median, mean, pstdev
from typing import Optional, List, Tuple

# -------------------- Defaults (Freenove mapping) --------------------
DEF_TRIG = 27          # Freenove: TRIG on BCM27
DEF_ECHO = 22          # Freenove: ECHO on BCM22
DEF_BUZZ = None        # set to 17 to beep on near hits
DEF_RATE = 10.0        # readings per second
DEF_BURST = 5          # samples per reading (median-of-N)
DEF_SAMPLES = 50       # number of readings (0 = run forever)
DEF_TEMP = 20.0        # ambient °C for speed-of-sound
DEF_THRESH = 0.28      # meters (near-field safety threshold)
DEF_TIMEOUT = 0.03     # seconds per edge wait
DEF_SETTLE = 0.05      # seconds settle before first ping
DEF_TRIG_PULSE = 10e-6 # seconds (10 μs HIGH)

# -------------------- Utilities --------------------
def speed_of_sound_mps(temp_c: float) -> float:
    return 331.3 + 0.606 * temp_c

def now_ns() -> int:
    return time.perf_counter_ns()

# -------------------- GPIO Backend Abstraction --------------------
class GpioBackend:
    def setup_output(self, pin: int): ...
    def setup_input(self, pin: int): ...
    def write(self, pin: int, level: int): ...
    def read(self, pin: int) -> int: ...
    def close(self): ...

# ---- Backend: lgpio ----
class LgpioBackend(GpioBackend):
    def __init__(self, chip_index: int = 0):
        import lgpio  # type: ignore
        self._lgpio = lgpio
        self._h = lgpio.gpiochip_open(chip_index)
        self._outs = set(); self._ins = set()

    def setup_output(self, pin: int):
        self._lgpio.gpio_claim_output(self._h, pin, 0)
        self._outs.add(pin)

    def setup_input(self, pin: int):
        self._lgpio.gpio_claim_input(self._h, pin)
        self._ins.add(pin)

    def write(self, pin: int, level: int):
        self._lgpio.gpio_write(self._h, pin, 1 if level else 0)

    def read(self, pin: int) -> int:
        return 1 if self._lgpio.gpio_read(self._h, pin) else 0

    def close(self):
        try:
            for p in list(self._outs):
                self._lgpio.gpio_write(self._h, p, 0)
        except Exception:
            pass
        try:
            self._lgpio.gpiochip_close(self._h)
        except Exception:
            pass

# ---- Backend: libgpiod (gpiod) ----
class GpiodBackend(GpioBackend):
    def __init__(self, chip_name: str = "gpiochip0"):
        import gpiod  # type: ignore
        self._gpiod = gpiod
        self._chip = gpiod.Chip(chip_name)
        self._outs = {}; self._ins = {}

    def setup_output(self, pin: int):
        line = self._chip.get_line(pin)
        line.request(consumer="ultra_out", type=self._gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self._outs[pin] = line

    def setup_input(self, pin: int):
        line = self._chip.get_line(pin)
        line.request(consumer="ultra_in", type=self._gpiod.LINE_REQ_DIR_IN)
        self._ins[pin] = line

    def write(self, pin: int, level: int):
        self._outs[pin].set_value(1 if level else 0)

    def read(self, pin: int) -> int:
        return 1 if self._ins[pin].get_value() else 0

    def close(self):
        for pin, line in list(self._outs.items()):
            try: line.set_value(0)
            except Exception: pass
            try: line.release()
            except Exception: pass
        for pin, line in list(self._ins.items()):
            try: line.release()
            except Exception: pass
        try: self._chip.close()
        except Exception: pass

def pick_backend(prefer: Optional[str] = None) -> GpioBackend:
    """
    prefer: 'lgpio' | 'gpiod' | None (auto)
    """
    if prefer == "lgpio":
        import importlib
        if importlib.util.find_spec("lgpio"):
            return LgpioBackend()
        raise RuntimeError("Requested backend 'lgpio' not available.")
    if prefer == "gpiod":
        import importlib
        if importlib.util.find_spec("gpiod"):
            return GpiodBackend()
        raise RuntimeError("Requested backend 'gpiod' not available.")
    # Auto: try lgpio → gpiod
    try: return LgpioBackend()
    except Exception: pass
    try: return GpiodBackend()
    except Exception: pass
    raise RuntimeError("No usable GPIO backend found (install python3-lgpio or python3-libgpiod).")

# -------------------- Timing helpers --------------------
def busy_wait_level(gpio: GpioBackend, pin: int, level: int, timeout_s: float) -> bool:
    t0 = now_ns(); limit = int(timeout_s * 1e9)
    while (now_ns() - t0) < limit:
        if gpio.read(pin) == level:
            return True
    return False

def pulse_width_s(gpio: GpioBackend, trig: int, echo: int, trig_pulse_s: float, timeout_s: float) -> Optional[float]:
    """
    Emit a TRIG pulse and measure echo HIGH width in seconds.
    Returns None on timeout or invalid width.
    """
    gpio.write(trig, 0)
    time.sleep(2e-6)
    # 10 µs HIGH pulse (busy-wait for precision)
    gpio.write(trig, 1)
    t0 = now_ns()
    while (now_ns() - t0) < int(trig_pulse_s * 1e9):
        pass
    gpio.write(trig, 0)

    if not busy_wait_level(gpio, echo, 1, timeout_s):
        return None
    t_rise = now_ns()
    if not busy_wait_level(gpio, echo, 0, timeout_s):
        return None
    t_fall = now_ns()
    width = (t_fall - t_rise) / 1e9
    return width if width > 0 else None

def measure_distance_m(gpio: GpioBackend, trig: int, echo: int, temp_c: float,
                       trig_pulse_s: float, timeout_s: float) -> Optional[Tuple[float, float]]:
    w = pulse_width_s(gpio, trig, echo, trig_pulse_s, timeout_s)
    if w is None:
        return None
    c = speed_of_sound_mps(temp_c)
    d = 0.5 * c * w  # out-and-back
    return (d, w)

def median_burst(gpio: GpioBackend, trig: int, echo: int, temp_c: float,
                 burst: int, trig_pulse_s: float, timeout_s: float,
                 inter_sample_delay_s: float = 0.01) -> Optional[Tuple[float, float, List[float], List[float]]]:
    ds: List[float] = []; ws: List[float] = []
    for _ in range(burst):
        r = measure_distance_m(gpio, trig, echo, temp_c, trig_pulse_s, timeout_s)
        if r is not None:
            d, w = r; ds.append(d); ws.append(w)
        time.sleep(inter_sample_delay_s)
    if not ds:
        return None
    return (median(ds), median(ws), ds, ws)

def grade_health(distances: List[float], thresh_m: float) -> Tuple[str, str]:
    if not distances:
        return ("FAIL", "No valid readings.")
    dmin, dmax = min(distances), max(distances)
    dmean = mean(distances)
    djit = pstdev(distances) if len(distances) > 1 else 0.0
    valid = (0.02 <= dmean <= 4.0)
    near_hits = sum(1 for d in distances if d <= thresh_m)
    if valid and djit <= 0.01: grade = "PASS"
    elif valid and djit <= 0.03: grade = "CAUTION"
    else: grade = "FAIL"
    msg = (f"range=[{dmin:.3f},{dmax:.3f}] m, mean={dmean:.3f} m, jitter={djit:.3f} m, "
           f"near({thresh_m:.2f} m) hits={near_hits}/{len(distances)}")
    return grade, msg

# -------------------- Main --------------------
def main():
    ap = argparse.ArgumentParser(description="Robot Savo ultrasonic tester (lgpio/libgpiod auto)")
    ap.add_argument("--trig", type=int, default=DEF_TRIG, help="TRIG BCM GPIO (default 27)")
    ap.add_argument("--echo", type=int, default=DEF_ECHO, help="ECHO BCM GPIO (default 22)")
    ap.add_argument("--buzzer", type=int, default=DEF_BUZZ, help="Optional buzzer BCM GPIO (e.g., 17)")
    ap.add_argument("--rate", type=float, default=DEF_RATE, help="Readings per second")
    ap.add_argument("--burst", type=int, default=DEF_BURST, help="Samples per reading (median-of-N)")
    ap.add_argument("--samples", type=int, default=DEF_SAMPLES, help="Number of readings (0=continuous)")
    ap.add_argument("--temp", type=float, default=DEF_TEMP, help="Ambient temperature in °C")
    ap.add_argument("--thresh", type=float, default=DEF_THRESH, help="Near-field safety threshold (m)")
    ap.add_argument("--timeout", type=float, default=DEF_TIMEOUT, help="Per-edge timeout (s)")
    ap.add_argument("--csv", type=str, default=None, help="CSV log path")
    ap.add_argument("--tag", type=str, default=None, help="Optional tag to include in CSV rows")
    ap.add_argument("--backend", type=str, choices=["auto", "lgpio", "gpiod"], default="auto",
                    help="GPIO backend (default auto)")
    ap.add_argument("--quiet", action="store_true", help="Reduce per-reading prints")
    args = ap.parse_args()

    # Select backend
    try:
        gpio = pick_backend(None if args.backend == "auto" else args.backend)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(2)

    # Claim lines
    try:
        gpio.setup_output(args.trig)
    except Exception as e:
        print(f"ERROR: TRIG={args.trig} output setup failed: {e}", file=sys.stderr)
        gpio.close(); sys.exit(3)
    try:
        gpio.setup_input(args.echo)
    except Exception as e:
        print(f"ERROR: ECHO={args.echo} input setup failed: {e}", file=sys.stderr)
        gpio.close(); sys.exit(4)

    buz = None
    if args.buzzer is not None:
        try:
            gpio.setup_output(args.buzzer); buz = args.buzzer
        except Exception as e:
            print(f"WARNING: buzzer setup failed ({e}). Continuing without buzzer.", file=sys.stderr)
            buz = None

    # CSV
    writer = None; fcsv = None
    if args.csv:
        try:
            fcsv = open(args.csv, "w", newline="")
            writer = csv.writer(fcsv)
            writer.writerow(["ts_iso", "tag", "median_m", "mean_m", "std_m",
                             "burst_valid", "burst_samples_m", "burst_pulses_s"])
        except Exception as e:
            print(f"WARNING: cannot open CSV '{args.csv}': {e}", file=sys.stderr)

    # Header
    print(f"[Ultrasonic] backend={gpio.__class__.__name__}  TRIG={args.trig}  ECHO={args.echo}"
          + (f"  BUZZER={buz}" if buz is not None else "")
          + f"  rate={args.rate:.2f} Hz  burst={args.burst}  temp={args.temp:.1f}°C  thresh={args.thresh:.2f} m")
    print(f"Speed of sound: {speed_of_sound_mps(args.temp):.2f} m/s")
    print("Note: Sensor is powered from Freenove HAT @5V; GPIO levels are board-buffered to 3.3V.\n")

    time.sleep(DEF_SETTLE)
    period = 1.0 / max(1e-3, args.rate)

    all_dist: List[float] = []
    n = 0
    try:
        while True:
            t0 = time.perf_counter()
            r = median_burst(gpio, args.trig, args.echo, args.temp,
                             args.burst, DEF_TRIG_PULSE, args.timeout)
            ts_iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

            if r is None:
                if not args.quiet:
                    print(f"{ts_iso}  median:  ---  (timeouts)")
                if writer:
                    writer.writerow([ts_iso, args.tag or "", "", "", "", 0, "[]", "[]"])
            else:
                d_med, w_med, ds, ws = r
                d_mean = mean(ds)
                d_std = pstdev(ds) if len(ds) > 1 else 0.0
                all_dist.append(d_med)

                if not args.quiet:
                    print(f"{ts_iso}  median={d_med:.3f} m  (mean={d_mean:.3f} σ={d_std:.3f})"
                          f"  burst={len(ds)}/{args.burst}")

                if writer:
                    writer.writerow([
                        ts_iso, args.tag or "",
                        f"{d_med:.6f}", f"{d_mean:.6f}", f"{d_std:.6f}",
                        len(ds),
                        "[" + ",".join(f"{x:.6f}" for x in ds) + "]",
                        "[" + ",".join(f"{x:.6e}" for x in ws) + "]"
                    ])

                if buz is not None and d_med <= args.thresh:
                    for _ in range(2):
                        gpio.write(buz, 1); time.sleep(0.04)
                        gpio.write(buz, 0); time.sleep(0.06)

                n += 1

            if args.samples > 0 and n >= args.samples:
                break

            # rate control
            dt = time.perf_counter() - t0
            sleep_left = period - dt
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Health summary
        grade, detail = grade_health(all_dist, args.thresh)
        if all_dist:
            print(f"\nHealth: {grade} | {detail}")
            print("Rules: PASS (jitter ≤ 0.01 m & plausible), CAUTION (≤ 0.03 m), else FAIL.")
        else:
            print("\nHealth: FAIL | No valid readings collected.")

        # Cleanup
        try:
            if buz is not None:
                gpio.write(buz, 0)
            gpio.write(args.trig, 0)
        except Exception:
            pass
        gpio.close()
        if fcsv:
            fcsv.close()

if __name__ == "__main__":
    main()
