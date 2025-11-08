#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ultrasonic_test.py — Robot Savo
HC-SR04-class ultrasonic tester with AUTO echo-polarity detection.
Defaults for Freenove HAT: TRIG=27, ECHO=22.

"""
import argparse, csv, sys, time
from statistics import median, mean, pstdev
from typing import Optional, List

# ---------------- Defaults (Freenove mapping) ----------------
DEF_TRIG = 27
DEF_ECHO = 22
DEF_BUZZ = None
DEF_RATE = 10.0
DEF_BURST = 5
DEF_SAMPLES = 40
DEF_TEMP = 20.0
DEF_THRESH = 0.28
DEF_TIMEOUT = 0.08
DEF_SETTLE = 0.05
DEF_TRIG_PULSE = 10e-6

# ---------------- Helpers ----------------
def speed_of_sound_mps(temp_c: float) -> float:
    return 331.3 + 0.606 * temp_c

def now_ns() -> int:
    return time.perf_counter_ns()

# ---------------- GPIO backend abstraction ----------------
class GpioBackend:
    def setup_output(self, pin: int): ...
    def setup_input(self, pin: int): ...
    def write(self, pin: int, level: int): ...
    def read(self, pin: int) -> int: ...
    def close(self): ...

class LgpioBackend(GpioBackend):
    def __init__(self, chip_index: int = 0):
        import lgpio  # type: ignore
        self._lgpio = lgpio
        self._h = lgpio.gpiochip_open(chip_index)
        self._outs = set()
    def setup_output(self,p): self._lgpio.gpio_claim_output(self._h,p,0); self._outs.add(p)
    def setup_input(self,p): self._lgpio.gpio_claim_input(self._h,p)
    def write(self,p,l): self._lgpio.gpio_write(self._h,p,1 if l else 0)
    def read(self,p): return 1 if self._lgpio.gpio_read(self._h,p) else 0
    def close(self):
        try:
            for p in list(self._outs): self._lgpio.gpio_write(self._h,p,0)
        except Exception: pass
        try: self._lgpio.gpiochip_close(self._h)
        except Exception: pass

class GpiodBackend(GpioBackend):
    def __init__(self, chip_name: str = "gpiochip0"):
        import gpiod  # type: ignore
        self._gpiod = gpiod
        self._chip = gpiod.Chip(chip_name)
        self._outs = {}; self._ins = {}
    def setup_output(self,p):
        ln = self._chip.get_line(p)
        ln.request(consumer="ultra_out", type=self._gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self._outs[p] = ln
    def setup_input(self,p):
        ln = self._chip.get_line(p)
        ln.request(consumer="ultra_in", type=self._gpiod.LINE_REQ_DIR_IN)
        self._ins[p] = ln
    def write(self,p,l): self._outs[p].set_value(1 if l else 0)
    def read(self,p): return 1 if self._ins[p].get_value() else 0
    def close(self):
        for p,ln in list(self._outs.items()):
            try: ln.set_value(0)
            except Exception: pass
            try: ln.release()
            except Exception: pass
        for p,ln in list(self._ins.items()):
            try: ln.release()
            except Exception: pass
        try: self._chip.close()
        except Exception: pass

def pick_backend(prefer: Optional[str] = None) -> GpioBackend:
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
    try: return LgpioBackend()
    except Exception: pass
    try: return GpiodBackend()
    except Exception: pass
    raise RuntimeError("No usable GPIO backend found (install python3-lgpio or python3-libgpiod).")

# ---------------- Timing + auto-polarity ----------------
def wait_level(gpio: GpioBackend, pin: int, level: int, timeout_s: float) -> bool:
    t0 = now_ns(); limit = int(timeout_s * 1e9)
    while (now_ns() - t0) < limit:
        if gpio.read(pin) == level:
            return True
    return False

def pulse_width_auto(gpio: GpioBackend, trig: int, echo: int, trig_pulse_s: float, timeout_s: float) -> Optional[float]:
    # Observe idle level
    idle = gpio.read(echo)
    # Try to stabilize (short)
    wait_level(gpio, echo, idle, 0.002)
    # Trigger pulse
    gpio.write(trig, 0); time.sleep(2e-6)
    gpio.write(trig, 1)
    t0 = now_ns()
    while (now_ns() - t0) < int(trig_pulse_s * 1e9): pass
    gpio.write(trig, 0)
    active = 1 - idle
    # wait for start of active pulse
    if not wait_level(gpio, echo, active, timeout_s):
        return None
    t_rise = now_ns()
    # wait for return to idle
    if not wait_level(gpio, echo, idle, timeout_s):
        return None
    t_fall = now_ns()
    w = (t_fall - t_rise) / 1e9
    return w if w > 0 else None

def measure_distance_m(gpio: GpioBackend, trig:int, echo:int, temp_c:float, trig_pulse_s:float, timeout_s:float) -> Optional[tuple]:
    w = pulse_width_auto(gpio, trig, echo, trig_pulse_s, timeout_s)
    if w is None: return None
    c = speed_of_sound_mps(temp_c)
    d = 0.5 * c * w
    return d, w

def median_burst(gpio: GpioBackend, trig:int, echo:int, temp_c:float, burst:int, trig_pulse_s:float, timeout_s:float, inter_delay:float=0.01):
    ds=[]; ws=[]
    for _ in range(burst):
        r = measure_distance_m(gpio, trig, echo, temp_c, trig_pulse_s, timeout_s)
        if r is not None:
            d,w = r; ds.append(d); ws.append(w)
        time.sleep(inter_delay)
    if not ds: return None
    return median(ds), median(ws), ds, ws

def grade_health(distances: List[float], thresh_m: float):
    if not distances: return ("FAIL", "No valid readings.")
    dmin, dmax = min(distances), max(distances)
    dmean = mean(distances)
    djitter = pstdev(distances) if len(distances) > 1 else 0.0
    valid_range = (0.02 <= dmean <= 4.0)
    near_hits = sum(1 for d in distances if d <= thresh_m)
    if valid_range and djitter <= 0.01: grade="PASS"
    elif valid_range and djitter <= 0.03: grade="CAUTION"
    else: grade="FAIL"
    msg = f"range=[{dmin:.3f},{dmax:.3f}] m, mean={dmean:.3f} m, jitter={djitter:.3f} m, near({thresh_m:.2f}) hits={near_hits}/{len(distances)}"
    return grade, msg

# ---------------- Main ----------------
def main():
    ap = argparse.ArgumentParser(description="Robot Savo ultrasonic tester (auto polarity)")
    ap.add_argument("--trig", type=int, default=DEF_TRIG)
    ap.add_argument("--echo", type=int, default=DEF_ECHO)
    ap.add_argument("--buzzer", type=int, default=DEF_BUZZ)
    ap.add_argument("--rate", type=float, default=DEF_RATE)
    ap.add_argument("--burst", type=int, default=DEF_BURST)
    ap.add_argument("--samples", type=int, default=DEF_SAMPLES)
    ap.add_argument("--temp", type=float, default=DEF_TEMP)
    ap.add_argument("--thresh", type=float, default=DEF_THRESH)
    ap.add_argument("--timeout", type=float, default=DEF_TIMEOUT)
    ap.add_argument("--csv", type=str, default=None)
    ap.add_argument("--tag", type=str, default=None)
    ap.add_argument("--backend", choices=["auto","lgpio","gpiod"], default="auto")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        gpio = pick_backend(None if args.backend == "auto" else args.backend)
    except Exception as e:
        print(f"ERROR selecting backend: {e}", file=sys.stderr); sys.exit(2)

    try:
        gpio.setup_output(args.trig); gpio.setup_input(args.echo)
    except Exception as e:
        print(f"ERROR claiming pins: {e}", file=sys.stderr); gpio.close(); sys.exit(3)

    buz = None
    if args.buzzer is not None:
        try:
            gpio.setup_output(args.buzzer); buz = args.buzzer
        except Exception:
            buz = None

    writer = None; fcsv = None
    if args.csv:
        try:
            fcsv = open(args.csv, "w", newline=""); writer = csv.writer(fcsv)
            writer.writerow(["ts_iso","tag","median_m","mean_m","std_m","burst_valid","burst_samples_m","burst_pulses_s"])
        except Exception as e:
            print("CSV open failed:", e, file=sys.stderr); writer = None

    print(f"[Ultrasonic] backend={gpio.__class__.__name__}  TRIG={args.trig}  ECHO={args.echo}  rate={args.rate:.2f}Hz  burst={args.burst}  temp={args.temp:.1f}C  thresh={args.thresh:.2f}m")
    print("Note: Sensor powered from Freenove HAT @5V; signals buffered to 3.3V.\n")

    time.sleep(DEF_SETTLE)
    period = 1.0 / max(1e-3, args.rate)
    all_dist=[]; n=0

    try:
        while True:
            loop_t0 = time.perf_counter()
            r = median_burst(gpio, args.trig, args.echo, args.temp, args.burst, DEF_TRIG_PULSE, args.timeout)
            ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
            if r is None:
                if not args.quiet: print(f"{ts}  median: --- (timeouts)")
                if writer: writer.writerow([ts, args.tag or "", "", "", "", 0, "[]", "[]"])
            else:
                d_med, w_med, ds, ws = r
                d_mean = mean(ds); d_std = pstdev(ds) if len(ds) > 1 else 0.0
                all_dist.append(d_med)
                if not args.quiet:
                    print(f"{ts}  median={d_med*100:.1f} cm  mean={d_mean*100:.1f} cm  σ={d_std*100:.1f} cm  burst={len(ds)}/{args.burst}")
                if writer: writer.writerow([ts, args.tag or "", f"{d_med:.6f}", f"{d_mean:.6f}", f"{d_std:.6f}", len(ds), "[" + ",".join(f"{x:.6f}" for x in ds) + "]", "[" + ",".join(f"{x:.6e}" for x in ws) + "]"])
                if buz is not None and d_med <= args.thresh:
                    for _ in range(2):
                        gpio.write(buz, 1); time.sleep(0.04); gpio.write(buz, 0); time.sleep(0.06)
                n += 1

            if args.samples > 0 and n >= args.samples:
                break
            dt = time.perf_counter() - loop_t0
            sleep_left = period - dt
            if sleep_left > 0: time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        grade, detail = grade_health(all_dist, args.thresh)
        if all_dist:
            print(f"\nHealth: {grade} | {detail}")
        else:
            print("\nHealth: FAIL | No valid readings collected.")
        try:
            if buz is not None: gpio.write(buz, 0)
            gpio.write(args.trig, 0)
        except Exception: pass
        gpio.close()
        if fcsv: fcsv.close()

if __name__ == "__main__":
    main()
