#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Encoders Test (lgpio, counts + direction + m/s + omega, CSV)
Raspberry Pi 5 • Ubuntu 24.04 • lgpio-based polling quadrature counter

Features
- Auto-detects a usable gpiochip unless --chip is provided.
- Gray-code state machine (legal transitions only), counts illegal jumps.
- Debounce: tries lgpio hardware debounce if available, else software time-debounce.
- Direction flags: --invert-left / --invert-right (so “forward” is positive without rewiring).
- Reports every --interval seconds:
    • total counts and per-interval deltas
    • wheel speeds L_v, R_v (m/s)
    • robot v (m/s) and ω (rad/s) using differential model: v=(vR+vL)/2, ω=(vR−vL)/track
- Optional CSV logging with the same metrics.
- Clean Ctrl+C and resource release.

Author: Robot Savo

"""

import argparse
import csv
import sys
import time
import signal
import math

try:
    import lgpio
except Exception:
    print("ERROR: python3-lgpio is required. Install with: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# Valid Gray-code transitions (A:bit1, B:bit0)
DELTA = {
    (0b00,0b01): +1, (0b01,0b11): +1, (0b11,0b10): +1, (0b10,0b00): +1,
    (0b01,0b00): -1, (0b11,0b01): -1, (0b10,0b11): -1, (0b00,0b10): -1
}

def autodetect_chip(pins, start=0, end=7, pullup=True):
    """
    Try gpiochip[start..end] and return (chip_index, handle) that can claim all pins.
    """
    last_error = None
    for chip in range(start, end + 1):
        try:
            h = lgpio.gpiochip_open(chip)
            claimed = []
            try:
                for p in pins:
                    lgpio.gpio_claim_input(h, p)
                    claimed.append(p)
                    if pullup:
                        try:
                            lgpio.gpio_set_flags(h, p, lgpio.BIAS_PULL_UP)
                        except Exception:
                            pass
                for p in claimed:
                    lgpio.gpio_free(h, p)
                lgpio.gpiochip_close(h)
                return chip, lgpio.gpiochip_open(chip)
            except Exception as e:
                last_error = e
                for p in claimed:
                    try: lgpio.gpio_free(h, p)
                    except: pass
                lgpio.gpiochip_close(h)
        except Exception as e:
            last_error = e
    raise RuntimeError(f"Could not find a gpiochip that can claim pins {pins}. Last error: {last_error}")

class QuadSide:
    def __init__(self, h, a, b, debounce_s, invert=1, use_hw_debounce=True, pullup=True):
        self.h, self.a, self.b = h, a, b
        self.debounce_s = max(0.0, debounce_s)
        self.use_hw_debounce = use_hw_debounce
        self.pullup = pullup
        self.invert = 1 if invert >= 0 else -1

        # Claim inputs
        lgpio.gpio_claim_input(h, a)
        lgpio.gpio_claim_input(h, b)

        # Pull-ups (harmless if ignored by the driver)
        if pullup:
            try:
                lgpio.gpio_set_flags(h, a, lgpio.BIAS_PULL_UP)
                lgpio.gpio_set_flags(h, b, lgpio.BIAS_PULL_UP)
            except Exception:
                pass

        # Hardware debounce (if available)
        self.hw_debounce = False
        if use_hw_debounce and self.debounce_s > 0:
            try:
                usec = int(self.debounce_s * 1e6)
                lgpio.gpio_set_debounce_micros(h, a, usec)
                lgpio.gpio_set_debounce_micros(h, b, usec)
                self.hw_debounce = True
            except Exception:
                self.hw_debounce = False

        # Software debounce bookkeeping
        now = time.monotonic()
        self.last_change = {a: now, b: now}
        self.state_bits = {a: lgpio.gpio_read(h, a), b: lgpio.gpio_read(h, b)}
        self.prev_state = ((self.state_bits[a] & 1) << 1) | (self.state_bits[b] & 1)

        self.count = 0      # signed count (includes invert)
        self.dir = 0        # last step dir: -1, 0, +1
        self.illegal = 0    # illegal Gray-code jumps

    def sample_once(self, now):
        a_val = lgpio.gpio_read(self.h, self.a)
        b_val = lgpio.gpio_read(self.h, self.b)

        if not self.hw_debounce and self.debounce_s > 0:
            if a_val != self.state_bits[self.a]:
                if (now - self.last_change[self.a]) >= self.debounce_s:
                    self.state_bits[self.a] = a_val
                    self.last_change[self.a] = now
                else:
                    a_val = self.state_bits[self.a]
            else:
                self.last_change[self.a] = now

            if b_val != self.state_bits[self.b]:
                if (now - self.last_change[self.b]) >= self.debounce_s:
                    self.state_bits[self.b] = b_val
                    self.last_change[self.b] = now
                else:
                    b_val = self.state_bits[self.b]
            else:
                self.last_change[self.b] = now

        s_new = ((a_val & 1) << 1) | (b_val & 1)
        if s_new != self.prev_state:
            d = DELTA.get((self.prev_state, s_new), None)
            if d is None:
                self.illegal += 1
            else:
                d *= self.invert
                self.count += d
                self.dir = 1 if d > 0 else -1
            self.prev_state = s_new

def run(args):
    pins = [args.l_a, args.l_b, args.r_a, args.r_b]

    # Pick gpiochip
    if args.chip is not None:
        h = lgpio.gpiochip_open(args.chip)
        chip_used = args.chip
    else:
        chip_used, h = autodetect_chip(pins, pullup=not args.no_pullup)

    print(f"[Encoders] gpiochip{chip_used}  Pins L({args.l_a},{args.l_b}) R({args.r_a},{args.r_b})")
    print(f"[Debounce] target={int(args.debounce_s*1e6)} µs  polling={int(args.poll_s*1e6)} µs")
    print(f"[Kinematics] wheel_dia={args.wheel_dia:.3f} m  CPR={args.cpr}  decoding={args.decoding}x  gear={args.gear:.3f}  track={args.track:.3f} m")
    edges_per_rev = args.cpr * args.decoding
    counts_per_wrev = max(1, int(edges_per_rev * args.gear))

    # Build sides (apply invert flags so forward is positive)
    L = QuadSide(h, args.l_a, args.l_b, debounce_s=args.debounce_s,
                 invert=(-1 if args.invert_left else +1),
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)
    R = QuadSide(h, args.r_a, args.r_b, debounce_s=args.debounce_s,
                 invert=(-1 if args.invert_right else +1),
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)

    # CSV setup
    csvf = None
    writer = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow([
            "t_s",
            "L_count","R_count",
            "L_delta","R_delta",
            "L_dir","R_dir",
            "L_v_mps","R_v_mps",
            "v_mps","omega_rad_s",
            "L_illegal","R_illegal"
        ])

    stop = False
    def on_sigint(sig, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    t0 = time.monotonic()
    last_print = t0
    lastL = L.count
    lastR = R.count

    # Pretty header
    print("\n t(s) |   L_cnt    R_cnt | dL  dR |  L_v   R_v  (m/s) |    v     ω(rad/s) | L_illeg R_illeg | dirL dirR")
    print("---------------------------------------------------------------------------------------------------------")

    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0 and (now - t0) >= args.duration:
                break

            # Tight polling for resolution
            L.sample_once(now)
            R.sample_once(now)

            # Emit at fixed intervals
            if (now - last_print) >= args.interval:
                dt = now - last_print
                dL = L.count - lastL
                dR = R.count - lastR
                lastL, lastR = L.count, R.count
                last_print = now

                # wheel linear speeds (m/s)
                # counts/sec -> rev/sec -> m/s
                L_v = (dL / max(1e-6, dt)) / counts_per_wrev * (math.pi * args.wheel_dia)
                R_v = (dR / max(1e-6, dt)) / counts_per_wrev * (math.pi * args.wheel_dia)

                # differential model (diagnostic)
                v = 0.5 * (L_v + R_v)
                omega = (R_v - L_v) / max(1e-9, args.track)

                dirL = "→" if L.dir > 0 else ("←" if L.dir < 0 else "•")
                dirR = "→" if R.dir > 0 else ("←" if R.dir < 0 else "•")

                print(f"{now - t0:5.1f} | {L.count:7d} {R.count:7d} | {dL:3d} {dR:3d} | "
                      f"{L_v: .3f} {R_v: .3f} | {v: .3f}  {omega: .3f}   | "
                      f"{L.illegal:7d} {R.illegal:7d} |  {dirL:^3s}  {dirR:^3s}")

                if writer:
                    writer.writerow([
                        f"{now - t0:.3f}",
                        L.count, R.count,
                        dL, dR,
                        L.dir, R.dir,
                        f"{L_v:.6f}", f"{R_v:.6f}",
                        f"{v:.6f}", f"{omega:.6f}",
                        L.illegal, R.illegal
                    ])

            time.sleep(args.poll_s)

    finally:
        # Cleanup
        for p in (args.l_a, args.l_b, args.r_a, args.r_b):
            try: lgpio.gpio_free(h, p)
            except: pass
        try: lgpio.gpiochip_close(h)
        except: pass
        if csvf: csvf.close()

    # Summary
    T = time.monotonic() - t0
    print("\n=== Summary ===")
    print(f"Duration: {T:.2f} s")
    print(f"L total: {L.count}   R total: {R.count}")
    print(f"L_illegal={L.illegal}  R_illegal={R.illegal}")
    if T > 0:
        print(f"Avg L rate: {L.count/T:.2f} counts/s   Avg R rate: {R.count/T:.2f} counts/s")
    print("Tip: If forward looks negative, add --invert-left and/or --invert-right.")

def parse_args():
    ap = argparse.ArgumentParser(
        description="Polling quadrature encoder test with speeds and omega (Pi 5 / lgpio)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # Pins
    ap.add_argument("--l-a", type=int, default=21, help="Left encoder A (BCM)")
    ap.add_argument("--l-b", type=int, default=20, help="Left encoder B (BCM)")
    ap.add_argument("--r-a", type=int, default=12, help="Right encoder A (BCM)")
    ap.add_argument("--r-b", type=int, default=26, help="Right encoder B (BCM)")

    # Direction
    ap.add_argument("--invert-left", action="store_true", help="Invert left wheel direction (make forward positive)")
    ap.add_argument("--invert-right", action="store_true", help="Invert right wheel direction (make forward positive)")

    # Timing / debounce
    ap.add_argument("--poll-s", type=float, default=0.001, help="Polling period (s)")
    ap.add_argument("--debounce-s", type=float, default=0.0003, help="Per-line debounce (s)")
    ap.add_argument("--interval", type=float, default=0.5, help="Print/CSV interval (s)")
    ap.add_argument("--duration", type=float, default=20.0, help="Duration (s); 0 = until Ctrl+C")

    # Hardware params (for speed math)
    ap.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter (m)")
    ap.add_argument("--cpr", type=int, default=20, help="Encoder counts per revolution (per channel)")
    ap.add_argument("--decoding", type=int, default=4, choices=[1,2,4], help="1/2/4x decoding assumed by count math")
    ap.add_argument("--gear", type=float, default=1.0, help="Motor:wheel gear ratio (counts * gear per wheel rev)")
    ap.add_argument("--track", type=float, default=0.165, help="Track width (m) for omega = (vR - vL)/track")

    # GPIO chip / pulls / debounce mode
    ap.add_argument("--chip", type=int, default=None, help="gpiochip index (auto if omitted)")
    ap.add_argument("--no-hw-debounce", action="store_true", help="Force software debounce only")
    ap.add_argument("--no-pullup", action="store_true", help="Do not request pull-ups (use external)")

    # CSV
    ap.add_argument("--csv", type=str, default="", help="CSV output path")
    return ap.parse_args()

if __name__ == "__main__":
    run(parse_args())
