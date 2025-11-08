#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Robot Savo — Quadrature Encoder Diagnostic (expert edition)
-----------------------------------------------------------
- Accurate edge timing via pigpio callbacks (kernel timestamped).
- Supports 1x/2x/4x decoding; per-wheel count and velocity.
- Illegal transition detection (noise / wiring / bounce).
- Glitch filtering (µs) for Hall or mechanical encoders.
- CSV logging with fixed-rate summaries.
- Clean shutdown and end-of-run stats.

Defaults are locked to your project GPIOs:
  Left  : L_A=21, L_B=20
  Right : R_A=12, R_B=26

Author: Robot Savo

"""

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    import pigpio
except Exception as e:
    print("ERROR: pigpio is required. Install with: sudo apt install -y pigpio python3-pigpio", file=sys.stderr)
    raise

# ---------------------------- Utils ----------------------------

def now_s():
    return time.monotonic()

# Quadrature transition table (Gray code) for 4x decoding
# States: 00,01,11,10. Valid steps are +/- 1 in the ring; others are illegal.
VALID_TRANSITIONS = {
    0b00: {0b01: +1, 0b10: -1},
    0b01: {0b11: +1, 0b00: -1},
    0b11: {0b10: +1, 0b01: -1},
    0b10: {0b00: +1, 0b11: -1},
}

@dataclass
class QuadState:
    a_pin: int
    b_pin: int
    invert: int = +1     # +1 normal, -1 invert direction
    decoding: int = 4    # 1, 2, or 4 (x decoding)
    # live
    last_state: int = 0
    count_edges: int = 0     # 4x edges count (raw steps)
    illegal: int = 0
    last_time: float = 0.0

    # For rate calculations
    sample_count_edges: int = 0
    sample_start_time: float = 0.0

class QuadDecoder:
    """
    Decodes quadrature signals with pigpio callbacks.
    """
    def __init__(self, pi: pigpio.pi, qs: QuadState, glitch_us: int = 300, pull_up=True):
        self.pi = pi
        self.qs = qs

        # Configure inputs
        pi.set_mode(qs.a_pin, pigpio.INPUT)
        pi.set_mode(qs.b_pin, pigpio.INPUT)
        if pull_up:
            pi.set_pull_up_down(qs.a_pin, pigpio.PUD_UP)
            pi.set_pull_up_down(qs.b_pin, pigpio.PUD_UP)

        if glitch_us > 0:
            # Reject pulses shorter than glitch_us (µs)
            pi.set_glitch_filter(qs.a_pin, glitch_us)
            pi.set_glitch_filter(qs.b_pin, glitch_us)

        # Initialize last state
        a = pi.read(qs.a_pin)
        b = pi.read(qs.b_pin)
        qs.last_state = (a << 1) | b
        qs.last_time = now_s()
        qs.sample_start_time = qs.last_time

        # Callbacks
        self.cb_a = pi.callback(qs.a_pin, pigpio.EITHER_EDGE, self._cb)
        self.cb_b = pi.callback(qs.b_pin, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        # Read both lines to form current state
        a = self.pi.read(self.qs.a_pin)
        b = self.pi.read(self.qs.b_pin)
        s_new = (a << 1) | b
        s_old = self.qs.last_state

        if s_new != s_old:
            d = VALID_TRANSITIONS.get(s_old, {}).get(s_new, 0)
            if d == 0:
                # Illegal transition (noise / bounce)
                self.qs.illegal += 1
            else:
                self.qs.count_edges += d * self.qs.invert
                self.qs.sample_count_edges += d * self.qs.invert
            self.qs.last_state = s_new
            self.qs.last_time = now_s()

    def cancel(self):
        self.cb_a.cancel()
        self.cb_b.cancel()

# ---------------------------- Kinematics ----------------------------

@dataclass
class Mech:
    wheel_dia: float   # meters
    track: float       # meters (wheel separation)
    cpr: int           # counts per revolution of the raw disk (single channel)
    gear: float        # gearbox ratio (wheel rev per shaft rev = 1/gear, or supply effective multiplier below)
    decoding: int      # 1x, 2x, 4x

    def edges_per_wheel_rev(self) -> float:
        # A/B quadrature effective counts per raw disk rev:
        # 1x = CPR, 2x = 2*CPR, 4x = 4*CPR
        # After gearbox, wheel rev edges = decoding*CPR*gear
        return self.decoding * self.cpr * self.gear

    def wheel_circumference(self) -> float:
        return math.pi * self.wheel_dia

    def edges_to_meters(self, edges: float) -> float:
        return (edges / self.edges_per_wheel_rev()) * self.wheel_circumference()

# ---------------------------- Main Diagnostic ----------------------------

def run(args):
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpio daemon not running. Start it with: sudo systemctl start pigpiod", file=sys.stderr)
        sys.exit(1)

    # Configure left/right decoders
    ql = QuadState(a_pin=args.l_a, b_pin=args.l_b, invert=-1 if args.invert_left else +1, decoding=args.decoding)
    qr = QuadState(a_pin=args.r_a, b_pin=args.r_b, invert=-1 if args.invert_right else +1, decoding=args.decoding)

    dl = QuadDecoder(pi, ql, glitch_us=args.glitch_us, pull_up=not args.no_pullup)
    dr = QuadDecoder(pi, qr, glitch_us=args.glitch_us, pull_up=not args.no_pullup)

    mech = Mech(
        wheel_dia=args.wheel_dia,
        track=args.track,
        cpr=args.cpr,
        gear=args.gear,
        decoding=args.decoding,
    )

    # CSV setup
    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "t_s",
            "L_edges", "R_edges",
            "L_rate_edges_s", "R_rate_edges_s",
            "L_vel_m_s", "R_vel_m_s",
            "v_m_s", "omega_rad_s",
            "L_illegal", "R_illegal"
        ])

    # Print header
    print(f"[EncoderDiag] pins L({args.l_a},{args.l_b}) R({args.r_a},{args.r_b})  decoding={args.decoding}x  glitch={args.glitch_us}us")
    print(f"[Mechanics] wheel_dia={args.wheel_dia:.3f} m  track={args.track:.3f} m  CPR={args.cpr}  gear={args.gear}  edges/wheel_rev={mech.edges_per_wheel_rev():.1f}")
    print(f"[Notes] invert_left={args.invert_left}  invert_right={args.invert_right}  pullups={'off' if args.no_pullup else 'on'}")
    print(" t(s) |   L_cnt    R_cnt | L_rate  R_rate (edges/s) |  L_v   R_v (m/s) |   v   omega(rad/s) | L_illeg R_illeg")

    stop = False
    def handle_sigint(signum, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handle_sigint)

    t0 = now_s()
    next_print = t0
    last_edges_L = 0
    last_edges_R = 0

    while not stop:
        t = now_s()
        if args.duration > 0 and (t - t0) >= args.duration:
            break

        if t >= next_print:
            # Compute sample rates since last print interval
            dt = t - next_print + args.period  # approximate dt ~ period
            # Protect against zero/negative dt
            dt = max(dt, 1e-6)

            # Snapshot and reset sample counters atomically-enough
            L_edges = ql.count_edges
            R_edges = qr.count_edges
            sL = ql.sample_count_edges
            sR = qr.sample_count_edges
            ql.sample_count_edges = 0
            qr.sample_count_edges = 0

            L_rate = sL / dt
            R_rate = sR / dt

            # Convert to linear wheel velocities
            L_v = mech.edges_to_meters(L_rate)
            R_v = mech.edges_to_meters(R_rate)

            # Robot frame (v, omega) for differential base:
            v = 0.5 * (L_v + R_v)
            omega = (R_v - L_v) / mech.track if mech.track > 0 else 0.0

            if not args.quiet:
                print(f"{t - t0:5.1f} | {L_edges:7d} {R_edges:7d} | {L_rate:6.1f} {R_rate:6.1f}          | {L_v:5.3f} {R_v:5.3f}      | {v:5.3f}  {omega:7.3f}      | {ql.illegal:7d} {qr.illegal:7d}")

            if csv_writer:
                csv_writer.writerow([f"{t - t0:.3f}", L_edges, R_edges, f"{L_rate:.3f}", f"{R_rate:.3f}",
                                     f"{L_v:.6f}", f"{R_v:.6f}", f"{v:.6f}", f"{omega:.6f}", ql.illegal, qr.illegal])

            next_print += args.period

        # Sleep a bit to reduce CPU
        time.sleep(0.001)

    # Cleanup
    dl.cancel()
    dr.cancel()
    pi.stop()
    if csv_file:
        csv_file.close()

    # Summary
    total_t = now_s() - t0
    L_total = ql.count_edges
    R_total = qr.count_edges
    print("\n=== Summary ===")
    print(f"Duration: {total_t:.2f} s")
    print(f"L edges: {L_total}   R edges: {R_total}")
    print(f"L illegal: {ql.illegal}   R illegal: {qr.illegal}")
    if total_t > 0:
        print(f"Avg L rate: {L_total/total_t:.1f} edges/s   Avg R rate: {R_total/total_t:.1f} edges/s")

    # Basic guidance
    if ql.illegal or qr.illegal:
        print("\n[Hint] Non-zero illegal transitions usually indicate noise or wiring issues:")
        print("  • Check GND reference, shorten/twist A/B pairs, verify 3.3 V pull-ups.")
        print("  • Increase --glitch-us (e.g., 300–800 µs) for mechanical or noisy Hall signals.")
        print("  • Ensure shield only grounded at one end to avoid ground loops.")

def parse_args():
    p = argparse.ArgumentParser(
        description="Robot Savo quadrature encoder diagnostic (pigpio).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # GPIO pins (BCM)
    p.add_argument("--l-a", type=int, default=21, help="Left encoder A (BCM)")
    p.add_argument("--l-b", type=int, default=20, help="Left encoder B (BCM)")
    p.add_argument("--r-a", type=int, default=12, help="Right encoder A (BCM)")
    p.add_argument("--r-b", type=int, default=26, help="Right encoder B (BCM)")

    # Decoding + filtering
    p.add_argument("--decoding", type=int, choices=[1,2,4], default=4, help="Quadrature decoding (x1/x2/x4)")
    p.add_argument("--glitch-us", type=int, default=300, help="Glitch filter in microseconds (0=off)")
    p.add_argument("--no-pullup", action="store_true", help="Disable internal pull-ups (use external)")

    # Mechanics (for velocity/omega)
    p.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter (m)")
    p.add_argument("--track", type=float, default=0.165, help="Wheel separation (m)")
    p.add_argument("--cpr", type=int, default=20, help="Counts per revolution (raw disk, single channel)")
    p.add_argument("--gear", type=float, default=1.0, help="Gear ratio multiplier (edges scaled by gear)")
    p.add_argument("--invert-left", action="store_true", help="Invert left direction")
    p.add_argument("--invert-right", action="store_true", help="Invert right direction")

    # Runtime / output
    p.add_argument("--rate", type=float, default=20.0, help="Print/CSV update rate (Hz)")
    p.add_argument("--duration", type=float, default=30.0, help="Duration (s), 0 = run until Ctrl+C")
    p.add_argument("--csv", type=str, default="", help="CSV output path")
    p.add_argument("--quiet", action="store_true", help="Suppress line prints (still logs to CSV)")
    p.add_argument("--debug", action="store_true", help="Reserved for future per-edge debug")

    args = p.parse_args()
    args.period = 1.0 / max(1e-3, args.rate)
    return args

if __name__ == "__main__":
    run(parse_args())
