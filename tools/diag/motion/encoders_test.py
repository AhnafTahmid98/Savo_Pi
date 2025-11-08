#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Robot Savo — Quadrature Encoder Diagnostic (Pi 5 / lgpio)
---------------------------------------------------------
- Works on Raspberry Pi 5 (Ubuntu 24.04) using lgpio (libgpiod backend).
- Accurate quadrature via state machine (polling loop; fast + lightweight).
- Supports 1x/2x/4x decoding (internally counts 4x, scales to requested).
- Glitch filtering (µs) per line to reject noise/bounce.
- CSV logging at a fixed print/update rate.
- Computes per-wheel velocity (m/s), robot v, and omega (rad/s).
- Detects illegal transitions for wiring/noise diagnosis.
- Clean Ctrl+C and end-of-run summary.

Author: Robot Sabo

"""

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass

try:
    import lgpio  # sudo apt install -y python3-lgpio
except Exception as e:
    print("ERROR: python3-lgpio is required. Install with: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# Valid Gray-code transitions for 4x decoding (00->01->11->10->00 ring)
VALID = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

def now_s() -> float:
    return time.monotonic()

@dataclass
class Quad:
    h: int
    a: int
    b: int
    invert: int = 1
    glitch_us: int = 300
    use_pullup: bool = True

    state: int = 0
    edges_4x: int = 0
    illegal: int = 0

    def __post_init__(self):
        # Claim inputs
        lgpio.gpio_claim_input(self.h, self.a)
        lgpio.gpio_claim_input(self.h, self.b)
        # Bias pull-ups if supported (harmless if ignored by kernel)
        if self.use_pullup:
            try:
                lgpio.gpio_set_flags(self.h, self.a, lgpio.BIAS_PULL_UP)
                lgpio.gpio_set_flags(self.h, self.b, lgpio.BIAS_PULL_UP)
            except Exception:
                pass
        # Debounce (glitch) in microseconds, if available
        if self.glitch_us > 0:
            for pin in (self.a, self.b):
                try:
                    lgpio.gpio_set_debounce_micros(self.h, pin, self.glitch_us)
                except Exception:
                    pass
        # Initial state
        a = lgpio.gpio_read(self.h, self.a)
        b = lgpio.gpio_read(self.h, self.b)
        self.state = (a << 1) | b

    def sample_once(self):
        # Read both lines and update state machine
        a = lgpio.gpio_read(self.h, self.a)
        b = lgpio.gpio_read(self.h, self.b)
        s_new = (a << 1) | b
        if s_new != self.state:
            d = VALID.get(self.state, {}).get(s_new, 0)
            if d == 0:
                self.illegal += 1
            else:
                self.edges_4x += d * self.invert
            self.state = s_new

@dataclass
class Mech:
    wheel_dia: float   # meters
    track: float       # meters (wheel separation)
    cpr: int           # counts per revolution of the raw disk (single channel)
    gear: float        # gearbox multiplier (edges scaled by gear)
    decoding: int      # 1, 2, or 4

    def edges_per_wheel_rev(self) -> float:
        # Effective edges per wheel revolution at the selected decoding
        # raw disk edges per rev @ 4x = 4 * CPR
        # selected decoding scales that: CPR@1x, 2*CPR@2x, 4*CPR@4x
        return self.decoding * self.cpr * self.gear

    def wheel_circumference(self) -> float:
        return math.pi * self.wheel_dia

    def edges_to_m(self, edges_effective: float) -> float:
        return (edges_effective / self.edges_per_wheel_rev()) * self.wheel_circumference()

def run(args):
    # Open gpiochip0 (Pi 5’s user GPIOs)
    try:
        h = lgpio.gpiochip_open(0)
    except Exception as e:
        print("ERROR: cannot open gpiochip0 — are you on a Raspberry Pi with GPIO enabled?", file=sys.stderr)
        raise

    # Configure channels
    qL = Quad(h, args.l_a, args.l_b, invert=-1 if args.invert_left else 1,
              glitch_us=args.glitch_us, use_pullup=not args.no_pullup)
    qR = Quad(h, args.r_a, args.r_b, invert=-1 if args.invert_right else 1,
              glitch_us=args.glitch_us, use_pullup=not args.no_pullup)

    mech = Mech(args.wheel_dia, args.track, args.cpr, args.gear, args.decoding)

    # Helper: convert 4x edge counts to selected decoding
    scale = args.decoding / 4.0
    def eff_edges(e4: float) -> float:
        return e4 * scale

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

    # Header
    print(f"[EncoderDiag lgpio] pins L({args.l_a},{args.l_b}) R({args.r_a},{args.r_b})  decoding={args.decoding}x  glitch={args.glitch_us}us")
    print(f"[Mechanics] wheel_dia={args.wheel_dia:.3f} m  track={args.track:.3f} m  CPR={args.cpr}  gear={args.gear}  edges/wheel_rev={mech.edges_per_wheel_rev():.1f}")
    print(f"[Notes] invert_left={args.invert_left}  invert_right={args.invert_right}  pullups={'off' if args.no_pullup else 'on'}")
    print(" t(s) |   L_cnt    R_cnt | L_rate  R_rate (edges/s) |  L_v   R_v (m/s) |   v   omega(rad/s) | L_illeg R_illeg")

    # Timing
    t0 = now_s()
    period = 1.0 / max(1e-3, args.rate)
    nextp = t0 + period

    # Per-interval deltas (effective decoding)
    L_prev_e4 = qL.edges_4x
    R_prev_e4 = qR.edges_4x

    stop = False
    def on_sigint(signum, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while not stop:
            # Tight but light polling loop — fine for Hall encoders
            qL.sample_once()
            qR.sample_once()

            t = now_s()
            if args.duration > 0 and (t - t0) >= args.duration:
                break

            if t >= nextp:
                dt = max(t - (nextp - period), 1e-6)

                # Snapshot edges @4x
                L_e4 = qL.edges_4x
                R_e4 = qR.edges_4x

                # Convert to requested decoding for reporting/kinematics
                dL_eff = eff_edges(L_e4 - L_prev_e4)
                dR_eff = eff_edges(R_e4 - R_prev_e4)
                L_prev_e4, R_prev_e4 = L_e4, R_e4

                L_rate = dL_eff / dt
                R_rate = dR_eff / dt

                # Convert to linear velocities
                L_v = mech.edges_to_m(L_rate)
                R_v = mech.edges_to_m(R_rate)

                v = 0.5 * (L_v + R_v)
                omega = (R_v - L_v) / mech.track if mech.track > 0 else 0.0

                if not args.quiet:
                    print(f"{t - t0:5.1f} | {int(eff_edges(L_e4)):7d} {int(eff_edges(R_e4)):7d} | {L_rate:6.1f} {R_rate:6.1f}          | {L_v:5.3f} {R_v:5.3f}      | {v:5.3f}  {omega:7.3f}      | {qL.illegal:7d} {qR.illegal:7d}")

                if csv_writer:
                    csv_writer.writerow([
                        f"{t - t0:.3f}",
                        int(eff_edges(L_e4)), int(eff_edges(R_e4)),
                        f"{L_rate:.6f}", f"{R_rate:.6f}",
                        f"{L_v:.6f}", f"{R_v:.6f}",
                        f"{v:.6f}", f"{omega:.6f}",
                        qL.illegal, qR.illegal
                    ])

                nextp += period

            # micro-snooze to keep CPU low without missing edges
            time.sleep(0.0008)
    finally:
        lgpio.gpiochip_close(h)
        if csv_file:
            csv_file.close()

    # Summary
    T = now_s() - t0
    L_total_eff = int(eff_edges(qL.edges_4x))
    R_total_eff = int(eff_edges(qR.edges_4x))
    print("\n=== Summary ===")
    print(f"Duration: {T:.2f} s")
    print(f"L edges: {L_total_eff}   R edges: {R_total_eff}   (decoding={args.decoding}x)")
    print(f"L illegal: {qL.illegal}   R illegal: {qR.illegal}")
    if T > 0:
        print(f"Avg L rate: {L_total_eff/T:.1f} edges/s   Avg R rate: {R_total_eff/T:.1f} edges/s")
    if qL.illegal or qR.illegal:
        print("\n[Hint] Illegal transitions > 0 usually mean noise/bounce/wiring issues:")
        print("  • Shorten/twist A/B pairs, share GND close to the header.")
        print("  • Increase --glitch-us (e.g., 300–800 µs) for mechanical/noisy Hall.")
        print("  • Verify 3.3 V logic levels; use pull-ups (internal or external).")

def parse_args():
    p = argparse.ArgumentParser(
        description="Robot Savo quadrature encoder diagnostic (Pi 5 / lgpio).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # GPIO (BCM)
    p.add_argument("--l-a", type=int, default=21, help="Left encoder A (BCM)")
    p.add_argument("--l-b", type=int, default=20, help="Left encoder B (BCM)")
    p.add_argument("--r-a", type=int, default=12, help="Right encoder A (BCM)")
    p.add_argument("--r-b", type=int, default=26, help="Right encoder B (BCM)")

    # Signal handling
    p.add_argument("--glitch-us", type=int, default=300, help="Glitch/debounce per line in microseconds (0=off)")
    p.add_argument("--no-pullup", action="store_true", help="Do not request internal pull-ups via lgpio flags")

    # Decoding & mechanics
    p.add_argument("--decoding", type=int, choices=[1,2,4], default=4, help="Quadrature decoding level")
    p.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter (m)")
    p.add_argument("--track",     type=float, default=0.165, help="Wheel separation (m)")
    p.add_argument("--cpr",       type=int,   default=20,    help="Counts per revolution (raw disk, single channel)")
    p.add_argument("--gear",      type=float, default=1.0,   help="Gear ratio multiplier (edges scaled by gear)")
    p.add_argument("--invert-left",  action="store_true", help="Invert left direction")
    p.add_argument("--invert-right", action="store_true", help="Invert right direction")

    # Runtime
    p.add_argument("--rate", type=float, default=20.0, help="Update/CSV rate (Hz)")
    p.add_argument("--duration", type=float, default=30.0, help="Run time in seconds (0 = until Ctrl+C)")
    p.add_argument("--csv", type=str, default="", help="CSV output file")
    p.add_argument("--quiet", action="store_true", help="Suppress per-line prints (still logs CSV)")
    return p.parse_args()

if __name__ == "__main__":
    run(parse_args())
