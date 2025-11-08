#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Encoder + Odometry Diagnostic (Pi 5 / lgpio)
---------------------------------------------------------
- Works on Raspberry Pi 5 (Ubuntu 24.04) with python3-lgpio.
- Reads quadrature on BCM pins via Gray-code state machine.
- Debounce: hardware (if kernel supports) + fallback software.
- Signed counts (forward/reverse), per-wheel velocity (m/s).
- Differential-drive odometry: integrates x, y, theta (rad)
  using midpoint (body-centered) integration each update.
- CSV logging, gpiochip auto-detect or --chip override.
- Clean Ctrl+C and resource release.

Author: Robot Savo

"""

import argparse
import csv
import math
import sys
import time
import signal

try:
    import lgpio
except Exception:
    print("ERROR: python3-lgpio is required. Install with: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# ---------------- Quadrature decoding (Gray-code transitions) ----------------
DELTA = {
    (0b00,0b01): +1, (0b01,0b11): +1, (0b11,0b10): +1, (0b10,0b00): +1,
    (0b01,0b00): -1, (0b11,0b01): -1, (0b10,0b11): -1, (0b00,0b10): -1
}

def autodetect_chip(pins, start=0, end=7, pullup=True):
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
                        try: lgpio.gpio_set_flags(h, p, lgpio.BIAS_PULL_UP)
                        except Exception: pass
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
    """One wheel’s quadrature reader with debounce + invert."""
    def __init__(self, h, a, b, debounce_s, invert=1, use_hw_debounce=True, pullup=True):
        self.h, self.a, self.b = h, a, b
        self.debounce_s = max(0.0, debounce_s)
        self.use_hw_debounce = use_hw_debounce
        self.invert = 1 if invert >= 0 else -1

        # Claim inputs
        lgpio.gpio_claim_input(h, a)
        lgpio.gpio_claim_input(h, b)

        # Internal pull-ups request (harmless if ignored by kernel)
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

        # State
        now = time.monotonic()
        self.last_change = {a: now, b: now}
        self.state_bits = {a: lgpio.gpio_read(h, a), b: lgpio.gpio_read(h, b)}
        self.prev_state = ((self.state_bits[a] & 1) << 1) | (self.state_bits[b] & 1)

        self.count = 0     # signed count (at selected decoding scale)
        self.dir = 0       # last step dir: -1, 0, +1
        self.illegal = 0   # illegal transitions (noise indicator)

    def sample_once(self, now):
        a_val = lgpio.gpio_read(self.h, self.a)
        b_val = lgpio.gpio_read(self.h, self.b)

        if not self.hw_debounce and self.debounce_s > 0:
            # simple software debounce
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
            d = DELTA.get((self.prev_state, s_new), 0)
            if d != 0:
                d *= self.invert
                self.count += d
                self.dir = 1 if d > 0 else -1
            else:
                self.illegal += 1
            self.prev_state = s_new

# ---------------- Mechanics / odometry helpers ----------------
class Mechanics:
    def __init__(self, wheel_dia, track, cpr, gear, decoding):
        self.wheel_dia = wheel_dia   # meters
        self.track = track           # meters
        self.cpr = cpr               # counts / raw disk rev (single channel)
        self.gear = gear             # gearbox multiplier
        self.decoding = decoding     # 1,2,4
        self.edges_per_rev = max(1.0, decoding * cpr * gear)
        self.m_per_edge = (math.pi * wheel_dia) / self.edges_per_rev

    def edges_to_m(self, edges):
        return edges * self.m_per_edge

def run(args):
    pins = [args.l_a, args.l_b, args.r_a, args.r_b]

    # gpiochip
    if args.chip is not None:
        h = lgpio.gpiochip_open(args.chip); chip_used = args.chip
    else:
        chip_used, h = autodetect_chip(pins)

    mech = Mechanics(args.wheel_dia, args.track, args.cpr, args.gear, args.decoding)

    print(f"[Encoder+Odom] gpiochip{chip_used}  Pins L({args.l_a},{args.l_b}) R({args.r_a},{args.r_b})")
    print(f"[Mech] wheel_dia={args.wheel_dia:.3f} m  track={args.track:.3f} m  CPR={args.cpr}  gear={args.gear}  decoding={args.decoding}x  edges/rev={mech.edges_per_rev:.1f}")
    print(f"[Debounce] target={int(args.debounce_s*1e6)} µs  polling={int(args.poll_s*1e6)} µs")
    print(f"[Invert] left={args.invert_left}  right={args.invert_right}")
    print(" t(s) |   L_cnt    R_cnt |  L_v   R_v (m/s) |    v    ω(rad/s) |      x       y     θ(rad) | L_illeg R_illeg")

    # Build sides
    L = QuadSide(h, args.l_a, args.l_b, debounce_s=args.debounce_s,
                 invert=(-1 if args.invert_left else +1),
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)
    R = QuadSide(h, args.r_a, args.r_b, debounce_s=args.debounce_s,
                 invert=(-1 if args.invert_right else +1),
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)

    # CSV
    csvf = None; writer = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow(["t_s","L_count","R_count","L_vel_m_s","R_vel_m_s",
                         "v_m_s","omega_rad_s","x_m","y_m","theta_rad","L_illegal","R_illegal"])

    # Pose (meters, radians)
    x = args.x0; y = args.y0; th = args.th0

    # Timers
    t0 = time.monotonic()
    last = t0
    nextp = t0
    period = 1.0 / max(1e-3, args.rate)

    # Count snapshots for velocities
    L_prev = L.count; R_prev = R.count

    stop = False
    def on_sigint(sig, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0 and (now - t0) >= args.duration:
                break

            # Poll encoders
            L.sample_once(now)
            R.sample_once(now)

            # Integrate at the chosen rate
            if now >= nextp:
                dt = max(now - last, 1e-6)

                # Edges since last update (signed)
                dL_edges = L.count - L_prev
                dR_edges = R.count - R_prev
                L_prev, R_prev = L.count, R.count

                # Per-wheel distances (m)
                dL = mech.edges_to_m(dL_edges)
                dR = mech.edges_to_m(dR_edges)

                # Velocities (m/s)
                L_v = dL / dt
                R_v = dR / dt

                # Differential-drive body twist
                v = 0.5 * (L_v + R_v)
                dtheta = (dR - dL) / mech.track if mech.track > 0 else 0.0
                omega = dtheta / dt

                # Midpoint (body-centered) integration
                th_mid = th + 0.5 * dtheta
                dx = v * dt * math.cos(th_mid)
                dy = v * dt * math.sin(th_mid)
                x += dx
                y += dy
                th += dtheta
                # Wrap theta to [-pi, pi]
                if th > math.pi: th -= 2*math.pi
                elif th < -math.pi: th += 2*math.pi

                # Print line
                print(f"{now - t0:5.1f} | {L.count:7d} {R.count:7d} | {L_v:5.3f} {R_v:5.3f}      | {v:5.3f} {omega:8.3f} | {x:7.3f} {y:7.3f} {th:8.4f} | {L.illegal:7d} {R.illegal:7d}")

                if writer:
                    writer.writerow([f"{now - t0:.3f}", L.count, R.count,
                                     f"{L_v:.6f}", f"{R_v:.6f}",
                                     f"{v:.6f}", f"{omega:.6f}",
                                     f"{x:.6f}", f"{y:.6f}", f"{th:.6f}",
                                     L.illegal, R.illegal])

                last = now
                nextp += period

            time.sleep(args.poll_s)
    finally:
        # Cleanup
        for p in (args.l_a, args.l_b, args.r_a, args.r_b):
            try: lgpio.gpio_free(h, p)
            except: pass
        lgpio.gpiochip_close(h)
        if csvf: csvf.close()

    # Summary
    T = time.monotonic() - t0
    print("\n=== Summary ===")
    print(f"Duration: {T:.2f} s")
    print(f"Final pose: x={x:.3f} m  y={y:.3f} m  theta={th:.4f} rad")
    print(f"L total: {L.count}   R total: {R.count}   L_illegal={L.illegal}  R_illegal={R.illegal}")
    if T > 0:
        print(f"Avg L rate: {L.count/T:.1f} counts/s   Avg R rate: {R.count/T:.1f} counts/s")

def parse_args():
    p = argparse.ArgumentParser(
        description="Encoder + Odometry diagnostic (Pi 5 / lgpio).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # GPIO pins (BCM)
    p.add_argument("--l-a", type=int, default=21, help="Left encoder A (BCM)")
    p.add_argument("--l-b", type=int, default=20, help="Left encoder B (BCM)")
    p.add_argument("--r-a", type=int, default=12, help="Right encoder A (BCM)")
    p.add_argument("--r-b", type=int, default=26, help="Right encoder B (BCM)")

    # Invert to make 'forward' positive if needed
    p.add_argument("--invert-left", action="store_true", help="Invert left direction")
    p.add_argument("--invert-right", action="store_true", help="Invert right direction")

    # Debounce / polling / gpiochip
    p.add_argument("--debounce-s", type=float, default=0.0003, help="Per-line debounce (s)")
    p.add_argument("--poll-s", type=float, default=0.001, help="Polling sleep between samples (s)")
    p.add_argument("--chip", type=int, default=None, help="gpiochip index (auto if omitted)")
    p.add_argument("--no-hw-debounce", action="store_true", help="Force software debounce only")
    p.add_argument("--no-pullup", action="store_true", help="Do not request internal pull-ups")

    # Mechanics (odometry)
    p.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter (m)")
    p.add_argument("--track",     type=float, default=0.165, help="Wheel separation (m)")
    p.add_argument("--cpr",       type=int,   default=20,    help="Counts per revolution (single channel)")
    p.add_argument("--gear",      type=float, default=1.0,   help="Gear multiplier (e.g., 34 for 34:1)")
    p.add_argument("--decoding",  type=int,   choices=[1,2,4], default=4, help="Quadrature decoding level")

    # Pose init
    p.add_argument("--x0",  type=float, default=0.0, help="Initial x (m)")
    p.add_argument("--y0",  type=float, default=0.0, help="Initial y (m)")
    p.add_argument("--th0", type=float, default=0.0, help="Initial theta (rad)")

    # Runtime / logging
    p.add_argument("--rate", type=float, default=20.0, help="Update/print/CSV rate (Hz)")
    p.add_argument("--duration", type=float, default=30.0, help="Duration (s), 0 = until Ctrl+C")
    p.add_argument("--csv", type=str, default="", help="CSV output path")
    return p.parse_args()

if __name__ == "__main__":
    run(parse_args())
