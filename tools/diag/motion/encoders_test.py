#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Encoders Test (no odom)
- Pure quadrature counting with direction (polling-based, debounced by time).
- Works with libgpiod v2 (new API) and v1 (legacy API).
- Prints counts, per-interval deltas, simple velocities, and illegal transitions.
- No odometry math here (you’ll test that later).

Pins (locked from memory):
  Left  : A=21, B=20
  Right : A=12, B=26

Examples:
  python3 tools/diag/motion/encoders_test.py --duration 12
  # If forward shows negative, flip sign without rewiring:
  python3 tools/diag/motion/encoders_test.py --invert-left
  python3 tools/diag/motion/encoders_test.py --invert-right
  # CSV
  python3 tools/diag/motion/encoders_test.py --duration 15 --csv encoders.csv
"""

import argparse
import time
import math
import sys
import csv

try:
    import gpiod
except Exception as e:
    print("ERROR: python3-libgpiod is required. Install:\n  sudo apt install -y python3-libgpiod", file=sys.stderr)
    raise

# ---------------------------- Quadrature utils ----------------------------

# Valid Gray-code sequence for quadrature: 00→01→11→10→00 (and reverse)
STATE_ORDER = [0b00, 0b01, 0b11, 0b10]
STATE_INDEX = {s: i for i, s in enumerate(STATE_ORDER)}

def step_from(prev_state: int, curr_state: int) -> int:
    """Return +1, -1 or 0 for valid neighbor transitions; 0 on same; raise on illegal."""
    if curr_state == prev_state:
        return 0
    if curr_state not in STATE_INDEX or prev_state not in STATE_INDEX:
        raise ValueError("Invalid state")
    di = STATE_INDEX[curr_state] - STATE_INDEX[prev_state]
    if di == 1 or di == -3:
        return +1   # forward
    if di == -1 or di == 3:
        return -1   # reverse
    raise RuntimeError("Illegal quadrature jump: {} -> {}".format(bin(prev_state), bin(curr_state)))

# ----------------------- GPIO abstraction (v2 and v1) ----------------------

def libgpiod_is_v2() -> bool:
    # v2 exposes request_lines(); v1 doesn't
    return hasattr(gpiod, "request_lines")

class LinesV2:
    def __init__(self, chipname: str, offsets: list[int]):
        settings = {}
        for off in offsets:
            settings[off] = gpiod.LineSettings(
                direction=gpiod.LineDirection.INPUT,
                edge_detection=gpiod.LineEdge.NONE,
                bias=gpiod.LineBias.DISABLED  # you said you have 10k external pull-ups
            )
        self.req = gpiod.request_lines(
            f"/dev/{chipname}" if not chipname.startswith("/dev/") else chipname,
            consumer="encoders_test",
            config=settings
        )
        self.offsets = offsets

    def get_values(self) -> list[int]:
        vals = self.req.get_values()  # dict {offset: value}
        return [int(vals[o]) for o in self.offsets]

class LinesV1:
    def __init__(self, chipname: str, offsets: list[int]):
        self.chip = gpiod.Chip(chipname)
        self.lines = self.chip.get_lines(offsets)
        self.lines.request(consumer="encoders_test", type=gpiod.LINE_REQ_DIR_IN)
        self.offsets = offsets

    def get_values(self) -> list[int]:
        return list(map(int, self.lines.get_values()))

class GPIOBank:
    """Group two lines (A,B) for one wheel; supports v2/v1 uniformly."""
    def __init__(self, chipname: str, a: int, b: int, use_v2: bool):
        Impl = LinesV2 if use_v2 else LinesV1
        self.impl = Impl(chipname, [a, b])

    def read_ab(self) -> tuple[int, int]:
        a, b = self.impl.get_values()
        return a, b

# ------------------------------ Core counter ------------------------------

class QuadCounter:
    def __init__(self, name: str, gpio: GPIOBank, invert: bool, debounce_us: int):
        self.name = name
        self.gpio = gpio
        self.invert = invert
        self.debounce_s = max(0.0, debounce_us / 1_000_000.0)
        a, b = self.gpio.read_ab()
        self.prev_state = (a << 1) | b
        self.last_change_t = time.monotonic()
        self.count = 0
        self.illegal = 0

    def sample(self) -> int:
        """Poll and update count; returns delta (since last sample call) in steps."""
        a, b = self.gpio.read_ab()
        state = (a << 1) | b
        now = time.monotonic()
        d = 0
        if state != self.prev_state:
            # Simple time-based debounce: ignore changes faster than debounce_s
            if (now - self.last_change_t) >= self.debounce_s:
                try:
                    step = step_from(self.prev_state, state)
                    if self.invert:
                        step = -step
                    self.count += step
                    d = step
                except RuntimeError:
                    self.illegal += 1
                self.prev_state = state
                self.last_change_t = now
            # else: ignore as bounce
        return d

# ------------------------------ CLI / main --------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Robot Savo encoders test (no odom).")
    p.add_argument("--left-a", type=int, default=21, help="Left A GPIO (BCM)")
    p.add_argument("--left-b", type=int, default=20, help="Left B GPIO (BCM)")
    p.add_argument("--right-a", type=int, default=12, help="Right A GPIO (BCM)")
    p.add_argument("--right-b", type=int, default=26, help="Right B GPIO (BCM)")
    p.add_argument("--chip", default="gpiochip4", help="GPIO chip (pinctrl-rp1 is usually gpiochip4)")
    p.add_argument("--duration", type=float, default=12.0, help="Run time (s)")
    p.add_argument("--rate", type=float, default=20.0, help="Print rate (Hz)")
    p.add_argument("--debounce-us", type=int, default=300, help="Time debounce for A/B transitions (µs)")
    p.add_argument("--invert-left", action="store_true", help="Invert left direction")
    p.add_argument("--invert-right", action="store_true", help="Invert right direction")
    p.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter (m) for velocity estimate")
    p.add_argument("--cpr", type=int, default=20, help="Counts per revolution of encoder")
    p.add_argument("--decoding", type=int, default=4, choices=[1,2,4], help="1/2/4x decoding for edges/rev calc")
    p.add_argument("--gear", type=float, default=1.0, help="Gear ratio (motor:wheel), only affects velocity scaling")
    p.add_argument("--csv", type=str, default="", help="CSV output path")
    return p.parse_args()

def main():
    args = parse_args()

    v2 = libgpiod_is_v2()
    # Probe version string if available
    ver = getattr(gpiod, "version_string", lambda: "unknown")()
    print(f"[Encoders] libgpiod={'v2' if v2 else 'v1'} ({ver})  Chip={args.chip}  Pins L({args.left_a},{args.left_b}) R({args.right_a},{args.right_b})")

    edges_per_rev = args.cpr * args.decoding
    print(f"[Mech] wheel_dia={args.wheel-dia:.3f} m  CPR={args.cpr}  decoding={args.decoding}  gear={args.gear}  edges/rev={edges_per_rev:.1f}")
    print(f"[Debounce] {args.debounce_us} µs  [Invert] left={args.invert_left} right={args.invert_right}")

    # Build GPIO readers
    try:
        left_bank  = GPIOBank(args.chip, args.left_a,  args.left_b,  v2)
        right_bank = GPIOBank(args.chip, args.right_a, args.right_b, v2)
    except Exception as e:
        print(f"ERROR: could not open {args.chip} or request lines: {e}", file=sys.stderr)
        sys.exit(1)

    left  = QuadCounter("L", left_bank,  args.invert_left,  args.debounce_us)
    right = QuadCounter("R", right_bank, args.invert_right, args.debounce_us)

    # CSV
    writer = None
    if args.csv:
        f = open(args.csv, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["t_s", "L_cnt", "R_cnt", "L_d", "R_d", "L_illeg", "R_illeg", "L_v_mps", "R_v_mps"])

    # Print header
    print(" t(s) |   L_cnt    R_cnt |  L_d  R_d |  L_v   R_v (m/s) | L_illeg R_illeg")
    print("--------------------------------------------------------------------------")

    period = 1.0 / max(1e-3, args.rate)
    t0 = time.monotonic()
    t_last = t0
    last_L = left.count
    last_R = right.count

    try:
        while True:
            now = time.monotonic()
            t = now - t0
            if t >= args.duration:
                break

            # Poll fast internally to not miss quick transitions; limit to ~1 kHz
            # but keep CPU reasonable by sleeping small slices until next print tick.
            next_print = t_last + period
            while now < next_print:
                # sample both wheels
                left.sample()
                right.sample()
                time.sleep(0.0005)  # 0.5 ms inner poll
                now = time.monotonic()

            dt = now - t_last
            t_last = now

            # deltas since last print
            dL = left.count - last_L
            dR = right.count - last_R
            last_L = left.count
            last_R = right.count

            # velocity estimate (m/s) using counts/sec → rev/sec → m/s
            # counts per wheel revolution = edges_per_rev * gear
            # Note: if encoder is on motor shaft with gearbox, set gear accordingly.
            counts_per_wrev = max(1, int(edges_per_rev * args.gear))
            L_v = (dL / max(1e-6, dt)) / counts_per_wrev * (math.pi * args.wheel_dia)
            R_v = (dR / max(1e-6, dt)) / counts_per_wrev * (math.pi * args.wheel_dia)

            print(f"{t:5.1f} | {left.count:7d} {right.count:7d} | {dL:4d} {dR:4d} | {L_v: .3f} {R_v: .3f} | {left.illegal:7d} {right.illegal:7d}")

            if writer:
                writer.writerow([f"{t:.3f}", left.count, right.count, dL, dR, left.illegal, right.illegal, f"{L_v:.6f}", f"{R_v:.6f}"])

    except KeyboardInterrupt:
        pass
    finally:
        if writer:
            f.close()

    # Summary
    t_total = time.monotonic() - t0
    print("\n=== Summary ===")
    print(f"Duration: {t_total:.2f} s")
    print(f"L total: {left.count}   R total: {right.count}   L_illegal={left.illegal}  R_illegal={right.illegal}")
    print("Tip: positive/negative counts indicate forward/reverse; flip with --invert-left/--invert-right if needed.")

if __name__ == "__main__":
    main()
