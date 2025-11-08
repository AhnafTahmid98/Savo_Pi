#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Quadrature Encoders Test (no odom)
-----------------------------------------------
- Uses libgpiod edge events (interrupt-like) with software debouncing.
- Robust quadrature state machine (valid/invalid transition tracking).
- Per-wheel count and linear speed (m/s) — NO pose/odom here.
- Tunable CPR, decoding (1x/2x/4x), wheel geometry for speed calc.
- Optional CSV logging.
- Clean shutdown and summary.

Author: Robot Savo
"""

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import gpiod  # python3-libgpiod
except Exception as e:
    print("ERROR: python3-libgpiod is required. Install with: sudo apt install -y python3-libgpiod", file=sys.stderr)
    raise

# ----------------------------- Quadrature Helpers -----------------------------

# Valid next-state table for 2-bit quadrature (Gray code): 00 -> 01 -> 11 -> 10 -> 00 (forward)
# We map (prev<<2 | curr) -> step {-1, 0, +1}. 0 means no change or illegal (tracked separately).
_QSTEP = {
    0b0001: +1,  # 00 -> 01
    0b0011:  0,  # 00 -> 11 (illegal, but we count it elsewhere)
    0b0010: -1,  # 00 -> 10
    0b0100: -1,  # 01 -> 00
    0b0111: +1,  # 01 -> 11
    0b0110:  0,  # 01 -> 10 (illegal)
    0b1110: +1,  # 11 -> 10
    0b1100: -1,  # 11 -> 00
    0b1101:  0,  # 11 -> 01 (illegal)
    0b1011: -1,  # 10 -> 11
    0b1000: +1,  # 10 -> 00
    0b1001:  0,  # 10 -> 01 (illegal)
}

def qstate(a_val: int, b_val: int) -> int:
    """Pack A,B (0/1) into a 2-bit state."""
    return ((a_val & 1) << 1) | (b_val & 1)

# --------------------------------- Data Classes --------------------------------

@dataclass
class WheelConfig:
    a_pin: int
    b_pin: int
    invert: bool = False  # invert direction if needed (A/B swapped or wiring)
    name: str = "L"

@dataclass
class MechConfig:
    wheel_diam: float = 0.065  # m
    cpr: int = 20              # cycles per revolution of encoder (A channel)
    decoding: int = 4          # 1, 2, or 4 (edges per cycle)
    gear: float = 1.0          # gear ratio wheel:encoder (1.0 if on wheel shaft)

    @property
    def edges_per_rev(self) -> float:
        return float(self.cpr) * float(self.decoding) * float(self.gear)

    @property
    def wheel_circ(self) -> float:
        return math.pi * self.wheel_diam

# -------------------------------- Encoder Class --------------------------------

class QuadEncoder:
    def __init__(self, chip: gpiod.Chip, cfg: WheelConfig, debounce_us: int):
        self.cfg = cfg
        self.debounce_us = max(0, int(debounce_us))
        self.count = 0
        self.illegal = 0
        self.last_state: Optional[int] = None
        self.last_ts_us = 0

        # Request lines with edge events; bias to pull-up (if supported)
        # We use BOTH edges on both lines and always sample both A,B to compute state transitions.
        req = gpiod.line_request()
        req.consumer = f"enc_{cfg.name}"
        req.request_type = gpiod.line_request.EVENT_BOTH_EDGES
        req.flags = gpiod.line_request.FLAG_BIAS_PULL_UP  # safe even if no internal pull
        self.a_line = chip.get_line(cfg.a_pin)
        self.b_line = chip.get_line(cfg.b_pin)
        self.a_line.request(req)
        self.b_line.request(req)

        # Initialize last state
        a = self.a_line.get_value()
        b = self.b_line.get_value()
        self.last_state = qstate(a, b)
        self.last_ts_us = _monotonic_us()

    def close(self):
        try:
            self.a_line.release()
        except Exception:
            pass
        try:
            self.b_line.release()
        except Exception:
            pass

    def poll_once(self) -> None:
        """Poll without blocking: drain any queued events for A and B, apply debounced transitions."""
        # Drain both lines’ events, but always compute transitions from sampled A/B pair.
        # We simply check if any edge occurred; if so, we sample current A,B and update state machine.
        got_edge = False
        for ln in (self.a_line, self.b_line):
            while True:
                ev = ln.event_read_multiple()
                if not ev:
                    break
                got_edge = True

        if not got_edge:
            return

        now_us = _monotonic_us()
        if self.debounce_us and (now_us - self.last_ts_us) < self.debounce_us:
            # Debounce window: ignore all transitions too soon after the previous accepted one.
            return

        a = self.a_line.get_value()
        b = self.b_line.get_value()
        curr = qstate(a, b)
        prev = self.last_state
        if prev is None:
            self.last_state = curr
            self.last_ts_us = now_us
            return

        step = _QSTEP.get(((prev << 2) | curr), 0)
        if step == 0 and curr != prev:
            # Illegal transition
            self.illegal += 1
        else:
            # Valid quantized step (+1/-1)
            if self.cfg.invert:
                step = -step
            self.count += step

        self.last_state = curr
        self.last_ts_us = now_us

# ------------------------------ Utility Functions ------------------------------

def _monotonic_us() -> int:
    return int(time.monotonic_ns() // 1000)

def _open_chip(name_hint: Optional[str]) -> gpiod.Chip:
    # Try exact chip if provided, else pick the last available gpiochip (usual on Pi5 is gpiochip4)
    if name_hint:
        try:
            return gpiod.Chip(name_hint)
        except Exception:
            print(f"[Warn] Could not open '{name_hint}', falling back to auto.", file=sys.stderr)
    chips = gpiod.chip_iter()
    last = None
    for c in chips:
        last = c
    if last is None:
        raise RuntimeError("No gpiochip found")
    # Re-open by label to own a handle (iterator yields temporary objects)
    return gpiod.Chip(last.label())

def _fmt(v: float, width: int = 6, prec: int = 3) -> str:
    return f"{v:{width}.{prec}f}"

# ------------------------------------ Main -------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Robot Savo — Encoders Test (no odom)")
    # GPIO / chip
    ap.add_argument("--chip", default="gpiochip4", help="gpiochip name (default: gpiochip4; auto-fallsback if missing)")
    ap.add_argument("--left-a", type=int, default=21, help="Left encoder A GPIO (BCM)")
    ap.add_argument("--left-b", type=int, default=20, help="Left encoder B GPIO (BCM)")
    ap.add_argument("--right-a", type=int, default=12, help="Right encoder A GPIO (BCM)")
    ap.add_argument("--right-b", type=int, default=26, help="Right encoder B GPIO (BCM)")
    ap.add_argument("--invert-left", action="store_true", help="Invert left direction")
    ap.add_argument("--invert-right", action="store_true", help="Invert right direction")
    ap.add_argument("--debounce-us", type=int, default=300, help="Debounce window in microseconds (per accepted step)")

    # Mechanics (for speed only; no odom is computed)
    ap.add_argument("--wheel-diam", type=float, default=0.065, help="Wheel diameter (m)")
    ap.add_argument("--cpr", type=int, default=20, help="Encoder CPR (cycles per revolution on A)")
    ap.add_argument("--decoding", type=int, default=4, choices=[1,2,4], help="Decoding factor (1/2/4)")
    ap.add_argument("--gear", type=float, default=1.0, help="Gear ratio wheel:encoder (1.0 if direct)")

    # Run/print/log
    ap.add_argument("--duration", type=float, default=10.0, help="Duration seconds (<=0 for infinite)")
    ap.add_argument("--print-hz", type=float, default=5.0, help="Terminal print rate (Hz)")
    ap.add_argument("--csv", default="", help="Optional CSV path to log samples")

    args = ap.parse_args()

    # Open gpiochip
    chip = _open_chip(args.chip)
    try:
        chip_label = chip.label()
    except Exception:
        chip_label = args.chip
    print(f"[Encoders] Using {chip_label}  Pins L({args.left_a},{args.left_b}) R({args.right_a},{args.right_b})")
    mech = MechConfig(args.wheel_diam, args.cpr, args.decoding, args.gear)
    print(f"[Mech] wheel_dia={mech.wheel_diam:.3f} m  CPR={mech.cpr}  decoding={mech.decoding}  gear={mech.gear}  edges/rev={mech.edges_per_rev:.1f}")
    print(f"[Debounce] {args.debounce_us} µs  [Invert] left={args.invert_left} right={args.invert_right}")

    # Encoders
    encL = QuadEncoder(chip, WheelConfig(args.left_a, args.left_b, args.invert_left, "L"), args.debounce_us)
    encR = QuadEncoder(chip, WheelConfig(args.right_a, args.right_b, args.invert_right, "R"), args.debounce_us)

    # CSV
    csv_writer = None
    csv_fp = None
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        csv_fp = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_fp)
        csv_writer.writerow(["t_s", "L_cnt", "R_cnt", "L_dcnt", "R_dcnt", "dt_s", "L_v_mps", "R_v_mps", "L_illegal", "R_illegal"])

    # Print header
    print(" t(s) |   L_cnt    R_cnt | L_dcnt R_dcnt | dt(s) |  L_v(m/s)  R_v(m/s) | L_illeg R_illeg")
    print("-----------------------------------------------------------------------------------------")

    start_t = time.monotonic()
    last_t = start_t
    last_L = encL.count
    last_R = encR.count
    print_period = 1.0 / max(1e-3, args.print_hz)
    next_print = start_t

    try:
        while True:
            # Poll both encoders; non-blocking
            encL.poll_once()
            encR.poll_once()

            now = time.monotonic()
            if args.duration > 0 and (now - start_t) >= args.duration:
                break

            if now >= next_print:
                dt = now - last_t
                if dt <= 0:
                    dt = 1e-6

                L_cnt = encL.count
                R_cnt = encR.count
                dL = L_cnt - last_L
                dR = R_cnt - last_R

                # Convert count delta -> linear speed (m/s)
                # revs = dcounts / edges_per_rev ; distance = revs * wheel_circ ; v = distance / dt
                def speed(dcounts: int) -> float:
                    return (float(dcounts) / mech.edges_per_rev) * mech.wheel_circ / dt

                L_v = speed(dL)
                R_v = speed(dR)

                t_rel = now - start_t
                print(f"{_fmt(t_rel,5,1)} | {L_cnt:7d} {R_cnt:7d} | {dL:6d} {dR:6d} | {_fmt(dt,4,3)} | {_fmt(L_v,8,3)}  {_fmt(R_v,8,3)} | {encL.illegal:7d} {encR.illegal:7d}")

                if csv_writer:
                    csv_writer.writerow([f"{t_rel:.3f}", L_cnt, R_cnt, dL, dR, f"{dt:.6f}", f"{L_v:.6f}", f"{R_v:.6f}", encL.illegal, encR.illegal])

                # roll
                last_t = now
                last_L = L_cnt
                last_R = R_cnt
                next_print = now + print_period

            # Small sleep to reduce CPU, but keep latency low
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        encL.close()
        encR.close()
        if csv_fp:
            csv_fp.close()

    # Summary
    end_t = time.monotonic()
    dur = end_t - start_t
    print("\n=== Summary ===")
    print(f"Duration: {dur:.2f} s")
    print(f"L total: {encL.count}   R total: {encR.count}   L_illegal={encL.illegal}  R_illegal={encR.illegal}")
    print("Done.")

if __name__ == "__main__":
    main()
