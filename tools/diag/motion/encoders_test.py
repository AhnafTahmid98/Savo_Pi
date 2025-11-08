#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_encoders.py — Raspberry Pi 5 (Ubuntu 24.04), lgpio-based polling quadrature counter

Features
- Auto-detects gpiochip (tries 0..7) unless --chip is provided.
- Works with BCM pins (default: L_A=21, L_B=20, R_A=12, R_B=26).
- Quadrature via Gray-code state machine (legal transitions only).
- Hardware debounce via lgpio if supported, else software debounce.
- Prints total counts + per-0.5s deltas + live direction arrows.
- Optional CSV logging.
- Clean Ctrl+C and resource release.

Tips
- If a pin is "busy", check `gpioinfo` to see which overlay/driver holds it.
- On Pi 5, audio PWM often grabs GPIO12; SPI1 overlays may grab GPIO20/21.
"""

import argparse
import csv
import sys
import time
import signal

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

def pair_state(h, a, b):
    return ((lgpio.gpio_read(h, a) & 1) << 1) | (lgpio.gpio_read(h, b) & 1)

class QuadSide:
    def __init__(self, h, a, b, debounce_s, use_hw_debounce=True, pullup=True):
        self.h, self.a, self.b = h, a, b
        self.debounce_s = max(0.0, debounce_s)
        self.use_hw_debounce = use_hw_debounce
        self.pullup = pullup

        # Claim lines
        lgpio.gpio_claim_input(h, a)
        lgpio.gpio_claim_input(h, b)

        # Request pull-ups if kernel supports flags (harmless if ignored)
        if pullup:
            try:
                lgpio.gpio_set_flags(h, a, lgpio.BIAS_PULL_UP)
                lgpio.gpio_set_flags(h, b, lgpio.BIAS_PULL_UP)
            except Exception:
                pass

        # Hardware debounce (if supported by kernel/driver)
        self.hw_debounce = False
        if use_hw_debounce and self.debounce_s > 0:
            try:
                usec = int(self.debounce_s * 1e6)
                lgpio.gpio_set_debounce_micros(h, a, usec)
                lgpio.gpio_set_debounce_micros(h, b, usec)
                self.hw_debounce = True
            except Exception:
                self.hw_debounce = False  # fall back to software

        # Software debounce tracking
        now = time.monotonic()
        self.last_change = {a: now, b: now}
        self.state_bits = {
            a: lgpio.gpio_read(h, a),
            b: lgpio.gpio_read(h, b),
        }

        self.prev_state = ((self.state_bits[a] & 1) << 1) | (self.state_bits[b] & 1)
        self.count = 0
        self.dir = 0  # -1, 0, +1

    def sample_once(self, now):
        # Read with soft debounce if needed
        a_val = lgpio.gpio_read(self.h, self.a)
        b_val = lgpio.gpio_read(self.h, self.b)

        if not self.hw_debounce and self.debounce_s > 0:
            # Apply simple time-based debounce
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
                self.count += d
                self.dir = 1 if d > 0 else -1
            self.prev_state = s_new

def autodetect_chip(pins, start=0, end=7, pullup=True):
    """
    Try gpiochip[start..end] and return the first chip handle that can claim all pins.
    """
    last_error = None
    for chip in range(start, end + 1):
        try:
            h = lgpio.gpiochip_open(chip)
            claimed = []
            try:
                # Try to claim all; if any fails, release and continue
                for p in pins:
                    lgpio.gpio_claim_input(h, p)
                    claimed.append(p)
                    if pullup:
                        try:
                            lgpio.gpio_set_flags(h, p, lgpio.BIAS_PULL_UP)
                        except Exception:
                            pass
                # Success -> free and return handle freshly opened for main use
                for p in claimed:
                    lgpio.gpio_free(h, p)
                lgpio.gpiochip_close(h)
                # Re-open clean for the caller
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

def run(args):
    pins = [args.l_a, args.l_b, args.r_a, args.r_b]

    # Decide chip
    if args.chip is not None:
        h = lgpio.gpiochip_open(args.chip)
        chip_used = args.chip
    else:
        chip_used, h = autodetect_chip(pins)

    print(f"[Encoders] Using gpiochip{chip_used}  Pins L({args.l_a},{args.l_b}) R({args.r_a},{args.r_b})")
    print(f"[Debounce] target={int(args.debounce_s*1e6)} µs  polling={int(args.poll_s*1e6)} µs")

    # Build sides
    L = QuadSide(h, args.l_a, args.l_b, debounce_s=args.debounce_s,
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)
    R = QuadSide(h, args.r_a, args.r_b, debounce_s=args.debounce_s,
                 use_hw_debounce=not args.no_hw_debounce, pullup=not args.no_pullup)

    # CSV
    csvf = None
    writer = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow(["t_s", "L_count", "R_count", "L_delta_0.5s", "R_delta_0.5s", "L_dir", "R_dir"])

    stop = False
    def on_sigint(sig, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    t0 = time.monotonic()
    last_print = t0
    lastL = L.count
    lastR = R.count

    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0 and (now - t0) >= args.duration:
                break

            # Poll both sides
            L.sample_once(now)
            R.sample_once(now)

            # Print every 0.5 s
            if (now - last_print) >= 0.5:
                dLp = L.count - lastL
                dRp = R.count - lastR
                lastL, lastR = L.count, R.count
                dirL = "→" if L.dir > 0 else ("←" if L.dir < 0 else "•")
                dirR = "→" if R.dir > 0 else ("←" if R.dir < 0 else "•")
                print(f"L: {L.count:7d} (+{dLp:3d}/0.5s) {dirL}    "
                      f"R: {R.count:7d} (+{dRp:3d}/0.5s) {dirR}    "
                      f"[A,B: L=({lgpio.gpio_read(h,args.l_a)},{lgpio.gpio_read(h,args.l_b)}) "
                      f"R=({lgpio.gpio_read(h,args.r_a)},{lgpio.gpio_read(h,args.r_b)})]")

                if writer:
                    writer.writerow([f"{now - t0:.3f}", L.count, R.count, dLp, dRp, L.dir, R.dir])

                last_print = now

            time.sleep(args.poll_s)

    finally:
        # Cleanup
        try: lgpio.gpio_free(h, args.l_a)
        except: pass
        try: lgpio.gpio_free(h, args.l_b)
        except: pass
        try: lgpio.gpio_free(h, args.r_a)
        except: pass
        try: lgpio.gpio_free(h, args.r_b)
        except: pass
        lgpio.gpiochip_close(h)
        if csvf: csvf.close()

    # Summary
    T = time.monotonic() - t0
    print("\n=== Summary ===")
    print(f"Duration: {T:.2f} s")
    print(f"L total: {L.count}   R total: {R.count}")
    print(f"L last dir: {('→' if L.dir>0 else ('←' if L.dir<0 else '•'))}   "
          f"R last dir: {('→' if R.dir>0 else ('←' if R.dir<0 else '•'))}")
    if T > 0:
        print(f"Avg L rate: {L.count/T:.1f} counts/s   Avg R rate: {R.count/T:.1f} counts/s")

def parse_args():
    ap = argparse.ArgumentParser(
        description="Polling quadrature encoder test (Pi 5 / lgpio)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    ap.add_argument("--l-a", type=int, default=21, help="Left encoder A (BCM)")
    ap.add_argument("--l-b", type=int, default=20, help="Left encoder B (BCM)")
    ap.add_argument("--r-a", type=int, default=12, help="Right encoder A (BCM)")
    ap.add_argument("--r-b", type=int, default=26, help="Right encoder B (BCM)")
    ap.add_argument("--poll-s", type=float, default=0.001, help="Polling period (s)")
    ap.add_argument("--debounce-s", type=float, default=0.0003, help="Per-line debounce (s)")
    ap.add_argument("--chip", type=int, default=None, help="gpiochip index (auto if omitted)")
    ap.add_argument("--no-hw-debounce", action="store_true", help="Force software debounce only")
    ap.add_argument("--no-pullup", action="store_true", help="Do not request pull-ups (use external)")
    ap.add_argument("--duration", type=float, default=30.0, help="Duration in seconds (0 = until Ctrl+C)")
    ap.add_argument("--csv", type=str, default="", help="CSV output path")
    return ap.parse_args()

if __name__ == "__main__":
    run(parse_args())
