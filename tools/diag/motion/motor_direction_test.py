#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Motor Direction Test (PCA9685 pair-mode + Encoder Polling)
------------------------------------------------------------------------
Goal
  Verify encoder sign conventions:
    • Forward      ⇒ ΔL > 0 and ΔR > 0
    • Reverse      ⇒ ΔL < 0 and ΔR < 0
    • Rotate CCW   ⇒ ΔL < 0 and ΔR > 0  (optional)
    • Rotate CW    ⇒ ΔL > 0 and ΔR < 0  (optional)

Aligned with drive_manual_direct.py:
  • Pair-mode per wheel (NO PWM/IN1/IN2 triplets)
  • Channel pairs (LOCKED):
        FL(0,1)  RL(3,2)  FR(6,7)  RR(4,5)
  • Quench on sign flip to protect H-bridge
  • Defaults: pwm-freq=50 Hz, max-duty=3000, quench=18 ms

Hardware assumptions
  • PCA9685 @ 0x40 on I²C-1 (Pi 5)
  • Encoders (LOCKED) with external pull-ups:
        Left  : L_A=21, L_B=20
        Right : R_A=12, R_B=26
  • Robot on blocks (wheels free)

Why this updated version
  • Fixes Raspberry Pi 5 gpiochip mismatch issues ("GPIO busy" on wrong chip)
  • Adds gpiochip auto-detection (like your working encoders_test.py)
  • Supports explicit --chip override (recommended on Pi 5: --chip 4)
  • Cleans up claimed GPIO lines on exit

Usage
  # Recommended on Pi 5 (explicit chip + external pull-ups)
  python3 motor_direction_test.py --chip 4 --no-pullup --with-rotate

  # Auto-detect gpiochip (tries gpiochip0..7)
  python3 motor_direction_test.py --no-pullup --with-rotate
"""

from __future__ import annotations

import sys
import time
import math
import argparse
from dataclasses import dataclass
from typing import Tuple, Optional

# ============================ Encoders (lgpio) ================================
try:
    import lgpio
except Exception:
    print("ERROR: lgpio is required. Install: sudo apt install -y python3-lgpio", file=sys.stderr)
    raise

# LOCKED encoder pins (BCM)
L_A, L_B = 21, 20
R_A, R_B = 12, 26

# Gray-code step table (state -> state) increments
GRAY_NEXT = {
    (0, 1): +1, (1, 3): +1, (3, 2): +1, (2, 0): +1,
    (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1,
}


def _safe_set_pullup(chip_handle: int, pin: int) -> None:
    """Best-effort internal pull-up request (ignored if driver/kernel rejects it)."""
    try:
        lgpio.gpio_set_flags(chip_handle, pin, lgpio.BIAS_PULL_UP)
    except Exception:
        pass


def autodetect_gpiochip_for_pins(
    pins: Tuple[int, ...],
    start: int = 0,
    end: int = 7,
    pullup: bool = False,
) -> Tuple[int, int]:
    """
    Find a gpiochip that can claim all requested BCM lines.

    Returns:
        (chip_index, chip_handle_opened)

    Notes:
    - On Raspberry Pi 5, user GPIO lines are usually on gpiochip4.
    - This method prevents false success on gpiochip0 where line numbers may exist
      but refer to unrelated SoC/internal lines (e.g., power button GPIO).
    """
    last_error = None

    for chip_idx in range(start, end + 1):
        h = None
        claimed = []
        try:
            h = lgpio.gpiochip_open(chip_idx)
            for pin in pins:
                lgpio.gpio_claim_input(h, pin)
                claimed.append(pin)
                if pullup:
                    _safe_set_pullup(h, pin)

            # Success: release temporary claims, then re-open fresh handle for caller
            for pin in claimed:
                try:
                    lgpio.gpio_free(h, pin)
                except Exception:
                    pass
            lgpio.gpiochip_close(h)

            return chip_idx, lgpio.gpiochip_open(chip_idx)

        except Exception as e:
            last_error = e
            if h is not None:
                for pin in claimed:
                    try:
                        lgpio.gpio_free(h, pin)
                    except Exception:
                        pass
                try:
                    lgpio.gpiochip_close(h)
                except Exception:
                    pass

    raise RuntimeError(
        f"Could not find a gpiochip that can claim pins {list(pins)}. Last error: {last_error}"
    )


@dataclass
class QuadPoll:
    chip: int
    a: int
    b: int
    debounce_s: float
    count: int = 0
    last_state: int = 0
    illegal: int = 0
    _last_change_a: float = 0.0
    _last_change_b: float = 0.0
    _state_a: int = 0
    _state_b: int = 0
    _claimed: bool = False

    def init_state(self, pullup: bool = False):
        # Claim inputs (Pi 5: must be on correct gpiochip, typically gpiochip4)
        lgpio.gpio_claim_input(self.chip, self.a)
        lgpio.gpio_claim_input(self.chip, self.b)
        self._claimed = True

        if pullup:
            _safe_set_pullup(self.chip, self.a)
            _safe_set_pullup(self.chip, self.b)

        self._state_a = lgpio.gpio_read(self.chip, self.a)
        self._state_b = lgpio.gpio_read(self.chip, self.b)
        self.last_state = ((self._state_a & 1) << 1) | (self._state_b & 1)

        now = time.monotonic()
        self._last_change_a = now
        self._last_change_b = now

    def step(self, now: float):
        a_val = lgpio.gpio_read(self.chip, self.a)
        b_val = lgpio.gpio_read(self.chip, self.b)

        # Software debounce (simple hold-last-state)
        if self.debounce_s > 0:
            if a_val != self._state_a:
                if (now - self._last_change_a) >= self.debounce_s:
                    self._state_a = a_val
                    self._last_change_a = now
                else:
                    a_val = self._state_a
            else:
                self._last_change_a = now

            if b_val != self._state_b:
                if (now - self._last_change_b) >= self.debounce_s:
                    self._state_b = b_val
                    self._last_change_b = now
                else:
                    b_val = self._state_b
            else:
                self._last_change_b = now

        s_new = ((a_val & 1) << 1) | (b_val & 1)
        if s_new != self.last_state:
            d = GRAY_NEXT.get((self.last_state, s_new))
            if d is None:
                self.illegal += 1
            else:
                self.count += d
            self.last_state = s_new

    def close(self):
        if self._claimed:
            for pin in (self.a, self.b):
                try:
                    lgpio.gpio_free(self.chip, pin)
                except Exception:
                    pass
            self._claimed = False


@dataclass
class Encoders:
    chip: int                  # lgpio handle
    chip_idx: int              # gpiochip index (e.g., 4 on Pi 5)
    L: QuadPoll
    R: QuadPoll


def encoders_open_poll(
    chip_idx: Optional[int],
    pullup: bool,
    poll_debounce_s: float = 0.0003,
) -> Encoders:
    """
    Open encoder poller with optional gpiochip autodetect.

    If chip_idx is None:
      - tries to autodetect a gpiochip that can claim BCM 21,20,12,26
      - this avoids Pi 5 gpiochip0 false/invalid mapping issues
    """
    pins = (L_A, L_B, R_A, R_B)

    if chip_idx is not None:
        # Explicit path (recommended on Pi 5: --chip 4)
        chip_handle = lgpio.gpiochip_open(chip_idx)
        selected_idx = chip_idx
    else:
        selected_idx, chip_handle = autodetect_gpiochip_for_pins(pins, start=0, end=7, pullup=False)

    # Build pollers and claim lines
    L = QuadPoll(chip_handle, L_A, L_B, debounce_s=poll_debounce_s)
    R = QuadPoll(chip_handle, R_A, R_B, debounce_s=poll_debounce_s)

    try:
        L.init_state(pullup=pullup)
        R.init_state(pullup=pullup)
    except Exception:
        # Clean up partial claims on failure
        try:
            L.close()
        except Exception:
            pass
        try:
            R.close()
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(chip_handle)
        except Exception:
            pass
        raise

    return Encoders(chip=chip_handle, chip_idx=selected_idx, L=L, R=R)


def encoders_close(E: Encoders):
    try:
        E.L.close()
    except Exception:
        pass
    try:
        E.R.close()
    except Exception:
        pass
    try:
        lgpio.gpiochip_close(E.chip)
    except Exception:
        pass


# ============================= PCA9685 (pair) =================================
import smbus


class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, bus: int = 1, address: int = 0x40, debug: bool = False):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        if duty < 0:
            duty = 0
        if duty > 4095:
            duty = 4095
        self.set_pwm(channel, 0, duty)

    def close(self) -> None:
        self.bus.close()


class PairMotorDriver:
    """
    Pair-mode (matches drive_manual_direct.py):
      Wheels order: 0=FL, 1=RL, 2=FR, 3=RR
      Channel pairs (LOCKED): FL(0,1) RL(3,2) FR(6,7) RR(4,5)
    """
    PAIRS_A = [0, 3, 6, 4]  # first channel of each pair
    PAIRS_B = [1, 2, 7, 5]  # second channel of each pair

    def __init__(
        self,
        i2c_bus: int,
        addr: int,
        pwm_freq: float,
        max_duty: int = 3000,
        quench_ms: int = 18,
    ):
        self.pca = PCA9685(bus=i2c_bus, address=addr, debug=True)
        self.pca.set_pwm_freq(pwm_freq)
        self.max_duty = max(0, min(4095, int(max_duty)))
        self.quench_ms = max(0, int(quench_ms))
        self._last_sign = [0, 0, 0, 0]
        self.stop()

    def _wheel_cmd(self, idx: int, val_norm: float):
        # quench on sign flip
        sign = 0 if abs(val_norm) < 1e-6 else (1 if val_norm > 0 else -1)
        if self.quench_ms and self._last_sign[idx] and sign and (self._last_sign[idx] != sign):
            a = self.PAIRS_A[idx]
            b = self.PAIRS_B[idx]
            self.pca.set_motor_pwm(a, 4095)
            self.pca.set_motor_pwm(b, 4095)
            time.sleep(self.quench_ms / 1000.0)

        v = max(-1.0, min(1.0, float(val_norm)))
        duty = int(abs(v) * self.max_duty)
        a = self.PAIRS_A[idx]
        b = self.PAIRS_B[idx]

        if v > 0:
            self.pca.set_motor_pwm(a, 0)
            self.pca.set_motor_pwm(b, duty)
        elif v < 0:
            self.pca.set_motor_pwm(b, 0)
            self.pca.set_motor_pwm(a, duty)
        else:
            self.pca.set_motor_pwm(a, 4095)
            self.pca.set_motor_pwm(b, 4095)

        self._last_sign[idx] = sign

    def drive_side(self, indices: Tuple[int, ...], val_norm: float):
        for i in indices:
            self._wheel_cmd(i, val_norm)

    def stop(self):
        for a, b in zip(self.PAIRS_A, self.PAIRS_B):
            self.pca.set_motor_pwm(a, 4095)
            self.pca.set_motor_pwm(b, 4095)
        self._last_sign = [0, 0, 0, 0]

    def close(self):
        try:
            self.stop()
        finally:
            self.pca.close()


# ================================ Test Core ===================================
def measure_poll(E: Encoders, seconds: float):
    L0, R0 = E.L.count, E.R.count
    iL0, iR0 = E.L.illegal, E.R.illegal

    t0 = time.monotonic()
    while (time.monotonic() - t0) < seconds:
        now = time.monotonic()
        E.L.step(now)
        E.R.step(now)
        time.sleep(0.001)  # ~1 kHz polling

    dL = E.L.count - L0
    dR = E.R.count - R0
    iL = E.L.illegal - iL0
    iR = E.R.illegal - iR0
    return dL, dR, iL, iR


def print_step(name: str, dL: int, dR: int, iL: int, iR: int, expect_side: Optional[str]):
    extra = ""
    if expect_side in ("L", "R"):
        active = dL if expect_side == "L" else dR
        sign = "+" if active > 0 else "-" if active < 0 else "0"
        extra = f"  sign({expect_side})={sign}"
    print(f"{name:<14} | ΔL={dL:>6}  ΔR={dR:>6}  illegal(L/R)={iL}/{iR}{extra}")


def suggest(forwL: int, forwR: int) -> str:
    invL = forwL < 0
    invR = forwR < 0
    if invL and invR:
        return "Suggestion: invert BOTH sides in teleop (e.g., --invert-fl/rl and --invert-fr/rr)."
    if invL:
        return "Suggestion: invert LEFT side in teleop (e.g., --invert-fl and/or --invert-rl)."
    if invR:
        return "Suggestion: invert RIGHT side in teleop (e.g., --invert-fr and/or --invert-rr)."
    return "Suggestion: OK — forward is positive on both sides."


def parse_group(group_s: str) -> Tuple[int, ...]:
    vals = tuple(int(x.strip()) for x in group_s.split(",") if x.strip() != "")
    if not vals:
        raise ValueError("group cannot be empty")
    for v in vals:
        if v not in (0, 1, 2, 3):
            raise ValueError(f"group index must be one of 0,1,2,3; got {v}")
    return vals


# =================================== Main =====================================
def main():
    ap = argparse.ArgumentParser(description="Motor direction test (PCA9685 pair-mode) + encoder polling")

    # Mode & timings
    ap.add_argument(
        "--driver",
        choices=["manual", "pca"],
        default="pca",
        help="manual: you power motors; pca: use PCA9685 pair-mode (default)",
    )
    ap.add_argument("--seconds", type=float, default=2.0, help="Observation window per step")
    ap.add_argument("--settle", type=float, default=0.6, help="Idle settle time between steps")
    ap.add_argument("--duty", type=float, default=0.35, help="Normalized duty (0..1) in auto mode")
    ap.add_argument("--quench-ms", type=int, default=18, help="Neutral ms on sign flip")
    ap.add_argument("--with-rotate", action="store_true", help="Also test rotation CCW/CW")

    # PCA9685 addressing
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pwm-addr", type=lambda x: int(x, 0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=50.0)
    ap.add_argument("--max-duty", type=int, default=3000, help="12-bit cap (<=4095)")

    # Grouping by side (indices 0..3 = FL,RL,FR,RR)
    ap.add_argument("--left-group", type=str, default="0,1", help="Left side wheels (default: FL,RL)")
    ap.add_argument("--right-group", type=str, default="2,3", help="Right side wheels (default: FR,RR)")

    # Encoders / gpiochip
    ap.add_argument(
        "--chip",
        type=int,
        default=None,
        help="gpiochip index (Pi 5 commonly 4). If omitted, auto-detect is used.",
    )
    ap.add_argument(
        "--no-pullup",
        action="store_true",
        help="Skip internal pull-ups (recommended if you have external pull-ups)",
    )
    ap.add_argument("--poll-debounce-s", type=float, default=0.0003, help="Polling debounce per line")

    args = ap.parse_args()

    if not (0.0 <= args.duty <= 1.0):
        ap.error("--duty must be in [0, 1]")

    left_idxs = parse_group(args.left_group)
    right_idxs = parse_group(args.right_group)

    E = None
    drv = None

    try:
        # Encoders
        E = encoders_open_poll(
            chip_idx=args.chip,
            pullup=(not args.no_pullup),
            poll_debounce_s=args.poll_debounce_s,
        )

        print("[Motor Direction Test] Robot on blocks (wheels free).")
        print(f"Window: {args.seconds:.2f}s  Settle: {args.settle:.2f}s  Duty: {args.duty:.2f}")
        print(f"GPIO chip: gpiochip{E.chip_idx}  (explicit={args.chip is not None})")
        print(
            f"Encoder pull-ups: {'internal ON' if (not args.no_pullup) else 'internal OFF (external pull-ups expected)'}"
        )
        print("BCM pins: Left(21,20)  Right(12,26)")
        print("Expect overall: FORWARD => ΔL>0 and ΔR>0\n")

        if args.driver == "pca":
            drv = PairMotorDriver(
                i2c_bus=args.i2c_bus,
                addr=args.pwm_addr,
                pwm_freq=args.pwm_freq,
                max_duty=args.max_duty,
                quench_ms=args.quench_ms,
            )

        # --- LEFT FORWARD ---
        if drv is None:
            input(f">>> Step 1: LEFT FORWARD — power ONLY LEFT side forward for {args.seconds:.1f}s, then Enter...")
        else:
            drv.drive_side(left_idxs, +args.duty)
        dL1, dR1, iL1, iR1 = measure_poll(E, args.seconds)
        if drv:
            drv.stop()
        print_step("LEFT forward", dL1, dR1, iL1, iR1, "L")
        time.sleep(args.settle)

        # --- LEFT REVERSE ---
        if drv is None:
            input(f">>> Step 2: LEFT REVERSE — power ONLY LEFT side reverse for {args.seconds:.1f}s, then Enter...")
        else:
            drv.drive_side(left_idxs, -args.duty)
        dL2, dR2, iL2, iR2 = measure_poll(E, args.seconds)
        if drv:
            drv.stop()
        print_step("LEFT reverse", dL2, dR2, iL2, iR2, "L")
        time.sleep(args.settle)

        # --- RIGHT FORWARD ---
        if drv is None:
            input(f">>> Step 3: RIGHT FORWARD — power ONLY RIGHT side forward for {args.seconds:.1f}s, then Enter...")
        else:
            drv.drive_side(right_idxs, +args.duty)
        dL3, dR3, iL3, iR3 = measure_poll(E, args.seconds)
        if drv:
            drv.stop()
        print_step("RIGHT forward", dL3, dR3, iL3, iR3, "R")
        time.sleep(args.settle)

        # --- RIGHT REVERSE ---
        if drv is None:
            input(f">>> Step 4: RIGHT REVERSE — power ONLY RIGHT side reverse for {args.seconds:.1f}s, then Enter...")
        else:
            drv.drive_side(right_idxs, -args.duty)
        dL4, dR4, iL4, iR4 = measure_poll(E, args.seconds)
        if drv:
            drv.stop()
        print_step("RIGHT reverse", dL4, dR4, iL4, iR4, "R")

        # --- ROTATION (optional) ---
        dL_ccw = dR_ccw = iL_ccw = iR_ccw = 0
        dL_cw = dR_cw = iL_cw = iR_cw = 0

        if args.with_rotate:
            time.sleep(args.settle)

            # CCW: left -, right +
            if drv is None:
                input(f">>> Step 5: ROTATE CCW — LEFT reverse, RIGHT forward for {args.seconds:.1f}s, then Enter...")
            else:
                drv.drive_side(left_idxs, -args.duty)
                drv.drive_side(right_idxs, +args.duty)
            dL_ccw, dR_ccw, iL_ccw, iR_ccw = measure_poll(E, args.seconds)
            if drv:
                drv.stop()
            print_step("ROT CCW", dL_ccw, dR_ccw, iL_ccw, iR_ccw, None)  # expect L<0, R>0

            time.sleep(args.settle)

            # CW: left +, right -
            if drv is None:
                input(f">>> Step 6: ROTATE CW — LEFT forward, RIGHT reverse for {args.seconds:.1f}s, then Enter...")
            else:
                drv.drive_side(left_idxs, +args.duty)
                drv.drive_side(right_idxs, -args.duty)
            dL_cw, dR_cw, iL_cw, iR_cw = measure_poll(E, args.seconds)
            if drv:
                drv.stop()
            print_step("ROT CW", dL_cw, dR_cw, iL_cw, iR_cw, None)  # expect L>0, R<0

        # Summary
        print("\n=== SUMMARY ===")
        print(f"LEFT forward   ΔL={dL1:>6}  illegal={iL1}")
        print(f"LEFT reverse   ΔL={dL2:>6}  illegal={iL2}")
        print(f"RIGHT forward  ΔR={dR3:>6}  illegal={iR3}")
        print(f"RIGHT reverse  ΔR={dR4:>6}  illegal={iR4}")
        if args.with_rotate:
            print(f"ROT CCW        ΔL={dL_ccw:>6}  ΔR={dR_ccw:>6}  (expect L<0, R>0)")
            print(f"ROT CW         ΔL={dL_cw:>6}  ΔR={dR_cw:>6}  (expect L>0, R<0)")
        print(suggest(dL1, dR3))
        print("Target: forward motion => ΔL>0 and ΔR>0")

    finally:
        try:
            if drv:
                drv.close()
        except Exception:
            pass
        try:
            if E is not None:
                encoders_close(E)
        except Exception:
            pass


if __name__ == "__main__":
    main()