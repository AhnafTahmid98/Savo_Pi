#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/wheel_direction_check.py
-----------------------------------------------------------
Professional wheel-direction diagnostic for the Robot Savo Freenove/PCA9685 mecanum base.

Purpose
-------
Validate that each wheel spins in the expected direction for a commanded signed duty
before running higher-level teleop / nav stacks.

What it checks
--------------
- Drives ONE wheel at a time (FL, RL, FR, RR)
- Tests forward and reverse for each wheel
- Uses the locked Robot Savo Freenove channel mapping:
    FL -> (0,1)
    RL -> (3,2)
    FR -> (6,7)
    RR -> (4,5)
- Safe stop between steps + direction-change quench delay

Why this matters
----------------
If wheel polarity/channel mapping is wrong, mecanum motion will be unstable:
- forward may drift / rotate
- strafe may fail
- nav control may become unsafe

This diagnostic should be run BEFORE base_driver_node bringup on new hardware wiring,
after board replacement, or after motor driver code refactors.

Examples
--------
# Basic test (all wheels, default duty)
ros2 run savo_base wheel_direction_check.py

# Safer low-duty test
ros2 run savo_base wheel_direction_check.py --duty 1200 --step-time 0.8

# Test only front-left and front-right
ros2 run savo_base wheel_direction_check.py --wheels FL,FR

# Invert a wheel in software for the diagnostic (temporary)
ros2 run savo_base wheel_direction_check.py --invert-fl

# Dry-run (no hardware writes)
ros2 run savo_base wheel_direction_check.py --dry-run

Notes
-----
- This script uses direct PCA9685 I2C control (smbus2) and does NOT require ROS topics.
- Confirm the robot is lifted so wheels can spin freely.
- Keep clear of moving parts.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

try:
    import smbus2 as smbus  # preferred
except Exception:
    smbus = None


# =============================================================================
# Locked Robot Savo wheel/channel mapping (Freenove pair mode)
# =============================================================================
# Each wheel uses a pair of PCA9685 channels (A, B).
# Positive duty -> A=duty, B=0
# Negative duty -> A=0,   B=abs(duty)
WHEEL_CHANNELS: Dict[str, Tuple[int, int]] = {
    "FL": (0, 1),
    "RL": (3, 2),
    "FR": (6, 7),
    "RR": (4, 5),
}

WHEEL_ORDER: Tuple[str, ...] = ("FL", "RL", "FR", "RR")


# =============================================================================
# PCA9685 low-level driver (minimal + robust for diagnostics)
# =============================================================================
class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus: int = 1, addr: int = 0x40, dry_run: bool = False, verbose: bool = False):
        self.bus_num = int(bus)
        self.addr = int(addr)
        self.dry_run = bool(dry_run)
        self.verbose = bool(verbose)
        self.bus = None

        if self.dry_run:
            print(f"[DRYRUN] PCA9685 bus={self.bus_num} addr=0x{self.addr:02X}")
            return

        if smbus is None:
            raise RuntimeError(
                "smbus2 is not available. Install it first:\n"
                "  pip install smbus2\n"
                "or run with --dry-run"
            )

        self.bus = smbus.SMBus(self.bus_num)

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    def _write8(self, reg: int, val: int) -> None:
        if self.dry_run:
            if self.verbose:
                print(f"[DRYRUN] W8 reg=0x{reg:02X} val=0x{val & 0xFF:02X}")
            return
        self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)

    def _read8(self, reg: int) -> int:
        if self.dry_run:
            return 0x00
        return int(self.bus.read_byte_data(self.addr, reg & 0xFF))

    def set_pwm_freq(self, freq_hz: float) -> None:
        """Set PCA9685 PWM frequency (typically 50 Hz for this motor board setup)."""
        f = float(freq_hz)
        if f <= 0:
            raise ValueError("PWM frequency must be > 0")

        # PCA9685 oscillator ~25MHz, 12-bit resolution
        prescaleval = 25_000_000.0 / 4096.0 / f - 1.0
        prescale = int(round(prescaleval))
        if prescale < 3:
            prescale = 3
        if prescale > 255:
            prescale = 255

        oldmode = self._read8(self.MODE1)
        sleep_mode = (oldmode & 0x7F) | 0x10  # sleep
        self._write8(self.MODE1, sleep_mode)
        self._write8(self.PRESCALE, prescale)
        self._write8(self.MODE1, oldmode)
        time.sleep(0.005)
        self._write8(self.MODE1, oldmode | 0xA1)  # restart + auto-increment
        self._write8(self.MODE2, 0x04)  # totem-pole

        if self.verbose or self.dry_run:
            print(f"[INFO] PCA9685 freq set to {f:.1f} Hz (prescale={prescale})")

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Set raw 12-bit PWM for a channel."""
        if not (0 <= channel <= 15):
            raise ValueError(f"Invalid channel {channel}, expected 0..15")
        base = self.LED0_ON_L + 4 * int(channel)

        on &= 0x0FFF
        off &= 0x0FFF

        if self.dry_run:
            if self.verbose:
                print(f"[DRYRUN] ch{channel}: on={on} off={off}")
            return

        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_duty_12b(self, channel: int, duty: int) -> None:
        """Duty in [0..4095]."""
        d = int(duty)
        if d < 0:
            d = 0
        if d > 4095:
            d = 4095
        self.set_pwm(channel, 0, d)

    def all_stop(self) -> None:
        for ch in range(16):
            self.set_duty_12b(ch, 0)


# =============================================================================
# Freenove pair-mode motor helper
# =============================================================================
@dataclass
class WheelConfig:
    name: str
    ch_a: int
    ch_b: int
    invert: bool = False


class FreenovePairMotorBoard:
    """
    Minimal pair-mode motor board helper for diagnostics.

    Signed duty convention:
      +duty -> channel A active, channel B zero
      -duty -> channel A zero,   channel B active
    """
    def __init__(
        self,
        pca: PCA9685,
        *,
        quench_ms: int = 18,
        verbose: bool = False,
    ) -> None:
        self.pca = pca
        self.quench_s = max(0.0, float(quench_ms) / 1000.0)
        self.verbose = bool(verbose)
        self._last_sign: Dict[str, int] = {w: 0 for w in WHEEL_ORDER}

    def _apply_pair_signed_duty(self, cfg: WheelConfig, duty_signed: int) -> None:
        d = int(duty_signed)
        if cfg.invert:
            d = -d

        if d > 4095:
            d = 4095
        if d < -4095:
            d = -4095

        new_sign = 0 if d == 0 else (+1 if d > 0 else -1)
        prev_sign = self._last_sign.get(cfg.name, 0)

        # Quench on direction flip to reduce stress on H-bridge/motor
        if prev_sign != 0 and new_sign != 0 and prev_sign != new_sign:
            self.pca.set_duty_12b(cfg.ch_a, 0)
            self.pca.set_duty_12b(cfg.ch_b, 0)
            if self.verbose:
                print(f"[INFO] {cfg.name}: quench {self.quench_s*1000:.0f} ms (flip {prev_sign} -> {new_sign})")
            time.sleep(self.quench_s)

        if d > 0:
            self.pca.set_duty_12b(cfg.ch_a, d)
            self.pca.set_duty_12b(cfg.ch_b, 0)
        elif d < 0:
            self.pca.set_duty_12b(cfg.ch_a, 0)
            self.pca.set_duty_12b(cfg.ch_b, abs(d))
        else:
            self.pca.set_duty_12b(cfg.ch_a, 0)
            self.pca.set_duty_12b(cfg.ch_b, 0)

        self._last_sign[cfg.name] = new_sign

        if self.verbose:
            print(f"[DEBUG] {cfg.name}: duty={d:+d} -> ({cfg.ch_a},{cfg.ch_b})")

    def stop_all(self) -> None:
        self.pca.all_stop()
        for w in self._last_sign:
            self._last_sign[w] = 0


# =============================================================================
# Diagnostic routine
# =============================================================================
def parse_wheel_list(text: str) -> List[str]:
    if not text:
        return list(WHEEL_ORDER)
    items = [x.strip().upper() for x in text.split(",") if x.strip()]
    valid = set(WHEEL_ORDER)
    bad = [x for x in items if x not in valid]
    if bad:
        raise ValueError(f"Unknown wheel(s): {bad}. Valid: {', '.join(WHEEL_ORDER)}")
    # preserve canonical order
    return [w for w in WHEEL_ORDER if w in items]


def expected_note_for_direction(cmd_name: str) -> str:
    """
    Human-facing note for physical observation.
    We don't hard-code 'clockwise/counterclockwise' because mounting orientation
    can make that confusing. We focus on consistency and later mecanum mapping.
    """
    if cmd_name == "FORWARD_SIGN":
        return "Wheel should spin in the software-defined FORWARD direction for this wheel."
    if cmd_name == "REVERSE_SIGN":
        return "Wheel should spin in the software-defined REVERSE direction for this wheel."
    return "Observe direction."


def run_check(args: argparse.Namespace) -> int:
    wheels = parse_wheel_list(args.wheels)

    invert_flags = {
        "FL": bool(args.invert_fl),
        "RL": bool(args.invert_rl),
        "FR": bool(args.invert_fr),
        "RR": bool(args.invert_rr),
    }

    print("=" * 78)
    print("Robot SAVO Wheel Direction Check (Professional Diagnostic)")
    print("=" * 78)
    print(f"I2C bus           : {args.i2c_bus}")
    print(f"PCA9685 address   : 0x{args.addr:02X}")
    print(f"PWM frequency     : {args.pwm_freq:.1f} Hz")
    print(f"Test duty         : {args.duty}")
    print(f"Step time         : {args.step_time:.2f} s")
    print(f"Pause time        : {args.pause_time:.2f} s")
    print(f"Quench            : {args.quench_ms} ms")
    print(f"Dry run           : {args.dry_run}")
    print(f"Wheels            : {', '.join(wheels)}")
    print(
        "Inverts           : "
        f"FL={invert_flags['FL']} RL={invert_flags['RL']} FR={invert_flags['FR']} RR={invert_flags['RR']}"
    )
    print("-" * 78)
    print("Locked wheel->channel pairs (Robot Savo):")
    for w in WHEEL_ORDER:
        a, b = WHEEL_CHANNELS[w]
        print(f"  {w} -> ({a},{b})")
    print("-" * 78)
    print("SAFETY: Lift robot so wheels spin freely. Keep hands/clothes clear.")
    print("=" * 78)

    if not args.yes:
        try:
            ans = input("Type 'YES' to continue: ").strip()
        except KeyboardInterrupt:
            print("\nCancelled.")
            return 130
        if ans != "YES":
            print("Aborted by user.")
            return 1

    pca = None
    board = None

    try:
        pca = PCA9685(
            bus=args.i2c_bus,
            addr=args.addr,
            dry_run=args.dry_run,
            verbose=args.verbose,
        )
        pca.set_pwm_freq(args.pwm_freq)
        pca.all_stop()
        time.sleep(0.05)

        board = FreenovePairMotorBoard(
            pca,
            quench_ms=args.quench_ms,
            verbose=args.verbose,
        )

        # Build configs
        cfgs: Dict[str, WheelConfig] = {}
        for w in WHEEL_ORDER:
            ch_a, ch_b = WHEEL_CHANNELS[w]
            cfgs[w] = WheelConfig(name=w, ch_a=ch_a, ch_b=ch_b, invert=invert_flags[w])

        # Test sequence per wheel: +duty, stop, -duty, stop
        for idx, wheel in enumerate(wheels, start=1):
            cfg = cfgs[wheel]
            print()
            print(f"[{idx}/{len(wheels)}] Testing wheel {wheel}  channels=({cfg.ch_a},{cfg.ch_b}) invert={cfg.invert}")
            print(f"  +{args.duty}: {expected_note_for_direction('FORWARD_SIGN')}")
            board._apply_pair_signed_duty(cfg, +args.duty)
            time.sleep(args.step_time)

            print("  STOP")
            board._apply_pair_signed_duty(cfg, 0)
            time.sleep(args.pause_time)

            print(f"  -{args.duty}: {expected_note_for_direction('REVERSE_SIGN')}")
            board._apply_pair_signed_duty(cfg, -args.duty)
            time.sleep(args.step_time)

            print("  STOP")
            board._apply_pair_signed_duty(cfg, 0)
            time.sleep(args.pause_time)

            # Optional operator confirmation prompt per wheel
            if args.interactive_confirm:
                ans = input(
                    f"  Did {wheel} spin correctly in both directions? [y/n/skip]: "
                ).strip().lower()
                if ans in ("n", "no"):
                    print(f"  [WARN] {wheel} reported incorrect direction. Check invert flag or wiring.")
                elif ans in ("skip", "s"):
                    print(f"  [INFO] {wheel} result skipped.")
                else:
                    print(f"  [OK] {wheel} confirmed.")

        # Final all-stop
        board.stop_all()
        print()
        print("=" * 78)
        print("Wheel direction diagnostic completed.")
        print("Next professional step:")
        print("  - If any wheel direction is opposite, fix with invert flag in base_driver YAML")
        print("    or correct motor wiring (prefer stable wiring + software-documented invert).")
        print("  - Then run mecanum motion smoke test (forward/strafe/rotate).")
        print("=" * 78)
        return 0

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopping all motors...")
        try:
            if board is not None:
                board.stop_all()
            elif pca is not None:
                pca.all_stop()
        except Exception:
            pass
        return 130

    except Exception as e:
        print(f"[ERROR] {e}")
        try:
            if board is not None:
                board.stop_all()
            elif pca is not None:
                pca.all_stop()
        except Exception:
            pass
        return 1

    finally:
        try:
            if pca is not None:
                pca.close()
        except Exception:
            pass


# =============================================================================
# CLI
# =============================================================================
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="wheel_direction_check.py",
        description="Professional one-wheel-at-a-time direction diagnostic for Robot Savo Freenove mecanum base."
    )

    # Hardware
    p.add_argument("--i2c-bus", type=int, default=1, help="I2C bus number (default: 1)")
    p.add_argument("--addr", type=lambda x: int(str(x), 0), default=0x40, help="PCA9685 I2C address (default: 0x40)")
    p.add_argument("--pwm-freq", type=float, default=50.0, help="PCA9685 PWM frequency in Hz (default: 50.0)")
    p.add_argument("--quench-ms", type=int, default=18, help="Quench delay on direction flips in ms (default: 18)")

    # Test behavior
    p.add_argument("--duty", type=int, default=1500, help="Signed duty magnitude (0..4095), default: 1500")
    p.add_argument("--step-time", type=float, default=0.8, help="Duration of each spin step in seconds (default: 0.8)")
    p.add_argument("--pause-time", type=float, default=0.4, help="Pause after stop between steps in seconds (default: 0.4)")
    p.add_argument("--wheels", type=str, default="FL,RL,FR,RR", help="Comma list from FL,RL,FR,RR (default: all)")
    p.add_argument("--interactive-confirm", action="store_true", help="Ask manual y/n confirmation after each wheel")

    # Per-wheel invert (temporary diag override)
    p.add_argument("--invert-fl", action="store_true", help="Invert FL direction in this diagnostic")
    p.add_argument("--invert-rl", action="store_true", help="Invert RL direction in this diagnostic")
    p.add_argument("--invert-fr", action="store_true", help="Invert FR direction in this diagnostic")
    p.add_argument("--invert-rr", action="store_true", help="Invert RR direction in this diagnostic")

    # Modes
    p.add_argument("--dry-run", action="store_true", help="Do not write hardware, print actions only")
    p.add_argument("--verbose", action="store_true", help="Verbose debug prints")
    p.add_argument("--yes", action="store_true", help="Skip safety confirmation prompt")

    return p


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    # Sanity clamps
    if args.duty < 0:
        args.duty = abs(int(args.duty))
    if args.duty > 4095:
        args.duty = 4095
    args.step_time = max(0.05, float(args.step_time))
    args.pause_time = max(0.0, float(args.pause_time))
    args.quench_ms = max(0, int(args.quench_ms))

    return run_check(args)


if __name__ == "__main__":
    sys.exit(main())