#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/pwm_sweep_check.py
-----------------------------------------------------
Professional PWM/motor sweep diagnostic for Robot Savo Freenove mecanum base.

Purpose
-------
Safely validate the motor board write path (PCA9685/Freenove backend) and basic
wheel direction behavior by sending short, controlled signed-duty sweeps.

What this checks
----------------
- `savo_base.drivers.make_motor_board(...)` factory integration
- PCA9685 I2C communication and motor write path
- Per-wheel command mapping path (FL, RL, FR, RR order)
- Direction changes with configurable quench delay
- Optional sequence patterns: single-wheel, all-wheel forward/reverse, rotate

What this does NOT check
------------------------
- Closed-loop odometry accuracy
- Kinematics correctness under load
- Encoder sign correctness (use your encoder / wheel odom diagnostics for that)

Safety Notes (IMPORTANT)
------------------------
- Run with robot lifted (wheels off the ground) for initial testing.
- Keep a physical emergency stop / power cut ready.
- Start with low duty (e.g., 400–1000).
- This script is open-loop and can move the robot if on the ground.
- By default, dry-run is available and recommended first.

Examples
--------
# Dry-run smoke test (no hardware movement)
python3 pwm_sweep_check.py --dryrun --pattern single --verbose

# Real hardware, low duty single-wheel sweep (recommended first)
python3 pwm_sweep_check.py --backend auto --pattern single --max-duty 700 --step-duty 350

# All wheels forward/reverse sequence
python3 pwm_sweep_check.py --pattern all --max-duty 900 --step-duty 300

# Include rotate-style pattern (CCW / CW)
python3 pwm_sweep_check.py --pattern rotate --max-duty 900 --step-duty 300

# Custom board params
python3 pwm_sweep_check.py --backend freenove --i2c-bus 1 --addr 0x40 --pwm-freq-hz 50 --quench-ms 18
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# =============================================================================
# Optional imports from savo_base (preferred)
# =============================================================================
try:
    from savo_base.drivers import make_motor_board
    _HAS_BOARD_FACTORY = True
except Exception:
    make_motor_board = None
    _HAS_BOARD_FACTORY = False

# Constants are optional; fall back to known Robot Savo defaults
try:
    from savo_base.constants import (
        BOARD_BACKEND_DEFAULT,
        BOARD_NAME_DEFAULT,
        I2C_BUS_DEFAULT,
        PCA9685_ADDR_DEFAULT,
        PCA9685_PWM_FREQ_HZ_DEFAULT,
        QUENCH_MS_DEFAULT,
        MAX_DUTY_DEFAULT,
        MAX_DUTY_ABSOLUTE,
    )
except Exception:
    BOARD_BACKEND_DEFAULT = "auto"
    BOARD_NAME_DEFAULT = "robot_savo_freenove_mecanum"
    I2C_BUS_DEFAULT = 1
    PCA9685_ADDR_DEFAULT = 0x40
    PCA9685_PWM_FREQ_HZ_DEFAULT = 50.0
    QUENCH_MS_DEFAULT = 18
    MAX_DUTY_DEFAULT = 3000
    MAX_DUTY_ABSOLUTE = 4095


# =============================================================================
# Helpers
# =============================================================================
WHEEL_ORDER = ("FL", "RL", "FR", "RR")


def _parse_int_auto(text: str) -> int:
    return int(str(text), 0)


def _clamp_int(v: int, lo: int, hi: int) -> int:
    iv = int(v)
    return lo if iv < lo else hi if iv > hi else iv


def _sleep_s(seconds: float) -> None:
    if seconds > 0.0:
        time.sleep(seconds)


def _fmt_ms(ms: int) -> str:
    return f"{int(ms)} ms"


def _fmt_addr(addr: int) -> str:
    return f"0x{int(addr):02X}"


# =============================================================================
# Data model
# =============================================================================
@dataclass
class SweepConfig:
    backend: str
    board_name: str
    dryrun: bool
    i2c_bus: int
    addr: int
    pwm_freq_hz: float
    quench_ms: int
    inv: Tuple[int, int, int, int]

    pattern: str                 # single | all | rotate | full
    max_duty: int
    step_duty: int
    hold_s: float
    settle_s: float
    zero_between_steps: bool
    repeat: int
    verbose: bool
    no_confirm: bool


# =============================================================================
# Board adapter wrapper
# =============================================================================
class MotorBoardAdapter:
    """
    Thin wrapper for compatible motor board interfaces.

    Expected methods on actual board object:
    - set_motor_model(fl, rl, fr, rr)   (Freenove style)
    OR
    - write_wheel_duty(fl, rl, fr, rr)
    Optional:
    - stop()
    - close()
    """
    def __init__(self, board_obj, verbose: bool = False) -> None:
        self.board = board_obj
        self.verbose = bool(verbose)
        self.write_count = 0

    def write(self, fl: int, rl: int, fr: int, rr: int) -> None:
        if self.board is None:
            raise RuntimeError("Motor board not initialized")

        vals = (int(fl), int(rl), int(fr), int(rr))
        if self.verbose:
            print(f"[WRITE] FL={vals[0]:>5} RL={vals[1]:>5} FR={vals[2]:>5} RR={vals[3]:>5}")

        if hasattr(self.board, "set_motor_model"):
            self.board.set_motor_model(*vals)
            self.write_count += 1
            return
        if hasattr(self.board, "write_wheel_duty"):
            self.board.write_wheel_duty(*vals)
            self.write_count += 1
            return

        raise RuntimeError("Board object has no set_motor_model() or write_wheel_duty()")

    def stop(self) -> None:
        if self.board is None:
            return
        try:
            if hasattr(self.board, "stop"):
                self.board.stop()
            elif hasattr(self.board, "set_motor_model"):
                self.board.set_motor_model(0, 0, 0, 0)
            elif hasattr(self.board, "write_wheel_duty"):
                self.board.write_wheel_duty(0, 0, 0, 0)
        except Exception as e:
            print(f"[WARN] stop() failed: {e}", file=sys.stderr)

    def close(self) -> None:
        if self.board is None:
            return
        try:
            if hasattr(self.board, "close"):
                self.board.close()
        except Exception as e:
            print(f"[WARN] close() failed: {e}", file=sys.stderr)


# =============================================================================
# Board init
# =============================================================================
def build_board(cfg: SweepConfig) -> MotorBoardAdapter:
    if not _HAS_BOARD_FACTORY or make_motor_board is None:
        raise RuntimeError(
            "savo_base.drivers.make_motor_board is not available. "
            "Ensure your svo_base package is sourced and installed."
        )

    backend = "dryrun" if cfg.dryrun else cfg.backend

    attempts = [
        dict(
            backend=backend,
            board_name=cfg.board_name,
            i2c_bus=cfg.i2c_bus,
            addr=cfg.addr,
            pwm_freq=cfg.pwm_freq_hz,
            quench_ms=cfg.quench_ms,
            inv=cfg.inv,
        ),
        dict(
            backend=backend,
            i2c_bus=cfg.i2c_bus,
            addr=cfg.addr,
            pwm_freq=cfg.pwm_freq_hz,
            quench_ms=cfg.quench_ms,
            inv=cfg.inv,
        ),
        dict(
            backend=backend,
            name=cfg.board_name,
            i2c_bus=cfg.i2c_bus,
            pca9685_addr=cfg.addr,
            pwm_freq_hz=cfg.pwm_freq_hz,
            quench_ms=cfg.quench_ms,
            inv=cfg.inv,
        ),
    ]

    last_err: Optional[Exception] = None
    for kwargs in attempts:
        try:
            obj = make_motor_board(**kwargs)
            return MotorBoardAdapter(obj, verbose=cfg.verbose)
        except TypeError as e:
            last_err = e
            continue
        except Exception as e:
            last_err = e
            break

    raise RuntimeError(f"Failed to initialize motor board: {last_err}")


# =============================================================================
# Sweep execution primitives
# =============================================================================
def _write_and_hold(board: MotorBoardAdapter, vals: Tuple[int, int, int, int], hold_s: float) -> None:
    board.write(*vals)
    _sleep_s(hold_s)


def _zero_and_settle(board: MotorBoardAdapter, settle_s: float) -> None:
    board.stop()
    _sleep_s(settle_s)


def _duty_steps(step_duty: int, max_duty: int) -> List[int]:
    step = max(1, int(abs(step_duty)))
    maxd = max(1, int(abs(max_duty)))
    values = list(range(step, maxd + 1, step))
    if values[-1] != maxd:
        values.append(maxd)
    return values


def _wheel_vector(wheel_index: int, duty: int) -> Tuple[int, int, int, int]:
    vals = [0, 0, 0, 0]
    vals[int(wheel_index)] = int(duty)
    return tuple(vals)  # type: ignore[return-value]


def _print_step_header(title: str) -> None:
    print(f"\n--- {title} ---")


# =============================================================================
# Patterns
# =============================================================================
def run_single_wheel_pattern(board: MotorBoardAdapter, cfg: SweepConfig) -> None:
    _print_step_header("Pattern: SINGLE-WHEEL SWEEP (FL, RL, FR, RR)")
    steps = _duty_steps(cfg.step_duty, cfg.max_duty)

    for wheel_idx, wheel_name in enumerate(WHEEL_ORDER):
        print(f"\n[{wheel_name}] forward sweep")
        for d in steps:
            print(f"  {wheel_name}: +{d}")
            _write_and_hold(board, _wheel_vector(wheel_idx, +d), cfg.hold_s)
            if cfg.zero_between_steps:
                _zero_and_settle(board, cfg.settle_s)

        print(f"[{wheel_name}] reverse sweep")
        for d in steps:
            print(f"  {wheel_name}: -{d}")
            _write_and_hold(board, _wheel_vector(wheel_idx, -d), cfg.hold_s)
            if cfg.zero_between_steps:
                _zero_and_settle(board, cfg.settle_s)

        _zero_and_settle(board, cfg.settle_s)


def run_all_wheels_pattern(board: MotorBoardAdapter, cfg: SweepConfig) -> None:
    _print_step_header("Pattern: ALL-WHEELS FORWARD / REVERSE")
    steps = _duty_steps(cfg.step_duty, cfg.max_duty)

    print("\n[ALL] forward (same signed duty to all wheels)")
    for d in steps:
        print(f"  ALL: +{d}")
        _write_and_hold(board, (+d, +d, +d, +d), cfg.hold_s)
        if cfg.zero_between_steps:
            _zero_and_settle(board, cfg.settle_s)

    print("\n[ALL] reverse (same signed duty to all wheels)")
    for d in steps:
        print(f"  ALL: -{d}")
        _write_and_hold(board, (-d, -d, -d, -d), cfg.hold_s)
        if cfg.zero_between_steps:
            _zero_and_settle(board, cfg.settle_s)

    _zero_and_settle(board, cfg.settle_s)


def run_rotate_pattern(board: MotorBoardAdapter, cfg: SweepConfig) -> None:
    """
    Rotate test patterns use diff-sign style across left/right pairs in wheel order FL, RL, FR, RR.
    Convention here:
      CCW test: left negative, right positive  -> (-,-,+,+)
      CW  test: left positive, right negative  -> (+,+,-,-)
    """
    _print_step_header("Pattern: ROTATE (CCW / CW style)")
    steps = _duty_steps(cfg.step_duty, cfg.max_duty)

    print("\n[ROTATE] CCW style  (FL,RL=- ; FR,RR=+)")
    for d in steps:
        print(f"  CCW: {d}")
        _write_and_hold(board, (-d, -d, +d, +d), cfg.hold_s)
        if cfg.zero_between_steps:
            _zero_and_settle(board, cfg.settle_s)

    print("\n[ROTATE] CW style   (FL,RL=+ ; FR,RR=-)")
    for d in steps:
        print(f"  CW : {d}")
        _write_and_hold(board, (+d, +d, -d, -d), cfg.hold_s)
        if cfg.zero_between_steps:
            _zero_and_settle(board, cfg.settle_s)

    _zero_and_settle(board, cfg.settle_s)


def run_pattern(board: MotorBoardAdapter, cfg: SweepConfig) -> None:
    pattern = cfg.pattern.lower().strip()
    if pattern == "single":
        run_single_wheel_pattern(board, cfg)
    elif pattern == "all":
        run_all_wheels_pattern(board, cfg)
    elif pattern == "rotate":
        run_rotate_pattern(board, cfg)
    elif pattern == "full":
        run_single_wheel_pattern(board, cfg)
        run_all_wheels_pattern(board, cfg)
        run_rotate_pattern(board, cfg)
    else:
        raise ValueError(f"Unknown pattern: {cfg.pattern}")


# =============================================================================
# CLI
# =============================================================================
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Robot Savo PWM sweep diagnostic (Freenove/PCA9685 motor board)"
    )

    # Board/backend
    p.add_argument("--backend", default=BOARD_BACKEND_DEFAULT, help="auto | freenove | dryrun")
    p.add_argument("--board-name", default=BOARD_NAME_DEFAULT, help="Board profile/factory name")
    p.add_argument("--dryrun", action="store_true", help="Force dry-run backend (no hardware motion)")
    p.add_argument("--i2c-bus", type=int, default=I2C_BUS_DEFAULT, help="I2C bus number (default: 1)")
    p.add_argument("--addr", type=_parse_int_auto, default=PCA9685_ADDR_DEFAULT, help="PCA9685 addr (e.g., 0x40)")
    p.add_argument("--pwm-freq-hz", type=float, default=PCA9685_PWM_FREQ_HZ_DEFAULT, help="PWM frequency Hz")
    p.add_argument("--quench-ms", type=int, default=QUENCH_MS_DEFAULT, help="Direction-flip quench ms")

    # Inversions (software level)
    p.add_argument("--invert-fl", action="store_true")
    p.add_argument("--invert-rl", action="store_true")
    p.add_argument("--invert-fr", action="store_true")
    p.add_argument("--invert-rr", action="store_true")

    # Sweep behavior
    p.add_argument("--pattern", choices=("single", "all", "rotate", "full"), default="single")
    p.add_argument("--max-duty", type=int, default=min(1000, MAX_DUTY_DEFAULT), help="Max abs duty for sweep")
    p.add_argument("--step-duty", type=int, default=250, help="Duty step size")
    p.add_argument("--hold-s", type=float, default=0.50, help="Hold time per step (seconds)")
    p.add_argument("--settle-s", type=float, default=0.30, help="Zero/settle delay (seconds)")
    p.add_argument("--no-zero-between-steps", action="store_true", help="Do not zero between each step")
    p.add_argument("--repeat", type=int, default=1, help="Repeat entire selected pattern N times")

    # UX
    p.add_argument("--no-confirm", action="store_true", help="Skip interactive safety confirmation")
    p.add_argument("--verbose", action="store_true", help="Verbose board write output")

    return p


def _print_summary(cfg: SweepConfig) -> None:
    print("Robot SAVO — PWM Sweep Check")
    print("----------------------------")
    print(f"Backend         : {'dryrun' if cfg.dryrun else cfg.backend}")
    print(f"Board name      : {cfg.board_name}")
    print(f"I2C             : bus={cfg.i2c_bus}, addr={_fmt_addr(cfg.addr)}")
    print(f"PWM             : {cfg.pwm_freq_hz:.1f} Hz")
    print(f"Quench          : {_fmt_ms(cfg.quench_ms)}")
    print(f"Invert flags    : FL={cfg.inv[0]} RL={cfg.inv[1]} FR={cfg.inv[2]} RR={cfg.inv[3]} (+1 normal, -1 inverted)")
    print(f"Pattern         : {cfg.pattern}")
    print(f"Sweep           : step={cfg.step_duty}, max={cfg.max_duty}")
    print(f"Timing          : hold={cfg.hold_s:.2f}s, settle={cfg.settle_s:.2f}s")
    print(f"Zero between    : {cfg.zero_between_steps}")
    print(f"Repeat          : {cfg.repeat}")
    print(f"Verbose writes  : {cfg.verbose}")


def _confirm_or_abort(cfg: SweepConfig) -> None:
    if cfg.no_confirm or cfg.dryrun:
        return

    print("\nSAFETY CHECK")
    print("------------")
    print("Make sure:")
    print("  1) Robot is lifted (wheels off the ground)")
    print("  2) You can cut power / stop immediately")
    print("  3) Duty values are low for first test")
    resp = input("Type 'YES' to continue: ").strip()
    if resp != "YES":
        raise SystemExit("Aborted by user (safety confirmation not accepted).")


def _build_config_from_args(args: argparse.Namespace) -> SweepConfig:
    max_duty = _clamp_int(abs(int(args.max_duty)), 1, int(MAX_DUTY_ABSOLUTE))
    step_duty = _clamp_int(abs(int(args.step_duty)), 1, max_duty)
    hold_s = max(0.02, float(args.hold_s))
    settle_s = max(0.0, float(args.settle_s))
    repeat = max(1, int(args.repeat))

    inv = (
        -1 if bool(args.invert_fl) else +1,
        -1 if bool(args.invert_rl) else +1,
        -1 if bool(args.invert_fr) else +1,
        -1 if bool(args.invert_rr) else +1,
    )

    return SweepConfig(
        backend=str(args.backend),
        board_name=str(args.board_name),
        dryrun=bool(args.dryrun),
        i2c_bus=int(args.i2c_bus),
        addr=int(args.addr),
        pwm_freq_hz=float(args.pwm_freq_hz),
        quench_ms=int(args.quench_ms),
        inv=inv,
        pattern=str(args.pattern),
        max_duty=max_duty,
        step_duty=step_duty,
        hold_s=hold_s,
        settle_s=settle_s,
        zero_between_steps=not bool(args.no_zero_between_steps),
        repeat=repeat,
        verbose=bool(args.verbose),
        no_confirm=bool(args.no_confirm),
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    cfg = _build_config_from_args(args)

    _print_summary(cfg)
    _confirm_or_abort(cfg)

    board: Optional[MotorBoardAdapter] = None
    t0 = time.monotonic()
    writes_before = 0
    try:
        board = build_board(cfg)
        print("\n[OK] Motor board initialized.")
        writes_before = board.write_count

        # Always start from zero
        board.stop()
        _sleep_s(cfg.settle_s)

        for i in range(cfg.repeat):
            if cfg.repeat > 1:
                print(f"\n================ Repeat {i+1}/{cfg.repeat} ================")
            run_pattern(board, cfg)

        # Final zero
        board.stop()
        _sleep_s(cfg.settle_s)

        elapsed = time.monotonic() - t0
        writes_after = board.write_count
        print("\n[RESULT] PASS")
        print(f"Elapsed time   : {elapsed:.2f} s")
        print(f"Board writes   : {writes_after - writes_before}")
        print("Final state    : STOP (zero output sent)")
        return 0

    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Ctrl+C received, stopping motors...")
        if board is not None:
            try:
                board.stop()
            except Exception:
                pass
        return 130

    except Exception as e:
        print(f"\n[RESULT] FAIL: {e}", file=sys.stderr)
        if board is not None:
            try:
                board.stop()
            except Exception:
                pass
        return 2

    finally:
        if board is not None:
            try:
                board.stop()
            except Exception:
                pass
            try:
                board.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())