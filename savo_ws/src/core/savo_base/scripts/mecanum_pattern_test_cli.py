#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Direct Freenove mecanum movement-pattern diagnostic for Robot Savo."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

if TYPE_CHECKING:
    from savo_base.drivers.freenove_mecanum_board import FreenoveMecanumBoard


WheelPattern = Tuple[int, int, int, int]


@dataclass(frozen=True)
class Pattern:
    name: str
    fl: int
    rl: int
    fr: int
    rr: int
    label: str


# Locked Robot Savo Freenove mapping:
# Data1 = FL = channels (0,1)
# Data2 = RL = channels (3,2)
# Data3 = FR = channels (6,7)
# Data4 = RR = channels (4,5)
PATTERNS: Dict[str, Pattern] = {
    "forward": Pattern("forward", +1, +1, +1, +1, "Forward"),
    "backward": Pattern("backward", -1, -1, -1, -1, "Backward"),
    "turn_left": Pattern("turn_left", -1, -1, +1, +1, "Turn left"),
    "turn_right": Pattern("turn_right", +1, +1, -1, -1, "Turn right"),
    "move_left": Pattern("move_left", -1, +1, +1, -1, "Move left / strafe left"),
    "move_right": Pattern("move_right", +1, -1, -1, +1, "Move right / strafe right"),
    "left_forward": Pattern("left_forward", 0, +1, +1, 0, "Left and forward"),
    "right_backward": Pattern("right_backward", 0, -1, -1, 0, "Right and backward"),
    "right_forward": Pattern("right_forward", +1, 0, 0, +1, "Right and forward"),
    "left_backward": Pattern("left_backward", -1, 0, 0, -1, "Left and backward"),
    "stop": Pattern("stop", 0, 0, 0, 0, "Stop"),
}

GROUPS: Dict[str, List[str]] = {
    "basic": [
        "forward",
        "backward",
        "turn_left",
        "turn_right",
        "move_left",
        "move_right",
    ],
    "diagonal": [
        "left_forward",
        "right_backward",
        "right_forward",
        "left_backward",
    ],
    "all": [
        "forward",
        "backward",
        "turn_left",
        "turn_right",
        "move_left",
        "move_right",
        "left_forward",
        "right_backward",
        "right_forward",
        "left_backward",
        "stop",
    ],
}


def clamp_int(value: int, lo: int, hi: int) -> int:
    v = int(value)
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def parse_addr(text: str) -> int:
    return int(str(text).strip(), 0)


def resolve_sequence(text: str) -> List[Pattern]:
    key = str(text).strip().lower()

    if key in GROUPS:
        return [PATTERNS[name] for name in GROUPS[key]]

    names = [item.strip().lower() for item in key.split(",") if item.strip()]
    if not names:
        raise ValueError("empty pattern list")

    out: List[Pattern] = []
    for name in names:
        if name not in PATTERNS:
            valid = ", ".join(sorted(list(PATTERNS.keys()) + list(GROUPS.keys())))
            raise ValueError(f"unknown pattern '{name}'. Valid: {valid}")
        out.append(PATTERNS[name])
    return out


def pattern_to_duty(pattern: Pattern, duty: int) -> WheelPattern:
    return (
        pattern.fl * duty,
        pattern.rl * duty,
        pattern.fr * duty,
        pattern.rr * duty,
    )


def print_reference_table() -> None:
    print()
    print("Locked Robot Savo Freenove pattern table")
    print("Wheel order: FL/Data1, RL/Data2, FR/Data3, RR/Data4")
    print("-" * 78)
    print(f"{'Pattern':<17} {'FL':>5} {'RL':>5} {'FR':>5} {'RR':>5}  Movement")
    print("-" * 78)
    for name in GROUPS["all"]:
        p = PATTERNS[name]
        print(f"{p.name:<17} {p.fl:>+5d} {p.rl:>+5d} {p.fr:>+5d} {p.rr:>+5d}  {p.label}")
    print("-" * 78)
    print()


def confirm_or_abort(args: argparse.Namespace) -> None:
    if args.dry_run or args.yes:
        return

    print()
    print("SAFETY CHECK")
    print("------------")
    print("1) Robot must be lifted for first test.")
    print("2) Keep hands, clothes, wires away from wheels.")
    print("3) Keep motor power cut ready.")
    print("4) Do not run base_driver_node at the same time.")
    print()
    ans = input("Type YES to continue: ").strip()
    if ans != "YES":
        raise KeyboardInterrupt("operator did not confirm safety")


def write_pattern(
    board: Optional["FreenoveMecanumBoard"],
    pattern: Pattern,
    duty: int,
    *,
    dry_run: bool,
) -> None:
    dfl, drl, dfr, drr = pattern_to_duty(pattern, duty)
    print(
        f"[PATTERN] {pattern.name:<15} "
        f"FL={dfl:+5d} RL={drl:+5d} FR={dfr:+5d} RR={drr:+5d} | {pattern.label}"
    )

    if dry_run:
        return

    assert board is not None
    board.set_motor_model(dfl, drl, dfr, drr)


def stop_board(board: Optional["FreenoveMecanumBoard"], *, dry_run: bool) -> None:
    print("[STOP] FL=0 RL=0 FR=0 RR=0")
    if dry_run:
        return
    assert board is not None
    board.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mecanum_pattern_test_cli.py",
        description="Direct Freenove mecanum movement-pattern diagnostic for Robot Savo.",
    )

    parser.add_argument("--i2c-bus", type=int, default=1, help="PCA9685 I2C bus, default 1")
    parser.add_argument("--addr", type=parse_addr, default="0x40", help="PCA9685 address, default 0x40")
    parser.add_argument("--pwm-freq", type=float, default=50.0, help="PWM frequency, default 50 Hz")
    parser.add_argument("--quench-ms", type=int, default=18, help="Direction flip quench, default 18 ms")

    parser.add_argument(
        "--only",
        default="basic",
        help=(
            "Pattern group or comma list. Groups: basic, diagonal, all. "
            "Names: forward, backward, turn_left, turn_right, move_left, move_right, "
            "left_forward, right_backward, right_forward, left_backward, stop."
        ),
    )
    parser.add_argument("--duty", type=int, default=1000, help="Signed PWM magnitude, default 1000")
    parser.add_argument("--pulse", type=float, default=1.50, help="Pattern hold time in seconds, default 1.50")
    parser.add_argument("--gap", type=float, default=0.80, help="Stop gap between patterns, default 0.80")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat sequence count, default 1")

    parser.add_argument("--no-invert-fl", dest="invert_fl", action="store_false")
    parser.add_argument("--no-invert-rl", dest="invert_rl", action="store_false")
    parser.add_argument("--no-invert-fr", dest="invert_fr", action="store_false")
    parser.add_argument("--no-invert-rr", dest="invert_rr", action="store_false")

    parser.set_defaults(
        invert_fl=True,
        invert_rl=True,
        invert_fr=True,
        invert_rr=True,
    )

    parser.add_argument("--dry-run", action="store_true", help="Print patterns without touching hardware")
    parser.add_argument("--yes", action="store_true", help="Skip interactive safety confirmation")
    parser.add_argument("--verbose", action="store_true", help="Enable board debug output")

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        duty = clamp_int(args.duty, 0, 4095)
        pulse_s = max(0.05, float(args.pulse))
        gap_s = max(0.05, float(args.gap))
        repeat = max(1, int(args.repeat))
        sequence = resolve_sequence(args.only)
    except Exception as exc:
        print(f"[ERROR] Invalid arguments: {exc}", file=sys.stderr)
        return 2

    print("=" * 78)
    print("Robot Savo - Freenove Mecanum Pattern Test")
    print("=" * 78)
    print("Mapping: Data1=FL(0,1), Data2=RL(3,2), Data3=FR(6,7), Data4=RR(4,5)")
    print(f"I2C: bus={args.i2c_bus}, addr=0x{args.addr:02X}, pwm={args.pwm_freq:.1f} Hz")
    print(f"Duty: {duty} / 4095")
    print(f"Timing: pulse={pulse_s:.2f}s gap={gap_s:.2f}s repeat={repeat}")
    print(f"Dry run: {bool(args.dry_run)}")
    print("=" * 78)

    print_reference_table()

    board: Optional["FreenoveMecanumBoard"] = None

    try:
        confirm_or_abort(args)

        if not args.dry_run:
            from savo_base.drivers.freenove_mecanum_board import FreenoveMecanumBoard

            board = FreenoveMecanumBoard(
                i2c_bus=int(args.i2c_bus),
                addr=int(args.addr),
                pwm_freq_hz=float(args.pwm_freq),
                invert_fl=bool(args.invert_fl),
                invert_rl=bool(args.invert_rl),
                invert_fr=bool(args.invert_fr),
                invert_rr=bool(args.invert_rr),
                quench_ms=int(args.quench_ms),
                debug=bool(args.verbose),
            )
            if not board.ping():
                raise RuntimeError("PCA9685 ping failed")

            stop_board(board, dry_run=False)
            time.sleep(0.30)

        for round_idx in range(repeat):
            print()
            print(f"=== Round {round_idx + 1}/{repeat} ===")
            for pattern in sequence:
                write_pattern(board, pattern, duty, dry_run=bool(args.dry_run))
                time.sleep(pulse_s)
                stop_board(board, dry_run=bool(args.dry_run))
                time.sleep(gap_s)

        print()
        print("[RESULT] PASS")
        return 0

    except KeyboardInterrupt:
        print()
        print("[RESULT] ABORTED by operator")
        return 130
    except Exception as exc:
        print(f"[RESULT] FAIL: {exc}", file=sys.stderr)
        return 1
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
