#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robot Savo four-wheel encoder diagnostic for Raspberry Pi 5."""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from typing import Dict, List, Tuple

try:
    import lgpio
except Exception:
    print(
        "ERROR: python3-lgpio is required. Install with: sudo apt install -y python3-lgpio",
        file=sys.stderr,
    )
    raise


# Valid Gray-code transitions.
# A is bit 1, B is bit 0.
DELTA = {
    (0b00, 0b01): +1,
    (0b01, 0b11): +1,
    (0b11, 0b10): +1,
    (0b10, 0b00): +1,
    (0b01, 0b00): -1,
    (0b11, 0b01): -1,
    (0b10, 0b11): -1,
    (0b00, 0b10): -1,
}


def maybe_pullup_flags(use_internal_pullup: bool) -> int:
    if not use_internal_pullup:
        return 0
    return getattr(lgpio, "SET_PULL_UP", 0)


def validate_unique_pins(pin_pairs: Dict[str, Tuple[int, int]]) -> None:
    all_pins: List[int] = [pin for pair in pin_pairs.values() for pin in pair]
    duplicates = sorted({pin for pin in all_pins if all_pins.count(pin) > 1})

    if duplicates:
        raise ValueError(
            f"Duplicate GPIO pin assignment detected: {duplicates}. "
            "Each encoder channel must use a unique BCM GPIO pin."
        )


def autodetect_chip(pins, start=0, end=7, use_internal_pullup=False):
    last_error = None
    flags = maybe_pullup_flags(use_internal_pullup)

    for chip in range(start, end + 1):
        try:
            handle = lgpio.gpiochip_open(chip)
            claimed = []

            try:
                for pin in pins:
                    lgpio.gpio_claim_input(handle, pin, flags)
                    claimed.append(pin)

                for pin in claimed:
                    lgpio.gpio_free(handle, pin)

                lgpio.gpiochip_close(handle)
                return chip, lgpio.gpiochip_open(chip)

            except Exception as exc:
                last_error = exc

                for pin in claimed:
                    try:
                        lgpio.gpio_free(handle, pin)
                    except Exception:
                        pass

                lgpio.gpiochip_close(handle)

        except Exception as exc:
            last_error = exc

    raise RuntimeError(
        f"Could not find a gpiochip that can claim pins {pins}. Last error: {last_error}"
    )


class QuadEncoder:
    def __init__(
        self,
        handle,
        name,
        a_pin,
        b_pin,
        debounce_s,
        invert=False,
        use_hw_debounce=True,
        use_internal_pullup=False,
    ):
        self.handle = handle
        self.name = name
        self.a_pin = a_pin
        self.b_pin = b_pin
        self.debounce_s = max(0.0, debounce_s)
        self.invert = -1 if invert else +1

        flags = maybe_pullup_flags(use_internal_pullup)

        lgpio.gpio_claim_input(handle, a_pin, flags)
        lgpio.gpio_claim_input(handle, b_pin, flags)

        self.hw_debounce = False
        if use_hw_debounce and self.debounce_s > 0:
            try:
                debounce_us = int(self.debounce_s * 1_000_000)
                lgpio.gpio_set_debounce_micros(handle, a_pin, debounce_us)
                lgpio.gpio_set_debounce_micros(handle, b_pin, debounce_us)
                self.hw_debounce = True
            except Exception:
                self.hw_debounce = False

        now = time.monotonic()
        self.last_change = {a_pin: now, b_pin: now}
        self.state_bits = {
            a_pin: lgpio.gpio_read(handle, a_pin),
            b_pin: lgpio.gpio_read(handle, b_pin),
        }

        self.prev_state = self._state_from_bits()
        self.count = 0
        self.dir = 0
        self.illegal = 0

    def _state_from_bits(self):
        a_value = self.state_bits[self.a_pin] & 1
        b_value = self.state_bits[self.b_pin] & 1
        return (a_value << 1) | b_value

    def sample_once(self, now):
        a_value = lgpio.gpio_read(self.handle, self.a_pin)
        b_value = lgpio.gpio_read(self.handle, self.b_pin)

        if not self.hw_debounce and self.debounce_s > 0:
            a_value = self._software_debounce(self.a_pin, a_value, now)
            b_value = self._software_debounce(self.b_pin, b_value, now)

        new_state = ((a_value & 1) << 1) | (b_value & 1)

        if new_state == self.prev_state:
            return

        delta = DELTA.get((self.prev_state, new_state))

        if delta is None:
            self.illegal += 1
        else:
            delta *= self.invert
            self.count += delta
            self.dir = 1 if delta > 0 else -1

        self.prev_state = new_state

    def _software_debounce(self, pin, raw_value, now):
        if raw_value != self.state_bits[pin]:
            if (now - self.last_change[pin]) >= self.debounce_s:
                self.state_bits[pin] = raw_value
                self.last_change[pin] = now
            else:
                raw_value = self.state_bits[pin]
        else:
            self.last_change[pin] = now

        return raw_value


def direction_symbol(direction):
    if direction > 0:
        return "→"
    if direction < 0:
        return "←"
    return "•"


def wheel_speed_mps(delta_count, dt, counts_per_wheel_rev, wheel_dia_m):
    counts_per_second = delta_count / max(dt, 1e-9)
    rev_per_second = counts_per_second / max(counts_per_wheel_rev, 1)
    return rev_per_second * math.pi * wheel_dia_m


def estimate_mecanum_body_velocity(fl_v, fr_v, rl_v, rr_v, wheelbase_m, track_m):
    """Rough mecanum estimate; signs depend on final ROS calibration."""
    radius_sum = max(1e-9, wheelbase_m + track_m)

    vx = (fl_v + fr_v + rl_v + rr_v) / 4.0
    vy = (-fl_v + fr_v + rl_v - rr_v) / 4.0
    omega = (-fl_v + fr_v - rl_v + rr_v) / (4.0 * radius_sum)

    return vx, vy, omega


def run(args):
    pin_pairs = {
        "FL": (args.fl_a, args.fl_b),
        "FR": (args.fr_a, args.fr_b),
        "RL": (args.rl_a, args.rl_b),
        "RR": (args.rr_a, args.rr_b),
    }

    validate_unique_pins(pin_pairs)

    all_pins = [pin for pair in pin_pairs.values() for pin in pair]

    if args.chip is not None:
        chip_used = args.chip
        handle = lgpio.gpiochip_open(chip_used)
    else:
        chip_used, handle = autodetect_chip(
            all_pins,
            use_internal_pullup=args.internal_pullup,
        )

    print(f"[Encoders] gpiochip{chip_used}")
    print(
        "[Pins] "
        f"FL({args.fl_a},{args.fl_b})  "
        f"FR({args.fr_a},{args.fr_b})  "
        f"RL({args.rl_a},{args.rl_b})  "
        f"RR({args.rr_a},{args.rr_b})"
    )

    if args.internal_pullup:
        print("[Pull-up] Internal Pi pull-ups requested")
    else:
        print("[Pull-up] External pull-ups expected, e.g. 12kΩ to 3.3V")

    print(
        f"[Timing] debounce={int(args.debounce_s * 1e6)} µs  "
        f"poll={int(args.poll_s * 1e6)} µs  interval={args.interval:.3f}s"
    )

    counts_per_wheel_rev = max(1, int(args.cpr * args.decoding * args.gear))

    print(
        f"[Kinematics] wheel_dia={args.wheel_dia:.3f} m  "
        f"CPR={args.cpr}  decoding={args.decoding}x  gear={args.gear:.3f}  "
        f"counts_per_wheel_rev={counts_per_wheel_rev}"
    )

    encoders = {}
    csv_file = None

    try:
        encoders["FL"] = QuadEncoder(
            handle,
            "FL",
            args.fl_a,
            args.fl_b,
            args.debounce_s,
            invert=args.invert_fl,
            use_hw_debounce=not args.no_hw_debounce,
            use_internal_pullup=args.internal_pullup,
        )
        encoders["FR"] = QuadEncoder(
            handle,
            "FR",
            args.fr_a,
            args.fr_b,
            args.debounce_s,
            invert=args.invert_fr,
            use_hw_debounce=not args.no_hw_debounce,
            use_internal_pullup=args.internal_pullup,
        )
        encoders["RL"] = QuadEncoder(
            handle,
            "RL",
            args.rl_a,
            args.rl_b,
            args.debounce_s,
            invert=args.invert_rl,
            use_hw_debounce=not args.no_hw_debounce,
            use_internal_pullup=args.internal_pullup,
        )
        encoders["RR"] = QuadEncoder(
            handle,
            "RR",
            args.rr_a,
            args.rr_b,
            args.debounce_s,
            invert=args.invert_rr,
            use_hw_debounce=not args.no_hw_debounce,
            use_internal_pullup=args.internal_pullup,
        )

        writer = None

        if args.csv:
            csv_file = open(args.csv, "w", newline="")
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "t_s",
                    "FL_count",
                    "FR_count",
                    "RL_count",
                    "RR_count",
                    "FL_delta",
                    "FR_delta",
                    "RL_delta",
                    "RR_delta",
                    "FL_v_mps",
                    "FR_v_mps",
                    "RL_v_mps",
                    "RR_v_mps",
                    "vx_mps",
                    "vy_mps",
                    "omega_rad_s",
                    "FL_illegal",
                    "FR_illegal",
                    "RL_illegal",
                    "RR_illegal",
                ]
            )

        stop = False

        def on_sigint(_sig, _frame):
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, on_sigint)

        t0 = time.monotonic()
        last_print = t0
        last_counts = {name: encoder.count for name, encoder in encoders.items()}

        print()
        print(
            " t(s) |"
            "   FL_cnt   FR_cnt   RL_cnt   RR_cnt |"
            " dFL dFR dRL dRR |"
            "  FL_v   FR_v   RL_v   RR_v |"
            "   vx     vy      omega |"
            " illegal FL FR RL RR | dir FL FR RL RR"
        )
        print("-" * 138)

        while not stop:
            now = time.monotonic()

            if args.duration > 0 and (now - t0) >= args.duration:
                break

            for encoder in encoders.values():
                encoder.sample_once(now)

            if (now - last_print) >= args.interval:
                dt = now - last_print
                last_print = now

                counts = {name: encoder.count for name, encoder in encoders.items()}
                deltas = {name: counts[name] - last_counts[name] for name in encoders}
                last_counts = counts

                speeds = {
                    name: wheel_speed_mps(
                        deltas[name],
                        dt,
                        counts_per_wheel_rev,
                        args.wheel_dia,
                    )
                    for name in encoders
                }

                vx, vy, omega = estimate_mecanum_body_velocity(
                    speeds["FL"],
                    speeds["FR"],
                    speeds["RL"],
                    speeds["RR"],
                    args.wheelbase,
                    args.track,
                )

                print(
                    f"{now - t0:5.1f} |"
                    f" {counts['FL']:8d} {counts['FR']:8d} {counts['RL']:8d} {counts['RR']:8d} |"
                    f" {deltas['FL']:3d} {deltas['FR']:3d} {deltas['RL']:3d} {deltas['RR']:3d} |"
                    f" {speeds['FL']: .3f} {speeds['FR']: .3f} {speeds['RL']: .3f} {speeds['RR']: .3f} |"
                    f" {vx: .3f} {vy: .3f} {omega: .3f} |"
                    f" {encoders['FL'].illegal:3d} {encoders['FR'].illegal:3d}"
                    f" {encoders['RL'].illegal:3d} {encoders['RR'].illegal:3d} |"
                    f"  {direction_symbol(encoders['FL'].dir):^3s}"
                    f" {direction_symbol(encoders['FR'].dir):^3s}"
                    f" {direction_symbol(encoders['RL'].dir):^3s}"
                    f" {direction_symbol(encoders['RR'].dir):^3s}",
                    flush=True,
                )

                if writer:
                    writer.writerow(
                        [
                            f"{now - t0:.3f}",
                            counts["FL"],
                            counts["FR"],
                            counts["RL"],
                            counts["RR"],
                            deltas["FL"],
                            deltas["FR"],
                            deltas["RL"],
                            deltas["RR"],
                            f"{speeds['FL']:.6f}",
                            f"{speeds['FR']:.6f}",
                            f"{speeds['RL']:.6f}",
                            f"{speeds['RR']:.6f}",
                            f"{vx:.6f}",
                            f"{vy:.6f}",
                            f"{omega:.6f}",
                            encoders["FL"].illegal,
                            encoders["FR"].illegal,
                            encoders["RL"].illegal,
                            encoders["RR"].illegal,
                        ]
                    )

            time.sleep(args.poll_s)

    finally:
        for pin in all_pins:
            try:
                lgpio.gpio_free(handle, pin)
            except Exception:
                pass

        try:
            lgpio.gpiochip_close(handle)
        except Exception:
            pass

        if csv_file:
            csv_file.close()

    elapsed = time.monotonic() - t0

    print()
    print("=== Summary ===")
    print(f"Duration: {elapsed:.2f} s")

    for name in ("FL", "FR", "RL", "RR"):
        encoder = encoders[name]
        rate = encoder.count / elapsed if elapsed > 0 else 0.0
        print(
            f"{name}: total={encoder.count:+d}  "
            f"avg_rate={rate:+.2f} counts/s  "
            f"illegal={encoder.illegal}"
        )

    print()
    print("Robot Savo encoder diagnostic defaults:")
    print("  FL: A=GPIO20, B=GPIO21")
    print("  FR: A=GPIO13, B=GPIO25")
    print("  RL: A=GPIO23, B=GPIO24")
    print("  RR: A=GPIO12, B=GPIO26")
    print()
    print("Expected:")
    print("  Forward wheel rotation should count positive.")
    print("  Reverse wheel rotation should count negative.")
    print("  External pull-up resistors are expected by default.")
    print("  Use --invert-fl/fr/rl/rr only for temporary diagnostics.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Robot Savo 4-wheel quadrature encoder test for Raspberry Pi 5",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Robot Savo tested encoder GPIO map.
    #
    # Goal:
    # - Forward wheel rotation should count positive for each wheel.
    # - External pull-up resistors are used, so --internal-pullup is NOT used by default.
    #
    # Confirmed mapping from hardware tests:
    # FL physical wheel -> GPIO20/GPIO21
    # FR physical wheel -> GPIO13/GPIO25
    # RL physical wheel -> GPIO23/GPIO24
    # RR physical wheel -> GPIO12/GPIO26
    parser.add_argument("--fl-a", type=int, default=20, help="Front-left encoder A BCM GPIO")
    parser.add_argument("--fl-b", type=int, default=21, help="Front-left encoder B BCM GPIO")

    parser.add_argument("--fr-a", type=int, default=13, help="Front-right encoder A BCM GPIO")
    parser.add_argument("--fr-b", type=int, default=25, help="Front-right encoder B BCM GPIO")

    parser.add_argument("--rl-a", type=int, default=23, help="Rear-left encoder A BCM GPIO")
    parser.add_argument("--rl-b", type=int, default=24, help="Rear-left encoder B BCM GPIO")

    parser.add_argument("--rr-a", type=int, default=12, help="Rear-right encoder A BCM GPIO")
    parser.add_argument("--rr-b", type=int, default=26, help="Rear-right encoder B BCM GPIO")

    # Keep old working flag style:
    # default = no inversion.
    # --invert-* can be used only for temporary diagnostics.
    # --no-invert-* is accepted for backward compatibility.
    parser.add_argument(
        "--invert-fl",
        dest="invert_fl",
        action="store_true",
        help="Invert front-left encoder sign",
    )
    parser.add_argument(
        "--invert-fr",
        dest="invert_fr",
        action="store_true",
        help="Invert front-right encoder sign",
    )
    parser.add_argument(
        "--invert-rl",
        dest="invert_rl",
        action="store_true",
        help="Invert rear-left encoder sign",
    )
    parser.add_argument(
        "--invert-rr",
        dest="invert_rr",
        action="store_true",
        help="Invert rear-right encoder sign",
    )

    parser.add_argument(
        "--no-invert-fl",
        dest="invert_fl",
        action="store_false",
        help="Disable front-left encoder inversion",
    )
    parser.add_argument(
        "--no-invert-fr",
        dest="invert_fr",
        action="store_false",
        help="Disable front-right encoder inversion",
    )
    parser.add_argument(
        "--no-invert-rl",
        dest="invert_rl",
        action="store_false",
        help="Disable rear-left encoder inversion",
    )
    parser.add_argument(
        "--no-invert-rr",
        dest="invert_rr",
        action="store_false",
        help="Disable rear-right encoder inversion",
    )

    parser.set_defaults(
        invert_fl=False,
        invert_fr=False,
        invert_rl=False,
        invert_rr=False,
    )

    parser.add_argument("--poll-s", type=float, default=0.001, help="Polling period in seconds")
    parser.add_argument("--debounce-s", type=float, default=0.0003, help="Per-line debounce in seconds")
    parser.add_argument("--interval", type=float, default=0.5, help="Print interval in seconds")
    parser.add_argument("--duration", type=float, default=40.0, help="Duration in seconds; 0 means until Ctrl+C")

    parser.add_argument("--wheel-dia", type=float, default=0.065, help="Wheel diameter in meters")
    parser.add_argument("--cpr", type=int, default=20, help="Encoder counts per revolution per channel")
    parser.add_argument("--decoding", type=int, default=4, choices=[1, 2, 4], help="Quadrature decoding factor")
    parser.add_argument("--gear", type=float, default=1.0, help="Motor-to-wheel gear multiplier")

    parser.add_argument("--wheelbase", type=float, default=0.165, help="Front-rear wheel spacing in meters")
    parser.add_argument("--track", type=float, default=0.165, help="Left-right wheel spacing in meters")

    parser.add_argument("--chip", type=int, default=None, help="gpiochip index; auto if omitted")
    parser.add_argument("--internal-pullup", action="store_true", help="Request Pi internal pull-ups")
    parser.add_argument("--no-hw-debounce", action="store_true", help="Force software debounce only")

    parser.add_argument("--csv", type=str, default="", help="Optional CSV output path")

    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
