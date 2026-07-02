# -*- coding: utf-8 -*-

"""Manual pan-tilt diagnostic CLI for Robot Savo head."""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from typing import Optional

from savo_head.constants import (
    MANUAL_STEP_DEG_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    TILT_CENTER_DEG_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
)
from savo_head.drivers.pantilt_driver import (
    BACKEND_DRYRUN,
    BACKEND_PCA9685,
    PanTiltDriver,
    PanTiltDriverConfig,
)
from savo_head.models.pantilt_command import (
    PanTiltCommand,
    center_command,
    hold_command,
    manual_step_command,
    stop_command,
)


class RawKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self) -> "RawKeyReader":
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            return sys.stdin.read(1)
        return ""


def print_help() -> None:
    print("\nRobot Savo head manual controls")
    print("--------------------------------")
    print("  W / S     : tilt up / tilt down")
    print("  A / D     : pan left / pan right")
    print("  C         : center")
    print("  SPACE     : hold current position")
    print("  H         : help")
    print("  Q or ESC  : quit\n")


def command_from_key(key_raw: str, step_deg: int = MANUAL_STEP_DEG_DEFAULT) -> Optional[PanTiltCommand]:
    if not key_raw:
        return None

    code = ord(key_raw)
    key = key_raw.upper()

    if key == "Q" or code == 27:
        return stop_command(source="manual", stamp_s=time.monotonic(), reason="quit")

    if key == "C":
        return center_command(source="manual", stamp_s=time.monotonic(), reason="manual_center")

    if key_raw == " ":
        return hold_command(source="manual", stamp_s=time.monotonic(), reason="manual_hold")

    if key in ("W", "A", "S", "D"):
        cmd = manual_step_command(key, step_deg=step_deg)
        return PanTiltCommand(
            command_type=cmd.command_type,
            pan_delta_deg=cmd.pan_delta_deg,
            tilt_delta_deg=cmd.tilt_delta_deg,
            source=cmd.source,
            stamp_s=time.monotonic(),
            priority=cmd.priority,
            reason=cmd.reason,
        )

    return None


def make_driver(args: argparse.Namespace) -> PanTiltDriver:
    backend = BACKEND_DRYRUN if args.dryrun else BACKEND_PCA9685

    return PanTiltDriver(
        PanTiltDriverConfig(
            backend=backend,
            center_on_open=args.center_on_start,
            center_on_close=args.center_on_exit,
        )
    )


def print_driver_summary(driver: PanTiltDriver, step_deg: int) -> None:
    cal = driver.calibration

    print("\nRobot Savo — Head Manual CLI")
    print("----------------------------")
    print(f"Backend      : {driver.config.backend}")
    print(f"Pan channel  : logical {cal.pan.logical_channel} / PCA9685 {cal.pan.pca9685_channel}")
    print(f"Tilt channel : logical {cal.tilt.logical_channel} / PCA9685 {cal.tilt.pca9685_channel}")
    print(f"Center       : pan={cal.pan.center_deg}°, tilt={cal.tilt.center_deg}°")
    print(f"Tilt range   : {cal.tilt.min_deg} .. {cal.tilt.max_deg}°")
    print(f"Step         : {step_deg}°")
    print_help()


def run_manual_cli(args: argparse.Namespace) -> int:
    driver = make_driver(args)
    driver.open()
    print_driver_summary(driver, args.step)

    try:
        with RawKeyReader() as keys:
            while True:
                key_raw = keys.read_key(timeout_s=args.key_timeout)
                command = command_from_key(key_raw, step_deg=args.step)

                if command is None:
                    if key_raw and key_raw.upper() == "H":
                        print_help()
                    continue

                state = driver.apply_command(command)

                if command.reason == "quit":
                    print(f"[STOP] pan={state.pan_deg}°, tilt={state.tilt_deg}°")
                    break

                print(
                    f"[HEAD] pan={state.pan_deg}°, tilt={state.tilt_deg}°, "
                    f"mode={state.mode}, source={state.source}, reason={command.reason}"
                )

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        if args.center_on_exit:
            try:
                state = driver.apply_command(
                    center_command(source="shutdown", stamp_s=time.monotonic())
                )
                print(f"[CENTER] pan={state.pan_deg}°, tilt={state.tilt_deg}°")
            except Exception as exc:
                print(f"[WARN] Could not center on exit: {exc}")

        driver.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Robot Savo head manual pan-tilt diagnostic CLI"
    )

    parser.add_argument(
        "--dryrun",
        action="store_true",
        help="Do not access PCA9685 hardware.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=MANUAL_STEP_DEG_DEFAULT,
        help="Manual step in degrees.",
    )
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=0.05,
        help="Keyboard polling timeout in seconds.",
    )
    parser.add_argument(
        "--center-on-start",
        action="store_true",
        help="Move to calibrated center when opening the driver.",
    )
    parser.add_argument(
        "--no-center-on-exit",
        action="store_true",
        help="Do not recenter on exit.",
    )

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    args.center_on_exit = not bool(args.no_center_on_exit)

    if args.step <= 0:
        parser.error("--step must be positive")

    if args.step > 20:
        parser.error("--step is too large for manual head diagnostics")

    return run_manual_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
