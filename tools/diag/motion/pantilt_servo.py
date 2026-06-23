#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive and automatic PCA9685 pan-tilt servo diagnostic for Robot Savo."""

import argparse
import math
import select
import sys
import termios
import time
import tty

import smbus


class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, address: int = 0x40, debug: bool = False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10

        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_servo_pulse(self, channel: int, pulse_us: float) -> None:
        pulse = pulse_us * 4096.0 / 20000.0
        self.set_pwm(channel, 0, int(pulse))

    def close(self) -> None:
        self.bus.close()


class Servo:
    def __init__(self, addr: int = 0x40, debug: bool = True):
        self.pwm_frequency = 50

        # Freenove logical servo ports -> real PCA9685 channels.
        # Robot Savo validated mapping:
        #   logical 7 = pan  / left-right
        #   logical 6 = tilt / up-down
        self.pwm_channel_map = {
            "0": 8,
            "1": 9,
            "2": 10,
            "3": 11,
            "4": 12,
            "5": 13,
            "6": 14,
            "7": 15,
        }

        self.pwm_servo = PCA9685(addr, debug=debug)
        self.pwm_servo.set_pwm_freq(self.pwm_frequency)

    def angle_to_pulse(self, channel: str, angle: int, error: int = 10) -> float:
        angle = int(angle)

        if channel == "0":
            pulse = 2500.0 - float((angle + error) / 0.09)
        else:
            pulse = 500.0 + float((angle + error) / 0.09)

        return max(500.0, min(2500.0, pulse))

    def set_servo_angle(self, channel: str, angle: int, error: int = 10) -> None:
        if channel not in self.pwm_channel_map:
            raise ValueError(
                f"Invalid channel: {channel}. Valid: {list(self.pwm_channel_map.keys())}"
            )

        # Standard hobby servo command safety limit.
        angle = max(0, min(180, int(angle)))

        pulse = self.angle_to_pulse(channel, angle, error)
        pca_channel = self.pwm_channel_map[channel]
        self.pwm_servo.set_servo_pulse(pca_channel, pulse)

    def close(self) -> None:
        self.pwm_servo.close()


class RawKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.0) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            return sys.stdin.read(1)
        return ""


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def bounce(value: int, direction: int, step: int, low: int, high: int):
    value += direction * step

    if value >= high:
        value = high
        direction = -1
    elif value <= low:
        value = low
        direction = 1

    return value, direction


def servo_safe_angle(angle: int) -> int:
    return clamp(angle, 0, 180)


def print_help() -> None:
    print("\nControls:")
    print("  W / S     : tilt up / tilt down")
    print("  A / D     : pan left / pan right")
    print("  O         : toggle auto/manual")
    print("  SPACE     : pause/resume auto")
    print("  C         : calibrated center")
    print("  H         : show help")
    print("  Q or ESC  : quit\n")


def run_pantilt(args: argparse.Namespace) -> None:
    servo = Servo(addr=0x40, debug=True)

    pan_center = clamp(args.pan_center, 0, 180)
    tilt_center = clamp(args.tilt_center, 0, 180)

    auto_pan_min = args.auto_pan_min
    auto_pan_max = args.auto_pan_max
    auto_tilt_min = args.auto_tilt_min
    auto_tilt_max = args.auto_tilt_max

    if auto_pan_min > auto_pan_max:
        auto_pan_min, auto_pan_max = auto_pan_max, auto_pan_min

    if auto_tilt_min > auto_tilt_max:
        auto_tilt_min, auto_tilt_max = auto_tilt_max, auto_tilt_min

    pan_angle = pan_center
    tilt_angle = tilt_center

    pan_direction = 1
    tilt_direction = 1

    auto_paused = False
    mode = args.mode

    servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
    servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))

    print("\nRobot Savo — Pan-Tilt Servo Test")
    print("---------------------------------")
    print(f"Pan channel : '{args.pan_chan}' (PCA9685 ch {servo.pwm_channel_map[args.pan_chan]})")
    print(f"Tilt channel: '{args.tilt_chan}' (PCA9685 ch {servo.pwm_channel_map[args.tilt_chan]})")
    print(f"Pan center  : {pan_center} deg")
    print(f"Tilt center : {tilt_center} deg")
    print(f"Manual step : {args.step} deg")
    print(f"Auto axis   : {args.auto_axis}")
    print(f"Auto pan    : {auto_pan_min} .. {auto_pan_max} deg, step={args.auto_pan_step}")
    print(f"Auto tilt   : {auto_tilt_min} .. {auto_tilt_max} deg, step={args.auto_tilt_step}")
    print(f"Auto delay  : {args.auto_delay:.3f} s")
    print(f"Start mode  : {mode}")

    print_help()
    print(f"[INFO] Starting at pan={pan_angle}°, tilt={tilt_angle}°")

    last_auto_time = time.monotonic()

    try:
        with RawKeyReader() as kr:
            while True:
                key_raw = kr.read_key(timeout_s=0.02)

                if key_raw:
                    code = ord(key_raw)
                    key = key_raw.upper()

                    if key == "Q" or code == 27:
                        print("[INFO] Quit requested.")
                        break

                    if key == "H":
                        print_help()
                        continue

                    if key == "O":
                        mode = "auto" if mode == "manual" else "manual"
                        auto_paused = False
                        print(f"[MODE] {mode.upper()}")
                        continue

                    if key_raw == " ":
                        auto_paused = not auto_paused
                        print(f"[AUTO] paused={auto_paused}")
                        continue

                    if key == "C":
                        pan_angle = pan_center
                        tilt_angle = tilt_center
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[CENTER] pan={pan_angle}°, tilt={tilt_angle}°")
                        continue

                    if key == "A":
                        mode = "manual"
                        pan_angle = clamp(pan_angle - args.step, -180, 180)
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        print(f"[PAN] Left  → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "D":
                        mode = "manual"
                        pan_angle = clamp(pan_angle + args.step, -180, 180)
                        servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))
                        print(f"[PAN] Right → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "W":
                        mode = "manual"
                        tilt_angle = clamp(tilt_angle + args.step, 0, 180)
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[TILT] Up   → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "S":
                        mode = "manual"
                        tilt_angle = clamp(tilt_angle - args.step, 0, 180)
                        servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))
                        print(f"[TILT] Down → pan={pan_angle}°, tilt={tilt_angle}°")

                now = time.monotonic()

                if mode == "auto" and not auto_paused:
                    if now - last_auto_time >= args.auto_delay:
                        last_auto_time = now

                        if args.auto_axis in ("pan", "both"):
                            pan_angle, pan_direction = bounce(
                                pan_angle,
                                pan_direction,
                                args.auto_pan_step,
                                auto_pan_min,
                                auto_pan_max,
                            )
                            servo.set_servo_angle(args.pan_chan, servo_safe_angle(pan_angle))

                        if args.auto_axis in ("tilt", "both"):
                            tilt_angle, tilt_direction = bounce(
                                tilt_angle,
                                tilt_direction,
                                args.auto_tilt_step,
                                auto_tilt_min,
                                auto_tilt_max,
                            )
                            servo.set_servo_angle(args.tilt_chan, servo_safe_angle(tilt_angle))

                        print(f"[AUTO] {args.auto_axis} → pan={pan_angle}°, tilt={tilt_angle}°")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        try:
            servo.set_servo_angle(args.pan_chan, pan_center)
            servo.set_servo_angle(args.tilt_chan, tilt_center)
            print(f"[INFO] On exit: recentered to pan={pan_center}°, tilt={tilt_center}°")
        except Exception as e:
            print(f"[WARN] Could not recenter on exit: {e}")

        servo.close()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pan-Tilt Servo Test with manual and auto modes"
    )

    ap.add_argument(
        "--mode",
        choices=["manual", "auto"],
        default="manual",
        help="Start mode. Default: manual.",
    )

    ap.add_argument(
        "--pan-chan",
        default="7",
        help="Logical pan servo channel. Default: 7.",
    )
    ap.add_argument(
        "--tilt-chan",
        default="6",
        help="Logical tilt servo channel. Default: 6.",
    )

    ap.add_argument(
        "--pan-center",
        type=int,
        default=72,
        help="Calibrated pan center angle. Default: 72.",
    )
    ap.add_argument(
        "--tilt-center",
        type=int,
        default=55,
        help="Calibrated tilt center angle. Default: 55.",
    )

    ap.add_argument(
        "--step",
        type=int,
        default=5,
        help="Manual angle step per key press. Default: 5 degrees.",
    )

    ap.add_argument(
        "--auto-axis",
        choices=["pan", "tilt", "both"],
        default="both",
        help="Automatic sweep axis. Default: both.",
    )

    ap.add_argument(
        "--auto-pan-min",
        type=int,
        default=-180,
        help="Automatic pan minimum angle. Default: -180.",
    )
    ap.add_argument(
        "--auto-pan-max",
        type=int,
        default=170,
        help="Automatic pan maximum angle. Default: 170.",
    )
    ap.add_argument(
        "--auto-pan-step",
        type=int,
        default=5,
        help="Automatic pan step. Default: 5 degrees.",
    )

    ap.add_argument(
        "--auto-tilt-min",
        type=int,
        default=45,
        help="Automatic tilt minimum angle. Default: 45.",
    )
    ap.add_argument(
        "--auto-tilt-max",
        type=int,
        default=170,
        help="Automatic tilt maximum angle. Default: 170.",
    )
    ap.add_argument(
        "--auto-tilt-step",
        type=int,
        default=5,
        help="Automatic tilt step. Default: 5 degrees.",
    )

    ap.add_argument(
        "--auto-delay",
        type=float,
        default=0.12,
        help="Delay between automatic steps. Default: 0.12 seconds.",
    )

    args = ap.parse_args()
    run_pantilt(args)


if __name__ == "__main__":
    main()
