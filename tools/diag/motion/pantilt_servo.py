#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive and automatic PCA9685 pan-tilt servo diagnostic for Robot Savo."""

import math
import argparse
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


def print_help() -> None:
    print("\nControls:")
    print("  W / S     : tilt up / tilt down")
    print("  A / D     : pan left / pan right")
    print("  O         : toggle auto/manual sweep")
    print("  SPACE     : pause/resume auto sweep")
    print("  C         : calibrated center")
    print("  H         : show help")
    print("  Q or ESC  : quit\n")


def run_pantilt(args: argparse.Namespace) -> None:
    servo = Servo(addr=0x40, debug=True)

    pan_center = clamp(args.pan_center, args.pan_min, args.pan_max)
    tilt_center = clamp(args.tilt_center, args.tilt_min, args.tilt_max)

    auto_pan_min = clamp(args.auto_pan_min, args.pan_min, args.pan_max)
    auto_pan_max = clamp(args.auto_pan_max, args.pan_min, args.pan_max)

    if auto_pan_min > auto_pan_max:
        auto_pan_min, auto_pan_max = auto_pan_max, auto_pan_min

    pan_angle = pan_center
    tilt_angle = tilt_center
    auto_direction = 1
    auto_paused = False
    mode = args.mode

    servo.set_servo_angle(args.pan_chan, pan_angle)
    servo.set_servo_angle(args.tilt_chan, tilt_angle)

    print("\nRobot Savo — Pan-Tilt Servo Test")
    print("---------------------------------")
    print(f"Pan channel : '{args.pan_chan}' (PCA9685 ch {servo.pwm_channel_map[args.pan_chan]})")
    print(f"Tilt channel: '{args.tilt_chan}' (PCA9685 ch {servo.pwm_channel_map[args.tilt_chan]})")
    print(f"Pan range   : {args.pan_min} .. {args.pan_max} deg")
    print(f"Tilt range  : {args.tilt_min} .. {args.tilt_max} deg")
    print(f"Pan center  : {pan_center} deg")
    print(f"Tilt center : {tilt_center} deg")
    print(f"Manual step : {args.step} deg")
    print(f"Auto sweep  : pan {auto_pan_min} .. {auto_pan_max} deg")
    print(f"Auto step   : {args.auto_step} deg")
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
                        servo.set_servo_angle(args.pan_chan, pan_angle)
                        servo.set_servo_angle(args.tilt_chan, tilt_angle)
                        print(f"[CENTER] pan={pan_angle}°, tilt={tilt_angle}°")
                        continue

                    if key == "A":
                        mode = "manual"
                        pan_angle = clamp(pan_angle - args.step, args.pan_min, args.pan_max)
                        servo.set_servo_angle(args.pan_chan, pan_angle)
                        print(f"[PAN] Left  → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "D":
                        mode = "manual"
                        pan_angle = clamp(pan_angle + args.step, args.pan_min, args.pan_max)
                        servo.set_servo_angle(args.pan_chan, pan_angle)
                        print(f"[PAN] Right → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "W":
                        mode = "manual"
                        tilt_angle = clamp(tilt_angle + args.step, args.tilt_min, args.tilt_max)
                        servo.set_servo_angle(args.tilt_chan, tilt_angle)
                        print(f"[TILT] Up   → pan={pan_angle}°, tilt={tilt_angle}°")

                    elif key == "S":
                        mode = "manual"
                        tilt_angle = clamp(tilt_angle - args.step, args.tilt_min, args.tilt_max)
                        servo.set_servo_angle(args.tilt_chan, tilt_angle)
                        print(f"[TILT] Down → pan={pan_angle}°, tilt={tilt_angle}°")

                now = time.monotonic()

                if mode == "auto" and not auto_paused:
                    if now - last_auto_time >= args.auto_delay:
                        last_auto_time = now

                        pan_angle += auto_direction * args.auto_step

                        if pan_angle >= auto_pan_max:
                            pan_angle = auto_pan_max
                            auto_direction = -1

                        elif pan_angle <= auto_pan_min:
                            pan_angle = auto_pan_min
                            auto_direction = 1

                        servo.set_servo_angle(args.pan_chan, pan_angle)
                        servo.set_servo_angle(args.tilt_chan, tilt_angle)
                        print(f"[AUTO] Sweep → pan={pan_angle}°, tilt={tilt_angle}°")

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
    ap.add_argument("--mode", choices=["manual", "auto"], default="manual",
                    help="Start mode. Default: manual.")
    ap.add_argument("--pan-chan", default="7",
                    help="Logical pan servo channel. Default: 7.")
    ap.add_argument("--tilt-chan", default="6",
                    help="Logical tilt servo channel. Default: 6.")
    ap.add_argument("--pan-min", type=int, default=0,
                    help="Pan minimum angle. Default: 0.")
    ap.add_argument("--pan-max", type=int, default=180,
                    help="Pan maximum angle. Default: 180.")
    ap.add_argument("--tilt-min", type=int, default=45,
                    help="Tilt minimum angle. Default: 45.")
    ap.add_argument("--tilt-max", type=int, default=120,
                    help="Tilt maximum angle. Default: 120.")
    ap.add_argument("--pan-center", type=int, default=72,
                    help="Calibrated pan center angle. Default: 72.")
    ap.add_argument("--tilt-center", type=int, default=55,
                    help="Calibrated tilt center angle. Default: 55.")
    ap.add_argument("--step", type=int, default=3,
                    help="Manual angle step per key press. Default: 3 degrees.")
    ap.add_argument("--auto-pan-min", type=int, default=45,
                    help="Automatic sweep minimum pan angle. Default: 45.")
    ap.add_argument("--auto-pan-max", type=int, default=105,
                    help="Automatic sweep maximum pan angle. Default: 105.")
    ap.add_argument("--auto-step", type=int, default=1,
                    help="Automatic sweep step size. Default: 1 degree.")
    ap.add_argument("--auto-delay", type=float, default=0.08,
                    help="Delay between automatic sweep steps. Default: 0.08 seconds.")

    args = ap.parse_args()
    run_pantilt(args)


if __name__ == "__main__":
    main()
