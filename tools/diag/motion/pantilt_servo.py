#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Pan-Tilt Servo Test (PCA9685)
------------------------------------------
Interactive controls (default):
  W / S : tilt up / tilt down
  A / D : pan left / pan right
  C     : center both servos (pan=90°, tilt=55° by default)
  Q or ESC : quit

Ranges:
  Pan  : 0 .. 180 degrees
  Tilt : 45 .. 180 degrees (defaults, can be changed with CLI)
  Default tilt center = 55 degrees.

Author: Robot Savo
"""

import sys
import time
import math
import argparse
import termios
import tty

import smbus


# ---------------------------------------------------------------------------
# PCA9685 driver
# ---------------------------------------------------------------------------
class PCA9685:
    # Registers/etc.
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

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
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, pulse_us: float) -> None:
        # 50 Hz → 20 ms period = 20000 us
        pulse = pulse_us * 4096.0 / 20000.0
        self.set_pwm(channel, 0, int(pulse))

    def close(self) -> None:
        self.bus.close()


# ---------------------------------------------------------------------------
# Servo helper
# ---------------------------------------------------------------------------
class Servo:
    def __init__(self, addr: int = 0x40, debug: bool = True):
        self.pwm_frequency = 50
        self.initial_pulse = 1500
        self.pwm_channel_map = {
            '0': 8,
            '1': 9,
            '2': 10,
            '3': 11,
            '4': 12,
            '5': 13,
            '6': 14,
            '7': 15,
        }
        self.pwm_servo = PCA9685(addr, debug=debug)
        self.pwm_servo.set_pwm_freq(self.pwm_frequency)

        for channel in self.pwm_channel_map.values():
            self.pwm_servo.set_servo_pulse(channel, self.initial_pulse)

    def angle_to_pulse(self, channel: str, angle: int, error: int = 10) -> float:
        angle = int(angle)
        if channel == '0':
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
        ch = self.pwm_channel_map[channel]
        self.pwm_servo.set_servo_pulse(ch, pulse)

    def close(self) -> None:
        self.pwm_servo.close()


# ---------------------------------------------------------------------------
# Keyboard helpers
# ---------------------------------------------------------------------------
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

    def read_key(self) -> str:
        return sys.stdin.read(1)


# ---------------------------------------------------------------------------
# Main interactive test
# ---------------------------------------------------------------------------
def interactive_test(
    pan_chan: str,
    tilt_chan: str,
    tilt_min: int,
    tilt_max: int,
    step: int,
    tilt_center: int = 55,
) -> None:
    """
    Interactive WASD pan-tilt test.

    Pan range:  0 .. 180 deg
    Tilt range: tilt_min .. tilt_max (e.g. 45 .. 180)
    """
    PAN_MIN = 0
    PAN_MAX = 180

    servo = Servo(addr=0x40, debug=True)

    # Clamp center and initial angles into allowed range
    tilt_center = max(tilt_min, min(tilt_max, tilt_center))

    pan_angle = 90
    tilt_angle = tilt_center

    pan_angle = max(PAN_MIN, min(PAN_MAX, pan_angle))
    tilt_angle = max(tilt_min, min(tilt_max, tilt_angle))

    servo.set_servo_angle(pan_chan, pan_angle)
    servo.set_servo_angle(tilt_chan, tilt_angle)

    print("\nRobot Savo — Pan-Tilt Servo Test")
    print("---------------------------------")
    print(f"Pan channel : '{pan_chan}' (PCA9685 ch {servo.pwm_channel_map[pan_chan]})")
    print(f"Tilt channel: '{tilt_chan}' (PCA9685 ch {servo.pwm_channel_map[tilt_chan]})")
    print(f"Pan range   : {PAN_MIN} .. {PAN_MAX} deg")
    print(f"Tilt range  : {tilt_min} .. {tilt_max} deg")
    print(f"Tilt center : {tilt_center} deg")
    print(f"Step size   : {step} deg\n")
    print("Controls:")
    print("  W / S : tilt up / tilt down")
    print("  A / D : pan left / pan right")
    print("  C     : center (pan=90°, tilt=tilt_center)")
    print("  Q or ESC : quit\n")
    print(f"[INFO] Starting at pan={pan_angle}°, tilt={tilt_angle}°")
    print("[INFO] Press keys now.\n")

    try:
        with RawKeyReader() as kr:
            while True:
                ch = kr.read_key()
                if not ch:
                    continue

                code = ord(ch)
                ch_up = ch.upper()

                if ch_up == 'Q' or code == 27:  # ESC
                    print("[INFO] Quit requested.")
                    break

                if ch_up == 'C':
                    pan_angle = 90
                    tilt_angle = tilt_center
                    pan_angle = max(PAN_MIN, min(PAN_MAX, pan_angle))
                    tilt_angle = max(tilt_min, min(tilt_max, tilt_angle))
                    servo.set_servo_angle(pan_chan, pan_angle)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[CENTER] pan={pan_angle}°, tilt={tilt_angle}°")
                    continue

                # Pan: always 0..180
                if ch_up == 'A':
                    pan_angle = max(PAN_MIN, pan_angle - step)
                    servo.set_servo_angle(pan_chan, pan_angle)
                    print(f"[PAN] Left  → pan={pan_angle}°, tilt={tilt_angle}°")
                elif ch_up == 'D':
                    pan_angle = min(PAN_MAX, pan_angle + step)
                    servo.set_servo_angle(pan_chan, pan_angle)
                    print(f"[PAN] Right → pan={pan_angle}°, tilt={tilt_angle}°")

                # Tilt: 45..180 (or CLI overrides)
                elif ch_up == 'W':
                    tilt_angle = min(tilt_max, tilt_angle + step)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[TILT] Up   → pan={pan_angle}°, tilt={tilt_angle}°")
                elif ch_up == 'S':
                    tilt_angle = max(tilt_min, tilt_angle - step)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[TILT] Down → pan={pan_angle}°, tilt={tilt_angle}°")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        try:
            pan_angle = 90
            tilt_angle = tilt_center
            pan_angle = max(PAN_MIN, min(PAN_MAX, pan_angle))
            tilt_angle = max(tilt_min, min(tilt_max, tilt_angle))
            servo.set_servo_angle(pan_chan, pan_angle)
            servo.set_servo_angle(tilt_chan, tilt_angle)
            print(f"[INFO] On exit: recentered to pan={pan_angle}°, tilt={tilt_angle}°")
        except Exception as e:
            print(f"[WARN] Could not recenter on exit: {e}")
        servo.close()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Robot Savo — Pan-Tilt Servo Test (PCA9685)"
    )
    ap.add_argument("--pan-chan", default="0",
                    help="Logical pan servo channel (default: '0' -> PCA 8).")
    ap.add_argument("--tilt-chan", default="1",
                    help="Logical tilt servo channel (default: '1' -> PCA 9).")
    ap.add_argument("--tilt-min", type=int, default=45,
                    help="Tilt minimum angle (default: 45).")
    ap.add_argument("--tilt-max", type=int, default=180,
                    help="Tilt maximum angle (default: 180).")
    ap.add_argument("--step", type=int, default=5,
                    help="Angle step per key press (default: 5 degrees).")
    ap.add_argument("--tilt-center", type=int, default=55,
                    help="Tilt center angle (default: 55).")

    args = ap.parse_args()

    interactive_test(
        pan_chan=args.pan_chan,
        tilt_chan=args.tilt_chan,
        tilt_min=args.tilt_min,
        tilt_max=args.tilt_max,
        step=args.step,
        tilt_center=args.tilt_center,
    )


if __name__ == "__main__":
    main()
