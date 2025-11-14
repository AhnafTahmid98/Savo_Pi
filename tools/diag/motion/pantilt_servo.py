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

Author: Robot Savo

"""

import sys
import time
import math
import argparse
import termios
import tty
from typing import Tuple

import smbus


# ---------------------------------------------------------------------------
# PCA9685 driver (from your servo.py, slightly tidied)
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
        """Writes an 8-bit value to the specified register/address."""
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg: int) -> int:
        """Read an unsigned byte from the I2C device."""
        result = self.bus.read_byte_data(self.address, reg)
        return result

    def set_pwm_freq(self, freq: float) -> None:
        """Sets the PWM frequency."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Sets a single PWM channel."""
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Sets the PWM duty cycle for a motor (0..4095)."""
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, pulse_us: float) -> None:
        """
        Sets the Servo Pulse width in microseconds.
        Assumes PWM frequency is 50 Hz → period is 20,000 us.
        """
        pulse = pulse_us * 4096.0 / 20000.0  # 12-bit within 20ms
        self.set_pwm(channel, 0, int(pulse))

    def close(self) -> None:
        """Close the I2C bus."""
        self.bus.close()


# ---------------------------------------------------------------------------
# Servo helper (from your code, with small safety clamps)
# ---------------------------------------------------------------------------
class Servo:
    def __init__(self, addr: int = 0x40, debug: bool = True):
        self.pwm_frequency = 50
        self.initial_pulse = 1500
        # Logical channels mapped to PCA9685 channels 8..15
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

        # Initialize all defined channels to neutral position
        for channel in self.pwm_channel_map.values():
            self.pwm_servo.set_servo_pulse(channel, self.initial_pulse)

    def angle_to_pulse(self, channel: str, angle: int, error: int = 10) -> float:
        """
        Convert angle (0..180) to microsecond pulse.
        This replicates your original formula:
          - channel '0': 2500 - (angle+error)/0.09
          - others    :  500 + (angle+error)/0.09
        """
        angle = int(angle)
        if channel == '0':
            pulse = 2500.0 - float((angle + error) / 0.09)
        else:
            pulse = 500.0 + float((angle + error) / 0.09)
        # Clamp to a safe typical range ~500..2500 us
        return max(500.0, min(2500.0, pulse))

    def set_servo_angle(self, channel: str, angle: int, error: int = 10) -> None:
        """Set a servo to a given angle (0..180)."""
        if channel not in self.pwm_channel_map:
            raise ValueError(
                f"Invalid channel: {channel}. Valid channels: {list(self.pwm_channel_map.keys())}"
            )
        angle = max(0, min(180, int(angle)))
        pulse = self.angle_to_pulse(channel, angle, error)
        ch = self.pwm_channel_map[channel]
        self.pwm_servo.set_servo_pulse(ch, pulse)

    def close(self) -> None:
        self.pwm_servo.close()


# ---------------------------------------------------------------------------
# Keyboard helpers (raw key reading without Enter)
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
        """
        Read a single character from stdin (non-blocking echo).
        Returns the raw character (1-byte). ESC sequences for arrows are longer,
        but for this script we only use simple keys (WASD, C, Q).
        """
        ch = sys.stdin.read(1)
        return ch


# ---------------------------------------------------------------------------
# Main interactive test
# ---------------------------------------------------------------------------
def interactive_test(pan_chan: str,
                     tilt_chan: str,
                     min_angle: int,
                     max_angle: int,
                     step: int,
                     tilt_center: int = 55) -> None:
    """
    Interactive WASD pan-tilt test.

    tilt_center: default center angle for tilt (e.g. 55 deg for your mount).
    """
    servo = Servo(addr=0x40, debug=True)

    # Clamp center and initial angles into allowed range
    tilt_center = max(min_angle, min(max_angle, tilt_center))

    # Start centered: pan=90, tilt=tilt_center
    pan_angle = max(min_angle, min(max_angle, 90))
    tilt_angle = tilt_center
    servo.set_servo_angle(pan_chan, pan_angle)
    servo.set_servo_angle(tilt_chan, tilt_angle)

    print("\nRobot Savo — Pan-Tilt Servo Test")
    print("---------------------------------")
    print(f"Pan channel : '{pan_chan}' (PCA9685 ch {servo.pwm_channel_map[pan_chan]})")
    print(f"Tilt channel: '{tilt_chan}' (PCA9685 ch {servo.pwm_channel_map[tilt_chan]})")
    print(f"Angle range : {min_angle} .. {max_angle} deg")
    print(f"Tilt center : {tilt_center} deg")
    print(f"Step size   : {step} deg\n")
    print("Controls:")
    print("  W / S : tilt up / tilt down")
    print("  A / D : pan left / pan right")
    print("  C     : center both (pan=90°, tilt=tilt_center)")
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
                    pan_angle = max(min_angle, min(max_angle, 90))
                    tilt_angle = tilt_center
                    servo.set_servo_angle(pan_chan, pan_angle)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[CENTER] pan={pan_angle}°, tilt={tilt_angle}°")
                    continue

                # Pan
                if ch_up == 'A':
                    pan_angle = max(min_angle, pan_angle - step)
                    servo.set_servo_angle(pan_chan, pan_angle)
                    print(f"[PAN] Left  → pan={pan_angle}°, tilt={tilt_angle}°")
                elif ch_up == 'D':
                    pan_angle = min(max_angle, pan_angle + step)
                    servo.set_servo_angle(pan_chan, pan_angle)
                    print(f"[PAN] Right → pan={pan_angle}°, tilt={tilt_angle}°")

                # Tilt
                elif ch_up == 'W':
                    tilt_angle = min(max_angle, tilt_angle + step)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[TILT] Up   → pan={pan_angle}°, tilt={tilt_angle}°")
                elif ch_up == 'S':
                    tilt_angle = max(min_angle, tilt_angle - step)
                    servo.set_servo_angle(tilt_chan, tilt_angle)
                    print(f"[TILT] Down → pan={pan_angle}°, tilt={tilt_angle}°")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C caught, exiting...")

    finally:
        # On exit: recenter to pan=90, tilt=tilt_center
        try:
            pan_angle = max(min_angle, min(max_angle, 90))
            tilt_angle = tilt_center
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
    ap.add_argument("--min-angle", type=int, default=45,
                    help="Minimum angle (default: 45 for your tilt).")
    ap.add_argument("--max-angle", type=int, default=180,
                    help="Maximum angle (default: 180).")
    ap.add_argument("--step", type=int, default=5,
                    help="Angle step per key press (default: 5 degrees).")
    ap.add_argument("--tilt-center", type=int, default=55,
                    help="Tilt center angle (default: 55 degrees).")

    args = ap.parse_args()

    interactive_test(
        pan_chan=args.pan_chan,
        tilt_chan=args.tilt_chan,
        min_angle=args.min_angle,
        max_angle=args.max_angle,
        step=args.step,
        tilt_center=args.tilt_center,
    )


if __name__ == "__main__":
    main()
