#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — PCA9685 Mecanum Teleop (NO ROS2)

Keys
----
  W / S : forward / backward
  A / D : strafe left / right
  Q / E : rotate CCW / CW
  X     : stop (zero all)
  SPACE : stop (same as X)
  Z / C : speed scale down / up
  R     : reset scale to 1.0
  ESC   : quit

Notes
-----
- Uses your existing Ordinary_Car + pca9685 driver (duty in [-4095..4095]).
- Keeps your 50 Hz setup (don’t touch servos / channels 8–15).
- Mecanum mix (front-left, rear-left, front-right, rear-right):
    fl =  vx - vy - ωk
    rl =  vx + vy - ωk
    fr =  vx + vy + ωk
    rr =  vx - vy + ωk
  where vx,vy,ω are in [-1..1]; k (turn gain) ~ wheelbase factor.

- If your wheel direction is flipped, toggle the sign at the mapping section.
"""

import sys
import time
import tty
import termios
import select
import math

# ---------- your original code (unchanged) ----------
import time as _t
from pca9685 import PCA9685

class Ordinary_Car:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
    def duty_range(self, duty1, duty2, duty3, duty4):
        def clamp(v): 
            return 4095 if v>4095 else (-4095 if v<-4095 else int(v))
        return clamp(duty1), clamp(duty2), clamp(duty3), clamp(duty4)
    def left_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(0,0)
            self.pwm.set_motor_pwm(1,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(1,0)
            self.pwm.set_motor_pwm(0,abs(duty))
        else:
            self.pwm.set_motor_pwm(0,4095)
            self.pwm.set_motor_pwm(1,4095)
    def left_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(3,0)
            self.pwm.set_motor_pwm(2,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(2,0)
            self.pwm.set_motor_pwm(3,abs(duty))
        else:
            self.pwm.set_motor_pwm(2,4095)
            self.pwm.set_motor_pwm(3,4095)
    def right_upper_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(6,0)
            self.pwm.set_motor_pwm(7,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(7,0)
            self.pwm.set_motor_pwm(6,abs(duty))
        else:
            self.pwm.set_motor_pwm(6,4095)
            self.pwm.set_motor_pwm(7,4095)
    def right_lower_wheel(self,duty):
        if duty>0:
            self.pwm.set_motor_pwm(4,0)
            self.pwm.set_motor_pwm(5,duty)
        elif duty<0:
            self.pwm.set_motor_pwm(5,0)
            self.pwm.set_motor_pwm(4,abs(duty))
        else:
            self.pwm.set_motor_pwm(4,4095)
            self.pwm.set_motor_pwm(5,4095)
    def set_motor_model(self, duty1, duty2, duty3, duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_upper_wheel(duty1)   # Front-Left (FL)
        self.left_lower_wheel(duty2)   # Rear-Left  (RL)
        self.right_upper_wheel(duty3)  # Front-Right(FR)
        self.right_lower_wheel(duty4)  # Rear-Right (RR)
    def close(self):
        self.set_motor_model(0,0,0,0)
        self.pwm.close()
# ----------------------------------------------------

MAX_DUTY_DEFAULT = 3000     # a bit under 4095 for margin
TURN_GAIN_DEFAULT = 1.0     # scale ω contribution (increase if turns are weak)
STEP = 0.15                 # velocity increment per key press
DECAY = 0.85                # friction decay toward 0 when no key
HZ = 30                     # control loop Hz

def _kbhit(timeout=0.0):
    """Return a single char if pressed within timeout seconds, else ''."""
    dr,_,_ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''

class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def mix_mecanum(vx, vy, wz, turn_gain=TURN_GAIN_DEFAULT):
    """Return wheel commands (fl, rl, fr, rr) in [-1..1]."""
    w = turn_gain * wz
    fl =  vx - vy - w
    rl =  vx + vy - w
    fr =  vx + vy + w
    rr =  vx - vy + w
    # normalize if any exceeds magnitude 1
    m = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return fl/m, rl/m, fr/m, rr/m

def to_duties(nfl, nrl, nfr, nrr, max_duty):
    """Map normalized [-1..1] to integer duties [-max_duty..max_duty]."""
    return (int(nfl*max_duty), int(nrl*max_duty),
            int(nfr*max_duty), int(nrr*max_duty))

def main():
    car = Ordinary_Car()
    vx = vy = wz = 0.0
    max_duty = MAX_DUTY_DEFAULT
    turn_gain = TURN_GAIN_DEFAULT

    print("\nMecanum Teleop ready.")
    print("  WASD = move, Q/E = rotate, X/SPACE = stop, Z/C = speed -, +, R = reset, ESC = quit\n")
    print(f"  scale = {max_duty}/{4095}  turn_gain = {turn_gain}")

    try:
        with RawTerminal():
            last = time.time()
            while True:
                # read keys quickly without blocking the loop
                ch = _kbhit(0.0)

                if ch:
                    c = ch.lower()
                    if c == '\x1b':  # ESC
                        break
                    elif c == 'w':
                        vx = min(1.0, vx + STEP)
                    elif c == 's':
                        vx = max(-1.0, vx - STEP)
                    elif c == 'a':
                        vy = max(-1.0, vy - STEP)   # left is negative vy
                    elif c == 'd':
                        vy = min(1.0, vy + STEP)    # right is positive vy
                    elif c == 'q':
                        wz = min(1.0, wz + STEP)    # CCW +
                    elif c == 'e':
                        wz = max(-1.0, wz - STEP)   # CW  -
                    elif c in ('x', ' '):
                        vx = vy = wz = 0.0
                    elif c == 'z':
                        max_duty = max(600, int(max_duty * 0.85))
                        print(f"[scale↓] max_duty={max_duty}")
                    elif c == 'c':
                        max_duty = min(4095, int(max_duty * 1.15))
                        print(f"[scale↑] max_duty={max_duty}")
                    elif c == 'r':
                        vx = vy = wz = 0.0
                        max_duty = MAX_DUTY_DEFAULT
                        turn_gain = TURN_GAIN_DEFAULT
                        print("[reset] velocities zeroed; scale/turn_gain reset.")
                    # ignore other keys

                # passive decay toward zero for smoothness when no keys held
                now = time.time()
                dt = now - last
                last = now
                decay = DECAY ** (dt * HZ)  # make decay rate time-step independent
                vx *= decay
                vy *= decay
                wz *= decay

                # mix ➜ wheel normalized commands
                fl, rl, fr, rr = mix_mecanum(vx, vy, wz, turn_gain)

                # If a particular wheel is inverted in your hardware,
                # flip its sign here (e.g., fl *= -1).
                # fl *= 1; rl *= 1; fr *= 1; rr *= 1

                d_fl, d_rl, d_fr, d_rr = to_duties(fl, rl, fr, rr, max_duty)

                # send to motors (your mapping: FL, RL, FR, RR)
                car.set_motor_model(d_fl, d_rl, d_fr, d_rr)

                # run at ~HZ
                time.sleep(max(0.0, (1.0/HZ) - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        car.close()
        print("\nStopped. Bye.")

if __name__ == "__main__":
    main()
