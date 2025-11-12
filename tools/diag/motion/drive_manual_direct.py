#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Expert PCA9685 Mecanum Teleop (NO ROS2)
----------------------------------------------------
Controls
  • W / S           : forward / backward
  • A / D           : strafe left / right
  • Arrow keys      : same as WASD
  • Q / E           : rotate CCW / CW
  • X or SPACE      : immediate stop (all zero)
  • Z / C           : speed scale down / up
  • R               : reset scale + gains + velocities
  • ESC             : quit (safe stop)

Design goals
  • Embed a minimal PCA9685 driver (I²C @ 0x40) — no external deps besides smbus.
  • RobotSavo class abstracts 4 mecanum wheels (FL, RL, FR, RR) with your channel map.
  • Neutral “quench” on sign change to avoid H-bridge latching when reversing.
  • Simple tuning constants at top; robust clamping & smooth decay.
"""

import sys, time, tty, termios, select, math
import smbus

# ================== PCA9685 (embedded) ==================
class PCA9685:
    # Registers/etc.
    __SUBADR1      = 0x02
    __SUBADR2      = 0x03
    __SUBADR3      = 0x04
    __MODE1        = 0x00
    __PRESCALE     = 0xFE
    __LED0_ON_L    = 0x06
    __LED0_ON_H    = 0x07
    __LED0_OFF_L   = 0x08
    __LED0_OFF_H   = 0x09
    __ALLLED_ON_L  = 0xFA
    __ALLLED_ON_H  = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address: int = 0x40, debug: bool = False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq: float) -> None:
        """Set PWM frequency in Hz."""
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10          # sleep
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)   # restart

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Set channel with 12-bit edges."""
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Set duty 0..4095 on a channel (no phase shift)."""
        if duty < 0: duty = 0
        if duty > 4095: duty = 4095
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, us: float) -> None:
        """Set servo pulse at 50 Hz (microseconds)."""
        ticks = int(us * 4096.0 / 20000.0)
        self.set_pwm(channel, 0, ticks)

    def close(self) -> None:
        self.bus.close()

# ================== Tuning & mappings ==================
MAX_DUTY_DEFAULT   = 3000     # leave headroom under 4095
TURN_GAIN_DEFAULT  = 1.0
STEP               = 0.15
DECAY              = 0.85
HZ                 = 30

# Make +vx be physical forward on YOUR robot (fixes W/S inversion)
FORWARD_SIGN       = -1       # set to +1 if you later rewire to the standard
STRAFE_SIGN        = +1
ROTATE_SIGN        = +1

# Per-wheel invert (if a single wheel is flipped physically)
FL_INV, RL_INV, FR_INV, RR_INV = +1, +1, +1, +1

# Quench (neutral) parameters when a wheel changes sign (reverse → forward or vice versa)
QUENCH_ON_SIGN_FLIP = True
QUENCH_MS           = 18      # 10–25 ms is typical and safe

# ================== Robot motor wrapper ==================
class RobotSavo:
    """
    RobotSavo controls 4 mecanum wheels mapped to PCA9685 channels as per your original code:

      FL (front-left)  : (IN pair channels 0,1)
      RL (rear-left)   : (IN pair channels 3,2)
      FR (front-right) : (IN pair channels 6,7)
      RR (rear-right)  : (IN pair channels 4,5)

    Positive duty => INx=(0,duty), Negative => (duty goes to the opposite pin).
    Zero duty     => both pins 'off' (4095) for neutral/brake.
    """
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)      # keep 50 Hz unless your H-bridge requires higher
        # Track last wheel signs for quench logic
        self._last_sign = {'fl': 0, 'rl': 0, 'fr': 0, 'rr': 0}

    # ---- low-level primitives for each wheel ----
    def _wheel_fl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(0,0);   self.pwm.set_motor_pwm(1,d)
        elif d < 0:  self.pwm.set_motor_pwm(1,0);   self.pwm.set_motor_pwm(0,-d)
        else:        self.pwm.set_motor_pwm(0,4095);self.pwm.set_motor_pwm(1,4095)

    def _wheel_rl(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(3,0);   self.pwm.set_motor_pwm(2,d)
        elif d < 0:  self.pwm.set_motor_pwm(2,0);   self.pwm.set_motor_pwm(3,-d)
        else:        self.pwm.set_motor_pwm(2,4095);self.pwm.set_motor_pwm(3,4095)

    def _wheel_fr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(6,0);   self.pwm.set_motor_pwm(7,d)
        elif d < 0:  self.pwm.set_motor_pwm(7,0);   self.pwm.set_motor_pwm(6,-d)
        else:        self.pwm.set_motor_pwm(6,4095);self.pwm.set_motor_pwm(7,4095)

    def _wheel_rr(self, d):
        if   d > 0:  self.pwm.set_motor_pwm(4,0);   self.pwm.set_motor_pwm(5,d)
        elif d < 0:  self.pwm.set_motor_pwm(5,0);   self.pwm.set_motor_pwm(4,-d)
        else:        self.pwm.set_motor_pwm(4,4095);self.pwm.set_motor_pwm(5,4095)

    @staticmethod
    def _clamp4(d1,d2,d3,d4):
        def c(v): return 4095 if v>4095 else (-4095 if v<-4095 else int(v))
        return c(d1), c(d2), c(d3), c(d4)

    def _apply_quench_if_needed(self, name: str, prev_sign: int, new_val: int, apply_fn) -> int:
        """Insert a brief neutral if sign flips and quench is enabled."""
        new_sign = 0 if new_val == 0 else (1 if new_val > 0 else -1)
        if QUENCH_ON_SIGN_FLIP and prev_sign != 0 and new_sign != 0 and prev_sign != new_sign:
            # neutral
            apply_fn(0)
            time.sleep(QUENCH_MS / 1000.0)
        apply_fn(new_val)
        self._last_sign[name] = new_sign
        return new_sign

    def set_motor_model(self, d_fl, d_rl, d_fr, d_rr):
        """Send wheel duties (int, -4095..4095) with clamping, invert, and quench."""
        # Per-wheel invert
        d_fl *= FL_INV; d_rl *= RL_INV; d_fr *= FR_INV; d_rr *= RR_INV
        d_fl, d_rl, d_fr, d_rr = self._clamp4(d_fl, d_rl, d_fr, d_rr)

        # Apply with quench handling
        self._apply_quench_if_needed('fl', self._last_sign['fl'], d_fl, self._wheel_fl)
        self._apply_quench_if_needed('rl', self._last_sign['rl'], d_rl, self._wheel_rl)
        self._apply_quench_if_needed('fr', self._last_sign['fr'], d_fr, self._wheel_fr)
        self._apply_quench_if_needed('rr', self._last_sign['rr'], d_rr, self._wheel_rr)

    def stop(self):
        self.set_motor_model(0,0,0,0)

    def close(self):
        self.stop()
        self.pwm.close()

# ================== Teleop helpers ==================
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def read_key():
    """
    Return: 'w','a','s','d','q','e','x',' ','z','c','r','esc','up','down','left','right',''
    Supports ANSI arrow sequences.
    """
    dr,_,_ = select.select([sys.stdin], [], [], 0)
    if not dr: return ''
    ch = sys.stdin.read(1)
    if ch == '\x1b':     # ESC or arrows
        time.sleep(0.001)
        if select.select([sys.stdin], [], [], 0)[0]:
            ch2 = sys.stdin.read(1)
            if ch2 == '[' and select.select([sys.stdin], [], [], 0)[0]:
                ch3 = sys.stdin.read(1)
                return {'A':'up','B':'down','C':'right','D':'left'}.get(ch3,'')
            return ''
        return 'esc'
    return ch.lower()

def mix_mecanum(vx, vy, wz, turn_gain=TURN_GAIN_DEFAULT):
    """
    Return normalized wheel commands (fl, rl, fr, rr) in [-1..1].
    Mecanum kinematics (Holonomic):
        fl =  vx - vy - w
        rl =  vx + vy - w
        fr =  vx + vy + w
        rr =  vx - vy + w
    Signs FORWARD/STRAFE/ROTATE applied so +vx is your physical forward.
    """
    vx *= FORWARD_SIGN
    vy *= STRAFE_SIGN
    w  = ROTATE_SIGN * turn_gain * wz

    fl =  vx - vy - w
    rl =  vx + vy - w
    fr =  vx + vy + w
    rr =  vx - vy + w

    m = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    return fl/m, rl/m, fr/m, rr/m

def to_duties(nfl, nrl, nfr, nrr, max_duty):
    return int(nfl*max_duty), int(nrl*max_duty), int(nfr*max_duty), int(nrr*max_duty)

# ================== Main loop ==================
def main():
    bot = RobotSavo()
    vx = vy = wz = 0.0
    max_duty  = MAX_DUTY_DEFAULT
    turn_gain = TURN_GAIN_DEFAULT

    print("\nRobot Savo teleop ready.")
    print("W/S/A/D or Arrows; Q/E rotate; X/Space stop; Z/C scale -, +; R reset; ESC quit.")
    print(f"scale={max_duty}/4095  forward_sign={FORWARD_SIGN}  quench={QUENCH_MS}ms\n")

    try:
        with RawTerminal():
            last = time.time()
            while True:
                key = read_key()
                if key:
                    if key == 'esc': break
                    elif key in ('w','up'):      vx = min(1.0,  vx + STEP)
                    elif key in ('s','down'):    vx = max(-1.0, vx - STEP)
                    elif key in ('a','left'):    vy = max(-1.0, vy - STEP)
                    elif key in ('d','right'):   vy = min(1.0,  vy + STEP)
                    elif key == 'q':             wz = min(1.0,  wz + STEP)   # CCW
                    elif key == 'e':             wz = max(-1.0, wz - STEP)   # CW
                    elif key in ('x',' '):       vx = vy = wz = 0.0
                    elif key == 'z':
                        max_duty = max(600, int(max_duty * 0.85))
                        print(f"[scale↓] max_duty={max_duty}")
                    elif key == 'c':
                        max_duty = min(4095, int(max_duty * 1.15))
                        print(f"[scale↑] max_duty={max_duty}")
                    elif key == 'r':
                        vx = vy = wz = 0.0
                        max_duty  = MAX_DUTY_DEFAULT
                        turn_gain = TURN_GAIN_DEFAULT
                        print("[reset] zeroed; scale/turn_gain reset.")

                # smooth decay toward zero between keypresses
                now = time.time()
                dt  = now - last
                last = now
                decay = DECAY ** (dt * HZ)
                vx *= decay; vy *= decay; wz *= decay

                # mix and send
                fl, rl, fr, rr = mix_mecanum(vx, vy, wz, turn_gain)
                dfl, drl, dfr, drr = to_duties(fl, rl, fr, rr, max_duty)
                bot.set_motor_model(dfl, drl, dfr, drr)

                # ~HZ control loop
                time.sleep(max(0.0, (1.0/HZ) - (time.time()-now)))
    except KeyboardInterrupt:
        pass
    finally:
        bot.close()
        print("\nStopped. Bye.")

if __name__ == "__main__":
    main()
