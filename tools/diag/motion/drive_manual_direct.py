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

# ========== DEBUG ==========
DEBUG_KEYS = False  # set True to print raw key sequences we don't recognize

# ================== PCA9685 (embedded) ==================
class PCA9685:
    __SUBADR1=0x02; __SUBADR2=0x03; __SUBADR3=0x04
    __MODE1=0x00; __PRESCALE=0xFE
    __LED0_ON_L=0x06; __LED0_ON_H=0x07
    __LED0_OFF_L=0x08; __LED0_OFF_H=0x09
    __ALLLED_ON_L=0xFA; __ALLLED_ON_H=0xFB
    __ALLLED_OFF_L=0xFC; __ALLLED_OFF_H=0xFD

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
        prescaleval = 25_000_000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        base = self.__LED0_ON_L + 4 * channel
        self.write(base + 0, on & 0xFF)
        self.write(base + 1, (on >> 8) & 0x0F)
        self.write(base + 2, off & 0xFF)
        self.write(base + 3, (off >> 8) & 0x0F)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        if duty < 0: duty = 0
        if duty > 4095: duty = 4095
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, us: float) -> None:
        ticks = int(us * 4096.0 / 20000.0)
        self.set_pwm(channel, 0, ticks)

    def close(self) -> None:
        self.bus.close()

# ================== Tuning & mappings ==================
MAX_DUTY_DEFAULT   = 3000
TURN_GAIN_DEFAULT  = 1.0
STEP               = 0.15
DECAY              = 0.85
HZ                 = 30

# Fix W/S inversion for your wiring
FORWARD_SIGN       = -1
STRAFE_SIGN        = +1
ROTATE_SIGN        = +1

# Per-wheel invert toggles
FL_INV, RL_INV, FR_INV, RR_INV = +1, +1, +1, +1

# Quench (neutral) on sign flip
QUENCH_ON_SIGN_FLIP = True
QUENCH_MS           = 18

# ================== Robot motor wrapper ==================
class RobotSavo:
    """
    Channel map (as per your earlier code):
      FL: (0,1)   RL: (3,2)   FR: (6,7)   RR: (4,5)
    """
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.set_pwm_freq(50)
        self._last_sign = {'fl':0,'rl':0,'fr':0,'rr':0}

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

    def _apply_quench_if_needed(self, name: str, prev_sign: int, new_val: int, fn) -> int:
        new_sign = 0 if new_val == 0 else (1 if new_val > 0 else -1)
        if QUENCH_ON_SIGN_FLIP and prev_sign and new_sign and (prev_sign != new_sign):
            fn(0); time.sleep(QUENCH_MS/1000.0)
        fn(new_val)
        self._last_sign[name] = new_sign
        return new_sign

    def set_motor_model(self, d_fl, d_rl, d_fr, d_rr):
        d_fl *= FL_INV; d_rl *= RL_INV; d_fr *= FR_INV; d_rr *= RR_INV
        d_fl, d_rl, d_fr, d_rr = self._clamp4(d_fl, d_rl, d_fr, d_rr)
        self._apply_quench_if_needed('fl', self._last_sign['fl'], d_fl, self._wheel_fl)
        self._apply_quench_if_needed('rl', self._last_sign['rl'], d_rl, self._wheel_rl)
        self._apply_quench_if_needed('fr', self._last_sign['fr'], d_fr, self._wheel_fr)
        self._apply_quench_if_needed('rr', self._last_sign['rr'], d_rr, self._wheel_rr)

    def stop(self): self.set_motor_model(0,0,0,0)
    def close(self): self.stop(); self.pwm.close()

# ================== Terminal helpers ==================
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)  # raw mode captures full escape sequences
        return self
    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def _read_available(timeout=0):
    """Return bytes available on stdin within timeout."""
    b = bytearray()
    while True:
        r,_,_ = select.select([sys.stdin], [], [], timeout)
        if not r: break
        ch = sys.stdin.read(1)
        if not ch: break
        b.extend(ch.encode('latin1', 'ignore'))
        # after first read, keep gobbling without additional wait
        timeout = 0
    return bytes(b)

def read_key():
    """
    Robustly parse: letters, space, ESC, and arrow keys (CSI or SS3 forms).
    Returns: 'w','a','s','d','q','e','x',' ','z','c','r','esc','up','down','left','right',''
    """
    r,_,_ = select.select([sys.stdin], [], [], 0)
    if not r: return ''
    seq = _read_available(0)
    if not seq: return ''

    # Single-byte letters / space
    if len(seq) == 1:
        ch = seq.decode('latin1').lower()
        if ch == '\x1b':   # bare ESC
            return 'esc'
        return ch

    # Escape sequences (arrows etc.)
    # Common forms:
    #  CSI:  \x1b [ A/B/C/D
    #  SS3:  \x1b O A/B/C/D
    if seq.startswith(b'\x1b[') and len(seq) >= 3:
        code = seq[2:3]
        key = {b'A':'up', b'B':'down', b'C':'right', b'D':'left'}.get(code, '')
        if key: return key
    if seq.startswith(b'\x1bO') and len(seq) >= 3:
        code = seq[2:3]
        key = {b'A':'up', b'B':'down', b'C':'right', b'D':'left'}.get(code, '')
        if key: return key

    if DEBUG_KEYS:
        print(f"[keydebug] unknown seq: {seq!r}")
    return ''

# ================== Kinematics ==================
def mix_mecanum(vx, vy, wz, turn_gain=TURN_GAIN_DEFAULT):
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
    if not sys.stdin.isatty():
        print("⚠ This program needs a real TTY (not a pipe). Run in a terminal.")
    bot = RobotSavo()
    vx = vy = wz = 0.0
    max_duty  = MAX_DUTY_DEFAULT
    turn_gain = TURN_GAIN_DEFAULT

    print("\nRobot Savo teleop ready.")
    print("W/S/A/D or Arrows; Q/E rotate; X/Space stop; Z/C scale -, +; R reset; ESC quit.")
    print(f"scale={max_duty}/4095  forward_sign={FORWARD_SIGN}  quench={QUENCH_MS}ms  debug={DEBUG_KEYS}\n")

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
                        max_duty = max(600, int(max_duty*0.85))
                        print(f"[scale↓] max_duty={max_duty}")
                    elif key == 'c':
                        max_duty = min(4095, int(max_duty*1.15))
                        print(f"[scale↑] max_duty={max_duty}")
                    elif key == 'r':
                        vx = vy = wz = 0.0
                        max_duty  = MAX_DUTY_DEFAULT
                        turn_gain = TURN_GAIN_DEFAULT
                        print("[reset] zeroed; scale/turn_gain reset.")

                # smooth decay toward zero
                now = time.time(); dt = now - last; last = now
                decay = DECAY ** (dt * HZ)
                vx *= decay; vy *= decay; wz *= decay

                fl, rl, fr, rr = mix_mecanum(vx, vy, wz, turn_gain)
                dfl, drl, dfr, drr = to_duties(fl, rl, fr, rr, max_duty)
                bot.set_motor_model(dfl, drl, dfr, drr)

                time.sleep(max(0.0, (1.0/HZ) - (time.time()-now)))
    except KeyboardInterrupt:
        pass
    finally:
        bot.close()
        print("\nStopped. Bye.")

if __name__ == "__main__":
    main()
