#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Teleop (PAIR MODE, PCA9685 direct)
-----------------------------------------------
Use this when your board ties front+rear wheels together on each side
(i.e., one PWM+IN1+IN2 triad drives both left wheels, another drives both right).

Controls
  W / ↑ : both sides forward
  S / ↓ : both sides reverse
  Q     : turn left (CCW)   -> left reverse, right forward
  E     : turn right (CW)   -> left forward, right reverse
  SPACE/0: stop
  M     : print current duties
  1/2   : pulse LEFT forward/reverse
  3/4   : pulse RIGHT forward/reverse
  Esc / Ctrl+C : quit

Quality-of-life
  --duration N   : auto-quit after N seconds (0=never)
  --idle-exit N  : auto-quit if no keys for N seconds (0=never)
  --quiet        : suppress per-tick prints (still prints key events)
  Deadman: stops motors after --deadman seconds with no key press

Example (common Freenove-style wiring)
  python3 tools/diag/motion/teleop_pairmode_pca.py \
    --i2c-bus 1 --pca-addr 0x40 --pwm-freq 200 \
    --left-pwm 0  --left-in1 1  --left-in2 2 \
    --right-pwm 3 --right-in1 4 --right-in2 5 \
    --reserve 12,13,14,15 --idle-exit 10
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------- I2C ----------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ------------- PCA9685 mini driver -------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=False):
        self.addr = int(addr)
        self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV)
        self._w8(MODE1, AI)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def _raw(self, ch: int, on: int, off: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF)
        self._w8(base+1, (on >> 8) & 0x0F)
        self._w8(base+2, off & 0xFF)
        self._w8(base+3, (off >> 8) & 0x0F)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x00)
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:      self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else:             self._raw(ch, 0, off)

# ------------- H-bridge pair (one side) -------------
@dataclass
class Triplet:
    pwm: int
    in1: int
    in2: int

class SideDrive:
    def __init__(self, pca: PCA9685, ch: Triplet, invert=False, in_active_low=False, swap_in12=False, name="SIDE"):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.name = name
        self.stop()

    def _digital(self, ch: int, level: int):
        lvl = (level ^ 1) if self.in_active_low else level
        if lvl: self.pca.full_on(ch)
        else:   self.pca.full_off(ch)

    def drive(self, signed: float, verbose=False):
        d = max(-1.0, min(1.0, float(signed)))
        if self.invert: d = -d
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)

        if abs(d) < 1e-3:
            self._digital(a, 0); self._digital(b, 0); self.pca.set_duty(self.ch.pwm, 0.0)
            if verbose: print(f"[{self.name}] STOP (pwm=CH{self.ch.pwm})")
            return

        if d > 0:
            self._digital(a, 1); self._digital(b, 0); self.pca.set_duty(self.ch.pwm, d)
            if verbose: print(f"[{self.name}] FWD  inA=CH{a} inB=CH{b} duty={d:.2f}")
        else:
            self._digital(a, 0); self._digital(b, 1); self.pca.set_duty(self.ch.pwm, -d)
            if verbose: print(f"[{self.name}] REV  inA=CH{a} inB=CH{b} duty={-d:.2f}")

    def stop(self):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        self._digital(a, 0); self._digital(b, 0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------- Keyboard raw ----------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r,_,_ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd, 1) if r else None

    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b'\x1b':  # escape sequences
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                    else: out.extend(filter(None, [b'\x1b', a, b]))
                else:
                    out.append(b'\x1b')
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------- Teleop pair-mode ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

class TeleopPair:
    def __init__(self, a):
        self.hz = float(a.hz)
        self.deadman = float(a.deadman)
        self.scale_low = float(a.scale_low)
        self.scale_high = float(a.scale_high)
        self.mag = float(a.mag)
        self.ramp = float(a.ramp)

        self.duration   = float(a.duration)
        self.idle_exit  = float(a.idle_exit)
        self.verbose    = bool(a.verbose)
        self.quiet      = bool(a.quiet or (not self.verbose))

        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)

        # Reserve channels (e.g., servo outputs)
        if a.reserve:
            try:
                rs = [int(x) for x in str(a.reserve).split(',') if x.strip()!='']
                for ch in rs: self.pca.full_off(ch)
                if rs: print(f"[Reserve] FULL-OFF: {rs}")
            except Exception:
                pass

        self.left  = SideDrive(self.pca, Triplet(a.left_pwm,  a.left_in1,  a.left_in2),
                               invert=a.inv_left, in_active_low=a.in_active_low, swap_in12=a.swap_in12_left, name="LEFT")
        self.right = SideDrive(self.pca, Triplet(a.right_pwm, a.right_in1, a.right_in2),
                               invert=a.inv_right, in_active_low=a.in_active_low, swap_in12=a.swap_in12_right, name="RIGHT")

        if self.verbose:
            print("\n[WIRING] side   PWM  IN1  IN2   invert swap in_active_low")
            print(f"[WIRING] LEFT   {a.left_pwm:<4} {a.left_in1:<4} {a.left_in2:<4} {bool(a.inv_left):<6} {bool(a.swap_in12_left):<4} {bool(a.in_active_low)}")
            print(f"[WIRING] RIGHT  {a.right_pwm:<4} {a.right_in1:<4} {a.right_in2:<4} {bool(a.inv_right):<6} {bool(a.swap_in12_right):<4} {bool(a.in_active_low)}\n")

        self.kb = Keyboard()
        self._targetL = 0.0
        self._targetR = 0.0
        self._curL = 0.0
        self._curR = 0.0
        self.last_in = time.monotonic()
        self.start_time = self.last_in

        signal.signal(signal.SIGINT,  self._sig_exit)
        signal.signal(signal.SIGTERM, self._sig_exit)

        print("[Teleop] READY — Pair mode (W/S forward/back, Q/E yaw, Space stop, 1..4 pulses, Esc quits)")

    def _sig_exit(self, *_):
        print("\n[Teleop] Signal → safe stop & exit")
        self.close(); os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        try: self.left.stop()
        except Exception: pass
        try: self.right.stop()
        except Exception: pass
        try: self.pca.close()
        except Exception: pass

    def _set_targets(self, L: float, R: float, note=""):
        self._targetL = clamp(L, -1.0, 1.0)
        self._targetR = clamp(R, -1.0, 1.0)
        if not self.quiet and note:
            print(f"[CMD] {note} -> L={self._targetL:+.2f} R={self._targetR:+.2f}")

    def _pulse(self, which: str, duty: float, sec=0.30):
        if which == "L":
            self.left.drive(duty, verbose=self.verbose)
            time.sleep(sec)
            self.left.stop()
        elif which == "R":
            self.right.drive(duty, verbose=self.verbose)
            time.sleep(sec)
            self.right.stop()

    def _handle_key(self, ch):
        # quit
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True

        # stop
        if ch in (b' ', b'0'):
            self._set_targets(0.0, 0.0, "STOP"); return False

        # print duties
        if ch in (b'm', b'M'):
            print(f"[duty] L {self._curL:+.2f}  R {self._curR:+.2f}")
            return False

        # speed scaling via shift/ctrl
        slow = (isinstance(ch, bytes) and len(ch)==1 and 1 <= ch[0] <= 26)   # Ctrl
        fast = (isinstance(ch, bytes) and ch.isalpha() and ch.isupper())     # Shift
        scale = self.scale_low if slow else (self.scale_high if fast else 1.0)
        mag = clamp(self.mag * scale, 0.0, 1.0)

        # pulses for probing
        if ch == b'1': self._pulse("L", +0.20); return False
        if ch == b'2': self._pulse("L", -0.20); return False
        if ch == b'3': self._pulse("R", +0.20); return False
        if ch == b'4': self._pulse("R", -0.20); return False

        # arrows / WASD + Q/E
        if ch in (b'w', b'W', b'UP'):
            self._set_targets(+mag, +mag, "FWD")
        elif ch in (b's', b'S', b'DOWN'):
            self._set_targets(-mag, -mag, "REV")
        elif ch in (b'q', b'Q'):
            self._set_targets(-mag, +mag, "YAW CCW")
        elif ch in (b'e', b'E'):
            self._set_targets(+mag, -mag, "YAW CW")
        elif ch in (b'a', b'A', b'LEFT'):
            # optional: treat A as gentle CCW
            self._set_targets(-0.5*mag, +0.5*mag, "YAW CCW (A)")
        elif ch in (b'd', b'D', b'RIGHT'):
            # optional: treat D as gentle CW
            self._set_targets(+0.5*mag, -0.5*mag, "YAW CW (D)")

        return False

    def loop(self):
        period = 1.0 / self.hz
        try:
            while True:
                t0 = time.monotonic()

                # time-based exits
                if self.duration > 0 and (t0 - self.start_time) >= self.duration:
                    print("[Teleop] Duration reached → exit"); return
                if self.idle_exit > 0 and (t0 - self.last_in) >= self.idle_exit:
                    print("[Teleop] Idle timeout → exit"); return

                # read all pending keys
                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch): return
                    self.last_in = t0

                # deadman: stop motors if no key for a while (but keep running)
                if not got and (t0 - self.last_in) > self.deadman:
                    self._targetL = 0.0
                    self._targetR = 0.0

                # ramp towards targets for smoothness
                alpha = clamp(self.ramp, 0.0, 1.0)
                self._curL = (1.0 - alpha)*self._curL + alpha*self._targetL
                self._curR = (1.0 - alpha)*self._curR + alpha*self._targetR

                self.left.drive(self._curL,  verbose=(self.verbose and not self.quiet))
                self.right.drive(self._curR, verbose=(self.verbose and not self.quiet))

                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# ---------------- CLI ----------------
def build_ap():
    p = argparse.ArgumentParser(description="Robot Savo — Teleop Pair-Mode (PCA9685)")
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--set-freq', action='store_true', help="Apply prescale (default: don't change)")
    p.add_argument('--reserve', type=str, default='12,13,14,15')

    # pair channels
    p.add_argument('--left-pwm',  type=int, required=True)
    p.add_argument('--left-in1',  type=int, required=True)
    p.add_argument('--left-in2',  type=int, required=True)
    p.add_argument('--right-pwm', type=int, required=True)
    p.add_argument('--right-in1', type=int, required=True)
    p.add_argument('--right-in2', type=int, required=True)

    # polarity/logic helpers
    p.add_argument('--inv-left',  action='store_true')
    p.add_argument('--inv-right', action='store_true')
    p.add_argument('--in-active-low', action='store_true', help='IN pins are active-low')
    p.add_argument('--swap-in12-left',  action='store_true', help='Swap IN1/IN2 on LEFT side')
    p.add_argument('--swap-in12-right', action='store_true', help='Swap IN1/IN2 on RIGHT side')

    # motion & loop
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.25)
    p.add_argument('--mag', type=float, default=0.55, help='Base magnitude for commands (0..1)')
    p.add_argument('--ramp', type=float, default=0.7, help='Smoothing (0..1) per tick')
    p.add_argument('--scale-low', type=float, default=0.25, help='Ctrl scaling')
    p.add_argument('--scale-high', type=float, default=1.5, help='Shift scaling')

    # exits / verbosity
    p.add_argument('--duration', type=float, default=0.0, help='Quit after N seconds (0=never)')
    p.add_argument('--idle-exit', type=float, default=0.0, help='Quit after N seconds without keys (0=never)')
    p.add_argument('--quiet', action='store_true', help='Reduce prints even when --verbose')
    p.add_argument('--verbose', action='store_true')
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    t = TeleopPair(args)
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
