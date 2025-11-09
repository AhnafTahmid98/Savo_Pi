#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Pair-Mode Teleop (LEFT/RIGHT triplets on PCA9685)
--------------------------------------------------------------
Designed for HATs that gang both left motors together and both right motors together.
You get: forward/back, spot-turns, and arcs. No strafing (not possible on ganged HATs).

Keys:
  W / S      : forward / backward
  A / D      : steer left / right (while moving or standstill)
  Q / E      : spot turn CCW / CW
  SPACE or 0 : stop
  M          : print left/right duties
  1..2       : pulse LEFT forward / reverse
  3..4       : pulse RIGHT forward / reverse
  Shift      : fast scale,  Ctrl : slow scale
  Esc / Ctrl+C : quit (safe stop)

Flags to adapt your wiring:
  --left-pwm --left-in1 --left-in2
  --right-pwm --right-in1 --right-in2
  --inv-left --inv-right
  --in-active-low  (if your driver expects LOW=ON)
"""

import argparse, os, select, sys, termios, time, tty, signal, math
from dataclasses import dataclass

# ----------- I2C ----------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ----------- PCA9685 minimal ----------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
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
        if off <= 0:   self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else:          self._raw(ch, 0, off)

# ----------- H-bridge side (triplet) ----------
@dataclass
class Triplet:
    pwm: int; in1: int; in2: int

class Side:
    def __init__(self, pca: PCA9685, tri: Triplet, invert=False, in_active_low=False, name="LEFT", verbose=False):
        self.pca, self.t = pca, tri
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.name = name
        self.verbose = verbose
        self.stop()

    def _digital(self, channel: int, level: int):
        lvl = (level ^ 1) if self.in_active_low else level
        if lvl: self.pca.full_on(channel)
        else:   self.pca.full_off(channel)
        if self.verbose:
            state = "HIGH" if lvl else "LOW"
            print(f"[{self.name}] CH{channel}: {state} (in_active_low={self.in_active_low})")

    def drive(self, signed: float):
        d = max(-1.0, min(1.0, float(signed)))
        if self.invert: d = -d
        if abs(d) < 1e-3:
            self._digital(self.t.in1, 0); self._digital(self.t.in2, 0); self.pca.set_duty(self.t.pwm, 0.0)
            if self.verbose: print(f"[{self.name}] STOP pwm=CH{self.t.pwm} duty=0.00 (invert={self.invert})")
            return
        if d > 0:
            self._digital(self.t.in1, 1); self._digital(self.t.in2, 0); self.pca.set_duty(self.t.pwm, d)
            if self.verbose: print(f"[{self.name}] FWD  inA=CH{self.t.in1} inB=CH{self.t.in2} duty={d:.2f} (invert={self.invert})")
        else:
            self._digital(self.t.in1, 0); self._digital(self.t.in2, 1); self.pca.set_duty(self.t.pwm, -d)
            if self.verbose: print(f"[{self.name}] REV  inA=CH{self.t.in1} inB=CH{self.t.in2} duty={-d:.2f} (invert={self.invert})")

    def stop(self):
        self._digital(self.t.in1, 0); self._digital(self.t.in2, 0); self.pca.set_duty(self.t.pwm, 0.0)

# ----------- Keyboard ----------
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
            if ch == b'\x1b':
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                    else: out.extend(filter(None, [b'\x1b', a, b]))
                else:
                    out.append(b'\x1b'); 
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ----------- Teleop (pair-mode) ----------
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Limits:
    vx: float; wz: float; ax: float; az: float

class TeleopPair:
    def __init__(self, a):
        self.hz = float(a.hz)
        self.deadman = float(a.deadman)
        self.scale_low, self.scale_high = float(a.scale_low), float(a.scale_high)
        self.scale = 1.0
        self.lim = Limits(a.max_vx, a.max_omega, a.accel_x, a.accel_z)
        self.verbose = a.verbose

        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=True)

        # reserve channels (e.g., 12..15)
        try:
            reserved = [int(x) for x in str(a.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Reserve] FULL-OFF: {reserved}")
        except Exception: pass

        self.left  = Side(self.pca, Triplet(a.left_pwm,  a.left_in1,  a.left_in2),  invert=a.inv_left,  in_active_low=a.in_active_low, name="LEFT",  verbose=self.verbose)
        self.right = Side(self.pca, Triplet(a.right_pwm, a.right_in1, a.right_in2), invert=a.inv_right, in_active_low=a.in_active_low, name="RIGHT", verbose=self.verbose)

        self.kb = Keyboard()
        self.last_in = time.monotonic()

        # targets & current with ramping
        self.tgtL = 0.0; self.tgtR = 0.0
        self.curL = 0.0; self.curR = 0.0

        signal.signal(signal.SIGINT, self._sig_exit)
        signal.signal(signal.SIGTERM, self._sig_exit)

        print("[Teleop] READY — pair-mode: W/S forward/back, A/D steer, Q/E spot turn, Space stop, M print, 1..4 pulses")

    def _sig_exit(self, *_):
        print("\n[Teleop] Signal → safe stop & exit")
        self.close(); os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        try: self.left.stop(); self.right.stop()
        except Exception: pass
        try: self.pca.close()
        except Exception: pass

    # ramp helper
    def _slew(self, cur, tgt, rate, dt):
        step = rate * dt
        if tgt > cur: return min(tgt, cur + step)
        if tgt < cur: return max(tgt, cur - step)
        return cur

    def _apply(self):
        self.left.drive(self.curL)
        self.right.drive(self.curR)
        if self.verbose:
            print(f"[duty] L {self.curL:+.2f}  R {self.curR:+.2f}")

    def _update(self, dt):
        # ramp limits
        ax = max(0.2, self.lim.ax)
        self.curL = self._slew(self.curL, self.tgtL, ax, dt)
        self.curR = self._slew(self.curR, self.tgtR, ax, dt)
        self._apply()

    def _handle_key(self, ch):
        # quit
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True
        # stop
        if ch in (b' ', b'0'):
            self.tgtL = 0.0; self.tgtR = 0.0; print("[Cmd] STOP"); return False
        # print duties
        if ch in (b'm', b'M'):
            print(f"[duty] L {self.curL:+.2f}  R {self.curR:+.2f}")
            return False

        # speed scaling
        slow = (isinstance(ch, bytes) and len(ch)==1 and 1 <= ch[0] <= 26)   # Ctrl
        fast = (isinstance(ch, bytes) and ch.isalpha() and ch.isupper())     # Shift
        self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)
        vmax = clamp(self.lim.vx * self.scale, 0.0, 1.0)
        wmax = clamp(self.lim.wz * self.scale, 0.0, 1.0)

        # pulses
        if ch == b'1': self._pulse(side="L", duty=+0.25); return False
        if ch == b'2': self._pulse(side="L", duty=-0.25); return False
        if ch == b'3': self._pulse(side="R", duty=+0.25); return False
        if ch == b'4': self._pulse(side="R", duty=-0.25); return False

        # maps
        if ch in (b'w', b'W', b'UP'):
            self.tgtL = +vmax; self.tgtR = +vmax; print("[Move] Forward")
        elif ch in (b's', b'S', b'DOWN'):
            self.tgtL = -vmax; self.tgtR = -vmax; print("[Move] Backward")
        elif ch in (b'a', b'A', b'LEFT'):
            # steer left: slow left, speed right (works at standstill too)
            base = max(abs(self.tgtL), abs(self.tgtR), 0.3*vmax)
            self.tgtL = clamp(+base - 0.5*wmax, -1.0, 1.0)
            self.tgtR = clamp(+base + 0.5*wmax, -1.0, 1.0)
            print("[Steer] Left")
        elif ch in (b'd', b'D', b'RIGHT'):
            base = max(abs(self.tgtL), abs(self.tgtR), 0.3*vmax)
            self.tgtL = clamp(+base + 0.5*wmax, -1.0, 1.0)
            self.tgtR = clamp(+base - 0.5*wmax, -1.0, 1.0)
            print("[Steer] Right")
        elif ch in (b'q', b'Q'):
            self.tgtL = -wmax; self.tgtR = +wmax; print("[Turn] CCW")
        elif ch in (b'e', b'E'):
            self.tgtL = +wmax; self.tgtR = -wmax; print("[Turn] CW")

        return False

    def _pulse(self, side="L", duty=+0.20, t=0.30):
        print(f"[Pulse] {side} {('FWD' if duty>0 else 'REV')}")
        if side == "L":
            self.left.drive(duty)
            time.sleep(t)
            self.left.stop()
        else:
            self.right.drive(duty)
            time.sleep(t)
            self.right.stop()

    def loop(self):
        period = 1.0 / self.hz
        try:
            while True:
                t0 = time.monotonic()
                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch): return
                    self.last_in = t0

                if not got and (time.monotonic() - self.last_in) > self.deadman:
                    if abs(self.curL) > 1e-3 or abs(self.curR) > 1e-3:
                        print("[Deadman] stop")
                    self.tgtL = 0.0; self.tgtR = 0.0

                dt = time.monotonic() - t0
                self._update(dt)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# ----------- CLI ----------
def build_ap():
    p = argparse.ArgumentParser(description="Robot Savo — Pair-Mode Teleop (PCA9685 LEFT/RIGHT)")
    # motion limits & rate
    p.add_argument('--max-vx', type=float, default=0.9)        # overall speed scale (0..1)
    p.add_argument('--max-omega', type=float, default=0.9)     # how strong spot turns steer
    p.add_argument('--accel-x', type=float, default=1.5)       # ramp (units: duty/s)
    p.add_argument('--accel-z', type=float, default=1.5)
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.6)
    p.add_argument('--scale-low', type=float, default=0.35)
    p.add_argument('--scale-high', type=float, default=1.4)
    # I2C / PCA9685
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Channels to FULL-OFF (servo pins)')
    # Triplets (LEFT and RIGHT)
    p.add_argument('--left-pwm',  type=int, required=True)
    p.add_argument('--left-in1',  type=int, required=True)
    p.add_argument('--left-in2',  type=int, required=True)
    p.add_argument('--right-pwm', type=int, required=True)
    p.add_argument('--right-in1', type=int, required=True)
    p.add_argument('--right-in2', type=int, required=True)
    # polarity / logic
    p.add_argument('--inv-left', action='store_true')
    p.add_argument('--inv-right', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    # misc
    p.add_argument('--verbose', action='store_true')
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    # init
    t = TeleopPair(args)
    # banner wiring
    print("\n[WIRING] Side  PWM  IN1  IN2   invert in_active_low")
    print(f"[WIRING] LEFT  {args.left_pwm:<4} {args.left_in1:<4} {args.left_in2:<4} {bool(args.inv_left):<6} {bool(args.in_active_low)}")
    print(f"[WIRING] RIGHT {args.right_pwm:<4} {args.right_in1:<4} {args.right_in2:<4} {bool(args.inv_right):<6} {bool(args.in_active_low)}\n")
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
