#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (FNK0043 table-accurate, DC motors only)
-------------------------------------------------------------------
Implements EXACT Freenove table for mecanum:
Forward=[+,+,+,+], Back=[-,-,-,-],
TurnL=[-,-,+,+], TurnR=[+,+,-,-],
Left=[-,+,+,-], Right=[+,-,-,+].

NO servo code. We do NOT touch channels except the 12 you give for motors.
Default: we DO NOT change PCA9685 frequency (use --set-freq if you really want).
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------------- PCA9685 minimal ----------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV = 0x80, 0x10, 0x20, 0x04

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=False):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV)        # push-pull
        self._w8(MODE1, AI)            # auto-inc
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI))

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _w8(self, r, v): self.bus.write_byte_data(self.addr, r & 0xFF, v & 0xFF)
    def _r8(self, r):    return self.bus.read_byte_data(self.addr, r & 0xFF)

    def set_pwm_freq(self, f):
        f = float(max(40.0, min(1500.0, f)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI))

    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8)&0x0F)
        self._w8(base+2, off & 0xFF); self._w8(base+3, (off>>8)&0x0F)

    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0); self._w8(base+1, 0)
        self._w8(base+2, 0); self._w8(base+3, 0x10)

    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0); self._w8(base+1, 0x10)
        self._w8(base+2, 0); self._w8(base+3, 0)

    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

# ---------------- H-Bridge ----------------
@dataclass
class MotorCH: pwm: int; in1: int; in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.inv = bool(invert); self.al = bool(in_active_low); self.swap = bool(swap_in12)
        self.stop()

    def _dir(self, a_on, b_on):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        if self.al: a_on ^= 1; b_on ^= 1
        (self.pca.full_on if a_on else self.pca.full_off)(a)
        (self.pca.full_on if b_on else self.pca.full_off)(b)

    def drive(self, s):
        d = max(-1.0, min(1.0, float(s)))
        if self.inv: d = -d
        if abs(d) < 1e-3: self.stop(); return
        if d > 0: self._dir(1,0); self.pca.set_duty(self.ch.pwm, d)
        else:     self._dir(0,1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        self._dir(0,0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------- Keyboard ----------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r, _, _ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd, 1) if r else None

    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b"\x1b":
                a = self._read1(0.03)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b"\x1b");  out.append(a) if a else None
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------- Teleop core ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Limits: vx: float; vy: float; wz: float; ax: float; ay: float; az: float

class Teleop:
    def __init__(self, a):
        self.lim = Limits(a.max_vx, a.max_vy, a.max_omega, a.accel_x, a.accel_y, a.accel_z)
        self.hz = float(a.hz); self.deadman = float(a.deadman)
        self.scale_low, self.scale_high = float(a.scale_low), float(a.scale_high)
        self.scale = 1.0

        # per-wheel sign multipliers (Data1..Data4 = M1..M4)
        self.ws = [int(x) for x in a.wheel_signs.split(',')]
        if len(self.ws) != 4 or any(s not in (-1,1) for s in self.ws):
            raise ValueError("--wheel-signs must be 4 values in {-1,1}")

        # PCA (no freq change by default)
        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)

        # Motors
        self.m = [
            HBridge(self.pca, MotorCH(a.fl_pwm, a.fl_in1, a.fl_in2), invert=a.inv_fl,
                    in_active_low=a.in_active_low, swap_in12=a.swap_in12_fl),
            HBridge(self.pca, MotorCH(a.fr_pwm, a.fr_in1, a.fr_in2), invert=a.inv_fr,
                    in_active_low=a.in_active_low, swap_in12=a.swap_in12_fr),
            HBridge(self.pca, MotorCH(a.rl_pwm, a.rl_in1, a.rl_in2), invert=a.inv_rl,
                    in_active_low=a.in_active_low, swap_in12=a.swap_in12_rl),
            HBridge(self.pca, MotorCH(a.rr_pwm, a.rr_in1, a.rr_in2), invert=a.inv_rr,
                    in_active_low=a.in_active_low, swap_in12=a.swap_in12_rr),
        ]

        self.kb = Keyboard()
        self.last_in = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        print("[Teleop] READY (Freonove table). W/S forward/back, A/D left/right, Q/E turn. Esc=quit")

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for x in self.m:
            try: x.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    # pattern vectors (per your table)
    V_F = [+1, +1, +1, +1]
    V_B = [-1, -1, -1, -1]
    V_L = [-1, +1, +1, -1]
    V_R = [+1, -1, -1, +1]
    V_TL= [-1, -1, +1, +1]
    V_TR= [+1, +1, -1, -1]

    def _apply(self, vx, vy, wz):
        # normalize magnitudes 0..1 and scale
        sx = abs(vx) / max(self.lim.vx, 1e-6)
        sy = abs(vy) / max(self.lim.vy, 1e-6)
        so = abs(wz) / max(self.lim.wz, 1e-6)
        k = self.scale

        acc = [0.0, 0.0, 0.0, 0.0]
        if vx > 1e-6:
            for i,v in enumerate(self.V_F): acc[i] += v * sx * k
        elif vx < -1e-6:
            for i,v in enumerate(self.V_B): acc[i] += v * sx * k

        if vy > 1e-6:
            for i,v in enumerate(self.V_L): acc[i] += v * sy * k
        elif vy < -1e-6:
            for i,v in enumerate(self.V_R): acc[i] += v * sy * k

        if wz > 1e-6:
            for i,v in enumerate(self.V_TL): acc[i] += v * so * k
        elif wz < -1e-6:
            for i,v in enumerate(self.V_TR): acc[i] += v * so * k

        # apply per-wheel signs and clamp
        for i in range(4):
            acc[i] = self.ws[i] * clamp(acc[i], -1.0, 1.0)
            self.m[i].drive(acc[i])
        self._duties = acc

    def _handle_key(self, ch):
        if ch in (b"\x1b", b"\x03"): print("[Teleop] Quit"); return True
        if ch == b' ': self.t_vx = self.t_vy = self.t_wz = 0.0; print("[Cmd] Stop"); return False
        if ch in (b'm', b'M'): d=getattr(self,'_duties',[0,0,0,0]); print(f"[duties] M1 {d[0]:+.2f} M2 {d[1]:+.2f} M3 {d[2]:+.2f} M4 {d[3]:+.2f}"); return False

        # speed scaling: Ctrl=slow, Shift=fast
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
            self.scale = 0.20 if slow else (1.5 if fast else 1.0)

        if ch in (b'w', b'W', b'UP'):      self.t_vx = +self.lim.vx
        elif ch in (b's', b'S', b'DOWN'):  self.t_vx = -self.lim.vx
        elif ch in (b'a', b'A', b'LEFT'):  self.t_vy = +self.lim.vy
        elif ch in (b'd', b'D', b'RIGHT'): self.t_vy = -self.lim.vy
        elif ch in (b'q', b'Q'):           self.t_wz = +self.lim.wz
        elif ch in (b'e', b'E'):           self.t_wz = -self.lim.wz
        elif ch in (b'x', b'X'):           self.t_vx = self.t_vy = 0.0
        elif ch in (b'z', b'Z'):           self.t_wz = 0.0
        return False

    def loop(self):
        period = 1.0 / self.hz
        try:
            while True:
                t0 = time.monotonic()
                key = False
                for ch in self.kb.read():
                    key = True
                    if self._handle_key(ch): return
                    self.last_in = t0
                if not key and (t0 - self.last_in) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # accel limiting
                dt = max(1e-3, t0 - getattr(self,'_last',t0)); self._last = t0
                ax, ay, az = self.lim.ax*dt, self.lim.ay*dt, self.lim.az*dt
                def filt(c,t,amax): 
                    return min(c + amax, t) if t>c else max(c - amax, t)
                self.c_vx = filt(self.c_vx, self.t_vx, ax)
                self.c_vy = filt(self.c_vy, self.t_vy, ay)
                self.c_wz = filt(self.c_wz, self.t_wz, az)

                self._apply(self.c_vx, self.c_vy, self.c_wz)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# ---------------- CLI ----------------
def build_ap():
    p = argparse.ArgumentParser(description="FNK0043 DC-motor teleop (table-accurate)")
    # limits
    p.add_argument('--max-vx', type=float, default=0.12)
    p.add_argument('--max-vy', type=float, default=0.12)
    p.add_argument('--max-omega', type=float, default=0.6)
    p.add_argument('--accel-x', type=float, default=0.9)
    p.add_argument('--accel-y', type=float, default=0.9)
    p.add_argument('--accel-z', type=float, default=1.8)
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.25)
    p.add_argument('--scale-low', type=float, default=0.20)
    p.add_argument('--scale-high', type=float, default=1.5)
    # PCA / I2C
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', action='store_true', help="Actually set PCA9685 frequency (default: don't touch)")
    # channels (Data1..Data4 = M1..M4)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # polarity helpers
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # wheel forward signs (default all +1; change if any wheel wiring is reversed)
    p.add_argument('--wheel-signs', type=str, default='+1,+1,+1,+1', help='CSV of -1/1 for (M1,M2,M3,M4)')
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    t = Teleop(args)
    try: t.loop()
    except KeyboardInterrupt: pass
    finally: t.close()

if __name__ == '__main__':
    main()
