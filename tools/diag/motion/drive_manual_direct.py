#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for DC Motors via PCA9685 + H-Bridge
---------------------------------------------------------------------------
MOTOR-ONLY (no servo control). Safe on FNK0043 (shared PCA9685).

Keys
  W / ↑       : Forward
  S / ↓       : Backward
  A / ←       : Strafe Left
  D / →       : Strafe Right
  Q / E       : Turn CCW / CW
  SPACE       : Stop immediately
  M           : Print wheel duties
  1..4 / 5..8 : Pulse FL/FR/RL/RR forward / reverse
  Shift       : Fast scale
  Ctrl        : Slow scale
  Esc / Ctrl+C: Quit

Notes
- Default reserves channels 12,13,14,15 (servo area) → FULL-OFF.
- Pass --set-freq if you actually want to change the PCA9685 prescale.
- W is “forward” on FNK0043 by adding --flip-vx (you can keep it default True later).
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------- I2C ----------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------- PCA9685 low-level ----------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus: int, addr: int, freq_hz: float, set_freq: bool):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        # push-pull outputs; auto-increment; clear ALLCALL
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
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f) - 1.0))))
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
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

# ---------- H-Bridge ----------
@dataclass
class MotorCH: pwm: int; in1: int; in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.stop()

    def _dig(self, ch: int, level: int):
        if self.in_active_low: level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(ch)

    def drive(self, signed: float):
        d = max(-1.0, min(1.0, float(signed)))
        if self.invert: d = -d
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        if abs(d) < 1e-3:
            self._dig(a,0); self._dig(b,0); self.pca.set_duty(self.ch.pwm, 0.0); return
        if d > 0:
            self._dig(a,1); self._dig(b,0); self.pca.set_duty(self.ch.pwm, d)
        else:
            self._dig(a,0); self._dig(b,1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        self._dig(a,0); self._dig(b,0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------- Keyboard ----------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, timeout=0.0):
        r,_,_ = select.select([sys.stdin], [], [], timeout)
        return os.read(self.fd,1) if r else None

    def read(self):
        out=[]; ch=self._read1(0.0)
        while ch is not None:
            if ch == b"\x1b":
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows={b'A':b'UP', b'B':b'DOWN', b'C':b'RIGHT', b'D':b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                    else: 
                        out.append(b"\x1b"); 
                        if a: out.append(a)
                        if b: out.append(b)
                else:
                    out.append(b"\x1b"); 
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------- Helpers ----------
def clamp(x, lo, hi): return max(lo, min(hi, x))
def step(curr, target, max_step):
    return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

@dataclass
class Limits: vx: float; vy: float; wz: float; ax: float; ay: float; az: float
@dataclass
class MecanumGeom: r: float; L: float  # r=wheel radius (m), L=(Lx+Ly)/2 (m)

# ---------- Teleop ----------
class Teleop:
    # Freenove pattern vectors (M1,M2,M3,M4) = (FL,FR,RL,RR)
    V_F  = (+1, +1, +1, +1)   # forward
    V_B  = (-1, -1, -1, -1)   # backward
    V_L  = (-1, +1, +1, -1)   # strafe left
    V_R  = (+1, -1, -1, +1)   # strafe right
    V_TL = (-1, -1, +1, +1)   # turn CCW (left back, right fwd)
    V_TR = (+1, +1, -1, -1)   # turn CW

    def __init__(self, a):
        self.lim = Limits(a.max_vx, a.max_vy, a.max_omega, a.accel_x, a.accel_y, a.accel_z)
        self.geom = MecanumGeom(a.wheel_radius, a.L)
        self.hz = float(a.hz); self.deadman=float(a.deadman)
        self.scale_low, self.scale_high = float(a.scale_low), float(a.scale_high)
        self.scale = 1.0

        # axis flips
        self.flip_vx = bool(a.flip_vx)
        self.flip_vy = bool(a.flip_vy)
        self.flip_om = bool(a.flip_omega)

        # PCA + reserve OFF (e.g., 12,13,14,15 for servos)
        self.pca = PCA9685(a.i2c_bus, a.pca_addr, a.pwm_freq, a.set_freq)
        reserved = [int(x) for x in str(a.reserve).split(',') if x.strip()!='']
        for ch in reserved: self.pca.full_off(ch)
        if reserved: print(f"[Teleop] Reserved channels FULL-OFF: {reserved}")

        # motors
        self.FL = HBridge(self.pca, MotorCH(a.fl_pwm, a.fl_in1, a.fl_in2), a.inv_fl, a.in_active_low, a.swap_in12_fl)
        self.FR = HBridge(self.pca, MotorCH(a.fr_pwm, a.fr_in1, a.fr_in2), a.inv_fr, a.in_active_low, a.swap_in12_fr)
        self.RL = HBridge(self.pca, MotorCH(a.rl_pwm, a.rl_in1, a.rl_in2), a.inv_rl, a.in_active_low, a.swap_in12_rl)
        self.RR = HBridge(self.pca, MotorCH(a.rr_pwm, a.rr_in1, a.rr_in2), a.inv_rr, a.in_active_low, a.swap_in12_rr)

        self.kb = Keyboard()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()
        self.last_key = time.monotonic()
        self._duties = [0,0,0,0]

        print("[Teleop] READY. W/S forward/back, A/D strafe, Q/E turn. SPACE stop, M print, Esc quit.")

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in (self.FL, self.FR, self.RL, self.RR):
            try: m.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    def _pattern(self, vec, mag):
        d = [clamp(vec[i]*mag, -1.0, 1.0) for i in range(4)]
        self.FL.drive(d[0]); self.FR.drive(d[1]); self.RL.drive(d[2]); self.RR.drive(d[3])
        self._duties = d

    def _apply_geom(self, vx, vy, wz):
        # flips (so W is forward if flip_vx True)
        if self.flip_vx: vx = -vx
        if self.flip_vy: vy = -vy
        if self.flip_om: wz = -wz

        r = max(1e-6, self.geom.r); L = self.geom.L
        w_fl = ( vx - vy - L*wz ) / r
        w_fr = ( vx + vy + L*wz ) / r
        w_rl = ( vx + vy - L*wz ) / r
        w_rr = ( vx - vy + L*wz ) / r
        ws   = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))

        mag  = max(abs(vx)/max(self.lim.vx,1e-6),
                   abs(vy)/max(self.lim.vy,1e-6),
                   abs(wz)/max(self.lim.wz,1e-6))
        d = [clamp((w/max_w)*mag, -1.0, 1.0) for w in ws]
        self.FL.drive(d[0]); self.FR.drive(d[1]); self.RL.drive(d[2]); self.RR.drive(d[3])
        self._duties = d

    def _print_status(self):
        d = self._duties
        print(f"[duties] FL {d[0]:+0.2f}  FR {d[1]:+0.2f}  RL {d[2]:+0.2f}  RR {d[3]:+0.2f}   "
              f"vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}")

    def _pulse(self, mask, duty=0.12, t=0.30):
        M = (self.FL, self.FR, self.RL, self.RR)
        for i,m in enumerate(M): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in M: m.stop()

    def _handle_key(self, k):
        # quit / stop
        if k in (b"\x03", b"\x1b"): print("[Teleop] Quit"); return True
        if k == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0; self.FL.stop(); self.FR.stop(); self.RL.stop(); self.RR.stop()
            print("[Cmd] STOP"); return False
        if k in (b'm', b'M'): self._print_status(); return False

        # scale
        slow = isinstance(k, bytes) and len(k)==1 and (1 <= k[0] <= 26)   # Ctrl
        fast = isinstance(k, bytes) and k.isalpha() and k.isupper()
        self.scale = (0.20 if slow else (1.5 if fast else 1.0))

        # intents
        if k in (b'w', b'W', b'UP'):    self.t_vx = +self.lim.vx; print("[Move] Forward")
        elif k in (b's', b'S', b'DOWN'):self.t_vx = -self.lim.vx; print("[Move] Backward")
        elif k in (b'a', b'A', b'LEFT'):self.t_vy = +self.lim.vy; print("[Move] Left (strafe)")
        elif k in (b'd', b'D', b'RIGHT'):self.t_vy = -self.lim.vy; print("[Move] Right (strafe)")
        elif k in (b'q', b'Q'):         self.t_wz = +self.lim.wz; print("[Turn] CCW")
        elif k in (b'e', b'E'):         self.t_wz = -self.lim.wz; print("[Turn] CW")
        elif k in (b'x', b'X'):         self.t_vx = 0.0; self.t_vy = 0.0; print("[Cmd] Zero XY")
        elif k in (b'z', b'Z'):         self.t_wz = 0.0; print("[Cmd] Zero Yaw")
        # diagnostics pulses
        elif k == b'1': self._pulse([1,0,0,0]); 
        elif k == b'2': self._pulse([0,1,0,0]); 
        elif k == b'3': self._pulse([0,0,1,0]); 
        elif k == b'4': self._pulse([0,0,0,1]); 
        elif k == b'5': self._pulse([1,0,0,0], duty=-0.12); 
        elif k == b'6': self._pulse([0,1,0,0], duty=-0.12); 
        elif k == b'7': self._pulse([0,0,1,0], duty=-0.12); 
        elif k == b'8': self._pulse([0,0,0,1], duty=-0.12);
        return False

    def loop(self):
        period = 1.0 / max(1e-3, self.hz)
        try:
            while True:
                t0 = time.monotonic()
                got=False
                for k in self.kb.read():
                    got=True
                    if self._handle_key(k): return
                    self.last_key = t0

                # deadman
                if not got and (t0 - self.last_key) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # target with scaling + clamps
                vx_t = clamp(self.t_vx * self.scale, -self.lim.vx, self.lim.vx)
                vy_t = clamp(self.t_vy * self.scale, -self.lim.vy, self.lim.vy)
                wz_t = clamp(self.t_wz * self.scale, -self.lim.wz, self.lim.wz)

                # accel limiting
                dt = max(1e-3, t0 - getattr(self, "_last_upd", t0)); self._last_upd=t0
                self.c_vx = step(self.c_vx, vx_t, self.lim.ax*dt)
                self.c_vy = step(self.c_vy, vy_t, self.lim.ay*dt)
                self.c_wz = step(self.c_wz, wz_t, self.lim.az*dt)

                # Apply geometry mixer (mecanum IK)
                self._apply_geom(self.c_vx, self.c_vy, self.c_wz)

                # maintain rate
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, period - elapsed))
        finally:
            self.close()

# ---------- CLI ----------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (PCA9685 + H-bridge, motor-only)')
    # Geometry (now recognized)
    p.add_argument('--wheel-radius', type=float, default=0.0325, help='Wheel radius r (m) (65 mm wheel → 0.0325)')
    p.add_argument('--L', type=float, default=0.115, help='(Lx+Ly)/2 in meters (~0.115 for your base)')
    # Limits
    p.add_argument('--max-vx', type=float, default=0.12)
    p.add_argument('--max-vy', type=float, default=0.00)  # start 0.00 for FNK0043 paired board
    p.add_argument('--max-omega', type=float, default=0.6)
    p.add_argument('--accel-x', type=float, default=0.9)
    p.add_argument('--accel-y', type=float, default=0.9)
    p.add_argument('--accel-z', type=float, default=1.8)
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.25)
    p.add_argument('--scale-low', type=float, default=0.20)
    p.add_argument('--scale-high', type=float, default=1.5)
    # I2C / PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', action='store_true', help='Actually set PCA prescale (otherwise leave as-is)')
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Comma list of PCA channels to FULL-OFF (servo pins)')
    # Channels (M1..M4 = FL,FR,RL,RR)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Polarity / logic fixes
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Axis flips (make W=forward on your kit)
    p.add_argument('--flip-vx', action='store_true', default=True)
    p.add_argument('--flip-vy', action='store_true', default=False)
    p.add_argument('--flip-omega', action='store_true', default=False)
    return p

def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = Teleop(args)
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
