#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (FNK0043 table-accurate, DC motors only)
-------------------------------------------------------------------
Implements EXACT Freenove mecanum table using 4 DC motors via PCA9685:

  Forward     = [+,+,+,+]
  Backward    = [-,-,-,-]
  Turn left   = [-,-,+,+]
  Turn right  = [+,+,-,-]
  Move left   = [-,+,+,-]
  Move right  = [+,-,-,+]

Keys
----
W / ↑ : Forward
S / ↓ : Backward
A / ← : Move left
D / → : Move right
Q     : Turn left  (CCW)
E     : Turn right (CW)
SPACE : Stop now
M     : Print per-wheel duties
1..4  : Pulse M1..M4 forward
5..8  : Pulse M1..M4 reverse
Esc or Ctrl+C : Quit (safe stop)

Notes
-----
• NO servo code. This script NEVER writes to any servo channels and DOES NOT
  change PCA9685 frequency unless you pass --set-freq.
• If any wheel is electrically inverted, fix with --wheel-signs (CSV of -1/1).
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------- PCA9685 minimal ----------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV = 0x80, 0x10, 0x20, 0x04

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=False):
        self.addr = int(addr)
        self.bus  = SMBus(int(bus))
        # push-pull outputs; auto-increment
        self._w8(MODE2, OUTDRV)
        self._w8(MODE1, AI)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        # ensure restart + AI
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI))

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
        self._w8(MODE1, (old | RESTART | AI))

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

# ---------------- H-Bridge primitive ----------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.stop()

    def _dir(self, a_on: int, b_on: int):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        if self.in_active_low:
            a_on ^= 1; b_on ^= 1
        (self.pca.full_on if a_on else self.pca.full_off)(a)
        (self.pca.full_on if b_on else self.pca.full_off)(b)

    def drive(self, signed_duty: float):
        d = max(-1.0, min(1.0, float(signed_duty)))
        if self.invert:
            d = -d
        if abs(d) < 1e-3:
            self.stop(); return
        if d > 0:
            self._dir(1, 0); self.pca.set_duty(self.ch.pwm, d)
        else:
            self._dir(0, 1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        self._dir(0, 0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------- Keyboard helper ----------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return os.read(self.fd, 1) if r else None

    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b"\x1b":  # ESC / arrows
                a = self._read1(0.03)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows:
                        out.append(arrows[b])
                else:
                    out.append(b"\x1b")
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------- Teleop core ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

class Teleop:
    # pattern vectors per Freenove table (M1,M2,M3,M4)
    V_F  = (+1, +1, +1, +1)
    V_B  = (-1, -1, -1, -1)
    V_L  = (-1, +1, +1, -1)
    V_R  = (+1, -1, -1, +1)
    V_TL = (-1, -1, +1, +1)  # turn left (CCW): left side back, right side fwd
    V_TR = (+1, +1, -1, -1)  # turn right (CW)

    def __init__(self, a):
        self.lim = Limits(a.max_vx, a.max_vy, a.max_omega, a.accel_x, a.accel_y, a.accel_z)
        self.hz = float(a.hz)
        self.deadman = float(a.deadman)
        self.scale_low, self.scale_high = float(a.scale_low), float(a.scale_high)
        self.scale = 1.0

        self.ws = [int(x) for x in a.wheel_signs.split(',')]
        if len(self.ws) != 4 or any(s not in (-1, 1) for s in self.ws):
            raise ValueError("--wheel-signs must be CSV of four values in {-1,1}")

        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)

        self.M = [
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
        self._duties = [0.0, 0.0, 0.0, 0.0]
        print("[Teleop] READY. W/S forward/back, A/D left/right, Q/E turn. Esc=quit")

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in self.M:
            try: m.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    def _apply_pattern(self, vec, mag):
        # apply vector (tuple of +/-1) with magnitude 0..1
        d = [self.ws[i] * clamp(vec[i] * mag, -1.0, 1.0) for i in range(4)]
        for i, m in enumerate(self.M):
            m.drive(d[i])
        self._duties = d

    def _mix_and_apply(self, vx, vy, wz):
        # magnitudes 0..1
        sx = abs(vx) / max(self.lim.vx, 1e-6)
        sy = abs(vy) / max(self.lim.vy, 1e-6)
        so = abs(wz) / max(self.lim.wz, 1e-6)
        k  = self.scale

        # start at zero, then add each pattern contribution
        acc = [0.0, 0.0, 0.0, 0.0]
        def add(vec, s):
            for i in range(4):
                acc[i] += vec[i] * s

        if vx > 1e-6:   add(self.V_F,  +sx * k)
        if vx < -1e-6:  add(self.V_B,  +sx * k)
        if vy > 1e-6:   add(self.V_L,  +sy * k)
        if vy < -1e-6:  add(self.V_R,  +sy * k)
        if wz > 1e-6:   add(self.V_TL, +so * k)
        if wz < -1e-6:  add(self.V_TR, +so * k)

        # apply wheel signs and clamp
        for i in range(4):
            acc[i] = self.ws[i] * clamp(acc[i], -1.0, 1.0)
            self.M[i].drive(acc[i])
        self._duties = acc

    def _print_status(self, label=None):
        d = self._duties
        prefix = f"[{label}] " if label else ""
        print(f"{prefix}M1 {d[0]:+0.2f}  M2 {d[1]:+0.2f}  M3 {d[2]:+0.2f}  M4 {d[3]:+0.2f}")

    def _handle_key(self, ch):
        # quit
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True
        # stop
        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0
            self._apply_pattern((0,0,0,0), 0.0)
            print("[Cmd] Stop")
            return False
        if ch in (b'm', b'M'):
            self._print_status("duties"); return False

        # speed scaling: Ctrl=slow, Shift=fast
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
            self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)

        # directional intents (with prints)
        if ch in (b'w', b'W', b'UP'):
            self.t_vx = +self.lim.vx; print("[Move] Forward")
        elif ch in (b's', b'S', b'DOWN'):
            self.t_vx = -self.lim.vx; print("[Move] Backward")
        elif ch in (b'a', b'A', b'LEFT'):
            self.t_vy = +self.lim.vy; print("[Move] Left (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):
            self.t_vy = -self.lim.vy; print("[Move] Right (strafe)")
        elif ch in (b'q', b'Q'):
            self.t_wz = +self.lim.wz; print("[Turn] Left (CCW)")
        elif ch in (b'e', b'E'):
            self.t_wz = -self.lim.wz; print("[Turn] Right (CW)")
        elif ch in (b'x', b'X'):
            self.t_vx = 0.0; self.t_vy = 0.0; print("[Cmd] Zero VX/VY")
        elif ch in (b'z', b'Z'):
            self.t_wz = 0.0; print("[Cmd] Zero WZ")

        # diagnostics: single-wheel pulses
        elif ch in (b'1',): self._pulse([1,0,0,0], +0.5)
        elif ch in (b'2',): self._pulse([0,1,0,0], +0.5)
        elif ch in (b'3',): self._pulse([0,0,1,0], +0.5)
        elif ch in (b'4',): self._pulse([0,0,0,1], +0.5)
        elif ch in (b'5',): self._pulse([1,0,0,0], -0.5)
        elif ch in (b'6',): self._pulse([0,1,0,0], -0.5)
        elif ch in (b'7',): self._pulse([0,0,1,0], -0.5)
        elif ch in (b'8',): self._pulse([0,0,0,1], -0.5)

        return False

    def _pulse(self, mask, duty, t=0.5):
        for i, m in enumerate(self.M):
            m.drive(self.ws[i] * (duty if mask[i] else 0.0))
        time.sleep(t)
        for m in self.M: m.stop()

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

                if not got and (t0 - self.last_in) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # accel limiting
                dt = max(1e-3, t0 - getattr(self, "_tlast", t0))
                self._tlast = t0
                def filt(c, t, amax): 
                    return min(c + amax, t) if t > c else max(c - amax, t)
                self.c_vx = filt(self.c_vx, self.t_vx, self.lim.ax*dt)
                self.c_vy = filt(self.c_vy, self.t_vy, self.lim.ay*dt)
                self.c_wz = filt(self.c_wz, self.t_wz, self.lim.az*dt)

                self._mix_and_apply(self.c_vx, self.c_vy, self.c_wz)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# ---------------- CLI ----------------
def build_ap():
    p = argparse.ArgumentParser(description="FNK0043 DC-motor teleop (table-accurate)")
    # motion limits
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
    # channels (M1..M4 = FL,FR,RL,RR)
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
    # per-wheel sign fix (if wiring flips a motor)
    p.add_argument('--wheel-signs', type=str, default='+1,+1,+1,+1', help='CSV of -1/1 for (M1,M2,M3,M4)')
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    t = Teleop(args)
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
