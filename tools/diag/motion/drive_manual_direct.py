#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) — Pair mode for Freenove FNK0043
---------------------------------------------------------------------
Motor-only (PCA9685 + H-bridge). Never touches servos.
Channels 8..15 are forced OFF at startup.

PAIR MODE (default):
  W / ↑  => ONLY M2 & M3 spin  (forward)
  S / ↓  => ONLY M1 & M4 spin  (backward)
  A / ←  => ONLY M1 & M2 spin  (strafe-left)
  D / →  => ONLY M3 & M4 spin  (strafe-right)
  Q / E  => turn (L side reverse, R side forward), sums with others

Keys
  W/S or ↑/↓  = forward/back using pairs
  A/D or ←/→  = strafe using pairs
  Q/E         = turn CCW/CW (side mix)
  1..4 / 5..8 = pulse FL/FR/RL/RR forward / reverse
  G           = gentle forward pulse (all wheels)
  M           = print current duties
  SPACE       = stop   |   Esc/Ctrl+C = quit
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, ALLCALL, OUTDRV = 0x80, 0x10, 0x20, 0x01, 0x04

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._write8(MODE2, OUTDRV)
        self._write8(MODE1, AI)              # auto-inc (also clears ALLCALL)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._read8(MODE1)
        self._write8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._read8(MODE1)
        self._write8(MODE1, (old & ~RESTART) | SLEEP)
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._write8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def set_pwm(self, ch: int, on: int, off: int):
        base = LED0_ON_L + 4 * int(ch)
        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self.set_pwm(ch, 0, off)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4 * int(ch)
        self._write8(base + 0, 0x00); self._write8(base + 1, 0x00)
        self._write8(base + 2, 0x00); self._write8(base + 3, 0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4 * int(ch)
        self._write8(base + 0, 0x00); self._write8(base + 1, 0x10)
        self._write8(base + 2, 0x00); self._write8(base + 3, 0x00)

# ---------------------- Motor / H-Bridge ----------------------
@dataclass
class MotorCH: pwm: int; in1: int; in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap_in12 = bool(swap_in12)
        self.stop()

    def _digital(self, ch: int, level: int):
        if self.in_active_low: level ^= 1
        if level: self.pca.full_on(ch)
        else:     self.pca.full_off(ch)

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert: d = -d
        inA, inB = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)
        if abs(d) < 1e-3: self.stop(); return
        if d > 0:
            self._digital(inA, 1); self._digital(inB, 0); self.pca.set_duty(self.ch.pwm, d)
        else:
            self._digital(inA, 0); self._digital(inB, 1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        inA, inB = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)
        self._digital(inA, 0); self._digital(inB, 0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- Keyboard ----------------------
class Keyboard:
    """Non-blocking; handles CSI (ESC [ A) and SS3 (ESC O A) arrows."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, timeout: float = 0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r: return None
        return os.read(self.fd, 1)

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
                    else: out.extend(filter(None, [b"\x1b", a, b]))
                else:
                    out.append(b"\x1b");  out.append(a) if a else None
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------------- Helpers ----------------------
def clamp(x, lo, hi): return max(lo, min(hi, x))
def step(curr, target, max_step): return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

# ---------------------- Teleop core ----------------------
@dataclass
class Limits: vx: float; vy: float; wz: float; ax: float; ay: float; az: float

class DirectTeleop:
    def __init__(self, args):
        self.hz, self.deadman = float(args.hz), float(args.deadman)
        self.scale_low, self.scale_high = float(args.scale_low), float(args.scale_high)
        self.scale = 1.0
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega, args.accel_x, args.accel_y, args.accel_z)

        # Per-wheel sign multipliers (fl,fr,rl,rr). Default = (-1,+1,+1,-1)
        self.ws = [int(x) for x in args.wheel_signs.split(',')]
        if len(self.ws) != 4 or any(s not in (-1,1) for s in self.ws):
            raise ValueError("--wheel-signs must be 4 items of -1/1")

        # Pair sets (indices FL=0, FR=1, RL=2, RR=3)
        self.pairs = {
            'forward':      [1,2],  # M2,M3
            'backward':     [0,3],  # M1,M4
            'strafe_left':  [0,1],  # M1,M2
            'strafe_right': [2,3],  # M3,M4
            'left_side':    [0,2],  # M1,M3
            'right_side':   [1,3],  # M2,M4
        }

        # state
        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c = [0.0, 0.0, 0.0]  # filtered (vx,vy,wz)
        self._last = time.monotonic()

        # PCA
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr,
                           freq_hz=args.pwm_freq, set_freq=(not args.no_freq_change))
        # reserve OFF
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception: pass

        # Motors
        self.mot = [
            HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2),
                    invert=args.inv_fl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fl),
            HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2),
                    invert=args.inv_fr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fr),
            HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2),
                    invert=args.inv_rl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rl),
            HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2),
                    invert=args.inv_rr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rr),
        ]

        signal.signal(signal.SIGINT, self._sig_quit); signal.signal(signal.SIGTERM, self._sig_quit)
        print("[Teleop] READY (pair mode). W/S->(M2,M3)/(M1,M4), A/D strafe, Q/E turn.")

    def _sig_quit(self, *_):
        print("\n[Teleop] SIGINT/SIGTERM → safe stop and exit")
        self.close(); os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in self.mot:
            try: m.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    def loop(self):
        try:
            period = 1.0 / self.hz
            while True:
                t0 = time.monotonic()
                got_key = False
                for ch in self.kb.read():
                    got_key = True
                    if self._handle_key(ch): return
                    self.last_input = t0
                if not got_key and (t0 - self.last_input) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # filter
                dt = max(1e-3, t0 - self._last); self._last = t0
                for i, (tgt, acc) in enumerate(zip((self.t_vx, self.t_vy, self.t_wz),
                                                   (self.lim.ax, self.lim.ay, self.lim.az))):
                    stepmax = acc * dt
                    self.c[i] = min(self.c[i] + stepmax, tgt) if tgt > self.c[i] else max(self.c[i] - stepmax, tgt)

                self._apply(self.c[0], self.c[1], self.c[2])
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

    # ------------------ input handling ------------------
    def _handle_key(self, ch) -> bool:
        if ch in (b"\x1b", b"\x03"): print("[Teleop] Quit"); return True
        if ch == b' ': self.t_vx = self.t_vy = self.t_wz = 0.0; print("[Cmd] STOP"); return False
        if ch in (b'm', b'M'): self._print_status(); return False
        if ch in (b'g', b'G'): self._gentle_forward(); return False

        # Shift = fast, Ctrl = slow
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
            if slow: self.scale = 0.20
            elif fast: self.scale = 1.5
            else: self.scale = 1.0

        if ch in (b'w', b'W', b'UP'):     self.t_vx = +self.lim.vx
        elif ch in (b's', b'S', b'DOWN'): self.t_vx = -self.lim.vx
        elif ch in (b'a', b'A', b'LEFT'): self.t_vy = +self.lim.vy
        elif ch in (b'd', b'D', b'RIGHT'):self.t_vy = -self.lim.vy
        elif ch in (b'q', b'Q'):          self.t_wz = +self.lim.wz
        elif ch in (b'e', b'E'):          self.t_wz = -self.lim.wz
        elif ch in (b'x', b'X'):          self.t_vx = 0.0; self.t_vy = 0.0
        elif ch in (b'z', b'Z'):          self.t_wz = 0.0
        elif ch == b'1': self._pulse([1,0,0,0]); return False
        elif ch == b'2': self._pulse([0,1,0,0]); return False
        elif ch == b'3': self._pulse([0,0,1,0]); return False
        elif ch == b'4': self._pulse([0,0,0,1]); return False
        elif ch == b'5': self._pulse([1,0,0,0], duty=-0.15); return False
        elif ch == b'6': self._pulse([0,1,0,0], duty=-0.15); return False
        elif ch == b'7': self._pulse([0,0,1,0], duty=-0.15); return False
        elif ch == b'8': self._pulse([0,0,0,1], duty=-0.15); return False
        return False

    # ------------------ pair mapping core ------------------
    def _apply(self, vx, vy, wz):
        # base magnitudes (scaled)
        sx = abs(vx) / max(self.lim.vx, 1e-6)
        sy = abs(vy) / max(self.lim.vy, 1e-6)
        so = abs(wz) / max(self.lim.wz, 1e-6)
        scale = self.scale

        duties = [0.0, 0.0, 0.0, 0.0]  # FL,FR,RL,RR

        # forward/back pairs
        if vx > 1e-6:
            for i in self.pairs['forward']: duties[i] += +sx * scale
        elif vx < -1e-6:
            for i in self.pairs['backward']: duties[i] += +sx * scale

        # strafe pairs
        if vy > 1e-6:
            for i in self.pairs['strafe_left']: duties[i] += +sy * scale
        elif vy < -1e-6:
            for i in self.pairs['strafe_right']: duties[i] += +sy * scale

        # turn mix (left side reverse, right side forward)
        if wz > 1e-6:
            for i in self.pairs['left_side']:  duties[i] += -so * scale
            for i in self.pairs['right_side']: duties[i] += +so * scale
        elif wz < -1e-6:
            for i in self.pairs['left_side']:  duties[i] += +so * scale
            for i in self.pairs['right_side']: duties[i] += -so * scale

        # Apply per-wheel signs and clamp
        for i in range(4):
            duties[i] = self.ws[i] * clamp(duties[i], -1.0, +1.0)

        self._duties = duties
        # drive
        for i, m in enumerate(self.mot):
            m.drive(duties[i])

    # ------------------ utils ------------------
    def _pulse(self, mask, duty=0.12, t=0.30):
        for i, m in enumerate(self.mot): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in self.mot: m.stop()

    def _gentle_forward(self, duty=0.10, t=0.50):
        print("[Test] Gentle forward pulse (all)")
        for m in self.mot: m.drive(duty)
        time.sleep(t)
        for m in self.mot: m.stop()

    def _print_status(self):
        d = getattr(self, '_duties', [0,0,0,0])
        print(f"[duties] FL {d[0]:+.2f}  FR {d[1]:+.2f}  RL {d[2]:+.2f}  RR {d[3]:+.2f}")

# ---------------------- CLI ----------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (Pair Mode for FNK0043)')
    # Limits & rate
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
    # I2C / PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--no-freq-change', action='store_true', help="Do NOT change PCA9685 frequency")
    p.add_argument('--reserve', type=str, default='8,9,10,11,12,13,14,15', help='Channels to FULL-OFF at startup (servos)')
    # Channels (FL,FR,RL,RR)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Polarity tweaks
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Per-wheel signs (default implements your forward pattern)
    p.add_argument('--wheel-signs', type=str, default='-1,+1,+1,-1',
                   help='Comma of -1/1 for (FL,FR,RL,RR). Default = M2&M3 fwd, M1&M4 back.')
    return p

def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = DirectTeleop(args)
    try: t.loop()
    except KeyboardInterrupt: pass
    finally: t.close()

if __name__ == '__main__':
    main()
