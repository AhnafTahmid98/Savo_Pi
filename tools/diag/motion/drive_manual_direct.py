#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for Mecanum (PCA9685 + H-Bridge)
----------------------------------------------------------------------
- Keyboard → direct PCA9685 motor drive (no ROS2).
- Arrow keys + WASD/QE.
- Accel limiting, deadman, per-wheel invert, shutdown-safe.
- **Diagnostics** built in to fix your mapping fast:
    • 1/2/3/4  → pulse FL/FR/RL/RR **forward**
    • 5/6/7/8  → pulse FL/FR/RL/RR **reverse**
    • F/H/T    → demo patterns (Forward / strafe-Left / Turn CCW)
    • M        → print current wheel duties + targets
- **Sign toggles** for chassis conventions: --flip-vy, --flip-omega
- **Robust quit**: Esc or Ctrl+C always stops motors and exits.

Key map (US)
  W/S or ↑/↓  = +vx / −vx   (forward/back)
  A/D or ←/→  = +vy / −vy   (strafe left/right)
  Q/E         = +ω  / −ω    (CCW/CW turn)
  1..4 / 5..8 = pulse single wheel fwd / rev (FL,FR,RL,RR)
  F/H/T       = forward / strafe-left / turn-CCW pattern demo
  M           = print current command + wheel duties
  SPACE       = stop immediately
  Shift       = fast scale    |   Ctrl = slow scale
  R           = reset scale to 1.0
  Esc / Ctrl+C= quit (safe)

Conventions
  +vx forward, +vy left, +ω CCW (can flip vy/ω with flags).
  Wheel order: FL, FR, RL, RR.

Kinematics (inverse, mecanum)
  r = wheel radius;  L = (Lx + Ly)/2 (half of track + wheelbase)
  ω_FL = ( vx − s_y*vy − L*s_o*ωz ) / r
  ω_FR = ( vx + s_y*vy + L*s_o*ωz ) / r
  ω_RL = ( vx + s_y*vy − L*s_o*ωz ) / r
  ω_RR = ( vx − s_y*vy + L*s_o*ωz ) / r
Where s_y ∈ {+1,-1} from --flip-vy, s_o ∈ {+1,-1} from --flip-omega.

Usage example
  sudo -E python3 tools/diag/motion/drive_manual_direct.py \
      --fl-pwm 0 --fl-in1 1 --fl-in2 2 \
      --fr-pwm 3 --fr-in1 4 --fr-in2 5 \
      --rl-pwm 6 --rl-in1 7 --rl-in2 8 \
      --rr-pwm 9 --rr-in1 10 --rr-in2 11 \
      --hz 60 --max-vx 0.6 --max-vy 0.6 --max-omega 1.8 \
      --wheel-radius 0.0325 --L 0.115

Dependencies
  sudo apt install -y python3-smbus  # or: pip3 install smbus2
"""

import argparse
import os
import select
import sys
import termios
import time
import tty
import signal
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install with: sudo apt install python3-smbus or pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 minimal driver ----------------------
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

class PCA9685:
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 1000.0):
        self.addr = addr
        self.bus = SMBus(int(bus))
        self._write8(MODE1, 0x00)  # normal mode
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)

    def _write8(self, reg, val):
        reg = int(reg) & 0xFF
        val = int(val) & 0xFF
        self.bus.write_byte_data(self.addr, reg, val)

    def _read8(self, reg):
        reg = int(reg) & 0xFF
        return self.bus.read_byte_data(self.addr, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale_val = 25_000_000.0 / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(max(3, min(255, round(prescale_val))))
        old = self._read8(MODE1)
        self._write8(MODE1, (old & 0x7F) | 0x10)  # sleep
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, old)
        time.sleep(0.005)
        self._write8(MODE1, old | 0xA1)  # auto-inc, allcall

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * int(channel)
        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_duty(self, channel: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        self.set_pwm(channel, 0, off)

# ------------------------- Motor abstraction ------------------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert: bool = False):
        self.pca = pca
        self.ch = ch
        self.invert = bool(invert)
        self.stop()

    def _gpio_set(self, channel: int, level: int):
        try:
            if level:
                self.pca.set_pwm(int(channel), 4096, 0)  # ON
            else:
                self.pca.set_pwm(int(channel), 0, 0)     # OFF
        except Exception:
            pass

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert:
            d = -d
        if abs(d) < 1e-3:
            self.stop()
            return
        try:
            if d > 0:
                self._gpio_set(self.ch.in1, 1); self._gpio_set(self.ch.in2, 0)
                self.pca.set_duty(self.ch.pwm, d)
            else:
                self._gpio_set(self.ch.in1, 0); self._gpio_set(self.ch.in2, 1)
                self.pca.set_duty(self.ch.pwm, -d)
        except Exception:
            pass

    def stop(self):
        try:
            self._gpio_set(self.ch.in1, 0)
            self._gpio_set(self.ch.in2, 0)
            self.pca.set_duty(self.ch.pwm, 0.0)
        except Exception:
            pass

# ---------------------- Keyboard + helpers --------------------------
class Keyboard:
    """Non‑blocking keyboard reader that parses arrow keys robustly.
    Returns tokens: bytes OR b'UP'/b'DOWN'/b'LEFT'/b'RIGHT'.
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, timeout: float = 0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        return os.read(self.fd, 1)

    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b"":  # ESC or arrow seq
                a = self._read1(0.02)
                if a == b'[':
                    b = self._read1(0.02)
                    if b in (b'A', b'B', b'C', b'D'):
                        out.append({b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}[b])
                    else:
                        out.extend(filter(None, [b"", a, b]))
                else:
                    out.append(b"")
                    if a:
                        out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def step(curr, target, max_step):
    return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

# -------------------------- Teleop core -----------------------------
@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

@dataclass
class MecanumGeom:
    r: float   # wheel radius (m)
    L: float   # (Lx + Ly)/2 (m)

class DirectTeleop:
    def __init__(self, args):
        self.hz = float(args.hz)
        self.deadman = float(args.deadman)
        self.scale_low = float(args.scale_low)
        self.scale_high = float(args.scale_high)
        self.scale = 1.0
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega, args.accel_x, args.accel_y, args.accel_z)
        self.geom = MecanumGeom(args.wheel_radius, args.L)
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        self.kb = Keyboard()
        self.last_input = 0.0
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq)
        self.FL = HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2), invert=args.inv_fl)
        self.FR = HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2), invert=args.inv_fr)
        self.RL = HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2), invert=args.inv_rl)
        self.RR = HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2), invert=args.inv_rr)

        self.estop_force = bool(args.estop_force)

        # trap SIGINT/SIGTERM for safe stop even if raw tty eats Ctrl+C
        signal.signal(signal.SIGINT, self._signal_quit)
        signal.signal(signal.SIGTERM, self._signal_quit)

        print(
            "[Teleop] READY (no ROS2). Esc/Ctrl+C to quit."
            f" rate={self.hz} Hz  deadman={self.deadman}s  max(vx,vy,wz)=({self.lim.vx},{self.lim.vy},{self.lim.wz})"
            f" accel(x,y,z)=({self.lim.ax},{self.lim.ay},{self.lim.az})  scale_low={self.scale_low}  scale_high={self.scale_high}"
            f" geom: r={self.geom.r} m, L={self.geom.L} m  PCA9685@0x{args.pca_addr:02X} bus={args.i2c_bus}"
            f" flips: vy={'-vy' if args.flip_vy else 'vy'}  omega={'-ω' if args.flip_omega else 'ω'}"
            " Keys: WASD/QE or Arrows, 1..4/5..8 wheel test fwd/rev, F/H/T demos, M print, Space stop, R reset"
        )

    def _signal_quit(self, *_):
        print("[Teleop] SIGINT/SIGTERM → safe stop and exit")
        self.close()
        os._exit(0)

    def close(self):
        try:
            if self.kb:
                self.kb.restore()
        except Exception:
            pass
        for m in (getattr(self, 'FL', None), getattr(self, 'FR', None), getattr(self, 'RL', None), getattr(self, 'RR', None)):
            try:
                if m:
                    m.stop()
            except Exception:
                pass
        try:
            if hasattr(self.pca, 'bus') and self.pca.bus:
                self.pca.bus.close()
        except Exception:
            pass

    def loop(self):
        try:
            period = 1.0 / self.hz
            while True:
                t0 = time.monotonic()
                for ch in self.kb.read():
                    if self._handle_key(ch):
                        return
                    self.last_input = t0

                if (t0 - self.last_input) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                scale = self.scale
                vx_t = clamp(self.t_vx * scale, -self.lim.vx, self.lim.vx)
                vy_t = clamp(self.t_vy * scale, -self.lim.vy, self.lim.vy)
                wz_t = clamp(self.t_wz * scale, -self.lim.wz, self.lim.wz)

                dt = max(1e-3, t0 - self._last); self._last = t0
                self.c_vx = step(self.c_vx, vx_t, self.lim.ax * dt)
                self.c_vy = step(self.c_vy, vy_t, self.lim.ay * dt)
                self.c_wz = step(self.c_wz, wz_t, self.lim.az * dt)

                if self.estop_force:
                    self.c_vx = self.c_vy = self.c_wz = 0.0

                self._apply(self.c_vx, self.c_vy, self.c_wz)

                dt_done = time.monotonic() - t0
                time.sleep(max(0.0, period - dt_done))
        finally:
            self.close()

    # ------------------ internals ------------------
    def _handle_key(self, ch) -> bool:
        if ch in (b"", b""):  # Esc or Ctrl+C
            print("[Teleop] Quit")
            return True
        if ch == b' ':  # Space stop
            self.t_vx = self.t_vy = self.t_wz = 0.0
            return False
        if ch in (b'r', b'R', b""):
            self.scale = 1.0
            return False

        # diagnostics: single-wheel and patterns
        if ch == b'1': self._pulse_wheels([1,0,0,0]); return False
        if ch == b'2': self._pulse_wheels([0,1,0,0]); return False
        if ch == b'3': self._pulse_wheels([0,0,1,0]); return False
        if ch == b'4': self._pulse_wheels([0,0,0,1]); return False
        if ch == b'5': self._pulse_wheels([1,0,0,0], duty=-0.5); return False
        if ch == b'6': self._pulse_wheels([0,1,0,0], duty=-0.5); return False
        if ch == b'7': self._pulse_wheels([0,0,1,0], duty=-0.5); return False
        if ch == b'8': self._pulse_wheels([0,0,0,1], duty=-0.5); return False
        if ch in (b'f', b'F'): self._demo(vx=0.4, vy=0.0, wz=0.0); return False
        if ch in (b'h', b'H'): self._demo(vx=0.0, vy=0.4, wz=0.0); return False
        if ch in (b't', b'T'): self._demo(vx=0.0, vy=0.0, wz=0.8); return False
        if ch in (b'm', b'M'): self._print_status(); return False

        # speed scaling
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
        else:
            slow = False; fast = False
        if slow:
            self.scale = clamp(self.scale_low, 0.05, 1.0)
        elif fast:
            self.scale = clamp(self.scale_high, 1.0, 3.0)
        else:
            self.scale = 1.0

        # movement
        if ch in (b'w', b'W', b'UP'):
            self.t_vx = +self.lim.vx
        elif ch in (b's', b'S', b'DOWN'):
            self.t_vx = -self.lim.vx
        elif ch in (b'a', b'A', b'LEFT'):
            self.t_vy = +self.lim.vy
        elif ch in (b'd', b'D', b'RIGHT'):
            self.t_vy = -self.lim.vy
        elif ch in (b'q', b'Q'):
            self.t_wz = +self.lim.wz
        elif ch in (b'e', b'E'):
            self.t_wz = -self.lim.wz
        elif ch in (b'x', b'X'):
            self.t_vx = 0.0; self.t_vy = 0.0
        elif ch in (b'z', b'Z'):
            self.t_wz = 0.0
        return False

    def _apply(self, vx, vy, wz):
        r = self.geom.r; L = self.geom.L; sy = self.sy; so = self.so
        # compute desired wheel angular velocities
        w_fl = ( vx - sy*vy - L*so*wz ) / r
        w_fr = ( vx + sy*vy + L*so*wz ) / r
        w_rl = ( vx + sy*vy - L*so*wz ) / r
        w_rr = ( vx - sy*vy + L*so*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        # duty proportional to command magnitude
        mag = max(abs(vx)/max(self.lim.vx,1e-6), abs(vy)/max(self.lim.vy,1e-6), abs(wz)/max(self.lim.wz,1e-6))
        duties = [clamp((w/max_w) * mag, -1.0, 1.0) for w in ws]
        self._duties = duties  # store for M print
        self.FL.drive(duties[0]); self.FR.drive(duties[1]); self.RL.drive(duties[2]); self.RR.drive(duties[3])

    def _pulse_wheels(self, mask, duty=0.5, t=0.5):
        motors = [self.FL, self.FR, self.RL, self.RR]
        for i, m in enumerate(motors):
            m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in motors:
            m.stop()

    def _demo(self, vx, vy, wz, t=0.8):
        self._apply(vx, vy, wz)
        time.sleep(t)
        self._apply(0.0, 0.0, 0.0)

    def _print_status(self):
        try:
            duties = getattr(self, '_duties', [0,0,0,0])
            print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}  duties=[FL {duties[0]:+.2f}, FR {duties[1]:+.2f}, RL {duties[2]:+.2f}, RR {duties[3]:+.2f}]")
        except Exception:
            pass

# --------------------------- CLI -----------------------------------

def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (no ROS2, PCA9685)')
    # Geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325, help='Wheel radius r (m) (65 mm wheel → 0.0325)')
    p.add_argument('--L', type=float, default=0.115, help='(Lx+Ly)/2 in meters (≈0.115 for ~165 mm track + ~65 mm wheel)')
    # Limits
    p.add_argument('--max-vx', type=float, default=0.60)
    p.add_argument('--max-vy', type=float, default=0.60)
    p.add_argument('--max-omega', type=float, default=1.80)
    p.add_argument('--accel-x', type=float, default=1.2)
    p.add_argument('--accel-y', type=float, default=1.2)
    p.add_argument('--accel-z', type=float, default=2.5)
    p.add_argument('--hz', type=float, default=50.0)
    p.add_argument('--deadman', type=float, default=0.4)
    p.add_argument('--scale-low', type=float, default=0.35)
    p.add_argument('--scale-high', type=float, default=1.7)
    # I2C / PCA9685
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x: int(x, 0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=1000.0, help='PWM carrier frequency (Hz). 1 kHz safe for many H‑bridges')
    # Channel mapping
    p.add_argument('--fl-pwm', type=int, required=True)
    p.add_argument('--fl-in1', type=int, required=True)
    p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True)
    p.add_argument('--fr-in1', type=int, required=True)
    p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True)
    p.add_argument('--rl-in1', type=int, required=True)
    p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True)
    p.add_argument('--rr-in1', type=int, required=True)
    p.add_argument('--rr-in2', type=int, required=True)
    # Polarity fixes
    p.add_argument('--inv-fl', action='store_true', help='Invert FL direction')
    p.add_argument('--inv-fr', action='store_true', help='Invert FR direction')
    p.add_argument('--inv-rl', action='store_true', help='Invert RL direction')
    p.add_argument('--inv-rr', action='store_true', help='Invert RR direction')
    # Convention flips
    p.add_argument('--flip-vy', action='store_true', help='Flip strafe axis sign (vy ←→ −vy)')
    p.add_argument('--flip-omega', action='store_true', help='Flip yaw sign (ω ←→ −ω)')
    # Safety (software)
    p.add_argument('--estop-force', action='store_true', help='Force software e‑stop (zero all outputs)')
    return p


def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = DirectTeleop(args)
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()


if __name__ == '__main__':
    main()
