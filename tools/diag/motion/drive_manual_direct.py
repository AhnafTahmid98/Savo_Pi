#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop  for Mecanum (PCA9685 + H-Bridge)
----------------------------------------------------------------------
- Reads keyboard directly (raw TTY) and drives motors via PCA9685 @ 0x40.
- **Arrow keys supported** in addition to WASD/QE.
- No ROS 2 required; ideal for bring‑up/offline testing on blocks.
- Smooth accel limiting, deadman timeout, software e‑stop, clear kinematics.

Key map (US)
  W/S or ↑/↓  = +vx / −vx   (forward/back)
  A/D or ←/→  = +vy / −vy   (strafe left/right)
  Q/E         = +ω  / −ω    (CCW/CW turn)
  SPACE       = stop immediately
  Shift       = fast scale    |   Ctrl = slow scale
  R           = reset scale to 1.0
  Esc / Ctrl+C= quit

Conventions
  +vx forward, +vy left, +ω CCW.
  Wheel order: FL, FR, RL, RR.

Kinematics (inverse, mecanum)
  r = wheel radius;  L = (Lx + Ly)/2 (half of track + wheelbase)
  ω_FL = ( vx − vy − L*ωz ) / r
  ω_FR = ( vx + vy + L*ωz ) / r
  ω_RL = ( vx + vy − L*ωz ) / r
  ω_RR = ( vx − vy + L*ωz ) / r

Hardware assumptions
  - PCA9685 @ 0x40 driving H‑bridge inputs (IN1/IN2) and a PWM per wheel.
  - Provide per‑wheel channels via CLI (e.g., --fl-pwm 0 --fl-in1 1 --fl-in2 2 ...).

Safety
  - Software e‑stop: --estop-force zeroes all outputs.
  - For a hardware e‑stop input, we can add GPIO (lgpio) on request.

Usage example
  sudo -E python3 tools/scripts/drive_manual_direct.py \
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
        self.bus = SMBus(bus)
        self._write8(MODE1, 0x00)  # normal mode
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale_val = 25_000_000.0 / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(max(3, min(255, round(prescale_val))))
        old = self._read8(MODE1)
        self._write8(MODE1, (old & 0x7F) | 0x10)  # sleep
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, old)
        time.sleep(0.005)
        self._write8(MODE1, old | 0xA1)  # auto‑inc, allcall

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * channel
        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_duty(self, channel: int, duty: float):
        duty = max(0.0, min(1.0, duty))
        off = int(round(duty * 4095))
        self.set_pwm(channel, 0, off)

# ------------------------- Motor abstraction ------------------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH):
        self.pca = pca
        self.ch = ch
        self.stop()

    def _gpio_set(self, channel: int, level: int):
        # emulate digital via PWM full‑on/full‑off
        if level:
            self.pca.set_pwm(channel, 4096, 0)  # ON
        else:
            self.pca.set_pwm(channel, 0, 0)     # OFF

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, duty_signed))
        if abs(d) < 1e-3:
            self.stop()
            return
        if d > 0:
            self._gpio_set(self.ch.in1, 1); self._gpio_set(self.ch.in2, 0)
            self.pca.set_duty(self.ch.pwm, d)
        else:
            self._gpio_set(self.ch.in1, 0); self._gpio_set(self.ch.in2, 1)
            self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        self._gpio_set(self.ch.in1, 0)
        self._gpio_set(self.ch.in2, 0)
        self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- Keyboard + helpers --------------------------
class Keyboard:
    """Non‑blocking keyboard reader that also parses ANSI arrow keys.
    Returns list of tokens: one‑byte bytes OR b'UP'/b'DOWN'/b'LEFT'/b'RIGHT'.
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        return os.read(self.fd, 1)

    def read(self):
        out = []
        ch = self._read1()
        while ch is not None:
            if ch == b"":  # ESC
                a = self._read1(); b = self._read1()
                if a == b'[' and b in (b'A', b'B', b'C', b'D'):
                    out.append({b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}[b])
                else:
                    out.append(b"")
                    if a: out.append(a)
                    if b: out.append(b)
            else:
                out.append(ch)
            ch = self._read1()
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

        self.kb = Keyboard()
        self.last_input = 0.0
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq)
        self.FL = HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2))
        self.FR = HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2))
        self.RL = HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2))
        self.RR = HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2))

        self.estop_force = bool(args.estop_force)

        print(
            "[Teleop] READY (no ROS2). Esc to quit."
            f" rate={self.hz} Hz  deadman={self.deadman}s  max(vx,vy,wz)=({self.lim.vx},{self.lim.vy},{self.lim.wz})"
            f" accel(x,y,z)=({self.lim.ax},{self.lim.ay},{self.lim.az})  scale_low={self.scale_low}  scale_high={self.scale_high}"
            f" geom: r={self.geom.r} m, L={self.geom.L} m  PCA9685@0x{args.pca_addr:02X} bus={args.i2c_bus}"
            " Keys: WASD/QE or Arrows, Space=stop, R=reset, Shift=fast, Ctrl=slow"
        )

    def close(self):
        try:
            self.FL.stop(); self.FR.stop(); self.RL.stop(); self.RR.stop()
            if self.kb: self.kb.restore()
        finally:
            try: self.pca.bus.close()
            except Exception: pass

    def loop(self):
        try:
            period = 1.0 / self.hz
            while True:
                t0 = time.monotonic()
                for ch in self.kb.read():
                    if self._handle_key(ch):
                        return
                    self.last_input = t0

                # deadman → zero targets
                if (t0 - self.last_input) > self.deadman:
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # accel limit toward scaled targets
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

                # loop pacing
                dt_done = time.monotonic() - t0
                time.sleep(max(0.0, period - dt_done))
        finally:
            self.close()

    # ------------------ internals ------------------
    def _handle_key(self, ch: bytes) -> bool:
        # return True to quit
        if ch in (b"", b""):  # Esc or Ctrl+C
            print("[Teleop] Quit")
            return True
        if ch == b' ':  # Space → stop
            self.t_vx = self.t_vy = self.t_wz = 0.0
            return False
        if ch in (b'r', b'R', b""):  # reset scale (r/R or Ctrl+R)
            self.scale = 1.0
            return False

        # speed scaling heuristics
        slow = isinstance(ch, bytes) and (1 <= ch[0] <= 26)   # control chars → slow
        fast = isinstance(ch, bytes) and ch.isalpha() and ch.isupper()  # uppercase → fast
        if slow:
            self.scale = clamp(self.scale_low, 0.05, 1.0)
        elif fast:
            self.scale = clamp(self.scale_high, 1.0, 3.0)
        else:
            self.scale = 1.0

        # Movement keys (WASD/QE + Arrows)
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
        # inverse kinematics → wheel angular rates
        r = self.geom.r; L = self.geom.L
        w_fl = ( vx - vy - L*wz ) / r
        w_fr = ( vx + vy + L*wz ) / r
        w_rl = ( vx + vy - L*wz ) / r
        w_rr = ( vx - vy + L*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        duties = [clamp(w/max_w, -1.0, 1.0) for w in ws]
        self.FL.drive(duties[0]); self.FR.drive(duties[1]); self.RL.drive(duties[2]); self.RR.drive(duties[3])

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
