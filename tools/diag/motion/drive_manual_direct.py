#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for DC Motors via PCA9685 + H-Bridge
---------------------------------------------------------------------------
MOTOR-ONLY (no servo control). Designed to be safe on FNK0043 (shared PCA9685).

Fixes / Features
- W/S forward/back fixed: new --flip-vx (DEFAULT ON) makes W = forward on FNK0043.
- Arrow keys robust: supports CSI (ESC [ A) and SS3 (ESC O A).
- Clears ALLCALL, sets OUTDRV; reserves channels (FULL-OFF) so servos don't twitch.
- H-bridge control with proper FULL-ON/FULL-OFF on IN pins and PWM on ENA/ENB.
- Per-wheel invert + IN1/IN2 swap + active-LOW compatibility.
- Safe defaults (slow speeds, deadman, gentle pulses).

Keys
  W/S or ↑/↓  = Forward / Back
  Q/E         = Turn CCW / CW
  A/D or ←/→  = Strafe L/R  (set --max-vy 0.0 on paired boards like FNK0043)
  G           = Gentle forward pulse
  1..4 / 5..8 = Pulse FL/FR/RL/RR forward / reverse
  M           = Print current command + wheel duties
  SPACE       = Stop immediately
  Shift       = fast scale,  Ctrl = slow scale
  R           = Reset scale to 1.0
  Esc / Ctrl+C= Quit (safe)
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
    print("ERROR: smbus2 is required. Install with: sudo apt install -y python3-smbus || pip3 install smbus2",
          file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1     = 0x00
MODE2     = 0x01
PRESCALE  = 0xFE
LED0_ON_L = 0x06

# Bits
RESTART = 0x80
SLEEP   = 0x10
AI      = 0x20
ALLCALL = 0x01
OUTDRV  = 0x04  # totem-pole outputs

class PCA9685:
    """
    Minimal PCA9685 driver with:
      - OUTDRV push-pull
      - Auto-increment
      - ALLCALL cleared (no broadcast)
      - helpers: set_pwm(), set_duty(), full_off(), full_on()
    """
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 800.0):
        self.addr = int(addr)
        self.bus = SMBus(int(bus))
        # OUTDRV + AI; clear ALLCALL
        self._write8(MODE2, OUTDRV)
        self._write8(MODE1, AI)  # (clears ALLCALL)
        time.sleep(0.003)
        self.set_pwm_freq(freq_hz)
        # ensure ALLCALL cleared
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
        freq_hz = float(max(40.0, min(1500.0, freq_hz)))
        prescale_val = 25_000_000.0 / (4096.0 * freq_hz) - 1.0
        prescale = int(max(3, min(255, round(prescale_val))))
        old = self._read8(MODE1)
        self._write8(MODE1, (old & ~RESTART) | SLEEP)   # sleep
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, old & ~SLEEP)               # wake
        time.sleep(0.003)
        self._write8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * int(channel)
        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_duty(self, channel: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:
            self.full_off(channel)
        elif off >= 4095:
            self.full_on(channel)
        else:
            self.set_pwm(channel, 0, off)

    def full_off(self, channel: int):
        """Digital LOW (no PWM): ON=0, OFF=full-off bit set."""
        base = LED0_ON_L + 4 * int(channel)
        self._write8(base + 0, 0x00)
        self._write8(base + 1, 0x00)
        self._write8(base + 2, 0x00)
        self._write8(base + 3, 0x10)  # OFF_H full-off bit

    def full_on(self, channel: int):
        """Digital HIGH (no PWM): ON=full-on bit set, OFF=0."""
        base = LED0_ON_L + 4 * int(channel)
        self._write8(base + 0, 0x00)
        self._write8(base + 1, 0x10)  # ON_H full-on bit
        self._write8(base + 2, 0x00)
        self._write8(base + 3, 0x00)

# ------------------------- Motor abstraction ------------------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    """
    H-bridge control via PCA9685 channels:
      - Direction: IN1/IN2 digital full-on/full-off (optionally active-LOW)
      - Speed: PWM duty on PWM channel (0..1)
      - swap_in12=True swaps the two direction lines (wiring fix)
      - invert=True flips the motor sign convention (mechanical fix)
    """
    def __init__(self, pca: PCA9685, ch: MotorCH, invert: bool = False,
                 in_active_low: bool = False, swap_in12: bool = False):
        self.pca = pca
        self.ch = ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap_in12 = bool(swap_in12)
        self.stop()

    def _digital(self, channel: int, level: int):
        # level=1 (assert) / 0 (deassert), before active-low inversion
        if self.in_active_low:
            level ^= 1
        if level:
            self.pca.full_on(channel)
        else:
            self.pca.full_off(channel)

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert:
            d = -d
        inA, inB = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)

        if abs(d) < 1e-3:
            self.stop()
            return
        if d > 0:
            # Forward
            self._digital(inA, 1); self._digital(inB, 0)
            self.pca.set_duty(self.ch.pwm, d)
        else:
            # Reverse
            self._digital(inA, 0); self._digital(inB, 1)
            self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        inA, inB = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)
        self._digital(inA, 0)
        self._digital(inB, 0)
        self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- Keyboard + helpers --------------------------
class Keyboard:
    """Non-blocking keyboard reader that parses arrow keys robustly (CSI & SS3)."""
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
            if ch == b"\x1b":  # ESC
                a = self._read1(0.02)
                if a in (b'[', b'O'):  # CSI or SS3
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows:
                        out.append(arrows[b])
                    else:
                        out.extend(filter(None, [b"\x1b", a, b]))
                else:
                    out.append(b"\x1b")
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
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega,
                          args.accel_x, args.accel_y, args.accel_z)
        self.geom = MecanumGeom(args.wheel_radius, args.L)

        # Axis sign flips (W/S fix here: flip-vx default True)
        self.sx = -1.0 if args.flip_vx else +1.0
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        # PCA + reserve OFF
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq)
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip() != '']
            for ch in reserved:
                self.pca.full_off(ch)
            if reserved:
                print(f"[Teleop] Reserved channels OFF: {reserved}")
        except Exception:
            pass

        # Motors
        self.FL = HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2),
                          invert=args.inv_fl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fl)
        self.FR = HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2),
                          invert=args.inv_fr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fr)
        self.RL = HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2),
                          invert=args.inv_rl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rl)
        self.RR = HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2),
                          invert=args.inv_rr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rr)

        self.estop_force = bool(args.estop_force)

        signal.signal(signal.SIGINT, self._signal_quit)
        signal.signal(signal.SIGTERM, self._signal_quit)

        print(
            "[Teleop] READY (no ROS2). Esc/Ctrl+C to quit.\n"
            f" rate={self.hz} Hz  deadman={self.deadman}s  max(vx,vy,wz)=({self.lim.vx},{self.lim.vy},{self.lim.wz})\n"
            f" accel(x,y,z)=({self.lim.ax},{self.lim.ay},{self.lim.az})  scale_low={self.scale_low}  scale_high={self.scale_high}\n"
            f" geom: r={self.geom.r} m, L={self.geom.L} m  PCA9685@0x{args.pca_addr:02X} bus={args.i2c_bus}\n"
            f" flips: vx={'-vx' if args.flip_vx else 'vx'}  vy={'-vy' if args.flip_vy else 'vy'}  omega={'-ω' if args.flip_omega else 'ω'}\n"
            f" IN logic: {'ACTIVE-LOW' if args.in_active_low else 'ACTIVE-HIGH'}  swap(FL/FR/RL/RR)={args.swap_in12_fl}/{args.swap_in12_fr}/{args.swap_in12_rl}/{args.swap_in12_rr}\n"
            " Keys: WASD/QE or Arrows, G gentle forward, 1..8 wheel pulses, M print, Space stop, R reset\n"
        )

    def _signal_quit(self, *_):
        print("\n[Teleop] SIGINT/SIGTERM → safe stop and exit")
        self.close()
        os._exit(0)

    def close(self):
        try:
            if self.kb: self.kb.restore()
        except Exception: pass
        for m in (getattr(self, 'FL', None), getattr(self, 'FR', None),
                  getattr(self, 'RL', None), getattr(self, 'RR', None)):
            try:
                if m: m.stop()
            except Exception: pass
        try:
            if hasattr(self.pca, 'bus') and self.pca.bus: self.pca.close()
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
                    if (self.t_vx, self.t_vy, self.t_wz) != (0.0, 0.0, 0.0):
                        print("[Deadman] No input → stopping")
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

    def _announce(self, text): print(text)

    def _handle_key(self, ch) -> bool:
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit")
            return True
        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0; self._announce("[Cmd] STOP"); return False
        if ch in (b'r', b'R', b"\x12"):
            self.scale = 1.0; self._announce("[Scale] reset → 1.0"); return False
        if ch in (b'g', b'G'):
            self._gentle_forward(); return False
        if ch in (b'm', b'M'):
            self._print_status(); return False

        # Speed scaling (Shift=fast, Ctrl=slow)
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26   # Ctrl chars
            fast = ch.isalpha() and ch.isupper()
        else:
            slow = False; fast = False
        if slow: self.scale = clamp(self.scale_low, 0.05, 1.0)
        elif fast: self.scale = clamp(self.scale_high, 1.0, 3.0)
        else: self.scale = 1.0

        # Movement + cues (logical commands; sign flip applied later in _apply)
        if ch in (b'w', b'W', b'UP'):
            self.t_vx = +self.lim.vx; self._announce("[Cmd] FORWARD")
        elif ch in (b's', b'S', b'DOWN'):
            self.t_vx = -self.lim.vx; self._announce("[Cmd] BACK")
        elif ch in (b'a', b'A', b'LEFT'):
            self.t_vy = +self.lim.vy; self._announce("[Cmd] LEFT (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):
            self.t_vy = -self.lim.vy; self._announce("[Cmd] RIGHT (strafe)")
        elif ch in (b'q', b'Q'):
            self.t_wz = +self.lim.wz; self._announce("[Cmd] TURN CCW")
        elif ch in (b'e', b'E'):
            self.t_wz = -self.lim.wz; self._announce("[Cmd] TURN CW")
        elif ch in (b'x', b'X'):
            self.t_vx = 0.0; self.t_vy = 0.0; self._announce("[Cmd] HALT XY")
        elif ch in (b'z', b'Z'):
            self.t_wz = 0.0; self._announce("[Cmd] HALT YAW")
        # diagnostics: single-wheel pulses
        elif ch == b'1': self._pulse_wheels([1,0,0,0]); return False
        elif ch == b'2': self._pulse_wheels([0,1,0,0]); return False
        elif ch == b'3': self._pulse_wheels([0,0,1,0]); return False
        elif ch == b'4': self._pulse_wheels([0,0,0,1]); return False
        elif ch == b'5': self._pulse_wheels([1,0,0,0], duty=-0.12); return False
        elif ch == b'6': self._pulse_wheels([0,1,0,0], duty=-0.12); return False
        elif ch == b'7': self._pulse_wheels([0,0,1,0], duty=-0.12); return False
        elif ch == b'8': self._pulse_wheels([0,0,0,1], duty=-0.12); return False
        return False

    def _apply(self, vx, vy, wz):
        # Apply axis flips here (so W=forward with --flip-vx default ON)
        vx *= self.sx
        vy *= self.sy
        wz *= self.so

        r = self.geom.r; L = self.geom.L
        # inverse kinematics (mecanum)
        w_fl = ( vx - vy - L*wz ) / r
        w_fr = ( vx + vy + L*wz ) / r
        w_rl = ( vx + vy - L*wz ) / r
        w_rr = ( vx - vy + L*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        mag = max(abs(vx)/max(self.lim.vx,1e-6),
                  abs(vy)/max(self.lim.vy,1e-6),
                  abs(wz)/max(self.lim.wz,1e-6))
        duties = [clamp((w/max_w) * mag, -1.0, 1.0) for w in ws]
        self._duties = duties
        self.FL.drive(duties[0]); self.FR.drive(duties[1]); self.RL.drive(duties[2]); self.RR.drive(duties[3])

    def _pulse_wheels(self, mask, duty=0.12, t=0.30):
        motors = [self.FL, self.FR, self.RL, self.RR]
        for i, m in enumerate(motors):
            m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in motors:
            m.stop()

    def _gentle_forward(self, duty=0.10, t=0.50):
        self._announce("[Test] Gentle forward pulse")
        self.FL.drive(duty); self.FR.drive(duty); self.RL.drive(duty); self.RR.drive(duty)
        time.sleep(t)
        self.FL.stop(); self.FR.stop(); self.RL.stop(); self.RR.stop()

    def _print_status(self):
        duties = getattr(self, '_duties', [0,0,0,0])
        print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}  "
              f"duties=[FL {d
uties[0]:+.2f}, FR {duties[1]:+.2f}, RL {duties[2]:+.2f}, RR {duties[3]:+.2f}]")

# --------------------------- CLI -----------------------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (no ROS2, PCA9685 + H-bridge)')
    # Geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325, help='Wheel radius r (m) (65 mm wheel → 0.0325)')
    p.add_argument('--L', type=float, default=0.115, help='(Lx+Ly)/2 in meters (≈0.115 for ~165 mm track + ~65 mm wheel)')
    # Limits (safe defaults)
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
    # I2C / PCA9685
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x: int(x, 0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0, help='PWM carrier frequency (Hz): 600–1200 typical for H-bridges')
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Comma list of channels to FULL-OFF (servo pins) at startup')
    # Channel mapping
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Polarity & compatibility
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true', help='IN pins are active-LOW on your driver board')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Convention flips (W/S fixed here: flip-vx default True for FNK0043)
    p.add_argument('--flip-vx', action='store_true', default=True, help='Flip forward axis sign (W/S). DEFAULT: True')
    p.add_argument('--flip-vy', action='store_true', default=False, help='Flip strafe axis sign')
    p.add_argument('--flip-omega', action='store_true', default=False, help='Flip yaw sign')
    # Safety (software)
    p.add_argument('--estop-force', action='store_true', help='Force software e-stop (zero all outputs)')
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
