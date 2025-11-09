#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (FNK0043 table-accurate, DC motors only)
-------------------------------------------------------------------
MOTOR-ONLY (no servo control). Implements the Freenove mecanum mixer:

  FR =  VY - VX + W
  FL =  VY + VX - W
  BL =  VY - VX - W
  BR =  VY + VX + W

Key mapping (logical commands):
  W / ↑  : Forward
  S / ↓  : Backward
  A / ←  : Strafe Left
  D / →  : Strafe Right
  Q      : Turn Left  (CCW)
  E      : Turn Right (CW)
  SPACE  : Stop immediately
  M      : Print current duties
  1..4   : Pulse FL/FR/RL/RR forward
  5..8   : Pulse FL/FR/RL/RR reverse
  G      : Gentle forward pulse (all wheels small +)
  Shift  : Fast scale  |  Ctrl  : Slow scale
  R      : Reset scale to 1.0
  Esc / Ctrl+C : Quit (safe stop)

Notes
-----
• This script NEVER writes to servo channels. It explicitly FULL-OFFs any
  channels passed via --reserve (default: 12,13,14,15).
• The PCA9685 PWM frequency is left unchanged unless you pass --set-freq.
• Use --flip-vx/--flip-vy/--flip-omega if your axes feel reversed.
• Use --inv-*/--swap-in12-* to fix per-wheel wiring polarity.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------------- PCA9685 low-level ----------------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    """
    Minimal PCA9685 helper:
      - OUTDRV push-pull, Auto-Increment, ALLCALL cleared.
      - set_pwm_freq() only if requested by --set-freq.
      - full_on/full_off provide clean digital levels for IN pins.
    """
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=False):
        self.addr = int(addr)
        self.bus  = SMBus(int(bus))
        # Configure outputs & auto-increment; clear ALLCALL
        self._w8(MODE2, OUTDRV)
        self._w8(MODE1, AI)                 # (clears ALLCALL bit)
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        # ensure restart + AI; keep ALLCALL cleared
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    # I²C helpers
    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)  # sleep to change prescale
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)              # wake
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
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)   # OFF_H full-off bit

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)   # ON_H full-on bit
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

# ------------------------- Motor abstraction ------------------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    """
    H-bridge with PCA9685:
      - Direction via IN1/IN2 full-on/full-off (active-HIGH or LOW).
      - Speed via PWM duty on 'pwm'.
      - swap_in12 flips the two direction lines if wired cross-wise.
      - invert flips the motor sign convention if mechanically reversed.
    """
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.stop()

    def _set_in(self, channel: int, level: int):
        # level: 1 = assert, 0 = deassert (before active-low XOR)
        if self.in_active_low: level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(channel)

    def _dir(self, a_on: int, b_on: int):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        self._set_in(a, a_on); self._set_in(b, b_on)

    def drive(self, signed_duty: float):
        d = max(-1.0, min(1.0, float(signed_duty)))
        if self.invert: d = -d
        if abs(d) < 1e-3:
            self.stop(); return
        if d > 0:
            self._dir(1, 0); self.pca.set_duty(self.ch.pwm, d)
        else:
            self._dir(0, 1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        self._dir(0, 0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- Keyboard (robust arrows) --------------------
class Keyboard:
    """Non-blocking keyboard reader that parses arrows (CSI & SS3)."""
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
            if ch == b"\x1b":
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                    else:
                        if a: out.append(a)
                        if b: out.append(b)
                else:
                    out.append(b"\x1b")
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------------- Helpers & limits ---------------------------
def clamp(x, lo, hi): return max(lo, min(hi, x))
def step(curr, target, max_step):
    return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

# ---------------------- Teleop core (Freenove mixer) ---------------
class Teleop:
    def __init__(self, args):
        self.hz = float(args.hz)
        self.deadman = float(args.deadman)
        self.scale_low, self.scale_high = float(args.scale_low), float(args.scale_high)
        self.scale = 1.0
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega,
                          args.accel_x, args.accel_y, args.accel_z)

        # Axis sign flips; on FNK0043 we default flip-vx=True to make W/↑ = forward
        self.sx = -1.0 if args.flip_vx else +1.0
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        # PCA
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr,
                           freq_hz=args.pwm_freq, set_freq=args.set_freq)

        # Reserve channels (e.g., 12–15 are servo headers); keep them FULL-OFF
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip() != ""]
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved channels FULL-OFF: {reserved}")
        except Exception:
            pass

        # Motors: FL, FR, RL, RR (M1..M4)
        self.FL = HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2),
                          invert=args.inv_fl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fl)
        self.FR = HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2),
                          invert=args.inv_fr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fr)
        self.RL = HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2),
                          invert=args.inv_rl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rl)
        self.RR = HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2),
                          invert=args.inv_rr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rr)

        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._duties = [0.0, 0.0, 0.0, 0.0]
        self._last = time.monotonic()

        signal.signal(signal.SIGINT,  self._signal_quit)
        signal.signal(signal.SIGTERM, self._signal_quit)

        print("[Teleop] READY (motor-only). W/S/A/D/Q/E; Space stop; M print; 1..8 pulses; Esc quits.")

    # ---------- lifecycle ----------
    def _signal_quit(self, *_):
        print("\n[Teleop] SIGINT/SIGTERM → safe stop")
        self.close(); os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in (self.FL, self.FR, self.RL, self.RR):
            try: m.stop()
            except Exception: pass
        try: self.pca.close()
        except Exception: pass

    # ---------- mixing & driving ----------
    def _apply(self, vx, vy, wz):
        # Apply axis flips (so user-facing commands stay intuitive)
        VX = vy * self.sy   # Freenove's "VX" is our strafe
        VY = vx * self.sx   # Freenove's "VY" is our forward/back
        W  = wz * self.so   # CCW positive

        # Freenove mixer (table-accurate)
        FR =  VY - VX + W
        FL =  VY + VX - W
        BL =  VY - VX - W
        BR =  VY + VX + W

        vec = [FL, FR, BL, BR]
        maxmag = max(1.0, max(abs(x) for x in vec))
        duties = [clamp(x / maxmag, -1.0, 1.0) for x in vec]
        self._duties = duties

        self.FL.drive(duties[0]); self.FR.drive(duties[1])
        self.RL.drive(duties[2]); self.RR.drive(duties[3])

    # ---------- UI helpers ----------
    def _print_status(self):
        d = self._duties
        print(f"[duties] FL {d[0]:+0.2f}  FR {d[1]:+0.2f}  RL {d[2]:+0.2f}  RR {d[3]:+0.2f} | "
              f"cmd vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}")

    def _pulse(self, mask, duty, t=0.35):
        motors = [self.FL, self.FR, self.RL, self.RR]
        for i, m in enumerate(motors): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in motors: m.stop()

    def _gentle_forward(self, duty=0.10, t=0.40):
        print("[Test] gentle forward pulse")
        for m in (self.FL, self.FR, self.RL, self.RR): m.drive(duty)
        time.sleep(t)
        for m in (self.FL, self.FR, self.RL, self.RR): m.stop()

    # ---------- key handling ----------
    def _handle_key(self, ch):
        if ch in (b"\x1b", b"\x03"):  # ESC / Ctrl+C
            print("[Teleop] Quit"); return True
        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0; print("[Cmd] STOP"); return False
        if ch in (b'm', b'M'): self._print_status(); return False
        if ch in (b'g', b'G'): self._gentle_forward(); return False
        if ch in (b'r', b'R'): self.scale = 1.0; print("[Scale] reset → 1.0"); return False

        # speed scaling (Ctrl=slow, Shift=fast)
        slow = isinstance(ch, bytes) and len(ch)==1 and (1 <= ch[0] <= 26)
        fast = isinstance(ch, bytes) and ch.isalpha() and ch.isupper()
        self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)

        # directional intents
        if ch in (b'w', b'W', b'UP'):    self.t_vx = +self.lim.vx; print("[Cmd] FORWARD")
        elif ch in (b's', b'S', b'DOWN'): self.t_vx = -self.lim.vx; print("[Cmd] BACK")
        elif ch in (b'a', b'A', b'LEFT'): self.t_vy = +self.lim.vy; print("[Cmd] LEFT (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):self.t_vy = -self.lim.vy; print("[Cmd] RIGHT (strafe)")
        elif ch in (b'q', b'Q'):          self.t_wz = +self.lim.wz; print("[Cmd] TURN CCW")
        elif ch in (b'e', b'E'):          self.t_wz = -self.lim.wz; print("[Cmd] TURN CW")
        elif ch in (b'x', b'X'):          self.t_vx = self.t_vy = 0.0; print("[Cmd] zero VX/VY")
        elif ch in (b'z', b'Z'):          self.t_wz = 0.0; print("[Cmd] zero WZ")
        # wheel pulses (diagnostics)
        elif ch == b'1': self._pulse([1,0,0,0], +0.5)
        elif ch == b'2': self._pulse([0,1,0,0], +0.5)
        elif ch == b'3': self._pulse([0,0,1,0], +0.5)
        elif ch == b'4': self._pulse([0,0,0,1], +0.5)
        elif ch == b'5': self._pulse([1,0,0,0], -0.5)
        elif ch == b'6': self._pulse([0,1,0,0], -0.5)
        elif ch == b'7': self._pulse([0,0,1,0], -0.5)
        elif ch == b'8': self._pulse([0,0,0,1], -0.5)
        return False

    # ---------- main loop ----------
    def loop(self):
        period = 1.0 / self.hz
        try:
            while True:
                t0 = time.monotonic()
                got = False
                for ch in Keyboard.read(self.kb):
                    got = True
                    if self._handle_key(ch): return
                    self.last_input = t0

                if not got and (t0 - self.last_input) > self.deadman:
                    if (self.t_vx, self.t_vy, self.t_wz) != (0.0, 0.0, 0.0):
                        print("[Deadman] timeout → stop")
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                # acceleration limiting + scaling
                dt = max(1e-3, t0 - getattr(self, "_tlast", t0)); self._tlast = t0
                vx_t = clamp(self.t_vx * self.scale, -self.lim.vx, self.lim.vx)
                vy_t = clamp(self.t_vy * self.scale, -self.lim.vy, self.lim.vy)
                wz_t = clamp(self.t_wz * self.scale, -self.lim.wz, self.lim.wz)
                self.c_vx = step(self.c_vx, vx_t, self.lim.ax * dt)
                self.c_vy = step(self.c_vy, vy_t, self.lim.ay * dt)
                self.c_wz = step(self.c_wz, wz_t, self.lim.az * dt)

                self._apply(self.c_vx, self.c_vy, self.c_wz)

                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# --------------------------- CLI -----------------------------------
def build_argparser():
    p = argparse.ArgumentParser(description="Robot Savo — DC-motor teleop (Freenove mixer, motor-only)")
    # motion limits (safe)
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
    # PCA/I2C
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0, help='Only applied if --set-freq given')
    p.add_argument('--set-freq', action='store_true', help='Actually change PCA9685 frequency')
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Comma list of channels to FULL-OFF (servo headers)')
    # Channels (M1..M4 = FL, FR, RL, RR)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Polarity & compatibility
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true', help='IN pins active-LOW on your driver board')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Axis flips (W/↑ = forward on FNK0043 → flip-vx DEFAULT True)
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
