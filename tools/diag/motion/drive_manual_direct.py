#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for 4 DC motors via PCA9685 + H-Bridge
-----------------------------------------------------------------------------
Designed for mecanum “X” layout where FORWARD should be:
  M2 & M3 = forward,  M1 & M4 = backward  (default wheel-signs = -1,+1,+1,-1)

Features
- Per-wheel sign map (--wheel-signs fl,fr,rl,rr) to match any roller orientation.
- Robust arrow keys (CSI & SS3). WASD works too.
- Motor-only (never touches servos). Reserved channels are forced OFF.
- Proper H-bridge drive: IN1/IN2 digital (full-on/full-off), PWM for speed.
- Invert flags per wheel; IN active-low and IN1/IN2 swap per wheel.
- Safety: deadman timer, gentle pulse key, clean quit.

Keys
  W/S or ↑/↓  = Forward / Back
  A/D or ←/→  = Strafe Left / Right
  Q/E         = Turn CCW / CW
  G           = Gentle forward pulse
  1..4 / 5..8 = Pulse FL/FR/RL/RR forward / reverse
  M           = Print current command + wheel duties
  SPACE       = Stop immediately
  Esc / Ctrl+C= Quit (safe)
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
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._write8(MODE2, OUTDRV)              # push-pull
        self._write8(MODE1, AI)                  # auto-inc, (clears ALLCALL)
        time.sleep(0.003)
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
@dataclass
class MecanumGeom: r: float; L: float

class DirectTeleop:
    def __init__(self, args):
        self.hz, self.deadman = float(args.hz), float(args.deadman)
        self.scale_low, self.scale_high = float(args.scale_low), float(args.scale_high)
        self.scale = 1.0
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega, args.accel_x, args.accel_y, args.accel_z)
        self.geom = MecanumGeom(args.wheel_radius, args.L)

        # Axis flips (leave off; wheel-signs handles forward pattern)
        self.sx = -1.0 if args.flip_vx else +1.0
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        # Per-wheel sign multipliers (fl,fr,rl,rr). Default matches your diagram.
        self.ws_fl, self.ws_fr, self.ws_rl, self.ws_rr = (int(x) for x in args.wheel_signs.split(','))
        for s in (self.ws_fl, self.ws_fr, self.ws_rl, self.ws_rr):
            if s not in (-1, 1): raise ValueError("--wheel-signs must be comma list of -1/1")

        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq)
        # Reserve (servo) channels OFF
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception: pass

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
        signal.signal(signal.SIGINT, self._sig_quit); signal.signal(signal.SIGTERM, self._sig_quit)

        print(
            "[Teleop] READY (4-wheel). Esc/Ctrl+C to quit.\n"
            f" rate={self.hz} Hz  deadman={self.deadman}s  max(vx,vy,wz)=({self.lim.vx},{self.lim.vy},{self.lim.wz})\n"
            f" accel(x,y,z)=({self.lim.ax},{self.lim.ay},{self.lim.az})  scale_low={self.scale_low}  scale_high={self.scale_high}\n"
            f" geom: r={self.geom.r} m, L={self.geom.L} m  PCA9685@0x{args.pca_addr:02X} bus={args.i2c_bus}\n"
            f" wheel-signs (fl,fr,rl,rr)={self.ws_fl},{self.ws_fr},{self.ws_rl},{self.ws_rr}  flips: vx={args.flip_vx} vy={args.flip_vy} ω={args.flip_omega}\n"
            " Keys: WASD/Arrows (move), Q/E (turn), G pulse, SPACE stop, M print\n"
        )

    def _sig_quit(self, *_):
        print("\n[Teleop] SIGINT/SIGTERM → safe stop and exit")
        self.close(); os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        for m in (self.FL, self.FR, self.RL, self.RR):
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

                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

    def _announce(self, s): print(s)

    def _handle_key(self, ch) -> bool:
        if ch in (b"\x1b", b"\x03"): print("[Teleop] Quit"); return True
        if ch == b' ': self.t_vx = self.t_vy = self.t_wz = 0.0; self._announce("[Cmd] STOP"); return False
        if ch in (b'r', b'R', b"\x12"): self.scale = 1.0; self._announce("[Scale] reset → 1.0"); return False
        if ch in (b'g', b'G'): self._gentle_forward(); return False
        if ch in (b'm', b'M'): self._print_status(); return False

        # Speed scaling (Shift=fast, Ctrl=slow)
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
        else:
            slow = False; fast = False
        if slow: self.scale = clamp(self.scale_low, 0.05, 1.0)
        elif fast: self.scale = clamp(self.scale_high, 1.0, 3.0)
        else: self.scale = 1.0

        # Move (logical axes; sign flips and per-wheel signs applied in _apply)
        if ch in (b'w', b'W', b'UP'):     self.t_vx = +self.lim.vx; self._announce("[Cmd] FORWARD")
        elif ch in (b's', b'S', b'DOWN'): self.t_vx = -self.lim.vx; self._announce("[Cmd] BACK")
        elif ch in (b'a', b'A', b'LEFT'): self.t_vy = +self.lim.vy; self._announce("[Cmd] LEFT (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):self.t_vy = -self.lim.vy; self._announce("[Cmd] RIGHT (strafe)")
        elif ch in (b'q', b'Q'):          self.t_wz = +self.lim.wz; self._announce("[Cmd] TURN CCW")
        elif ch in (b'e', b'E'):          self.t_wz = -self.lim.wz; self._announce("[Cmd] TURN CW")
        elif ch in (b'x', b'X'):          self.t_vx = 0.0; self.t_vy = 0.0; self._announce("[Cmd] HALT XY")
        elif ch in (b'z', b'Z'):          self.t_wz = 0.0; self._announce("[Cmd] HALT YAW")
        # Diagnostics pulses
        elif ch == b'1': self._pulse([1,0,0,0]); return False
        elif ch == b'2': self._pulse([0,1,0,0]); return False
        elif ch == b'3': self._pulse([0,0,1,0]); return False
        elif ch == b'4': self._pulse([0,0,0,1]); return False
        elif ch == b'5': self._pulse([1,0,0,0], duty=-0.15); return False
        elif ch == b'6': self._pulse([0,1,0,0], duty=-0.15); return False
        elif ch == b'7': self._pulse([0,0,1,0], duty=-0.15); return False
        elif ch == b'8': self._pulse([0,0,0,1], duty=-0.15); return False
        return False

    def _apply(self, vx, vy, wz):
        # Apply global axis flips
        vx *= self.sx; vy *= self.sy; wz *= self.so

        # Standard mecanum inverse kinematics (X layout baseline)
        r, L = self.geom.r, self.geom.L
        w_fl = ( vx - vy - L*wz ) / r
        w_fr = ( vx + vy + L*wz ) / r
        w_rl = ( vx + vy - L*wz ) / r
        w_rr = ( vx - vy + L*wz ) / r

        # Normalize and apply per-wheel signs so forward matches your diagram
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        mag = max(abs(vx)/max(self.lim.vx,1e-6),
                  abs(vy)/max(self.lim.vy,1e-6),
                  abs(wz)/max(self.lim.wz,1e-6))
        duties = [(w/max_w) * mag for w in ws]

        # Per-wheel sign multipliers:
        duties[0] *= self.ws_fl   # FL
        duties[1] *= self.ws_fr   # FR
        duties[2] *= self.ws_rl   # RL
        duties[3] *= self.ws_rr   # RR

        # Clamp and drive
        duties = [clamp(d,-1.0,1.0) for d in duties]
        self._duties = duties
        self.FL.drive(duties[0]); self.FR.drive(duties[1]); self.RL.drive(duties[2]); self.RR.drive(duties[3])

    def _pulse(self, mask, duty=0.12, t=0.30):
        motors = [self.FL, self.FR, self.RL, self.RR]
        for i, m in enumerate(motors): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in motors: m.stop()

    def _gentle_forward(self, duty=0.10, t=0.50):
        self._announce("[Test] Gentle forward pulse")
        for m in (self.FL,self.FR,self.RL,self.RR): m.drive(duty)
        time.sleep(t)
        for m in (self.FL,self.FR,self.RL,self.RR): m.stop()

    def _print_status(self):
        d = getattr(self, '_duties', [0,0,0,0])
        print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}  "
              f"duties=[FL {d[0]:+.2f}, FR {d[1]:+.2f}, RL {d[2]:+.2f}, RR {d[3]:+.2f}]")

# ---------------------- CLI ----------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (PCA9685 + H-Bridge, 4 independent motors)')
    # Geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325)
    p.add_argument('--L', type=float, default=0.115)
    # Limits
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
    # I2C
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--reserve', type=str, default='12,13,14,15')
    # Channels (4 independent motors)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Polarity & wiring fixes
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Global axis flips (usually keep off if wheel-signs is set)
    p.add_argument('--flip-vx', action='store_true', default=False)
    p.add_argument('--flip-vy', action='store_true', default=False)
    p.add_argument('--flip-omega', action='store_true', default=False)
    # Per-wheel sign multipliers: default = -1,+1,+1,-1 to match your picture
    p.add_argument('--wheel-signs', type=str, default='-1,+1,+1,-1',
                   help='Comma list of -1/1 for (FL,FR,RL,RR); default matches FNK0043 forward pattern (M2,M3 fwd; M1,M4 back)')
    # Safety
    p.add_argument('--estop-force', action='store_true')
    return p

def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = DirectTeleop(args)
    try: t.loop()
    except KeyboardInterrupt: pass
    finally: t.close()

if __name__ == '__main__':
    main()
