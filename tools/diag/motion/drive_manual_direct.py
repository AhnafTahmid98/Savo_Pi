#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for DC Motors via PCA9685 + H-Bridge
---------------------------------------------------------------------------
MOTOR-ONLY (no servo control). Safe for FNK0043:
- Motor channels allowed: 0..7
- Servo rows: 8..15 (reserved/off by default)

Keys:
  W/S = forward/back
  A/D = strafe (side-paired boards will not truly strafe; this is blended)
  Q/E = turn CCW/CW
  Space = stop
  G = gentle forward pulse
  1..4 = pulse FL/FR/RL/RR forward
  5..8 = pulse FL/FR/RL/RR reverse
  M = print current command + wheel duties
  R = reset speed scale
  ESC / Ctrl+C = quit

Tip: Keep robot on blocks when testing. Start slow.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------------------- I2C ----------------------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install with: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 800.0, set_freq: bool = True):
        self.addr = int(addr)
        self.bus = SMBus(int(bus))
        # OUTDRV push-pull, auto-increment, clear ALLCALL
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

    def set_pwm_freq(self, f_hz: float):
        f = float(max(40.0, min(1500.0, f_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")

    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8)&0x0F)
        self._w8(base+2, off&0xFF);  self._w8(base+3, (off>>8)&0x0F)

    def set_duty(self, ch: int, duty: float):
        """duty in [0..1]"""
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:
            self.full_off(ch)
        elif off >= 4095:
            self.full_on(ch)
        else:
            self._raw(ch, 0, off)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0); self._w8(base+2,0); self._w8(base+3,0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0x10); self._w8(base+2,0); self._w8(base+3,0)

# ---------------------- H-bridge + wheels ----------------------
@dataclass
class MotorCH:
    pwm: int
    in1: int
    in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False, name=""):
        self.pca = pca; self.ch = ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap_in12 = bool(swap_in12)
        self.name = name or "M"
        self.stop()

    def _digital(self, channel: int, level: int):
        if self.in_active_low:
            level ^= 1
        (self.pca.full_on if level else self.pca.full_off)(channel)

    def drive(self, duty_signed: float):
        d = max(-1.0, min(1.0, float(duty_signed)))
        if self.invert: d = -d
        a,b = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)
        if abs(d) < 1e-3:
            self.stop(); return
        if d > 0:
            self._digital(a,1); self._digital(b,0); self.pca.set_duty(self.ch.pwm, d)
        else:
            self._digital(a,0); self._digital(b,1); self.pca.set_duty(self.ch.pwm, -d)

    def stop(self):
        a,b = (self.ch.in1, self.ch.in2) if not self.swap_in12 else (self.ch.in2, self.ch.in1)
        self._digital(a,0); self._digital(b,0); self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- keyboard ----------------------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r,_,_ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd,1) if r else None

    def read(self):
        out=[]; ch=self._read1(0.0)
        while ch is not None:
            if ch==b'\x1b':
                a=self._read1(0.02)
                if a in (b'[',b'O'):
                    b=self._read1(0.04)
                    arrows={b'A':b'UP',b'B':b'DOWN',b'C':b'RIGHT',b'D':b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b'\x1b'); 
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------------------- math helpers ----------------------
def clamp(x, lo, hi): return max(lo, min(hi, x))
def step(curr, target, max_step): return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

@dataclass
class MecanumGeom:
    r: float
    L: float

# ---------------------- main teleop ----------------------
class DirectTeleop:
    def __init__(self, args):
        self.hz = float(args.hz)
        self.deadman = float(args.deadman)
        self.idle_exit = float(args.idle_exit) if args.idle_exit > 0 else None
        self.scale_low = float(args.scale_low)
        self.scale_high = float(args.scale_high)
        self.scale = 1.0
        self.lim = Limits(args.max_vx, args.max_vy, args.max_omega,
                          args.accel_x, args.accel_y, args.accel_z)
        self.geom = MecanumGeom(args.wheel_radius, args.L)

        # flips (defaults for FNK0043: W=Forward)
        self.sx = -1.0 if args.flip_vx else +1.0
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0  # target commands
        self.c_vx = self.c_vy = self.c_wz = 0.0  # current (ramped)
        self._last = time.monotonic()
        self._running = True

        # PCA + reserve channels (servos 8..15 OFF by default)
        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq, set_freq=args.set_freq)
        try:
            reserved = [int(x) for x in str(args.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved OFF: {reserved}")
        except Exception as e:
            print(f"[Teleop] Warning: reserve failed: {e}")

        # Motors
        self.FL = HBridge(self.pca, MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2),
                          invert=args.inv_fl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fl, name="FL")
        self.FR = HBridge(self.pca, MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2),
                          invert=args.inv_fr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_fr, name="FR")
        self.RL = HBridge(self.pca, MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2),
                          invert=args.inv_rl, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rl, name="RL")
        self.RR = HBridge(self.pca, MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2),
                          invert=args.inv_rr, in_active_low=args.in_active_low, swap_in12=args.swap_in12_rr, name="RR")

        # Channel safety
        self._check_channels(args)

        self.estop_force = bool(args.estop_force)

        print("\n[WIRING] Wheel : PWM IN1 IN2 | invert swap active_low")
        print(f"[WIRING] FL    : {args.fl_pwm:>3} {args.fl_in1:>3} {args.fl_in2:>3} | {bool(args.inv_fl)!s:5} {bool(args.swap_in12_fl)!s:4} {bool(args.in_active_low)!s}")
        print(f"[WIRING] FR    : {args.fr_pwm:>3} {args.fr_in1:>3} {args.fr_in2:>3} | {bool(args.inv_fr)!s:5} {bool(args.swap_in12_fr)!s:4} {bool(args.in_active_low)!s}")
        print(f"[WIRING] RL    : {args.rl_pwm:>3} {args.rl_in1:>3} {args.rl_in2:>3} | {bool(args.inv_rl)!s:5} {bool(args.swap_in12_rl)!s:4} {bool(args.in_active_low)!s}")
        print(f"[WIRING] RR    : {args.rr_pwm:>3} {args.rr_in1:>3} {args.rr_in2:>3} | {bool(args.inv_rr)!s:5} {bool(args.swap_in12_rr)!s:4} {bool(args.in_active_low)!s}\n")

        # Signals
        def _sig(_s,_f): self._running=False
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print(
            "[Teleop] READY (no ROS2). Esc/Ctrl+C to quit.\n"
            f" rate={self.hz}Hz  deadman={self.deadman}s  idle_exit={self.idle_exit or 0}s\n"
            f" max(vx,vy,wz)=({self.lim.vx},{self.lim.vy},{self.lim.wz})  accel=({self.lim.ax},{self.lim.ay},{self.lim.az})\n"
            f" geom: r={self.geom.r}m L={self.geom.L}m  PCA=0x{args.pca_addr:02X} bus={args.i2c_bus} freq={args.pwm_freq}Hz\n"
            f" flips: vx={'-vx' if args.flip_vx else 'vx'} vy={'-vy' if args.flip_vy else 'vy'} ω={'-ω' if args.flip_omega else 'ω'}\n"
            " Keys: WASD/QE, Space stop, G gentle, 1..8 pulses, M print, R reset scale\n"
        )

    # ---------------- safety checks ----------------
    def _check_channels(self, args):
        reserved = set(int(x) for x in str(args.reserve).split(',') if x.strip()!='')
        motors = {
            "FL": MotorCH(args.fl_pwm, args.fl_in1, args.fl_in2),
            "FR": MotorCH(args.fr_pwm, args.fr_in1, args.fr_in2),
            "RL": MotorCH(args.rl_pwm, args.rl_in1, args.rl_in2),
            "RR": MotorCH(args.rr_pwm, args.rr_in1, args.rr_in2),
        }
        # 1) enforce motor channels only in 0..7
        for name, chs in motors.items():
            for label, ch in (("PWM",chs.pwm),("IN1",chs.in1),("IN2",chs.in2)):
                if ch < 0 or ch > 7:
                    raise ValueError(f"[ERROR] {name} {label}={ch} out of motor-safe range 0..7 (8..15 are servos).")
                if ch in reserved:
                    raise ValueError(f"[ERROR] {name} {label}={ch} is reserved ({sorted(reserved)}).")
        # 2) PWM must be unique per SIDE only (left uses 0, right uses 6 typically); never reused as IN
        pwms = [args.fl_pwm, args.fr_pwm, args.rl_pwm, args.rr_pwm]
        ins  = [args.fl_in1, args.fl_in2, args.fr_in1, args.fr_in2, args.rl_in1, args.rl_in2, args.rr_in1, args.rr_in2]
        bad = sorted(set(pwms) & set(ins))
        if bad:
            raise ValueError(f"[ERROR] PWM channels {bad} also used as IN pins — not allowed.")
        # 3) Warn if IN pins are shared across motors (side-paired boards will do this)
        per_motor = {
            "FL": {args.fl_in1, args.fl_in2},
            "FR": {args.fr_in1, args.fr_in2},
            "RL": {args.rl_in1, args.rl_in2},
            "RR": {args.rr_in1, args.rr_in2},
        }
        shared = {}
        names = list(per_motor.keys())
        for i in range(len(names)):
            for j in range(i+1, len(names)):
                a, b = names[i], names[j]
                overlap = per_motor[a] & per_motor[b]
                if overlap:
                    shared.setdefault(tuple(sorted(overlap)), set()).update([a, b])
        if shared:
            print(f"[WARN] Some IN pin(s) are shared across motors (side-paired). Mecanum strafe may be blended: {shared}")

    # ---------------- main loop ----------------
    def close(self):
        print("[Teleop] Shutting down safely...")
        try:
            for m in (self.FL, self.FR, self.RL, self.RR):
                try: m.stop()
                except: pass
            self.kb.restore()
            self.pca.close()
        except Exception as e:
            print(f"[Teleop] Close warning: {e}")

    def loop(self):
        period = 1.0 / max(1.0, self.hz)
        try:
            while self._running:
                now = time.monotonic()

                # input
                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch):  # request quit
                        self._running = False
                        break
                    self.last_input = now

                # deadman + optional idle-exit
                if not got:
                    if (now - self.last_input) > self.deadman:
                        if (self.t_vx, self.t_vy, self.t_wz) != (0.0, 0.0, 0.0):
                            print("[Deadman] stop")
                        self.t_vx = self.t_vy = self.t_wz = 0.0

                # scale + limits
                vx_t = clamp(self.t_vx * self.scale, -self.lim.vx, self.lim.vx)
                vy_t = clamp(self.t_vy * self.scale, -self.lim.vy, self.lim.vy)
                wz_t = clamp(self.t_wz * self.scale, -self.lim.wz, self.lim.wz)

                # ramps
                dt = max(1e-3, now - self._last); self._last = now
                self.c_vx = step(self.c_vx, vx_t, self.lim.ax * dt)
                self.c_vy = step(self.c_vy, vy_t, self.lim.ay * dt)
                self.c_wz = step(self.c_wz, wz_t, self.lim.az * dt)

                if self.estop_force:
                    self.c_vx = self.c_vy = self.c_wz = 0.0

                # apply
                self._apply(self.c_vx, self.c_vy, self.c_wz)

                # pace
                sleep_t = period - (time.monotonic() - now)
                if sleep_t > 0: time.sleep(sleep_t)
        finally:
            self.close()

    # ---------------- controls ----------------
    def _handle_key(self, ch) -> bool:
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True

        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0
            print("[Cmd] STOP"); return False

        if ch in (b'r', b'R', b"\x12"):
            self.scale = 1.0; print("[Scale] reset → 1.0"); return False

        if ch in (b'g', b'G'):
            self._gentle_forward(); return False

        if ch in (b'm', b'M'):
            self._print_status(); return False

        # scale via CTRL (slow) or UPPERCASE (fast)
        slow = (isinstance(ch,bytes) and len(ch)==1 and 1 <= ch[0] <= 26)
        fast = (isinstance(ch,bytes) and ch.isalpha() and ch.isupper())
        self.scale = clamp(self.scale_low if slow else (self.scale_high if fast else 1.0), 0.05, 3.0)

        # motion
        if ch in (b'w', b'W', b'UP'):    self.t_vx = +self.lim.vx; print("[Cmd] FORWARD")
        elif ch in (b's', b'S', b'DOWN'):self.t_vx = -self.lim.vx; print("[Cmd] BACK")
        elif ch in (b'a', b'A', b'LEFT'):self.t_vy = +self.lim.vy; print("[Cmd] LEFT (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):self.t_vy = -self.lim.vy; print("[Cmd] RIGHT (strafe)")
        elif ch in (b'q', b'Q'):         self.t_wz = +self.lim.wz; print("[Cmd] TURN CCW")
        elif ch in (b'e', b'E'):         self.t_wz = -self.lim.wz; print("[Cmd] TURN CW")
        elif ch in (b'x', b'X'):         self.t_vx = self.t_vy = 0.0; print("[Cmd] HALT XY")
        elif ch in (b'z', b'Z'):         self.t_wz = 0.0; print("[Cmd] HALT YAW")
        elif ch == b'1': self._pulse_wheels([1,0,0,0]);        return False
        elif ch == b'2': self._pulse_wheels([0,1,0,0]);        return False
        elif ch == b'3': self._pulse_wheels([0,0,1,0]);        return False
        elif ch == b'4': self._pulse_wheels([0,0,0,1]);        return False
        elif ch == b'5': self._pulse_wheels([1,0,0,0], -0.12); return False
        elif ch == b'6': self._pulse_wheels([0,1,0,0], -0.12); return False
        elif ch == b'7': self._pulse_wheels([0,0,1,0], -0.12); return False
        elif ch == b'8': self._pulse_wheels([0,0,0,1], -0.12); return False

        return False

    def _apply(self, vx, vy, wz):
        # flips
        vx *= self.sx; vy *= self.sy; wz *= self.so

        # mecanum mix (geometric form)
        r = self.geom.r; L = self.geom.L
        w_fl = ( vx - vy - L*wz ) / r
        w_fr = ( vx + vy + L*wz ) / r
        w_rl = ( vx + vy - L*wz ) / r
        w_rr = ( vx - vy + L*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]

        max_w = max(1e-6, max(abs(w) for w in ws))
        mag = max(abs(vx)/max(self.lim.vx,1e-6),
                  abs(vy)/max(self.lim.vy,1e-6),
                  abs(wz)/max(self.lim.wz,1e-6))
        duties = [clamp((w/max_w)*mag, -1.0, 1.0) for w in ws]
        self._duties = duties

        try:
            self.FL.drive(duties[0])
            self.FR.drive(duties[1])
            self.RL.drive(duties[2])
            self.RR.drive(duties[3])
        except Exception as e:
            print(f"[Teleop] Motor drive error: {e}")

    def _pulse_wheels(self, mask, duty=0.12, t=0.30):
        try:
            motors = [self.FL, self.FR, self.RL, self.RR]
            for i, m in enumerate(motors):
                if mask[i]: m.drive(duty)
            time.sleep(t)
            for m in motors: m.stop()
        except Exception as e:
            print(f"[Teleop] Wheel pulse error: {e}")

    def _gentle_forward(self, duty=0.10, t=0.50):
        print("[Test] Gentle forward pulse")
        try:
            for m in (self.FL, self.FR, self.RL, self.RR): m.drive(duty)
            time.sleep(t)
            for m in (self.FL, self.FR, self.RL, self.RR): m.stop()
        except Exception as e:
            print(f"[Teleop] Gentle forward error: {e}")

    def _print_status(self):
        d = getattr(self, "_duties", [0,0,0,0])
        print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}  "
              f"duties=[FL {d[0]:+.2f}, FR {d[1]:+.2f}, RL {d[2]:+.2f}, RR {d[3]:+.2f}]")

# ---------------------- CLI ----------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (no ROS2, PCA9685 + H-bridge)')

    # geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325)
    p.add_argument('--L', type=float, default=0.115)

    # speeds (LOW by default)
    p.add_argument('--max-vx', type=float, default=0.10)
    p.add_argument('--max-vy', type=float, default=0.10)
    p.add_argument('--max-omega', type=float, default=0.45)

    # ramps
    p.add_argument('--accel-x', type=float, default=0.8)
    p.add_argument('--accel-y', type=float, default=0.8)
    p.add_argument('--accel-z', type=float, default=1.2)

    # timing
    p.add_argument('--hz', type=float, default=20.0)
    p.add_argument('--deadman', type=float, default=0.40)
    p.add_argument('--idle-exit', type=float, default=0.0, help="Auto-exit after SEC idle (0=off)")

    # scaling shortcuts
    p.add_argument('--scale-low', type=float, default=0.25)
    p.add_argument('--scale-high', type=float, default=1.35)

    # I2C/PCA
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', dest='set_freq', action='store_true', default=True)
    p.add_argument('--no-set-freq', dest='set_freq', action='store_false')
    # reserve all servo pins by default (8..15)
    p.add_argument('--reserve', type=str, default='8,9,10,11,12,13,14,15')
    p.add_argument('--in-active-low', action='store_true')

    # channels (REQUIRED; must be 0..7)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)

    # polarity helpers
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')

    # motion flips (defaults chosen for FNK0043: “W=forward”)
    p.add_argument('--flip-vx', action='store_true', default=True)
    p.add_argument('--flip-vy', action='store_true', default=False)
    p.add_argument('--flip-omega', action='store_true', default=False)

    # hard e-stop freeze
    p.add_argument('--estop-force', action='store_true')
    return p

def main(argv=None):
    args = build_argparser().parse_args(argv)
    t = None
    try:
        t = DirectTeleop(args)
        t.loop()
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt")
    finally:
        if t: t.close()
    print("[Main] Exit complete")

if __name__ == '__main__':
    main()
