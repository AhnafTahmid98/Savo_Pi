#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for Mecanum (PCA9685 + H-Bridge)
- Fix: correct PCA9685 FULL-ON / FULL-OFF usage for IN pins (forward now works).
- Console cues for FORWARD/BACK/LEFT/RIGHT/TURN + deadman message.
"""

import argparse, os, select, sys, termios, time, tty, signal
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
    def __init__(self, bus: int = 1, addr: int = 0x40, freq_hz: float = 1000.0,
                 retries: int = 8, retry_delay: float = 0.004):
        self.addr = addr
        self.retries = int(max(1, retries))
        self.retry_delay = float(max(0.0, retry_delay))
        self.bus = SMBus(int(bus))
        self._write8(MODE1, 0x00)  # normal mode
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)

    def _xfer(self, fn, *args):
        last = None
        for _ in range(self.retries):
            try:
                return fn(*args)
            except (BlockingIOError, OSError) as e:
                last = e
                time.sleep(self.retry_delay)
        if last: raise last
        raise RuntimeError("I2C transfer failed")

    def _write8(self, reg, val):
        reg = int(reg) & 0xFF
        val = int(val) & 0xFF
        return self._xfer(self.bus.write_byte_data, self.addr, reg, val)

    def _read8(self, reg):
        reg = int(reg) & 0xFF
        return self._xfer(self.bus.read_byte_data, self.addr, reg)

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
    """
    Correct digital control of IN pins:
      - FULL ON  => set_pwm(ch, 4096, 0)
      - FULL OFF => set_pwm(ch, 0, 4096)
    Active-LOW boards invert the meaning of 'assert'.
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
        # level 1=assert, 0=deassert (before active-low inversion)
        if self.in_active_low:
            level ^= 1
        if level:  # assert → FULL ON
            self.pca.set_pwm(channel, 4096, 0)
        else:      # deassert → FULL OFF
            self.pca.set_pwm(channel, 0, 4096)

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
        # Brake-off (coast): both deasserted
        self._digital(inA, 0); self._digital(inB, 0)
        self.pca.set_duty(self.ch.pwm, 0.0)

# ---------------------- Keyboard + helpers --------------------------
class Keyboard:
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
                a = self._read1(0.02)
                if a == b'[':
                    b = self._read1(0.02)
                    if b in (b'A', b'B', b'C', b'D'):
                        out.append({b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}[b])
                    else:
                        out.extend(filter(None, [b"\x1b", a, b]))
                else:
                    out.append(b"\x1b")
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def clamp(x, lo, hi): return max(lo, min(hi, x))
def step(curr, target, max_step):
    return min(curr + max_step, target) if target > curr else max(curr - max_step, target)

# -------------------------- Teleop core -----------------------------
@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

@dataclass
class MecanumGeom:
    r: float
    L: float

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
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0

        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        self._last = time.monotonic()

        self.pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq,
                           retries=args.i2c_retries, retry_delay=args.i2c_delay)
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
            f" flips: vy={'-vy' if args.flip_vy else 'vy'}  omega={'-ω' if args.flip_omega else 'ω'}\n"
            f" IN logic: {'ACTIVE-LOW' if args.in_active_low else 'ACTIVE-HIGH'}  swap(FL/FR/RL/RR)={args.swap_in12_fl}/{args.swap_in12_fr}/{args.swap_in12_rl}/{args.swap_in12_rr}\n"
            " Keys: WASD/QE or Arrows, 1..4/5..8 wheel test, G gentle forward, M print, Space stop, R reset\n"
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
            if hasattr(self.pca, 'bus') and self.pca.bus: self.pca.bus.close()
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
        if ch in (b"\x1b", b"\x03"): print("[Teleop] Quit"); return True
        if ch == b' ':
            self.t_vx = self.t_vy = self.t_wz = 0.0; self._announce("[Cmd] STOP"); return False
        if ch in (b'r', b'R', b"\x12"):
            self.scale = 1.0; self._announce("[Scale] reset → 1.0"); return False

        # Diagnostics
        if ch == b'g' or ch == b'G': self._gentle_forward(); return False
        if ch == b'm' or ch == b'M': self._print_status(); return False

        # Speed scaling (Shift=fast, Ctrl=slow)
        if isinstance(ch, bytes) and len(ch) == 1:
            slow = 1 <= ch[0] <= 26
            fast = ch.isalpha() and ch.isupper()
        else:
            slow = False; fast = False
        if slow: self.scale = clamp(self.scale_low, 0.05, 1.0)
        elif fast: self.scale = clamp(self.scale_high, 1.0, 3.0)
        else: self.scale = 1.0

        # Movement + cues
        if ch in (b'w', b'W', b'UP'):
            self.t_vx = +self.lim.vx; self.t_vy = 0.0; self._announce("[Cmd] FORWARD")
        elif ch in (b's', b'S', b'DOWN'):
            self.t_vx = -self.lim.vx; self.t_vy = 0.0; self._announce("[Cmd] BACK")
        elif ch in (b'a', b'A', b'LEFT'):
            self.t_vy = +self.lim.vy; self.t_vx = 0.0; self._announce("[Cmd] LEFT")
        elif ch in (b'd', b'D', b'RIGHT'):
            self.t_vy = -self.lim.vy; self.t_vx = 0.0; self._announce("[Cmd] RIGHT")
        elif ch in (b'q', b'Q'):
            self.t_wz = +self.lim.wz; self._announce("[Cmd] TURN CCW")
        elif ch in (b'e', b'E'):
            self.t_wz = -self.lim.wz; self._announce("[Cmd] TURN CW")
        elif ch in (b'x', b'X'):
            self.t_vx = 0.0; self.t_vy = 0.0; self._announce("[Cmd] HALT XY")
        elif ch in (b'z', b'Z'):
            self.t_wz = 0.0; self._announce("[Cmd] HALT YAW")
        return False

    def _apply(self, vx, vy, wz):
        r = self.geom.r; L = self.geom.L; sy = self.sy; so = self.so
        w_fl = ( vx - sy*vy - L*so*wz ) / r
        w_fr = ( vx + sy*vy + L*so*wz ) / r
        w_rl = ( vx + sy*vy - L*so*wz ) / r
        w_rr = ( vx - sy*vy + L*so*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        mag = max(abs(vx)/max(self.lim.vx,1e-6),
                  abs(vy)/max(self.lim.vy,1e-6),
                  abs(wz)/max(self.lim.wz,1e-6))
        duties = [clamp((w/max_w) * mag, -1.0, 1.0) for w in ws]
        self._duties = duties
        self.FL.drive(duties[0]); self.FR.drive(duties[1]); self.RL.drive(duties[2]); self.RR.drive(duties[3])

    def _gentle_forward(self, duty=0.25, t=0.7):
        self._announce("[Test] Gentle forward pulse")
        self.FL.drive(duty); self.FR.drive(duty); self.RL.drive(duty); self.RR.drive(duty)
        time.sleep(t)
        self.FL.stop(); self.FR.stop(); self.RL.stop(); self.RR.stop()

    def _print_status(self):
        duties = getattr(self, '_duties', [0,0,0,0])
        print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f}  "
              f"duties=[FL {duties[0]:+.2f}, FR {duties[1]:+.2f}, RL {duties[2]:+.2f}, RR {duties[3]:+.2f}]")

# --------------------------- CLI -----------------------------------
def build_argparser():
    p = argparse.ArgumentParser(description='Robot Savo — Manual Teleop (no ROS2, PCA9685)')
    # Geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325)
    p.add_argument('--L', type=float, default=0.115)
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
    p.add_argument('--pwm-freq', type=float, default=1000.0)
    p.add_argument('--i2c-retries', type=int, default=8)
    p.add_argument('--i2c-delay', type=float, default=0.004)
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
    # Convention flips
    p.add_argument('--flip-vy', action='store_true'); p.add_argument('--flip-omega', action='store_true')
    # Safety (software)
    p.add_argument('--estop-force', action='store_true')
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
