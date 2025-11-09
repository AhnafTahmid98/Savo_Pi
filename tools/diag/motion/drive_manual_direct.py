#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (NO ROS2) for Mecanum (PCA9685-only, matches motor_direction_test.py)
- Pure PCA9685 control: each wheel uses 3 channels (PWM, IN1, IN2).
- Direction pins use FULL-ON/FULL-OFF exactly like motor_direction_test.py.
- No servo usage whatsoever unless you pass servo channels by mistake.
- Keys: W/S (fwd/back), A/D (strafe L/R), Q/E (turn CCW/CW), SPACE stop, M status.
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass
from typing import List

# ========================= PCA9685 (same semantics) ===========================
class PCA9685:
    MODE1      = 0x00
    MODE2      = 0x01
    LED0_ON_L  = 0x06
    PRE_SCALE  = 0xFE

    RESTART    = 0x80
    SLEEP      = 0x10
    ALLCALL    = 0x01
    OUTDRV     = 0x04
    AI         = 0x20     # Auto-Increment

    FULL_ON_H  = 0x10
    FULL_OFF_H = 0x10

    def __init__(self, bus, addr: int):
        self.bus  = bus
        self.addr = addr
        # MODE1: ALLCALL + Auto-Increment, wake
        self._write(self.MODE1, self.ALLCALL | self.AI)
        time.sleep(0.005)
        # MODE2: OUTDRV (totem pole)
        self._write(self.MODE2, self.OUTDRV)

    def set_freq(self, hz: int):
        hz = max(40, min(1500, int(hz)))
        prescale = int(round(25_000_000.0 / (4096 * hz)) - 1)
        oldmode = self._read(self.MODE1)
        sleep   = (oldmode & ~self.RESTART) | self.SLEEP
        self._write(self.MODE1, sleep)
        self._write(self.PRE_SCALE, prescale)
        self._write(self.MODE1, oldmode)
        time.sleep(0.005)
        self._write(self.MODE1, oldmode | self.RESTART | self.AI)

    def set_pwm_duty(self, ch: int, duty_12bit: int):
        """0..4095; 0 = full off, 4095 = full on, in-between = PWM."""
        duty = max(0, min(4095, int(duty_12bit)))
        reg  = self.LED0_ON_L + 4 * ch
        if duty == 0:
            # full off
            payload = [0x00, 0x00, 0x00, self.FULL_OFF_H]
        elif duty >= 4095:
            # full on
            payload = [0x00, self.FULL_ON_H, 0x00, 0x00]
        else:
            on_l, on_h = 0x00, 0x00
            off_l      = duty & 0xFF
            off_h      = (duty >> 8) & 0x0F
            payload    = [on_l, on_h, off_l, off_h]
        self.bus.write_i2c_block_data(self.addr, reg, payload)

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

# ============================= Motor driver ===================================
@dataclass
class MotorMap:
    pwm: int
    in1: int
    in2: int
    inv: bool = False
    swap_in12: bool = False

class PCAMecanum:
    """Exact same pin semantics as motor_direction_test: IN1/IN2 FULL-ON/OFF, PWM as duty."""
    def __init__(self, bus: int, addr: int, freq: int,
                 motors: List[MotorMap],
                 in_active_low: bool = False,
                 duty_limit: float = 1.0):
        try:
            from smbus2 import SMBus
        except Exception as e:
            print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus", file=sys.stderr)
            raise
        self.i2c = SMBus(bus)
        self.pca = PCA9685(self.i2c, addr)
        self.pca.set_freq(freq)
        self.motors = motors
        self.in_active_low = bool(in_active_low)
        self.duty_limit = max(0.0, min(1.0, duty_limit))
        self.stop()

    def _digital(self, ch: int, assert_level: int):
        # assert_level: 1=assert, 0=deassert before active-low inversion
        level = assert_level ^ (1 if self.in_active_low else 0)
        self.pca.set_pwm_duty(ch, 4095 if level else 0)

    def _cmd_one(self, m: MotorMap, val: float):
        v = max(-1.0, min(1.0, float(val))) * self.duty_limit
        if m.inv: v = -v
        duty = int(abs(v) * 4095)
        inA, inB = (m.in1, m.in2) if not m.swap_in12 else (m.in2, m.in1)
        if v > 0:
            # Forward = IN1=1, IN2=0 (post-swap)
            self._digital(inA, 1); self._digital(inB, 0)
            self.pca.set_pwm_duty(m.pwm, duty)
        elif v < 0:
            # Reverse = IN1=0, IN2=1
            self._digital(inA, 0); self._digital(inB, 1)
            self.pca.set_pwm_duty(m.pwm, duty)
        else:
            self.pca.set_pwm_duty(m.pwm, 0)
            self._digital(inA, 0); self._digital(inB, 0)

    def drive_all(self, vals):
        for m, v in zip(self.motors, vals):
            self._cmd_one(m, v)

    def stop(self):
        for m in self.motors:
            self.pca.set_pwm_duty(m.pwm, 0)
            self._digital(m.in1, 0)
            self._digital(m.in2, 0)

    def close(self):
        try:
            self.stop()
        finally:
            try: self.i2c.close()
            except Exception: pass

# ============================= Teleop core ====================================
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Geom:
    r: float
    L: float

class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, timeout=0.0):
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
                    out.append(b"\x1b"); 
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

class Teleop:
    def __init__(self, args):
        self.geom = Geom(args.wheel_radius, args.L)
        self.sy = -1.0 if args.flip_vy else +1.0
        self.so = -1.0 if args.flip_omega else +1.0
        self.max_vx, self.max_vy, self.max_wz = args.max_vx, args.max_vy, args.max_omega
        self.ax, self.ay, self.az = args.accel_x, args.accel_y, args.accel_z
        self.hz, self.deadman = args.hz, args.deadman
        self.scale_lo, self.scale_hi = args.scale_low, args.scale_high
        self.scale = 1.0
        self.kb = Keyboard()
        self.last_input = time.monotonic()
        self.t_vx = self.t_vy = self.t_wz = 0.0
        self.c_vx = self.c_vy = self.c_wz = 0.0
        # Motor map order: FL, FR, RL, RR
        motors = [
            MotorMap(args.fl_pwm, args.fl_in1, args.fl_in2, args.inv_fl, args.swap_in12_fl),
            MotorMap(args.fr_pwm, args.fr_in1, args.fr_in2, args.inv_fr, args.swap_in12_fr),
            MotorMap(args.rl_pwm, args.rl_in1, args.rl_in2, args.inv_rl, args.swap_in12_rl),
            MotorMap(args.rr_pwm, args.rr_in1, args.rr_in2, args.inv_rr, args.swap_in12_rr),
        ]
        self.driver = PCAMecanum(args.i2c_bus, args.pca_addr, int(args.pwm_freq),
                                 motors, in_active_low=args.in_active_low, duty_limit=1.0)

        signal.signal(signal.SIGINT, self._quit)
        signal.signal(signal.SIGTERM, self._quit)

        print("[Teleop] READY. Keys: W/S A/D Q/E, arrows, SPACE stop, M status. Esc/Ctrl+C quit.")
        print(f"  PCA9685@0x{args.pca_addr:02X} bus={args.i2c_bus}  PWM={int(args.pwm_freq)} Hz")
        print("  IMPORTANT: Make sure servo channels are NOT assigned here.")

    def _quit(self, *_):
        print("\n[Teleop] Quit → safe stop")
        self.close()
        os._exit(0)

    def close(self):
        try: self.kb.restore()
        except Exception: pass
        try: self.driver.close()
        except Exception: pass

    def _step(self, curr, target, a, dt):
        return min(curr + a*dt, target) if target > curr else max(curr - a*dt, target)

    def _apply(self, vx, vy, wz):
        r, L, sy, so = self.geom.r, self.geom.L, self.sy, self.so
        w_fl = ( vx - sy*vy - L*so*wz ) / r
        w_fr = ( vx + sy*vy + L*so*wz ) / r
        w_rl = ( vx + sy*vy - L*so*wz ) / r
        w_rr = ( vx - sy*vy + L*so*wz ) / r
        ws = [w_fl, w_fr, w_rl, w_rr]
        max_w = max(1e-6, max(abs(w) for w in ws))
        mag = max(abs(vx)/max(self.max_vx,1e-6),
                  abs(vy)/max(self.max_vy,1e-6),
                  abs(wz)/max(self.max_wz,1e-6))
        duties = [(w/max_w) * mag for w in ws]  # signed
        self.driver.drive_all(duties)
        self._duties = duties

    def _announce(self, s): print(s)

    def loop(self):
        try:
            period = 1.0 / self.hz
            last = time.monotonic()
            while True:
                t0 = time.monotonic()
                got = False
                for ch in self.kb.read():
                    got = True
                    if ch in (b"\x1b", b"\x03"): self._quit()
                    if ch == b' ':
                        self.t_vx = self.t_vy = self.t_wz = 0.0; self._announce("[Cmd] STOP")
                    elif ch in (b'm', b'M'):
                        d = getattr(self, "_duties", [0,0,0,0])
                        print(f"[cmd] vx={self.c_vx:+.3f} vy={self.c_vy:+.3f} wz={self.c_wz:+.3f} "
                              f"duties=[FL {d[0]:+.2f}, FR {d[1]:+.2f}, RL {d[2]:+.2f}, RR {d[3]:+.2f}]")
                    else:
                        # movement + cues
                        if ch in (b'w', b'W', b'UP'):
                            self.t_vx = +self.max_vx; self.t_vy = 0.0; self._announce("[Cmd] FORWARD")
                        elif ch in (b's', b'S', b'DOWN'):
                            self.t_vx = -self.max_vx; self.t_vy = 0.0; self._announce("[Cmd] BACK")
                        elif ch in (b'a', b'A', b'LEFT'):
                            self.t_vy = +self.max_vy; self.t_vx = 0.0; self._announce("[Cmd] LEFT")
                        elif ch in (b'd', b'D', b'RIGHT'):
                            self.t_vy = -self.max_vy; self.t_vx = 0.0; self._announce("[Cmd] RIGHT")
                        elif ch in (b'q', b'Q'):
                            self.t_wz = +self.max_wz; self._announce("[Cmd] TURN CCW")
                        elif ch in (b'e', b'E'):
                            self.t_wz = -self.max_wz; self._announce("[Cmd] TURN CW")
                    self.last_input = t0

                if not got and (t0 - self.last_input) > self.deadman:
                    if (self.t_vx, self.t_vy, self.t_wz) != (0.0, 0.0, 0.0):
                        print("[Deadman] No input → stopping")
                    self.t_vx = self.t_vy = self.t_wz = 0.0

                dt = max(1e-3, t0 - last); last = t0
                # accel limiting
                self.c_vx = self._step(self.c_vx, clamp(self.t_vx, -self.max_vx, self.max_vx), self.ax, dt)
                self.c_vy = self._step(self.c_vy, clamp(self.t_vy, -self.max_vy, self.max_vy), self.ay, dt)
                self.c_wz = self._step(self.c_wz, clamp(self.t_wz, -self.max_wz, self.max_wz), self.az, dt)

                self._apply(self.c_vx, self.c_vy, self.c_wz)

                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# =============================== CLI ==========================================
def build_args():
    p = argparse.ArgumentParser(description="Robot Savo — Manual Teleop (PCA9685-only)")
    # Geometry
    p.add_argument('--wheel-radius', type=float, default=0.0325)
    p.add_argument('--L', type=float, default=0.115)
    # Limits / rate
    p.add_argument('--max-vx', type=float, default=0.60)
    p.add_argument('--max-vy', type=float, default=0.60)
    p.add_argument('--max-omega', type=float, default=1.80)
    p.add_argument('--accel-x', type=float, default=1.2)
    p.add_argument('--accel-y', type=float, default=1.2)
    p.add_argument('--accel-z', type=float, default=2.5)
    p.add_argument('--hz', type=float, default=50.0)
    p.add_argument('--deadman', type=float, default=0.4)
    p.add_argument('--scale-low', type=float, default=0.35)   # kept for future (not used explicitly here)
    p.add_argument('--scale-high', type=float, default=1.7)
    # PCA9685
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=1000.0)
    p.add_argument('--in-active-low', action='store_true', help='Your H-bridge IN pins are active-LOW')
    # Channel mapping (FL, FR, RL, RR) — DO NOT use servo channels here
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # Per-wheel fixes
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # Convention flips
    p.add_argument('--flip-vy', action='store_true'); p.add_argument('--flip-omega', action='store_true')
    return p

def main(argv=None):
    args = build_args().parse_args(argv)
    t = Teleop(args)
    try:
        t.loop()
    except KeyboardInterrupt:
        pass
    finally:
        t.close()

if __name__ == '__main__':
    main()
