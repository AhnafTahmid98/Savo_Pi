#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Manual Teleop (Freenove 11-pattern table, DC motors only)
----------------------------------------------------------------------

Pure DC-motor control over PCA9685 H-bridges. NO servo control.

Mappings (M1=FL, M2=FR, M3=RL, M4=RR)
Basics:
  W / ↑ : (+,+,+,+)  Forward
  S / ↓ : (-,-,-,-)  Backward
  Q     : (-,-,+,+)  Turn left (CCW)
  E     : (+,+,-,-)  Turn right (CW)
  A / ← : (-,+,+,-)  Move left (strafe)
  D / → : (+,-,-,+)  Move right (strafe)

Two-wheel combos:
  Y     : (0,+,+,0)  Left and forward
  H     : (0,-,-,0)  Right and backward
  U     : (+,0,0,+)  Right and forward
  J     : (-,0,0,-)  Left and backward

Other:
  SPACE or 0 : Stop
  M          : Print wheel duties
  1..4 / 5..8: Pulse FL/FR/RL/RR forward / reverse (prints which)
  Shift      : fast scale,  Ctrl : slow scale
  Esc / Ctrl+C : Quit (safe stop)

Safety:
  - Deadman 0.25 s: no key input → stop
  - Reserves servo channels FULL-OFF (default: 12,13,14,15)
  - Does NOT change PCA9685 prescale unless --set-freq is used
"""

import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ----------- I2C ----------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ----------- PCA9685 minimal ----------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=False):
        self.addr = int(addr); self.bus = SMBus(int(bus))
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

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP)
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
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:   self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else:          self._raw(ch, 0, off)

# ----------- H-bridge ----------
@dataclass
class MotorCH:
    pwm: int; in1: int; in2: int

class HBridge:
    def __init__(self, pca: PCA9685, ch: MotorCH, invert=False, in_active_low=False, swap_in12=False, name="?", verbose=False):
        self.pca, self.ch = pca, ch
        self.invert = bool(invert)
        self.in_active_low = bool(in_active_low)
        self.swap = bool(swap_in12)
        self.name = name
        self.verbose = bool(verbose)
        # cache: last known levels per PCA channel and last duty set
        self._ch_level = {}    # {channel: 0/1}
        self._last_duty = None # float in [0..1]
        self.stop()

    def _digital(self, channel: int, level: int):
        lvl = (level ^ 1) if self.in_active_low else level
        # only write if changed
        if self._ch_level.get(channel) == lvl:
            return
        self._ch_level[channel] = lvl
        if lvl: self.pca.full_on(channel)
        else:   self.pca.full_off(channel)
        if self.verbose:
            state = "HIGH" if lvl else "LOW"
            print(f"[{self.name}] CH{channel}: {state} (in_active_low={self.in_active_low})")

    def _set_pwm_duty(self, duty: float):
        # only write if changed meaningfully (>1% or crossing zero)
        eps = 0.01
        prev = self._last_duty
        if prev is None or abs(duty - prev) > eps or (duty == 0.0) != (prev == 0.0):
            self.pca.set_duty(self.ch.pwm, duty)
            self._last_duty = duty
            if self.verbose:
                print(f"[{self.name}] PWM CH{self.ch.pwm} duty={duty:0.2f}")

    def drive(self, signed: float):
        d = max(-1.0, min(1.0, float(signed)))
        raw = d
        if self.invert: d = -d
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        if abs(d) < 1e-3:
            self._digital(a, 0); self._digital(b, 0); self._set_pwm_duty(0.0)
            if self.verbose:
                print(f"[{self.name}] STOP (invert={self.invert} swap={self.swap})")
            return
        if d > 0:
            self._digital(a, 1); self._digital(b, 0); self._set_pwm_duty(d)
            if self.verbose:
                print(f"[{self.name}] FWD  inA=CH{a} inB=CH{b}  duty={d:0.2f}  (invert={self.invert} swap={self.swap}, raw={raw:+0.2f})")
        else:
            self._digital(a, 0); self._digital(b, 1); self._set_pwm_duty(-d)
            if self.verbose:
                print(f"[{self.name}] REV  inA=CH{a} inB=CH{b}  duty={-d:0.2f} (invert={self.invert} swap={self.swap}, raw={raw:+0.2f})")

    def stop(self):
        a, b = (self.ch.in1, self.ch.in2) if not self.swap else (self.ch.in2, self.ch.in1)
        self._digital(a, 0); self._digital(b, 0); self._set_pwm_duty(0.0)

# ----------- Keyboard ----------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r,_,_ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd, 1) if r else None

    def read(self):
        out = []
        ch = self._read1(0.0)
        while ch is not None:
            if ch == b'\x1b':
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                    else: out.extend(filter(None, [b'\x1b', a, b]))
                else:
                    out.append(b'\x1b')
                    if a: out.append(a)
            else:
                out.append(ch)
            ch = self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ----------- Teleop (11 patterns) ----------
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Limits:
    vx: float; vy: float; wz: float
    ax: float; ay: float; az: float

class Teleop:
    # Basic table (M1,M2,M3,M4) = (FL,FR,RL,RR)
    V_F  = (+1, +1, +1, +1)
    V_B  = (-1, -1, -1, -1)
    V_TL = (-1, -1, +1, +1)
    V_TR = (+1, +1, -1, -1)
    V_SL = (-1, +1, +1, -1)
    V_SR = (+1, -1, -1, +1)

    # Two-wheel combos
    V_LF = ( 0, +1, +1,  0)  # Left & Forward
    V_RB = ( 0, -1, -1,  0)  # Right & Backward
    V_RF = (+1,  0,  0, +1)  # Right & Forward
    V_LB = (-1,  0,  0, -1)  # Left & Backward

    def __init__(self, a):
        self.hz = float(a.hz)
        self.deadman = float(a.deadman)
        self.scale_low, self.scale_high = float(a.scale_low), float(a.scale_high)
        self.scale = 1.0
        self.flip_vx = bool(a.flip_vx)
        self.flip_vy = bool(a.flip_vy)
        self.flip_wz = bool(a.flip_omega)
        self.lim = Limits(a.max_vx, a.max_vy, a.max_omega, a.accel_x, a.accel_y, a.accel_z)
        self.verbose = bool(a.verbose)

        self.pca = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)

        # Reserve servo outputs (never touched)
        reserved = []
        try:
            reserved = [int(x) for x in str(a.reserve).split(',') if x.strip()!='']
            for ch in reserved: self.pca.full_off(ch)
            if reserved: print(f"[Teleop] Reserved FULL-OFF: {reserved}")
        except Exception: pass
        self.reserved = set(reserved)

        # Motors
        self.FL = HBridge(self.pca, MotorCH(a.fl_pwm, a.fl_in1, a.fl_in2),
                          invert=a.inv_fl, in_active_low=a.in_active_low, swap_in12=a.swap_in12_fl,
                          name="FL", verbose=self.verbose)
        self.FR = HBridge(self.pca, MotorCH(a.fr_pwm, a.fr_in1, a.fr_in2),
                          invert=a.inv_fr, in_active_low=a.in_active_low, swap_in12=a.swap_in12_fr,
                          name="FR", verbose=self.verbose)
        self.RL = HBridge(self.pca, MotorCH(a.rl_pwm, a.rl_in1, a.rl_in2),
                          invert=a.inv_rl, in_active_low=a.in_active_low, swap_in12=a.swap_in12_rl,
                          name="RL", verbose=self.verbose)
        self.RR = HBridge(self.pca, MotorCH(a.rr_pwm, a.rr_in1, a.rr_in2),
                          invert=a.inv_rr, in_active_low=a.in_active_low, swap_in12=a.swap_in12_rr,
                          name="RR", verbose=self.verbose)

        self._wiring_report()

        self.kb = Keyboard()
        self.last_in = time.monotonic()
        self._duties = [0.0,0.0,0.0,0.0]

        self.vec = (0,0,0,0)
        self.mag = 0.0

        signal.signal(signal.SIGINT, self._sig_exit)
        signal.signal(signal.SIGTERM, self._sig_exit)

        print("[Teleop] READY — (W/S/A/D/Q/E + Y/H/U/J; Space/0 stop; M print; 1..8 pulses)")

    # ----- signal + cleanup -----
    def _sig_exit(self, *_):
        print("\n[Teleop] Signal → safe stop & exit")
        try:
            self.close()
        finally:
            os._exit(0)

    def close(self):
        # stop motors, restore keyboard, close I²C
        try:
            self.FL.stop(); self.FR.stop(); self.RL.stop(); self.RR.stop()
        except Exception:
            pass
        try:
            self.kb.restore()
        except Exception:
            pass
        try:
            self.pca.close()
        except Exception:
            pass

    # ----- wiring report + collision checks -----
    def _wiring_report(self):
        def row(name, m: HBridge):
            return (name, m.ch.pwm, m.ch.in1, m.ch.in2, m.invert, m.swap, m.in_active_low)
        rows = [row("FL", self.FL), row("FR", self.FR), row("RL", self.RL), row("RR", self.RR)]
        print("\n[WIRING] Wheel   PWM  IN1  IN2   invert swap in_active_low")
        for r in rows:
            print(f"[WIRING] {r[0]:<3}  {r[1]:>3}  {r[2]:>3}  {r[3]:>3}    {str(r[4]):<6} {str(r[5]):<4} {str(r[6]):<13}")
        used = []
        for _,p,i1,i2,_,_,_ in rows:
            used.extend([p,i1,i2])
        dup = [c for c in used if used.count(c) > 1]
        if dup:
            print(f"[WIRING][WARN] Channel collisions detected: {sorted(set(dup))}  (two signals share a PCA9685 channel)")
        bad = sorted(set(used) & self.reserved)
        if bad:
            print(f"[WIRING][ERROR] Using reserved/servo channels: {bad}. Move them off {sorted(self.reserved)} or change --reserve.")

    # ---- core helpers ----
    def _apply_vec(self, vec, mag):
        # compute desired duties
        new_d = [clamp(v*mag, -1.0, 1.0) for v in vec]
        # only act/print if something changed
        eps = 1e-3
        changed = any(abs(new_d[i] - self._duties[i]) > eps for i in range(4))
        if not changed:
            return
        if self.verbose:
            print(f"[CMD] vec={vec} mag={mag:0.2f} -> duty FL={new_d[0]:+0.2f} FR={new_d[1]:+0.2f} RL={new_d[2]:+0.2f} RR={new_d[3]:+0.2f}")
        self.FL.drive(new_d[0]); self.FR.drive(new_d[1]); self.RL.drive(new_d[2]); self.RR.drive(new_d[3])
        self._duties = new_d


    def _set_pattern(self, vec, mag):
        self.vec, self.mag = vec, clamp(mag, 0.0, 1.0)

    def _handle_key(self, ch):
        # quit
        if ch in (b"\x1b", b"\x03"):
            print("[Teleop] Quit"); return True
        # stop
        if ch in (b' ', b'0'):
            self._set_pattern((0,0,0,0), 0.0); print("[Cmd] STOP"); return False
        # print duties
        if ch in (b'm', b'M'):
            d = self._duties
            print(f"[duty] M1 {d[0]:+0.2f}  M2 {d[1]:+0.2f}  M3 {d[2]:+0.2f}  M4 {d[3]:+0.2f}")
            return False

        # speed scaling
        slow = (isinstance(ch, bytes) and len(ch)==1 and 1 <= ch[0] <= 26)   # Ctrl
        fast = (isinstance(ch, bytes) and ch.isalpha() and ch.isupper())     # Shift
        self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)

        base_mag = 1.0
        mag = clamp(base_mag * self.scale, 0.0, 1.0)

        def maybe_flip(vec, axis):
            if axis == 'x' and self.flip_vx: vec = tuple(-v for v in vec)
            if axis == 'y' and self.flip_vy: vec = tuple(-v for v in vec)
            if axis == 'o' and self.flip_wz: vec = tuple(-v for v in vec)
            return vec

        # basics
        if ch in (b'w', b'W', b'UP'):
            self._set_pattern(maybe_flip(self.V_F, 'x'), mag); print("[Move] Forward")
        elif ch in (b's', b'S', b'DOWN'):
            self._set_pattern(maybe_flip(self.V_B, 'x'), mag); print("[Move] Backward")
        elif ch in (b'a', b'A', b'LEFT'):
            self._set_pattern(maybe_flip(self.V_SL, 'y'), mag); print("[Move] Left (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):
            self._set_pattern(maybe_flip(self.V_SR, 'y'), mag); print("[Move] Right (strafe)")
        elif ch in (b'q', b'Q'):
            self._set_pattern(maybe_flip(self.V_TL, 'o'), mag); print("[Turn] CCW")
        elif ch in (b'e', b'E'):
            self._set_pattern(maybe_flip(self.V_TR, 'o'), mag); print("[Turn] CW")

        # two-wheel combos
        elif ch in (b'y', b'Y'):
            self._set_pattern(self.V_LF, mag); print("[Move] Left+Forward")
        elif ch in (b'h', b'H'):
            self._set_pattern(self.V_RB, mag); print("[Move] Right+Backward")
        elif ch in (b'u', b'U'):
            self._set_pattern(self.V_RF, mag); print("[Move] Right+Forward")
        elif ch in (b'j', b'J'):
            self._set_pattern(self.V_LB, mag); print("[Move] Left+Backward")

        # diagnostics pulses (announce which wheel)
        elif ch == b'1': print("[Pulse] FL forward"); self._pulse([1,0,0,0], +0.12); return False
        elif ch == b'2': print("[Pulse] FR forward"); self._pulse([0,1,0,0], +0.12); return False
        elif ch == b'3': print("[Pulse] RL forward"); self._pulse([0,0,1,0], +0.12); return False
        elif ch == b'4': print("[Pulse] RR forward"); self._pulse([0,0,0,1], +0.12); return False
        elif ch == b'5': print("[Pulse] FL reverse"); self._pulse([1,0,0,0], -0.12); return False
        elif ch == b'6': print("[Pulse] FR reverse"); self._pulse([0,1,0,0], -0.12); return False
        elif ch == b'7': print("[Pulse] RL reverse"); self._pulse([0,0,1,0], -0.12); return False
        elif ch == b'8': print("[Pulse] RR reverse"); self._pulse([0,0,0,1], -0.12); return False

        return False

    def _pulse(self, mask, duty, t=0.30):
        ms = [self.FL, self.FR, self.RL, self.RR]
        names = ["FL","FR","RL","RR"]
        who = [names[i] for i,v in enumerate(mask) if v]
        if self.verbose:
            print(f"[PULSE] {who} duty={duty:+0.2f} for {t:0.2f}s")
        for i, m in enumerate(ms): m.drive(duty if mask[i] else 0.0)
        time.sleep(t)
        for m in ms: m.stop()


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
                    if any(abs(x) > 1e-6 for x in self._duties): print("[Deadman] stop")
                    self._set_pattern((0,0,0,0), 0.0)

                self._apply_vec(self.vec, self.mag)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            self.close()

# ----------- CLI ----------
def build_ap():
    p = argparse.ArgumentParser(description="Robot Savo — Teleop (Freenove 11 patterns, no servos)")
    # motion limits & rate
    p.add_argument('--max-vx', type=float, default=0.12)
    p.add_argument('--max-vy', type=float, default=0.12)
    p.add_argument('--max-omega', type=float, default=0.6)
    p.add_argument('--accel-x', type=float, default=1.0)
    p.add_argument('--accel-y', type=float, default=1.0)
    p.add_argument('--accel-z', type=float, default=2.0)
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.25)
    p.add_argument('--scale-low', type=float, default=0.20)
    p.add_argument('--scale-high', type=float, default=1.5)
    # I2C / PCA9685
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=800.0)
    p.add_argument('--set-freq', action='store_true', help="Change PCA9685 prescale (default: don't)")
    p.add_argument('--reserve', type=str, default='12,13,14,15', help='Channels to FULL-OFF (servo pins)')
    # Channel mapping (independent channels REQUIRED for full table)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--rl-pwm', type=int, required=True); p.add_argument('--rl-in1', type=int, required=True); p.add_argument('--rl-in2', type=int, required=True)
    p.add_argument('--rr-pwm', type=int, required=True); p.add_argument('--rr-in1', type=int, required=True); p.add_argument('--rr-in2', type=int, required=True)
    # polarity / logic helpers
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-fr', action='store_true')
    p.add_argument('--inv-rl', action='store_true'); p.add_argument('--inv-rr', action='store_true')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-fr', action='store_true')
    p.add_argument('--swap-in12-rl', action='store_true'); p.add_argument('--swap-in12-rr', action='store_true')
    # axis flips
    p.add_argument('--flip-vx', action='store_true', default=False)
    p.add_argument('--flip-vy', action='store_true', default=False)
    p.add_argument('--flip-omega', action='store_true', default=False)
    # verbosity
    p.add_argument('--verbose', action='store_true', help='Print wiring & per-move channel activity')
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
