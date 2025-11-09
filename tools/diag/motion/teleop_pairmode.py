#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Minimal Mecanum Teleop (our own, DC motors only)
-------------------------------------------------------------
- Drives FOUR wheels simultaneously using raw PCA9685.
- API mirrors Freenove's set_motor_model(FL, BL, FR, BR) so we can swap later.
- W/S/A/D/Q/E + diagonals (U,J,Y,H). Space/0 = stop. ESC/Ctrl+C = quit.
- Deadman + optional idle-exit. Verbose wiring echo.

Wheel order (important):
  set_motor_model(FL, BL, FR, BR)   # same argument order Freenove uses

Each wheel needs 3 PCA9685 channels: PWM, IN1, IN2.
"""
import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---------- I2C ----------
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------- PCA9685 (minimal) ----------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True, verbose=False):
        self.addr = int(addr); self.bus = SMBus(int(bus)); self.verbose=verbose
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, freq_hz: float):
        f = float(max(40.0, min(1500.0, freq_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0 / (4096.0 * f) - 1.0))))
        old = self._r8(MODE1)
        self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale)
        self._w8(MODE1, old & ~SLEEP); time.sleep(0.003)
        self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        if self.verbose: print(f"[PCA] freq={f:.1f} Hz (prescale={prescale})")

    def _raw(self, ch: int, on: int, off: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF);    self._w8(base+1, (on >> 8) & 0x0F)
        self._w8(base+2, off & 0xFF);   self._w8(base+3, (off >> 8) & 0x0F)

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
        if off <= 0:         self.full_off(ch)
        elif off >= 4095:    self.full_on(ch)
        else:                self._raw(ch, 0, off)

# ---------- H-bridge ----------
@dataclass
class Triplet:
    pwm: int; in1: int; in2: int
    invert: bool=False
    in_active_low: bool=False
    swap_in12: bool=False

class Wheel:
    def __init__(self, pca: PCA9685, t: Triplet, name: str, verbose=False):
        self.p = pca; self.t = t; self.name=name; self.verbose=verbose
        self.stop()

    def _digital(self, ch: int, lvl: int):
        if self.t.in_active_low: lvl ^= 1
        if lvl: self.p.full_on(ch)
        else:   self.p.full_off(ch)
        if self.verbose:
            s = "HIGH" if lvl else "LOW"
            print(f"[{self.name}] CH{ch}: {s} (in_active_low={self.t.in_active_low})")

    def drive(self, signed: float, duty_max: float):
        d = max(-1.0, min(1.0, float(signed)))
        if self.t.invert: d = -d
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        if abs(d) < 1e-6:
            self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm, 0.0)
            if self.verbose: print(f"[{self.name}] STOP pwm=CH{self.t.pwm} duty=0.00")
            return
        if d > 0:
            self._digital(a,1); self._digital(b,0); self.p.set_duty(self.t.pwm, abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] FWD  duty={abs(d)*duty_max:.2f}")
        else:
            self._digital(a,0); self._digital(b,1); self.p.set_duty(self.t.pwm, abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] REV  duty={abs(d)*duty_max:.2f}")

    def stop(self):
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm, 0.0)

# ---------- Our motor adapter (drop-in for Freenove style) ----------
class Motor4:
    """
    set_motor_model(FL, BL, FR, BR)  # signed ints or floats in [-max, +max]
    """
    def __init__(self, pca: PCA9685, FL: Triplet, BL: Triplet, FR: Triplet, BR: Triplet, max_duty=1.0, verbose=False):
        self.FL = Wheel(pca, FL, "FL", verbose)
        self.BL = Wheel(pca, BL, "BL", verbose)
        self.FR = Wheel(pca, FR, "FR", verbose)
        self.BR = Wheel(pca, BR, "BR", verbose)
        self.max = float(max(0.05, min(1.0, max_duty)))

    def set_motor_model(self, FL, BL, FR, BR):
        # accept int in 0..4095 or normalized -1..+1
        def to_norm(v):
            v = float(v)
            if abs(v) > 1.01:  # assume raw 0..4095
                return max(-1.0, min(1.0, v/4095.0))
            return max(-1.0, min(1.0, v))
        fL, bL, fR, bR = map(to_norm, (FL, BL, FR, BR))
        self.FL.drive(fL, self.max)
        self.BL.drive(bL, self.max)
        self.FR.drive(fR, self.max)
        self.BR.drive(bR, self.max)

    def stop(self):
        for w in (self.FL, self.BL, self.FR, self.BR): w.stop()

# ---------- Keyboard ----------
class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def _read1(self, t=0.0):
        r,_,_ = select.select([sys.stdin], [], [], t)
        return os.read(self.fd, 1) if r else None

    def read(self):
        out=[]
        ch=self._read1(0.0)
        while ch is not None:
            if ch == b'\x1b':
                a = self._read1(0.02)
                if a in (b'[', b'O'):
                    b = self._read1(0.04)
                    arrows = {b'A': b'UP', b'B': b'DOWN', b'C': b'RIGHT', b'D': b'LEFT'}
                    if b in arrows: out.append(arrows[b])
                else:
                    out.append(b'\x1b')
                    if a: out.append(a)
            else:
                out.append(ch)
            ch=self._read1(0.0)
        return out

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---------- Teleop ----------
def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Patterns:
    # vectors for mecanum (signs for each wheel in set_motor_model order: FL, BL, FR, BR)
    F  = (+1, +1, +1, +1)
    B  = (-1, -1, -1, -1)
    TL = (-1, -1, +1, +1)  # CCW
    TR = (+1, +1, -1, -1)  # CW
    SL = (-1, -1, +1, +1)  # strafe left (same as CCW if you wire differently; flip keys if needed)
    SR = (+1, +1, -1, -1)  # strafe right
    # diagonals (two-wheel combos)
    LF = ( 0, +1, +1,  0)
    RB = ( 0, -1, -1,  0)
    RF = (+1,  0,  0, +1)
    LB = (-1,  0,  0, -1)
    STOP = (0,0,0,0)

class Teleop:
    def __init__(self, args):
        self.args = args
        self.pca = PCA9685(args.i2c_bus, args.pca_addr, args.pwm_freq, set_freq=args.set_freq, verbose=args.verbose)
        # reserve channels
        if args.reserve:
            chans=[int(x) for x in args.reserve.split(',') if x.strip()!='']
            for ch in chans: self.pca.full_off(ch)
            if args.verbose: print(f"[Reserve] FULL-OFF: {chans}")

        def T(pwm, in1, in2, invert, swap, ial):
            return Triplet(pwm=pwm, in1=in1, in2=in2, invert=invert, swap_in12=swap, in_active_low=ial)
        ial = args.in_active_low

        self.motor = Motor4(
            self.pca,
            FL=T(args.fl_pwm,args.fl_in1,args.fl_in2,args.inv_fl,args.swap_in12_fl,ial),
            BL=T(args.bl_pwm,args.bl_in1,args.bl_in2,args.inv_bl,args.swap_in12_bl,ial),
            FR=T(args.fr_pwm,args.fr_in1,args.fr_in2,args.inv_fr,args.swap_in12_fr,ial),
            BR=T(args.br_pwm,args.br_in1,args.br_in2,args.inv_br,args.swap_in12_br,ial),
            max_duty=clamp(args.max, 0.05, 1.0),
            verbose=args.verbose
        )

        # echo wiring
        print("\n[WIRING]  Wheel   PWM  IN1  IN2   invert swap in_active_low")
        print(f"[WIRING]  FL     {args.fl_pwm:2}  {args.fl_in1:2}  {args.fl_in2:2}   {bool(args.inv_fl)!s:5}  {bool(args.swap_in12_fl)!s:5} {bool(ial)!s:5}")
        print(f"[WIRING]  BL     {args.bl_pwm:2}  {args.bl_in1:2}  {args.bl_in2:2}   {bool(args.inv_bl)!s:5}  {bool(args.swap_in12_bl)!s:5} {bool(ial)!s:5}")
        print(f"[WIRING]  FR     {args.fr_pwm:2}  {args.fr_in1:2}  {args.fr_in2:2}   {bool(args.inv_fr)!s:5}  {bool(args.swap_in12_fr)!s:5} {bool(ial)!s:5}")
        print(f"[WIRING]  BR     {args.br_pwm:2}  {args.br_in1:2}  {args.br_in2:2}   {bool(args.inv_br)!s:5}  {bool(args.swap_in12_br)!s:5} {bool(ial)!s:5}\n")

        self.kb = Keyboard()
        self.deadman = float(args.deadman)
        self.idle_exit = float(args.idle_exit) if args.idle_exit>0 else None
        self.scale_low, self.scale_high = float(args.scale_low), float(args.scale_high)
        self.scale = 1.0
        self.mag = clamp(args.mag, 0.05, 1.0)
        self.last_in = time.monotonic()
        self.pattern = Patterns.STOP

        signal.signal(signal.SIGINT, self._sig_exit)
        signal.signal(signal.SIGTERM, self._sig_exit)

        print("[Teleop] READY: W/S/A/D  Q/E  U/J/Y/H  (Space=stop, M=print duties, ESC/Ctrl+C=quit)")

    def _sig_exit(self, *_):
        print("\n[Teleop] Signal → safe stop")
        self.close(); os._exit(0)

    def _apply(self):
        v = self.pattern
        m = self.scale * self.mag
        # call using Freenove order (FL, BL, FR, BR)
        self.motor.set_motor_model(v[0]*m, v[1]*m, v[2]*m, v[3]*m)

    def _handle_key(self, ch):
        if ch in (b'\x1b', b'\x03'):   # ESC or Ctrl+C
            print("[Teleop] Quit"); return True
        if ch in (b' ', b'0'):
            self.pattern = Patterns.STOP; print("[Cmd] STOP"); return False
        if ch in (b'm', b'M'):
            print(f"[mag] base={self.mag:.2f} scale={self.scale:.2f} -> eff={self.mag*self.scale:.2f}")
            return False

        slow = (isinstance(ch, bytes) and len(ch)==1 and 1 <= ch[0] <= 26)   # Ctrl
        fast = (isinstance(ch, bytes) and ch.isalpha() and ch.isupper())     # Shift
        self.scale = self.scale_low if slow else (self.scale_high if fast else 1.0)

        if ch in (b'w', b'W', b'UP'):     self.pattern = Patterns.F;  print("[Move] Forward")
        elif ch in (b's', b'S', b'DOWN'): self.pattern = Patterns.B;  print("[Move] Backward")
        elif ch in (b'a', b'A', b'LEFT'): self.pattern = Patterns.SL; print("[Move] Left (strafe)")
        elif ch in (b'd', b'D', b'RIGHT'):self.pattern = Patterns.SR; print("[Move] Right (strafe)")
        elif ch in (b'q', b'Q'):          self.pattern = Patterns.TL; print("[Turn] CCW")
        elif ch in (b'e', b'E'):          self.pattern = Patterns.TR; print("[Turn] CW")
        elif ch in (b'u', b'U'):          self.pattern = Patterns.RF; print("[Diag] Right+Forward (2-wheel)")
        elif ch in (b'j', b'J'):          self.pattern = Patterns.LB; print("[Diag] Left+Backward (2-wheel)")
        elif ch in (b'y', b'Y'):          self.pattern = Patterns.LF; print("[Diag] Left+Forward (2-wheel)")
        elif ch in (b'h', b'H'):          self.pattern = Patterns.RB; print("[Diag] Right+Backward (2-wheel)")
        return False

    def loop(self, hz=30.0, duration=None):
        period = 1.0/max(1.0, float(hz))
        t0 = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                got = False
                for ch in self.kb.read():
                    got = True
                    if self._handle_key(ch): return
                    self.last_in = now

                if not got:
                    if (now - self.last_in) > self.deadman and self.pattern != Patterns.STOP:
                        print("[Deadman] stop"); self.pattern = Patterns.STOP
                    if self.idle_exit and (now - self.last_in) > self.idle_exit:
                        print("[Idle] exit"); return

                self._apply()

                if duration and (now - t0) > duration: return
                time.sleep(max(0.0, period - (time.monotonic() - now)))
        finally:
            self.close()

    def close(self):
        try: self.motor.stop()
        except: pass
        try: self.kb.restore()
        except: pass
        try: self.pca.close()
        except: pass

# ---------- CLI ----------
def build_ap():
    p = argparse.ArgumentParser(description="Robot Savo — Mecanum Teleop (our own, DC only)")
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--set-freq', action='store_true', help="Write prescale (default: yes)", default=True)
    p.add_argument('--reserve', type=str, default='12,13,14,15')
    p.add_argument('--mag', type=float, default=0.6, help="base magnitude (0..1)")
    p.add_argument('--max', type=float, default=1.0, help="max duty (0..1)")
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.5)
    p.add_argument('--idle-exit', type=float, default=0.0, help="exit after idle seconds (0=never)")
    p.add_argument('--scale-low', type=float, default=0.35)
    p.add_argument('--scale-high', type=float, default=1.5)
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--verbose', action='store_true')

    # channel mapping (PCA ch)
    p.add_argument('--fl-pwm', type=int, required=True); p.add_argument('--fl-in1', type=int, required=True); p.add_argument('--fl-in2', type=int, required=True)
    p.add_argument('--bl-pwm', type=int, required=True); p.add_argument('--bl-in1', type=int, required=True); p.add_argument('--bl-in2', type=int, required=True)
    p.add_argument('--fr-pwm', type=int, required=True); p.add_argument('--fr-in1', type=int, required=True); p.add_argument('--fr-in2', type=int, required=True)
    p.add_argument('--br-pwm', type=int, required=True); p.add_argument('--br-in1', type=int, required=True); p.add_argument('--br-in2', type=int, required=True)

    # polarity helpers
    p.add_argument('--inv-fl', action='store_true'); p.add_argument('--inv-bl', action='store_true')
    p.add_argument('--inv-fr', action='store_true'); p.add_argument('--inv-br', action='store_true')
    p.add_argument('--swap-in12-fl', action='store_true'); p.add_argument('--swap-in12-bl', action='store_true')
    p.add_argument('--swap-in12-fr', action='store_true'); p.add_argument('--swap-in12-br', action='store_true')
    return p

def main(argv=None):
    args = build_ap().parse_args(argv)
    t = Teleop(args)
    try:
        t.loop(hz=args.hz)
    finally:
        t.close()

if __name__ == '__main__':
    main()
