#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Mecanum Teleop/Tests (finite modes included)
- Modes:
  teleop : W/S/A/D/Q/E, ESC quits (default)
  demo   : plays FWD, BWD, STRAFE L/R, TURN CCW/CW then exits
  pulse  : pulses each wheel FWD/REV to verify mapping then exits
"""
import argparse, os, select, sys, termios, time, tty, signal
from dataclasses import dataclass

# ---- I2C ----
try:
    from smbus2 import SMBus
except Exception:
    print("Install smbus2: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# ---- PCA9685 minimal ----
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True, verbose=False):
        self.addr = int(addr); self.bus = SMBus(int(bus)); self.verbose=verbose
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1); self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)
    def close(self):
        try: self.bus.close()
        except: pass
    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)
    def set_pwm_freq(self, f):
        f = float(max(40.0, min(1500.0, f)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
        old = self._r8(MODE1); self._w8(MODE1, (old & ~RESTART)|SLEEP)
        self._w8(PRESCALE, prescale); self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003); self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        if self.verbose: print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")
    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8)&0x0F)
        self._w8(base+2, off&0xFF);  self._w8(base+3, (off>>8)&0x0F)
    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0); self._w8(base+2,0); self._w8(base+3,0x10)
    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0,0); self._w8(base+1,0x10); self._w8(base+2,0); self._w8(base+3,0)
    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty*4095))
        if off<=0: self.full_off(ch)
        elif off>=4095: self.full_on(ch)
        else: self._raw(ch,0,off)

# ---- H-bridge + wheels ----
@dataclass
class Triplet:
    pwm:int; in1:int; in2:int
    invert:bool=False; in_active_low:bool=False; swap_in12:bool=False

class Wheel:
    def __init__(self, p: PCA9685, t: Triplet, name:str, verbose=False):
        self.p, self.t, self.name, self.verbose = p, t, name, verbose
        self.stop()
    def _digital(self, ch, lvl):
        if self.t.in_active_low: lvl ^= 1
        (self.p.full_on if lvl else self.p.full_off)(ch)
        if self.verbose: print(f"[{self.name}] CH{ch}: {'HIGH' if lvl else 'LOW'} (active_low={self.t.in_active_low})")
    def drive(self, signed, duty_max):
        d = max(-1.0, min(1.0, float(signed)))
        if self.t.invert: d = -d
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        if abs(d) < 1e-6:
            self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)
            if self.verbose: print(f"[{self.name}] STOP")
            return
        if d>0:
            self._digital(a,1); self._digital(b,0); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] FWD duty={abs(d)*duty_max:.2f}")
        else:
            self._digital(a,0); self._digital(b,1); self.p.set_duty(self.t.pwm,abs(d)*duty_max)
            if self.verbose: print(f"[{self.name}] REV duty={abs(d)*duty_max:.2f}")
    def stop(self):
        a,b = (self.t.in1, self.t.in2) if not self.t.swap_in12 else (self.t.in2, self.t.in1)
        self._digital(a,0); self._digital(b,0); self.p.set_duty(self.t.pwm,0.0)

class Motor4:
    """Freeno-ve compatible order: set_motor_model(FL, BL, FR, BR) in [-1..+1] or [-4095..4095]"""
    def __init__(self, p: PCA9685, FL:Triplet, BL:Triplet, FR:Triplet, BR:Triplet, max_duty=1.0, verbose=False):
        self.FL, self.BL, self.FR, self.BR = Wheel(p,FL,"FL",verbose), Wheel(p,BL,"BL",verbose), Wheel(p,FR,"FR",verbose), Wheel(p,BR,"BR",verbose)
        self.max = float(max(0.05, min(1.0, max_duty)))
    def _norm(self, v):
        v = float(v)
        return max(-1.0, min(1.0, v/4095.0)) if abs(v)>1.01 else max(-1.0, min(1.0, v))
    def set_motor_model(self, FL, BL, FR, BR):
        fL,bL,fR,bR = map(self._norm, (FL,BL,FR,BR))
        self.FL.drive(fL, self.max); self.BL.drive(bL, self.max); self.FR.drive(fR, self.max); self.BR.drive(bR, self.max)
    def stop(self):
        for w in (self.FL,self.BL,self.FR,self.BR): w.stop()

# ---- keyboard (teleop only) ----
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
            else: out.append(ch)
            ch=self._read1(0.0)
        return out
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---- patterns ----
@dataclass
class P:
    F=(+1,+1,+1,+1); B=(-1,-1,-1,-1)
    SL=(-1,-1,+1,+1); SR=(+1,+1,-1,-1)
    TL=(-1,-1,+1,+1); TR=(+1,+1,-1,-1)
    STOP=(0,0,0,0)

# ---- main controller ----
class App:
    def __init__(self, a):
        self.a=a
        self.pca = PCA9685(a.i2c_bus, a.pca_addr, a.pwm_freq, set_freq=a.set_freq, verbose=a.verbose)
        # reserve (e.g., servo channels)
        if a.reserve:
            r=[int(x) for x in a.reserve.split(',') if x.strip()!='']; [self.pca.full_off(ch) for ch in r]
            if a.verbose: print(f"[Reserve] FULL-OFF: {r}")
        ial=a.in_active_low
        def T(pwm,in1,in2,inv,sw): return Triplet(pwm,in1,in2,inv,ial,sw)
        self.mot = Motor4(
            self.pca,
            FL=T(a.fl_pwm,a.fl_in1,a.fl_in2,a.inv_fl,a.swap_in12_fl),
            BL=T(a.bl_pwm,a.bl_in1,a.bl_in2,a.inv_bl,a.swap_in12_bl),
            FR=T(a.fr_pwm,a.fr_in1,a.fr_in2,a.inv_fr,a.swap_in12_fr),
            BR=T(a.br_pwm,a.br_in1,a.br_in2,a.inv_br,a.swap_in12_br),
            max_duty=max(0.05, min(1.0, a.max)),
            verbose=a.verbose
        )
        print("\n[WIRING] Wheel  PWM IN1 IN2  invert swap in_active_low")
        print(f"[WIRING] FL     {a.fl_pwm:2}  {a.fl_in1:2} {a.fl_in2:2}  {bool(a.inv_fl)!s:5} {bool(a.swap_in12_fl)!s:4} {bool(ial)!s}")
        print(f"[WIRING] BL     {a.bl_pwm:2}  {a.bl_in1:2} {a.bl_in2:2}  {bool(a.inv_bl)!s:5} {bool(a.swap_in12_bl)!s:4} {bool(ial)!s}")
        print(f"[WIRING] FR     {a.fr_pwm:2}  {a.fr_in1:2} {a.fr_in2:2}  {bool(a.inv_fr)!s:5} {bool(a.swap_in12_fr)!s:4} {bool(ial)!s}")
        print(f"[WIRING] BR     {a.br_pwm:2}  {a.br_in1:2} {a.br_in2:2}  {bool(a.inv_br)!s:5} {bool(a.swap_in12_br)!s:4} {bool(ial)!s}\n")

    def setv(self, vec, mag): self.mot.set_motor_model(vec[0]*mag, vec[1]*mag, vec[2]*mag, vec[3]*mag)
    def stop(self): self.mot.stop()

    def run_demo(self):
        seq=[("Forward",P.F),("Backward",P.B),("Strafe Left",P.SL),("Strafe Right",P.SR),("Turn CCW",P.TL),("Turn CW",P.TR)]
        for name,vec in seq:
            print(f"[DEMO] {name}"); self.setv(vec, self.a.mag); time.sleep(self.a.step)
            self.setv(P.STOP,0); time.sleep(0.25)
        print("[DEMO] done"); self.stop()

    def run_pulse(self):
        tests=[("FL",(1,0,0,0)),("BL",(0,1,0,0)),("FR",(0,0,1,0)),("BR",(0,0,0,1))]
        for name,mask in tests:
            print(f"[PULSE] {name} FWD"); self.mot.set_motor_model(*(m*self.a.mag for m in mask)); time.sleep(0.4); self.stop(); time.sleep(0.15)
            print(f"[PULSE] {name} REV"); self.mot.set_motor_model(*(-m*self.a.mag for m in mask)); time.sleep(0.4); self.stop(); time.sleep(0.25)
        print("[PULSE] done"); self.stop()

    def run_teleop(self):
        kb=Keyboard()
        deadman=self.a.deadman
        idle=self.a.idle_exit if self.a.idle_exit>0 else None
        start=time.monotonic(); last=time.monotonic(); pattern=P.STOP; scale=1.0
        print("[Teleop] W/S/A/D  Q/E  (Space=stop, ESC quits)")
        try:
            while True:
                now=time.monotonic()
                if self.a.duration>0 and (now-start)>=self.a.duration:
                    print("[Teleop] duration timeout"); return
                got=False
                for ch in kb.read():
                    got=True; last=now
                    if ch in (b'\x1b', b'\x03'):
                        print("[Teleop] quit"); return
                    if ch in (b' ', b'0'):
                        pattern=P.STOP; print("[Cmd] STOP"); continue
                    # speed scaling: ctrl/lowcase vs UPPERCASE
                    slow = (isinstance(ch,bytes) and len(ch)==1 and 1<=ch[0]<=26)  # control keys
                    fast = (isinstance(ch,bytes) and ch.isalpha() and ch.isupper())
                    scale = self.a.scale_low if slow else (self.a.scale_high if fast else 1.0)
                    if ch in (b'w',b'W',b'UP'):      pattern=P.F;  print("[Move] FWD")
                    elif ch in (b's',b'S',b'DOWN'):  pattern=P.B;  print("[Move] BWD")
                    elif ch in (b'a',b'A',b'LEFT'):  pattern=P.SL; print("[Move] STRAFE L")
                    elif ch in (b'd',b'D',b'RIGHT'): pattern=P.SR; print("[Move] STRAFE R")
                    elif ch in (b'q',b'Q'):          pattern=P.TL; print("[Turn] CCW")
                    elif ch in (b'e',b'E'):          pattern=P.TR; print("[Turn] CW")
                if not got:
                    if (now-last)>deadman and pattern!=P.STOP:
                        pattern=P.STOP; print("[Deadman] stop")
                    if idle and (now-last)>idle:
                        print("[Idle] exit"); return
                self.setv(pattern, self.a.mag*scale)
                time.sleep(max(0.0, 1.0/max(1.0,self.a.hz)))
        finally:
            try: kb.restore()
            except: pass
            self.stop()

# ---- CLI ----
def build_ap():
    p=argparse.ArgumentParser(description="Robot Savo — Mecanum Teleop/Tests (finite modes)")
    p.add_argument('--mode', choices=['teleop','demo','pulse'], default='teleop')
    p.add_argument('--duration', type=float, default=0.0, help="Auto-exit after SEC (teleop)")
    p.add_argument('--idle-exit', type=float, default=0.0, help="Exit if idle for SEC (teleop)")
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--mag', type=float, default=0.6)
    p.add_argument('--max', type=float, default=1.0)
    p.add_argument('--step', type=float, default=1.0, help="demo step seconds")
    p.add_argument('--deadman', type=float, default=2.0, help="Auto STOP if no key for SEC")
    p.add_argument('--scale-low', type=float, default=0.35, help="Speed scale when holding control keys")
    p.add_argument('--scale-high', type=float, default=1.25, help="Speed scale when using UPPERCASE letters")
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    p.add_argument('--pwm-freq', type=float, default=200.0)
    p.add_argument('--set-freq', action='store_true', default=True)
    p.add_argument('--reserve', type=str, default='12,13,14,15')
    p.add_argument('--in-active-low', action='store_true')
    p.add_argument('--verbose', action='store_true')
    # wheel channels (PCA)
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
    a = build_ap().parse_args(argv)
    # expose scales on the object (used in teleop)
    setattr(a, 'scale_low', getattr(a, 'scale_low', 0.35))
    setattr(a, 'scale_high', getattr(a, 'scale_high', 1.25))
    app = App(a)
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if a.mode=='demo':    app.run_demo()
        elif a.mode=='pulse': app.run_pulse()
        else:                 app.run_teleop()
    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C")
    finally:
        app.stop()

if __name__=='__main__': main()
