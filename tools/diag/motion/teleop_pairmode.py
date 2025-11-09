#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Freenove Mecanum Teleop (11 patterns, DC motors only)
- Uses Freenove motor driver via car.Car().motor.set_motor_model(FL, FR, RL, RR)
- NO servo control. Smooth ramping, deadman, slow/fast scales.

Keys (M1=FL, M2=FR, M3=RL, M4=RR):
  W / ↑ : (+,+,+,+)  Forward
  S / ↓ : (-,-,-,-)  Backward
  Q     : (-,-,+,+)  Turn left (CCW)
  E     : (+,+,-,-)  Turn right (CW)
  A / ← : (-,+,+,-)  Strafe left
  D / → : (+,-,-,+)  Strafe right
  Y     : (0,+,+,0)  Left and forward
  H     : (0,-,-,0)  Right and backward
  U     : (+,0,0,+)  Right and forward
  J     : (-,0,0,-)  Left and backward
  SPACE/0: stop,  M: print duties,  Esc/Ctrl+C: quit
  Shift: fast scale  Ctrl: slow scale
"""

import sys, os, time, select, termios, tty, signal, argparse
from dataclasses import dataclass

# --- Freenove stack (uses your existing modules) ---
from car import Car   # <- gives car.motor.set_motor_model(FL, FR, RL, RR)

def clamp(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class Params:
    hz: float
    deadman: float
    scale_low: float
    scale_high: float
    mag: float
    ramp: float

class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
    def read_all(self, t=0.0):
        out=[]; r,_,_=select.select([sys.stdin],[],[],t)
        while r:
            b=os.read(self.fd,1)
            if not b: break
            # arrow decode
            if b == b'\x1b':
                a=os.read(self.fd,1) if select.select([sys.stdin],[],[],0.01)[0] else b''
                if a in (b'[',b'O'):
                    c=os.read(self.fd,1) if select.select([sys.stdin],[],[],0.02)[0] else b''
                    arrows={b'A':b'UP', b'B':b'DOWN', b'C':b'RIGHT', b'D':b'LEFT'}
                    out.append(arrows.get(c,b'\x1b'))
                else:
                    out.append(b)
            else:
                out.append(b)
            r,_,_=select.select([sys.stdin],[],[],0.0)
        return out
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

class Teleop:
    # 11-pattern table
    V_F  = (+1,+1,+1,+1)
    V_B  = (-1,-1,-1,-1)
    V_TL = (-1,-1,+1,+1)
    V_TR = (+1,+1,-1,-1)
    V_SL = (-1,+1,+1,-1)
    V_SR = (+1,-1,-1,+1)
    V_LF = ( 0,+1,+1, 0)
    V_RB = ( 0,-1,-1, 0)
    V_RF = (+1, 0, 0,+1)
    V_LB = (-1, 0, 0,-1)

    def __init__(self, p: Params):
        self.p = p
        self.kb = Keyboard()
        self.car = Car()     # Freenove driver initializes HAT/ADC/etc.
        self.last_input = time.monotonic()
        self.vec = (0,0,0,0)
        self.tgt = [0.0,0.0,0.0,0.0]
        self.cur = [0.0,0.0,0.0,0.0]
        print("[Teleop] READY — Freenove mecanum via set_motor_model().")

    def _apply_motor(self, d):
        # Freenove expects (FL, FR, RL, RR) as integers (duty-like).
        # We scale [-1..+1] by 1000 then by mag.
        scale = 1000
        FL = int(round(d[0]*scale)); FR = int(round(d[1]*scale))
        RL = int(round(d[2]*scale)); RR = int(round(d[3]*scale))
        self.car.motor.set_motor_model(FL, FR, RL, RR)

    def _set_vec(self, vec, mag):
        self.vec = vec
        m = clamp(mag, 0.0, 1.0)
        self.tgt = [clamp(v*m, -1.0, 1.0) for v in vec]

    def _slew(self, cur, tgt, rate, dt):
        if cur < tgt: return min(cur+rate*dt, tgt)
        if cur > tgt: return max(cur-rate*dt, tgt)
        return cur

    def _update_outputs(self, dt):
        rate = max(0.05, min(1.0, self.p.ramp))   # per-second duty change (0..1)
        for i in range(4):
            self.cur[i] = self._slew(self.cur[i], self.tgt[i], rate, dt)
        self._apply_motor(self.cur)

    def _stop(self):
        self._set_vec((0,0,0,0), 0.0)
        self._apply_motor([0,0,0,0])

    def _handle_key(self, b):
        # modifiers → scale
        scale = 1.0
        if b.isalpha() and b.isupper(): scale = self.p.scale_high   # Shift
        if 1 <= b[0] <= 26:             scale = self.p.scale_low    # Ctrl

        mag = self.p.mag * scale

        # commands
        if b in (b' ', b'0'):
            print("[STOP]"); self._stop(); return False
        if b in (b'\x1b', b'\x03'):
            print("[QUIT]"); raise KeyboardInterrupt
        if b in (b'm', b'M'):
            print("[duty] FL={:+.2f} FR={:+.2f} RL={:+.2f} RR={:+.2f}".format(*self.cur))
            return False

        # arrows
        if b == b'UP':    b = b'w'
        if b == b'DOWN':  b = b's'
        if b == b'LEFT':  b = b'a'
        if b == b'RIGHT': b = b'd'

        # mapping
        if   b in (b'w', b'W'): self._set_vec(self.V_F,  mag); print("[W] Forward")
        elif b in (b's', b'S'): self._set_vec(self.V_B,  mag); print("[S] Backward")
        elif b in (b'a', b'A'): self._set_vec(self.V_SL, mag); print("[A] Strafe Left")
        elif b in (b'd', b'D'): self._set_vec(self.V_SR, mag); print("[D] Strafe Right")
        elif b in (b'q', b'Q'): self._set_vec(self.V_TL, mag); print("[Q] Turn CCW")
        elif b in (b'e', b'E'): self._set_vec(self.V_TR, mag); print("[E] Turn CW")
        elif b in (b'y', b'Y'): self._set_vec(self.V_LF, mag); print("[Y] Left+Forward")
        elif b in (b'h', b'H'): self._set_vec(self.V_RB, mag); print("[H] Right+Backward")
        elif b in (b'u', b'U'): self._set_vec(self.V_RF, mag); print("[U] Right+Forward")
        elif b in (b'j', b'J'): self._set_vec(self.V_LB, mag); print("[J] Left+Backward")
        else: return False
        return True

    def loop(self):
        period = 1.0 / self.p.hz
        try:
            while True:
                t0 = time.monotonic()
                got = False
                for b in self.kb.read_all(0.0):
                    got = True
                    self._handle_key(b)
                    self.last_input = t0
                # deadman
                if not got and (t0 - self.last_input) > self.p.deadman:
                    if any(abs(x) > 1e-3 for x in self.cur):
                        print("[Deadman] stop")
                    self._stop()
                # ramp update
                self._update_outputs(period)
                # pace
                elapsed = time.monotonic() - t0
                if elapsed < period: time.sleep(period - elapsed)
        finally:
            try: self.kb.restore()
            except: pass
            self._stop()
            try: self.car.close()
            except: pass

def build_ap():
    p = argparse.ArgumentParser("Freenove mecanum teleop (11 patterns, ramped)")
    p.add_argument('--hz', type=float, default=30.0)
    p.add_argument('--deadman', type=float, default=0.6)
    p.add_argument('--scale-low', type=float, default=0.35)
    p.add_argument('--scale-high', type=float, default=1.25)
    p.add_argument('--mag', type=float, default=0.65, help="Base magnitude 0..1")
    p.add_argument('--ramp', type=float, default=0.8, help="Duty slew rate per second (0..1)")
    return p

def main(argv=None):
    a = build_ap().parse_args(argv)
    t = Teleop(Params(a.hz, a.deadman, a.scale_low, a.scale_high, a.mag, a.ramp))
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        t.loop()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
