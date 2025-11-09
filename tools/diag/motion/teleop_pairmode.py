#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Pair-Mode Teleop (Left/Right sides on PCA9685)
Keys:
  W/S : both sides forward / reverse
  Q/E : turn CCW / CW (left rev, right fwd) / (left fwd, right rev)
  SPACE: stop
  1/2 : Left side pulse fwd/rev
  3/4 : Right side pulse fwd/rev
Notes:
- Map EACH SIDE to ONE triplet (PWM, IN1, IN2). Works even if your HAT only has 3 DC triplets.
- No servos touched. Channels 12..15 FULL-OFF by default.
"""

import sys, os, time, select, termios, tty, argparse, signal
from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV); self._w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1); self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, f_hz: float):
        f = float(max(40.0, min(1500.0, f_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f) - 1.0))))
        old = self._r8(MODE1); self._w8(MODE1, (old & ~RESTART) | SLEEP)
        self._w8(PRESCALE, prescale); self._w8(MODE1, old & ~SLEEP)
        time.sleep(0.003); self._w8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def _raw(self, ch: int, on: int, off: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, on & 0xFF); self._w8(base+1, (on>>8) & 0x0F)
        self._w8(base+2, off & 0xFF); self._w8(base+3, (off>>8) & 0x0F)

    def full_off(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x00)
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        d = max(0.0, min(1.0, float(duty)))
        if d <= 0.0: self.full_off(ch); return
        if d >= 1.0: self.full_on(ch); return
        off = int(round(d * 4095)); self._raw(ch, 0, off)

class Side:
    def __init__(self, pca: PCA9685, pwm: int, in1: int, in2: int, invert=False, swap_in12=False, active_low=False):
        self.pca = pca; self.pwm=pwm
        self.in1=in1; self.in2=in2
        self.invert=bool(invert); self.swap=bool(swap_in12); self.active_low=bool(active_low)
        self.stop()

    def _digital(self, ch, level):
        lvl = (level ^ 1) if self.active_low else level
        if lvl: self.pca.full_on(ch)
        else:   self.pca.full_off(ch)

    def drive(self, signed: float):
        d = max(-1.0, min(1.0, float(signed)))
        if self.invert: d = -d
        a,b = (self.in1,self.in2) if not self.swap else (self.in2,self.in1)
        if abs(d) < 1e-3:
            self._digital(a,0); self._digital(b,0); self.pca.set_duty(self.pwm,0.0); return
        if d > 0:
            self._digital(a,1); self._digital(b,0); self.pca.set_duty(self.pwm,d)
        else:
            self._digital(a,0); self._digital(b,1); self.pca.set_duty(self.pwm,-d)

    def stop(self):
        a,b = (self.in1,self.in2) if not self.swap else (self.in2,self.in1)
        self._digital(a,0); self._digital(b,0); self.pca.set_duty(self.pwm,0.0)

class Kb:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
    def read_all(self, t=0.0):
        out=[]; r,_,_ = select.select([sys.stdin],[],[],t)
        while r:
            b=os.read(self.fd,1); 
            if not b: break
            out.append(b)
            r,_,_=select.select([sys.stdin],[],[],0.0)
        return out
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def main():
    ap = argparse.ArgumentParser("Teleop Pair-Mode (Left/Right)")
    ap.add_argument('--i2c-bus', type=int, default=1)
    ap.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    ap.add_argument('--pwm-freq', type=float, default=200.0)
    ap.add_argument('--set-freq', action='store_true')
    ap.add_argument('--reserve', type=str, default='12,13,14,15')
    # LEFT triplet
    ap.add_argument('--left-pwm', type=int, required=True)
    ap.add_argument('--left-in1', type=int, required=True)
    ap.add_argument('--left-in2', type=int, required=True)
    ap.add_argument('--inv-left', action='store_true')
    ap.add_argument('--swap-in12-left', action='store_true')
    # RIGHT triplet
    ap.add_argument('--right-pwm', type=int, required=True)
    ap.add_argument('--right-in1', type=int, required=True)
    ap.add_argument('--right-in2', type=int, required=True)
    ap.add_argument('--inv-right', action='store_true')
    ap.add_argument('--swap-in12-right', action='store_true')
    # control
    ap.add_argument('--hz', type=float, default=30.0)
    ap.add_argument('--deadman', type=float, default=1.5)
    ap.add_argument('--scale-low', type=float, default=0.25)
    ap.add_argument('--scale-high', type=float, default=1.0)
    a = ap.parse_args()

    p = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=a.set_freq)
    try:
        # reserve servos
        try:
            rs = [int(x) for x in a.reserve.split(',') if x.strip()!='']
            for ch in rs: p.full_off(ch)
            if rs: print(f"[Reserve] FULL-OFF: {rs}")
        except: pass

        L = Side(p, a.left_pwm,  a.left_in1,  a.left_in2,  invert=a.inv_left,  swap_in12=a.swap_in12_left)
        R = Side(p, a.right_pwm, a.right_in1, a.right_in2, invert=a.inv_right, swap_in12=a.swap_in12_right)

        def stop_all():
            L.stop(); R.stop()

        def pulse(side, duty, t=0.35):
            side.drive(duty); time.sleep(t); side.stop()

        kb = Kb()
        signal.signal(signal.SIGINT, lambda *_: (print("\n[Quit]"), kb.restore(), stop_all(), sys.exit(0)))
        print("[Ready] Pair-mode: W/S/Q/E, Space=stop, 1/2=Left fwd/rev, 3/4=Right fwd/rev")

        last = time.monotonic(); scale=1.0; mag=0.8
        while True:
            now = time.monotonic()
            if (now - last) > a.deadman:
                stop_all()
            for b in kb.read_all(0.02):
                last = now
                # modifiers
                if b.isalpha() and b.isupper(): scale = a.scale_high
                elif 1 <= b[0] <= 26:           scale = a.scale_low
                else:                            scale = 1.0
                # actions
                if b in (b' ',):
                    print("[Cmd] STOP"); stop_all()
                elif b in (b'w', b'W'):
                    L.drive(+mag*scale); R.drive(+mag*scale); print("[W] forward")
                elif b in (b's', b'S'):
                    L.drive(-mag*scale); R.drive(-mag*scale); print("[S] backward")
                elif b in (b'q', b'Q'):
                    L.drive(-mag*scale); R.drive(+mag*scale); print("[Q] turn CCW")
                elif b in (b'e', b'E'):
                    L.drive(+mag*scale); R.drive(-mag*scale); print("[E] turn CW")
                elif b == b'1':
                    print("[Pulse] LEFT +"); pulse(L, +0.35)
                elif b == b'2':
                    print("[Pulse] LEFT -"); pulse(L, -0.35)
                elif b == b'3':
                    print("[Pulse] RIGHT +"); pulse(R, +0.35)
                elif b == b'4':
                    print("[Pulse] RIGHT -"); pulse(R, -0.35)
                elif b in (b'\x1b', b'\x03'):
                    print("[Quit]"); kb.restore(); stop_all(); return
            time.sleep(max(0.0, 1.0/a.hz))
    finally:
        try: p.close()
        except: pass

if __name__ == "__main__":
    main()
