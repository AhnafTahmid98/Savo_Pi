#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — PCA9685 Triplet Mapper (DC motors)
Probes the three likely DC triplets: (0,1,2), (3,4,5), (6,7,8).
For each triplet:
  - Forward:  IN1=HIGH, IN2=LOW, PWM ~35% for 1.0s
  - Stop:     0.4s
  - Reverse:  IN1=LOW,  IN2=HIGH, PWM ~35% for 1.0s
It prints a big banner before each action so you can note which physical wheel moves.

NOTE:
- This does NOT touch channels 12..15 (kept FULL-OFF for servos).
- If your H-bridge happens to be active-low, you should still see motion on one of the two directions.
"""

import time, argparse
from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=200.0, set_freq=True):
        self.addr = int(addr)
        self.bus = SMBus(int(bus))
        self._w8(MODE2, OUTDRV)     # totem-pole
        self._w8(MODE1, AI)         # auto-increment
        time.sleep(0.003)
        if set_freq:
            self.set_pwm_freq(freq_hz)
        m1 = self._r8(MODE1)
        self._w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except: pass

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def _r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, f_hz: float):
        f = float(max(40.0, min(1500.0, f_hz)))
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
        self._w8(base+2, 0x00); self._w8(base+3, 0x10)  # OFF bit

    def full_on(self, ch: int):
        base = LED0_ON_L + 4*int(ch)
        self._w8(base+0, 0x00); self._w8(base+1, 0x10)  # ON bit
        self._w8(base+2, 0x00); self._w8(base+3, 0x00)

    def set_duty(self, ch: int, duty: float):
        d = max(0.0, min(1.0, float(duty)))
        if d <= 0.0:
            self.full_off(ch); return
        if d >= 1.0:
            self.full_on(ch);  return
        off = int(round(d * 4095))
        self._raw(ch, 0, off)

def triplet_forward(pca, t, duty, secs):
    pwm, in1, in2 = t
    print(f"\n=== TRIPLET {t} FORWARD: IN1=HIGH, IN2=LOW, PWM={duty:.2f} for {secs:.1f}s ===")
    pca.full_on(in1); pca.full_off(in2); pca.set_duty(pwm, duty); time.sleep(secs)
    pca.full_off(in1); pca.full_off(in2); pca.set_duty(pwm, 0.0)

def triplet_reverse(pca, t, duty, secs):
    pwm, in1, in2 = t
    print(f"=== TRIPLET {t} REVERSE: IN1=LOW, IN2=HIGH, PWM={duty:.2f} for {secs:.1f}s ===")
    pca.full_off(in1); pca.full_on(in2); pca.set_duty(pwm, duty); time.sleep(secs)
    pca.full_off(in1); pca.full_off(in2); pca.set_duty(pwm, 0.0)

def main():
    ap = argparse.ArgumentParser("PCA9685 Triplet Mapper")
    ap.add_argument('--i2c-bus', type=int, default=1)
    ap.add_argument('--pca-addr', type=lambda x:int(x,0), default=0x40)
    ap.add_argument('--pwm-freq', type=float, default=200.0)
    ap.add_argument('--duty', type=float, default=0.35)
    ap.add_argument('--hold', type=float, default=1.0)
    ap.add_argument('--gap', type=float, default=0.4)
    ap.add_argument('--reserve', type=str, default='12,13,14,15')
    a = ap.parse_args()

    p = PCA9685(bus=a.i2c_bus, addr=a.pca_addr, freq_hz=a.pwm_freq, set_freq=True)
    try:
        # keep servo channels off
        try:
            rs = [int(x) for x in a.reserve.split(',') if x.strip()!='']
            for ch in rs: p.full_off(ch)
            if rs: print(f"[Reserve] FULL-OFF: {rs}")
        except: pass

        triplets = [(0,1,2),(3,4,5),(6,7,8)]
        print("\nPlace robot on blocks. WATCH which wheel moves on each step and WRITE IT DOWN.")
        print("Order: (0,1,2) forward/reverse → gap → (3,4,5) → gap → (6,7,8) → done\n")

        for t in triplets:
            triplet_forward(p, t, a.duty, a.hold)
            time.sleep(a.gap)
            triplet_reverse(p, t, a.duty, a.hold)
            time.sleep(a.gap)

        print("\nDONE. Now you should have notes like:")
        print("  (0,1,2) → Left side? FL/RL? Which direction was forward?")
        print("  (3,4,5) → Right side? FR/RR? Which direction was forward?")
        print("  (6,7,8) → Which wheel(s) moved?")
        print("\nNext: use teleop_pairmode.py with the two triplets that clearly drive LEFT and RIGHT.")
    finally:
        p.close()

if __name__ == "__main__":
    main()
