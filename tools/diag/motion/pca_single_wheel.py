#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 single-wheel pulse (safe quench + settle)
- Drives ONE wheel using a (PWM, IN1, IN2) triplet.
- dir=fwd -> IN1=1, IN2=0 ; dir=rev -> IN1=0, IN2=1
- Adds: preclear, settle-ms, channel validation, servo-lane warning.

Usage (example):
  python3 tools/diag/motion/pca_single_wheel.py \
    --pca-addr 0x40 --i2c-bus 1 --pwm-freq 800 \
    --pwm 0 --in1 2 --in2 7 --dir fwd --duty 0.35 --secs 2
"""

import argparse, time, sys
from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

MOTOR_SAFE_MIN = 0
MOTOR_SAFE_MAX = 15   # hardware range; we’ll warn for 8..15 by default
SERVO_LANES    = set(range(8,16))

class PCA:
    def __init__(self, bus=1, addr=0x40, freq=200.0, set_freq=True):
        self.addr, self.bus = addr, SMBus(bus)
        self.w8(MODE2, OUTDRV)
        self.w8(MODE1, AI)
        time.sleep(0.003)
        if set_freq:
            prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*freq)-1.0))))
            old = self.r8(MODE1)
            self.w8(MODE1, (old & ~RESTART)|SLEEP)
            self.w8(PRESCALE, prescale)
            self.w8(MODE1, old & ~SLEEP)
            time.sleep(0.003)
            self.w8(MODE1, (old | RESTART | AI) & ~ALLCALL)

    def w8(self, r, v): self.bus.write_byte_data(self.addr, r & 0xFF, v & 0xFF)
    def r8(self, r):    return self.bus.read_byte_data(self.addr, r & 0xFF)

    def raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, on & 0xFF); self.w8(base+1, (on>>8)&0x0F)
        self.w8(base+2, off & 0xFF); self.w8(base+3, (off>>8)&0x0F)

    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, 0); self.w8(base+1, 0x10)  # ON_H bit4=1
        self.w8(base+2, 0); self.w8(base+3, 0)

    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, 0); self.w8(base+1, 0)
        self.w8(base+2, 0); self.w8(base+3, 0x10)  # OFF_H bit4=1

    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0:
            self.full_off(ch)
        elif off >= 4095:
            self.full_on(ch)
        else:
            self.raw(ch, 0, off)

def digital(pca: PCA, ch: int, level: int, active_low=False):
    if active_low:
        level ^= 1
    (pca.full_on if level else pca.full_off)(ch)

def validate_channels(pwm, in1, in2):
    for label, ch in (("PWM",pwm),("IN1",in1),("IN2",in2)):
        if ch < MOTOR_SAFE_MIN or ch > MOTOR_SAFE_MAX:
            print(f"[ERROR] {label} channel {ch} outside 0..15", file=sys.stderr); sys.exit(2)
    if len({pwm, in1, in2}) < 3:
        print(f"[WARN] PWM/IN overlap inside triplet (pwm={pwm}, in1={in1}, in2={in2}). Verify board wiring.")
    hits = [x for x in (pwm,in1,in2) if x in SERVO_LANES]
    if hits:
        print(f"[WARN] Using servo lanes {sorted(set(hits))}. If these are servos, reserve/avoid 8..15.")

def preclear_all(p: PCA):
    # Turn everything fully off at startup (0..15)
    for ch in range(16):
        p.full_off(ch)

def main():
    ap = argparse.ArgumentParser("PCA9685 single-wheel pulse (safe)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pca-addr", type=lambda x:int(x,0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=200.0)
    ap.add_argument("--set-freq", action="store_true", default=True)
    ap.add_argument("--pwm", type=int, required=True)
    ap.add_argument("--in1", type=int, required=True)
    ap.add_argument("--in2", type=int, required=True)
    ap.add_argument("--dir", choices=["fwd","rev"], default="fwd")
    ap.add_argument("--duty", type=float, default=0.50)
    ap.add_argument("--secs", type=float, default=1.0)
    ap.add_argument("--active-low", action="store_true")
    ap.add_argument("--preclear", action="store_true", help="Fully OFF all 16 channels at start")
    ap.add_argument("--settle-ms", type=float, default=8.0, help="Delay after IN lines before PWM (ms)")
    args = ap.parse_args()

    validate_channels(args.pwm, args.in1, args.in2)

    p = PCA(args.i2c_bus, args.pca_addr, args.pwm_freq, args.set_freq)

    try:
        if args.preclear:
            preclear_all(p)
            print("[Init] Precleared all channels to OFF")

        # Quench everything for this triplet first
        p.set_duty(args.pwm, 0.0)
        digital(p, args.in1, 0, args.active_low)
        digital(p, args.in2, 0, args.active_low)

        # Drive direction: set inputs first, wait, then apply PWM
        if args.dir == "fwd":
            digital(p, args.in1, 1, args.active_low)
            digital(p, args.in2, 0, args.active_low)
        else:
            digital(p, args.in1, 0, args.active_low)
            digital(p, args.in2, 1, args.active_low)

        time.sleep(max(0.0, args.settle_ms) / 1000.0)  # small settle
        p.set_duty(args.pwm, max(0.0, min(1.0, args.duty)))
        time.sleep(max(0.0, args.secs))

    finally:
        # STOP: PWM=0 then both inputs LOW
        p.set_duty(args.pwm, 0.0)
        digital(p, args.in1, 0, args.active_low)
        digital(p, args.in2, 0, args.active_low)

if __name__ == "__main__":
    main()
