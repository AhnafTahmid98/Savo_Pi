#!/usr/bin/env python3
import argparse, time
from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA:
    def __init__(self, bus=1, addr=0x40, freq=200.0, set_freq=True):
        self.addr, self.bus = addr, SMBus(bus)
        self.w8(MODE2, OUTDRV); self.w8(MODE1, AI); time.sleep(0.003)
        if set_freq:
            prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*freq)-1.0))))
            old = self.r8(MODE1); self.w8(MODE1, (old & ~RESTART)|SLEEP)
            self.w8(PRESCALE, prescale); self.w8(MODE1, old & ~SLEEP)
            time.sleep(0.003); self.w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
    def w8(self, r, v): self.bus.write_byte_data(self.addr, r&0xFF, v&0xFF)
    def r8(self, r):    return self.bus.read_byte_data(self.addr, r&0xFF)
    def raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, on & 0xFF); self.w8(base+1, (on>>8)&0x0F)
        self.w8(base+2, off&0xFF);  self.w8(base+3, (off>>8)&0x0F)
    def full_on(self, ch):  self.raw(ch, 0, 0); self.w8(LED0_ON_L+4*ch+1, 0x10)
    def full_off(self, ch): self.raw(ch, 0, 0); self.w8(LED0_ON_L+4*ch+3, 0x10)
    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, duty)); off = int(round(duty*4095))
        if off<=0: self.full_off(ch)
        elif off>=4095: self.full_on(ch)
        else: self.raw(ch, 0, off)

def digital(pca, ch, level, active_low=False):
    if active_low: level ^= 1
    (pca.full_on if level else pca.full_off)(ch)

def main():
    ap = argparse.ArgumentParser("PCA9685 single-wheel pulse")
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
    args = ap.parse_args()

    p = PCA(args.i2c_bus, args.pca_addr, args.pwm_freq, args.set_freq)

    try:
        # idle LOW on both inputs
        digital(p, args.in1, 0, args.active_low)
        digital(p, args.in2, 0, args.active_low)
        p.set_duty(args.pwm, 0.0)

        # drive
        if args.dir == "fwd":
            digital(p, args.in1, 1, args.active_low)
            digital(p, args.in2, 0, args.active_low)
        else:
            digital(p, args.in1, 0, args.active_low)
            digital(p, args.in2, 1, args.active_low)
        p.set_duty(args.pwm, max(0.0, min(1.0, args.duty)))
        time.sleep(args.secs)

    finally:
        # STOP
        digital(p, args.in1, 0, args.active_low)
        digital(p, args.in2, 0, args.active_low)
        p.set_duty(args.pwm, 0.0)

if __name__ == "__main__":
    main()

