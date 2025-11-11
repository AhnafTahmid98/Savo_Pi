#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 Triplet Mapper — multi-answer edition (0..7 defaults)
-------------------------------------------------------------
Map (PWM, IN1, IN2) triplets that move each wheel (FL/FR/RL/RR) in + (forward) or - (reverse).

Answer tokens (space-separated, one line per pulse):
  fl+ fl- fr+ fr- rl+ rl- rr+ rr-     (optionally add '?' weak or '!' strong)
Special: r=repeat  n=next  q=quit  none=log no-move for this triplet

Defaults:
  pwm-cands: 0..7
  in-cands : 0..7
  reserve  : 8..15 (servos off)
  duty: 0.35   secs: 3.0   pause: 0.5
"""

import argparse, csv, datetime as dt, os, sys, time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 missing. Install: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# ---------------- PCA9685 low-level ----------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA9685:
    def __init__(self, bus=1, addr=0x40, freq_hz=800.0, set_freq=True):
        self.addr = int(addr); self.bus = SMBus(int(bus))
        self.w8(MODE2, OUTDRV); self.w8(MODE1, AI); time.sleep(0.003)
        if set_freq: self.set_pwm_freq(freq_hz)
        m1 = self.r8(MODE1); self.w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def close(self):
        try: self.bus.close()
        except Exception: pass

    def w8(self, reg, val): self.bus.write_byte_data(self.addr, reg & 0xFF, val & 0xFF)
    def r8(self, reg):      return self.bus.read_byte_data(self.addr, reg & 0xFF)

    def set_pwm_freq(self, f_hz):
        f = float(max(40.0, min(1500.0, f_hz)))
        prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
        old = self.r8(MODE1)
        self.w8(MODE1, (old & ~RESTART) | SLEEP)
        self.w8(PRESCALE, prescale)
        self.w8(MODE1, old & ~SLEEP)
        time.sleep(0.003)
        self.w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")

    def _raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, on & 0xFF); self.w8(base+1, (on>>8)&0x0F)
        self.w8(base+2, off&0xFF);  self.w8(base+3, (off>>8)&0x0F)

    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0,0); self.w8(base+1,0); self.w8(base+2,0); self.w8(base+3,0x10)

    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0,0); self.w8(base+1,0x10); self.w8(base+2,0); self.w8(base+3,0)

    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty)))
        off = int(round(duty * 4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self._raw(ch, 0, off)

# ---------------- helpers ----------------
def digital(pca: PCA9685, ch: int, level: int, active_low: bool):
    if active_low: level ^= 1
    (pca.full_on if level else pca.full_off)(ch)

def ts_now():
    return dt.datetime.now().isoformat(timespec="seconds")

def parse_csv_list(s: str):
    return [int(x.strip()) for x in s.split(',') if x.strip()!='']

def safe_sleep(sec: float):
    t0 = time.monotonic()
    while (time.monotonic() - t0) < sec:
        time.sleep(0.01)

# ---------------- CLI ----------------
def build_argparser():
    ap = argparse.ArgumentParser("PCA9685 triplet mapper (multi-answer, 0..7 defaults)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pca-addr", type=lambda x:int(x,0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=800.0)
    ap.add_argument("--set-freq", action="store_true", default=True)

    ap.add_argument("--reserve", type=str, default="8,9,10,11,12,13,14,15", help="Channels to FULL-OFF at start (servos)")
    ap.add_argument("--pwm-cands", type=str, default="0,1,2,3,4,5,6,7", help="Comma list of PWM candidates")
    ap.add_argument("--in-cands",  type=str, default="0,1,2,3,4,5,6,7", help="Comma list of IN candidates")

    ap.add_argument("--duty", type=float, default=0.35)
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--pause", type=float, default=0.5)
    ap.add_argument("--active-low", action="store_true")
    ap.add_argument("--csv", type=str, default="/tmp/pca_map.csv")
    ap.add_argument("--phases", type=str, default="fwd,rev", help="Subset of {fwd,rev}")
    return ap

# ---------------- core ----------------
def main():
    args = build_argparser().parse_args()
    pwm_cands = parse_csv_list(args.pwm_cands)
    in_cands  = parse_csv_list(args.in_cands)
    phases = [p.strip() for p in args.phases.split(',') if p.strip()]

    pca = PCA9685(bus=args.i2c_bus, addr=args.pca_addr, freq_hz=args.pwm_freq, set_freq=args.set_freq)

    # reserve OFF (servos)
    reserved = set(parse_csv_list(args.reserve)) if args.reserve else set()
    for ch in reserved:
        pca.full_off(ch)
    if reserved:
        print(f"[Reserve] FULL-OFF: {sorted(reserved)}")

    # CSV init
    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    new_file = not os.path.exists(args.csv)
    f = open(args.csv, "a", newline="")
    writer = csv.writer(f)
    header = ["timestamp","phase","intended_sign","+1/-1","pwm","in1","in2","duty","pulse",
              "answer_tokens","answer_wheel","answer_sign","+1/-1","strength"]
    if new_file:
        writer.writerow(header); f.flush()

    def quench(pwm, in1, in2):
        digital(pca, in1, 0, args.active_low)
        digital(pca, in2, 0, args.active_low)
        pca.set_duty(pwm, 0.0)

    try:
        total = 0
        for pwm in pwm_cands:
            for in1 in in_cands:
                for in2 in in_cands:
                    if in1 == in2:
                        continue
                    for phase in phases:
                        total += 1
                        ts = ts_now()
                        intended_sign = '+' if phase == 'fwd' else '-'
                        intended_val  =  1 if phase == 'fwd' else -1
                        triplet = (pwm, in1, in2)

                        # prepare
                        quench(pwm, in1, in2)

                        # drive
                        if phase == 'fwd':
                            digital(pca, in1, 1, args.active_low)
                            digital(pca, in2, 0, args.active_low)
                        else:
                            digital(pca, in1, 0, args.active_low)
                            digital(pca, in2, 1, args.active_low)

                        print(f"[TEST #{total}] {phase.upper():3} ({'+' if intended_val>0 else '−'})  triplet={triplet}  duty={args.duty:.2f}  for {args.secs:.2f}s")
                        pca.set_duty(pwm, args.duty)
                        safe_sleep(args.secs)
                        quench(pwm, in1, in2)

                        # ask
                        while True:
                            ans = input("  moved? (fl+/fl-/fr+/fr-/rl+/rl-/rr+/rr- [add ?/!], or none; r=repeat, n=next, q=quit): ").strip()
                            if not ans:
                                continue
                            low = ans.lower()
                            if low == 'q':
                                print("[Quit]"); f.flush(); f.close(); pca.close(); return
                            if low == 'r':
                                # repeat pulse
                                if phase == 'fwd':
                                    digital(pca, in1, 1, args.active_low)
                                    digital(pca, in2, 0, args.active_low)
                                else:
                                    digital(pca, in1, 0, args.active_low)
                                    digital(pca, in2, 1, args.active_low)
                                print(f"[REPEAT] {phase.upper():3}  triplet={triplet}  duty={args.duty:.2f}  for {args.secs:.2f}s")
                                pca.set_duty(pwm, args.duty)
                                safe_sleep(args.secs)
                                quench(pwm, in1, in2)
                                continue
                            if low in ('n','none'):
                                if low == 'none':
                                    writer.writerow([ts, phase, intended_sign, intended_val,
                                                     pwm, in1, in2, args.duty, args.secs,
                                                     'none', '', '', '', ''])
                                    f.flush()
                                break

                            # multi-token parse
                            tokens = ans.split()
                            ok_any = False
                            for tok in tokens:
                                tok = tok.strip().lower()
                                if tok in ('r','n','q','none'):
                                    continue
                                wheel = None; sign = None; strength = ''
                                if len(tok) >= 3 and tok[:2] in ('fl','fr','rl','rr'):
                                    wheel = tok[:2]
                                    rest = tok[2:]
                                    if not rest:
                                        print("  → add + or - after wheel (e.g., fl+, rr-)."); continue
                                    if   rest[0] == '+': sign = '+'
                                    elif rest[0] == '-': sign = '-'
                                    else:
                                        print("  → token must include + or - (e.g., fr+ or rl-)."); continue
                                    if len(rest) >= 2 and rest[1] in ('?','!'):
                                        strength = {'?':'weak','!':'strong'}[rest[1]]
                                else:
                                    print("  → use fl+/fr-/rl+/rr- (optional ?/!)."); continue

                                ok_any = True
                                signed = 1 if sign == '+' else -1
                                writer.writerow([ts, phase, intended_sign, intended_val,
                                                 pwm, in1, in2, args.duty, args.secs,
                                                 ans, wheel, sign, signed, strength or 'normal'])
                                f.flush()

                            if not ok_any:
                                continue
                            break

                        safe_sleep(args.pause)

    finally:
        try: f.flush(); f.close()
        except: pass
        pca.close()
        print(f"[CSV] Saved: {args.csv}")

if __name__ == "__main__":
    main()
