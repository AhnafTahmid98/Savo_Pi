#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 Triplet Mapper + Pairing Detector (with sign labels)
------------------------------------------------------------
- Systematically tests (PWM, IN1, IN2) for FWD and REV.
- Prints sign for each pulse: FWD → "+"  |  REV → "−"
- You can respond: fl, fr, rl, rr, none, multi
  OR include sign: fl+, fl-, fr+, fr-, rl+, rl-, rr+, rr-
- Summarizes best triplets per wheel with signs:
    FL: + (p,i1,i2)    - (p,i1,i2)
- Infers pairing: INDEPENDENT / SIDE-PAIRED / DIAGONAL-PAIRED / MIXED

Safety:
- Low duty + short pulses; quench between tests.
- Skips reserved 8..15 by default (servos).

Examples:
  python3 tools/diag/motion/pca_triplet_mapper.py \
    --pca-addr 0x40 --i2c-bus 1 --pwm-freq 800 \
    --reserve 8,9,10,11,12,13,14,15 \
    --pulse 3.0 --duty 0.22 --pause 0.70 \
    --pwm-cands 0,1,2,6,7 \
    --in-cands 0,1,3,4,5,6,7
"""

import argparse, sys, time, itertools
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus  OR  pip3 install smbus2", file=sys.stderr)
    raise

# ---------- PCA9685 minimal ----------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA:
    def __init__(self, bus=1, addr=0x40, freq=800.0, set_freq=True):
        self.addr, self.bus = int(addr), SMBus(int(bus))
        self.w8(MODE2, OUTDRV); self.w8(MODE1, AI); time.sleep(0.003)
        if set_freq:
            prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*freq)-1.0))))
            old = self.r8(MODE1); self.w8(MODE1, (old & ~RESTART)|SLEEP)
            self.w8(PRESCALE, prescale); self.w8(MODE1, old & ~SLEEP)
            time.sleep(0.003); self.w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
        m1 = self.r8(MODE1)
        self.w8(MODE1, (m1 | RESTART | AI) & ~ALLCALL)

    def w8(self, r, v): self.bus.write_byte_data(self.addr, r & 0xFF, v & 0xFF)
    def r8(self, r):    return self.bus.read_byte_data(self.addr, r & 0xFF)

    def raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, on & 0xFF); self.w8(base+1, (on>>8)&0x0F)
        self.w8(base+2, off & 0xFF);  self.w8(base+3, (off>>8)&0x0F)

    def full_on(self, ch):  self.raw(ch, 0, 0); self.w8(LED0_ON_L+4*ch+1, 0x10)
    def full_off(self, ch): self.raw(ch, 0, 0); self.w8(LED0_ON_L+4*ch+3, 0x10)

    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty))); off = int(round(duty*4095))
        if off <= 0: self.full_off(ch)
        elif off >= 4095: self.full_on(ch)
        else: self.raw(ch, 0, off)

# ---------- helpers ----------
def digital(p: PCA, ch: int, level: int, active_low=False):
    if active_low: level ^= 1
    (p.full_on if level else p.full_off)(ch)

def quench_triplet(p: PCA, pwm: int, in1: int, in2: int, active_low=False):
    digital(p, in1, 0, active_low); digital(p, in2, 0, active_low)
    p.set_duty(pwm, 0.0)

def reserve_channels(p: PCA, reserve: str):
    if not reserve: return
    chans = [int(x) for x in str(reserve).split(',') if x.strip()!='']
    for ch in chans: p.full_off(ch)
    if chans: print(f"[Reserve] OFF: {sorted(chans)}")

def parse_list(arg: Optional[str], default: List[int]) -> List[int]:
    if not arg: return list(default)
    out = []
    for tok in arg.split(','):
        tok = tok.strip()
        if tok == '': continue
        out.append(int(tok))
    return out

def ask_moved(prompt: str) -> Tuple[str, Optional[int]]:
    """
    Returns (wheel, sign_override)
     - wheel: 'fl','fr','rl','rr','none','multi'
     - sign_override: +1, -1, or None (if no explicit +/- provided)
    Accepts entries like: fl, fr, rl, rr, none, multi, or fl+, fr-, rl+, rr-
    """
    valid = {'fl','fr','rl','rr','none','multi',
             'fl+','fl-','fr+','fr-','rl+','rl-','rr+','rr-'}
    while True:
        s = input(prompt).strip().lower()
        if s in valid:
            if s.endswith('+'): return (s[:-1], +1)
            if s.endswith('-'): return (s[:-1], -1)
            return (s, None)
        print("Please type: fl, fr, rl, rr, none, multi OR add sign: fl+, fr-, rl+, rr-")

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser("PCA9685 Triplet Mapper + Pairing Detector (signed)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pca-addr", type=lambda x:int(x,0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=800.0)
    ap.add_argument("--set-freq", action="store_true", default=True)
    ap.add_argument("--reserve", type=str, default="8,9,10,11,12,13,14,15")
    ap.add_argument("--active-low", action="store_true")

    # search spaces (motors generally 0..7)
    ap.add_argument("--pwm-cands", type=str, default="0,1,2,3,4,5,6,7")
    ap.add_argument("--in-cands",  type=str, default="0,1,2,3,4,5,6,7")

    # pulse profile
    ap.add_argument("--duty", type=float, default=0.18)
    ap.add_argument("--pulse", type=float, default=0.50)
    ap.add_argument("--pause", type=float, default=0.30)

    # limit scans
    ap.add_argument("--max-tests", type=int, default=0, help="Stop after N triplets (0 = no limit)")
    args = ap.parse_args()

    p = PCA(args.i2c_bus, args.pca_addr, args.pwm_freq, args.set_freq)
    try:
        reserve_channels(p, args.reserve)
        reserved = set(int(x) for x in args.reserve.split(',') if x.strip()!='')
        pwm_cands = [c for c in parse_list(args.pwm_cands, list(range(8))) if c not in reserved]
        in_cands  = [c for c in parse_list(args.in_cands,  list(range(8))) if c not in reserved]

        print("\nRobot on blocks. I’ll try (PWM,IN1,IN2) combos.")
        print("I label sign by pulse type: FWD → '+', REV → '−'.")
        print("You can override with: fl+, fr-, rl+, rr- (if observed sign differs).")
        print("If multiple moved, answer 'multi'; if nothing, answer 'none'.\n")

        # Best hits per wheel/direction (we store first reliable)
        best = {
            'fl+': None, 'fl-': None,
            'fr+': None, 'fr-': None,
            'rl+': None, 'rl-': None,
            'rr+': None, 'rr-': None,
        }

        # For pairing inference
        answers = []  # list of (who, sign, triplet, phase)
        tested = 0

        for pwm, in1, in2 in itertools.product(pwm_cands, in_cands, in_cands):
            if in1 == in2: continue
            if pwm in (in1, in2): continue  # avoid overlap within a triplet

            # ---- FWD pulse: label '+' ----
            quench_triplet(p, pwm, in1, in2, args.active_low)
            time.sleep(0.05)
            digital(p, in1, 1, args.active_low)
            digital(p, in2, 0, args.active_low)
            p.set_duty(pwm, args.duty)
            print(f"[TEST] FWD  (+) → triplet=({pwm},{in1},{in2}) duty={args.duty:.2f} for {args.pulse:.2f}s")
            time.sleep(args.pulse)
            quench_triplet(p, pwm, in1, in2, args.active_low)

            who, override = ask_moved("  moved? (fl/fr/rl/rr/none/multi or fl+/fr-/...): ")
            eff_sign = override if override in (+1,-1) else +1
            answers.append((who, eff_sign, (pwm,in1,in2), 'fwd'))
            if who in ('fl','fr','rl','rr') and eff_sign == +1:
                key = f"{who}+"
                if best[key] is None:
                    best[key] = (pwm,in1,in2)

            tested += 1
            if args.max_tests and tested >= args.max_tests: break
            time.sleep(args.pause)

            # ---- REV pulse: label '−' ----
            quench_triplet(p, pwm, in1, in2, args.active_low)
            time.sleep(0.05)
            digital(p, in1, 0, args.active_low)
            digital(p, in2, 1, args.active_low)
            p.set_duty(pwm, args.duty)
            print(f"[TEST] REV  (−) → triplet=({pwm},{in1},{in2}) duty={args.duty:.2f} for {args.pulse:.2f}s")
            time.sleep(args.pulse)
            quench_triplet(p, pwm, in1, in2, args.active_low)

            who, override = ask_moved("  moved? (fl/fr/rl/rr/none/multi or fl+/fr-/...): ")
            eff_sign = override if override in (+1,-1) else -1
            answers.append((who, eff_sign, (pwm,in1,in2), 'rev'))
            if who in ('fl','fr','rl','rr') and eff_sign == -1:
                key = f"{who}-"
                if best[key] is None:
                    best[key] = (pwm,in1,in2)

            tested += 1
            if args.max_tests and tested >= args.max_tests: break
            time.sleep(args.pause)

        # ------------ Summary ------------
        def fmt(t): 
            return "None" if t is None else f"({t[0]},{t[1]},{t[2]})"

        print("\n=== SUGGESTED TRIPLETS (with signs) ===")
        print(f" FL: + {fmt(best['fl+'])}    - {fmt(best['fl-'])}")
        print(f" FR: + {fmt(best['fr+'])}    - {fmt(best['fr-'])}")
        print(f" RL: + {fmt(best['rl+'])}    - {fmt(best['rl-'])}")
        print(f" RR: + {fmt(best['rr+'])}    - {fmt(best['rr-'])}")

        # Pairing inference (coarse, from answer sequence)
        side_pairs = {('fl','rl'), ('rl','fl'), ('fr','rr'), ('rr','fr')}
        diag_pairs = {('fl','rr'), ('rr','fl'), ('fr','rl'), ('rl','fr')}
        side_votes = 0
        diag_votes = 0

        seen = [w for (w,_,_,_) in answers if w in ('fl','fr','rl','rr')]
        for a, b in zip(seen, seen[1:]):
            if (a,b) in side_pairs: side_votes += 1
            if (a,b) in diag_pairs: diag_votes += 1

        print("\n=== PAIRING RESULT ===")
        if side_votes >= diag_votes + 2:
            print(" Likely SIDE-PAIRED (FL=RL, FR=RR).")
        elif diag_votes >= side_votes + 2:
            print(" Likely DIAGONAL-PAIRED (FL=RR, FR=RL).")
        elif all(best[k] is not None for k in ('fl+','fl-','fr+','fr-','rl+','rl-','rr+','rr-')):
            print(" Likely INDEPENDENT (each wheel separate).")
        else:
            print(" Mixed/Ambiguous. Narrow search or re-test.")

        # Handy CLI lines to copy into your paired teleop later:
        print("\n=== COPY-PASTE (if correct) ===")
        def line(w):
            p = best[w+'+']; r = best[w+'-']
            if p and r:
                print(f" --{w}-fwd-pwm {p[0]} --{w}-fwd-in1 {p[1]} --{w}-fwd-in2 {p[2]}  "
                      f"--{w}-rev-pwm {r[0]} --{w}-rev-in1 {r[1]} --{w}-rev-in2 {r[2]}")
        for w in ('fl','fr','rl','rr'):
            line(w)

    finally:
        try: p.bus.close()
        except Exception: pass

if __name__ == "__main__":
    main()
