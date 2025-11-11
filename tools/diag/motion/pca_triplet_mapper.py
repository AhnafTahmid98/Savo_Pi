#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 Triplet Mapper + Pairing Detector (signed + CSV + redo)
---------------------------------------------------------------
- Systematically tests (PWM, IN1, IN2) for FWD and REV.
- Prints direction + sign per pulse: FWD → "+"  |  REV → "−"
- Interactive observation with redo/skip and strength tagging.
- Logs every pulse to CSV.
- Summarizes best (+/-) triplets per wheel (FL/FR/RL/RR) and infers pairing.

Controls / Answers:
  After each pulse, type one of:
    fl, fr, rl, rr, none, multi
    or with sign override: fl+, fr-, rl+, rr-
    optionally add strength:  fl+? (weak), fr-! (strong)
  Commands:
    r  -> repeat the same pulse
    n  -> next (skip/keep current observation)
    q  -> quit early (safe)

Examples:
  python3 tools/diag/motion/pca_triplet_mapper.py \
    --pca-addr 0x40 --i2c-bus 1 --pwm-freq 800 \
    --reserve 8,9,10,11,12,13,14,15 \
    --pulse 3.0 --duty 0.35 --pause 0.70 \
    --pwm-cands 0,1,2,6,7 \
    --in-cands 0,1,3,4,5,6,7 \
    --csv /tmp/pca_map.csv
"""

import argparse, sys, time, itertools, csv, datetime
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

def parse_answer(s: str) -> Tuple[str, Optional[int], str]:
    """
    Parse user answer string.
    Returns: (wheel, sign_override, strength)
      - wheel in {'fl','fr','rl','rr','none','multi','r','n','q'}
      - sign_override in {+1, -1, None}
      - strength in {'normal','weak','strong'}
    Syntax:
      wheel        -> fl, fr, rl, rr, none, multi
      wheel+/-     -> fl+, fr-, rl+, rr-
      strength     -> add '?' for weak, '!' for strong (e.g., fl+?, rr-!, fr!)
      commands     -> r (repeat), n (next), q (quit)
    """
    s = s.strip().lower()
    if s in ('r','n','q'):
        return (s, None, 'normal')
    strength = 'normal'
    if s.endswith('!'):
        strength = 'strong'
        s = s[:-1]
    elif s.endswith('?'):
        strength = 'weak'
        s = s[:-1]
    sign = None
    if s.endswith('+'): sign, s = +1, s[:-1]
    elif s.endswith('-'): sign, s = -1, s[:-1]
    valid = {'fl','fr','rl','rr','none','multi'}
    if s not in valid:
        return ('invalid', None, 'normal')
    return (s, sign, strength)

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser("PCA9685 Triplet Mapper + Pairing Detector (signed + CSV + redo)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pca-addr", type=lambda x:int(x,0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=800.0)
    ap.add_argument("--set-freq", action="store_true", default=True)
    ap.add_argument("--reserve", type=str, default="8,9,10,11,12,13,14,15")
    ap.add_argument("--active-low", action="store_true")

    # search spaces (motors generally 0..7)
    ap.add_argument("--pwm-cands", type=str, default="0,1,2,3,4,5,6,7")
    ap.add_argument("--in-cands",  type=str, default="0,1,2,3,4,5,6,7")

    # pulse profile (alias: --secs for --pulse)
    ap.add_argument("--duty", type=float, default=0.18)
    ap.add_argument("--pulse", type=float, default=0.50)
    ap.add_argument("--secs", type=float, default=None, help="Alias for --pulse")
    ap.add_argument("--pause", type=float, default=0.30)

    # limit scans
    ap.add_argument("--max-tests", type=int, default=0, help="Stop after N triplets (0 = no limit)")

    # CSV
    ap.add_argument("--csv", type=str, default="pca_triplet_map.csv")

    args = ap.parse_args()
    if args.secs is not None:
        args.pulse = float(args.secs)

    p = PCA(args.i2c_bus, args.pca_addr, args.pwm_freq, args.set_freq)
    csv_file = open(args.csv, "w", newline="")
    csvw = csv.writer(csv_file)
    csvw.writerow([
        "timestamp","phase","intended_sign","+1/-1",
        "pwm","in1","in2","duty","pulse",
        "answer_wheel","answer_sign","+1/-1","strength"
    ])

    try:
        reserve_channels(p, args.reserve)
        reserved = set(int(x) for x in args.reserve.split(',') if x.strip()!='')
        pwm_cands = [c for c in parse_list(args.pwm_cands, list(range(8))) if c not in reserved]
        in_cands  = [c for c in parse_list(args.in_cands,  list(range(8))) if c not in reserved]

        print("\nRobot on blocks. I’ll try (PWM,IN1,IN2) combos.")
        print("Labels: FWD (+)  |  REV (−)")
        print("Answer examples:")
        print("  fl+    (FL moved forward/positive)   fr-?  (FR moved reverse and weak)")
        print("  none   (no motion)                    multi (more than one moved)")
        print("Commands: r=repeat, n=next, q=quit\n")

        # Best hits per wheel/direction (first reliable)
        best = {
            'fl+': None, 'fl-': None,
            'fr+': None, 'fr-': None,
            'rl+': None, 'rl-': None,
            'rr+': None, 'rr-': None,
        }

        answers = []  # (wheel, sign, (pwm,in1,in2), phase, strength)
        tested = 0

        def do_pulse(phase, pwm, in1, in2):
            """phase in {'fwd','rev'}; returns (wheel, sign, strength) or control"""
            # Set signals
            quench_triplet(p, pwm, in1, in2, args.active_low)
            time.sleep(0.05)
            if phase == 'fwd':
                intended_sign = +1
                digital(p, in1, 1, args.active_low)
                digital(p, in2, 0, args.active_low)
                label = "FWD (+)"
            else:
                intended_sign = -1
                digital(p, in1, 0, args.active_low)
                digital(p, in2, 1, args.active_low)
                label = "REV (−)"

            p.set_duty(pwm, args.duty)
            print(f"[TEST] {label}  triplet=({pwm},{in1},{in2})  duty={args.duty:.2f}  for {args.pulse:.2f}s")
            t0 = time.time()
            time.sleep(args.pulse)
            quench_triplet(p, pwm, in1, in2, args.active_low)

            # Query until valid or control (r/n/q)
            while True:
                s = input("  moved? (fl/fr/rl/rr/none/multi or fl+/fr-/... ; r=repeat, n=next, q=quit): ")
                wheel, sign_override, strength = parse_answer(s)
                if wheel == 'invalid':
                    print("  Please type: fl, fr, rl, rr, none, multi (add +/-, ? weak, ! strong), or r/n/q")
                    continue
                if wheel == 'r':
                    # Repeat the very same pulse
                    return ('repeat', None, 'normal', intended_sign)
                if wheel == 'n':
                    # Mark as none; proceed
                    wheel, sign_override, strength = 'none', None, 'normal'
                elif wheel == 'q':
                    return ('quit', None, 'normal', intended_sign)

                # Effective sign = user override if provided, else intended
                eff_sign = sign_override if sign_override in (+1,-1) else intended_sign

                # CSV
                csvw.writerow([
                    datetime.datetime.now().isoformat(timespec="seconds"),
                    phase, '+' if intended_sign>0 else '-', intended_sign,
                    pwm, in1, in2, f"{args.duty:.3f}", f"{args.pulse:.3f}",
                    wheel, ('+' if (sign_override==+1) else ('-' if (sign_override==-1) else '')),
                    eff_sign, strength
                ])
                csv_file.flush()

                # Store best hits
                if wheel in ('fl','fr','rl','rr') and eff_sign in (+1,-1):
                    key = f"{wheel}{'+' if eff_sign>0 else '-'}"
                    if best[key] is None:
                        best[key] = (pwm, in1, in2)
                answers.append((wheel, eff_sign, (pwm,in1,in2), phase, strength))
                return (wheel, eff_sign, strength, intended_sign)

        # Iterate through triplets
        outer_break = False
        for pwm in pwm_cands:
            for in1 in in_cands:
                for in2 in in_cands:
                    if in1 == in2: 
                        continue
                    if pwm in (in1, in2):
                        continue  # avoid overlap within a triplet

                    # FWD then REV
                    while True:
                        ans = do_pulse('fwd', pwm, in1, in2)
                        if ans[0] == 'repeat': 
                            continue
                        if ans[0] == 'quit': 
                            outer_break = True
                        break
                    if outer_break: break

                    while True:
                        ans = do_pulse('rev', pwm, in1, in2)
                        if ans[0] == 'repeat': 
                            continue
                        if ans[0] == 'quit': 
                            outer_break = True
                        break
                    if outer_break: break

                    tested += 1
                    if args.max_tests and tested >= args.max_tests:
                        outer_break = True
                        break
                    time.sleep(args.pause)
                if outer_break: break
            if outer_break: break

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
        seen = [w for (w,_,_,_,_) in answers if w in ('fl','fr','rl','rr')]
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
            p_trip = best[w+'+']; r_trip = best[w+'-']
            if p_trip and r_trip:
                print(f" --{w}-fwd-pwm {p_trip[0]} --{w}-fwd-in1 {p_trip[1]} --{w}-fwd-in2 {p_trip[2]}  "
                      f"--{w}-rev-pwm {r_trip[0]} --{w}-rev-in1 {r_trip[1]} --{w}-rev-in2 {r_trip[2]}")
        for w in ('fl','fr','rl','rr'):
            line(w)

        print(f"\n[CSV] Saved: {args.csv}")

    finally:
        try: csv_file.close()
        except Exception: pass
        try: p.bus.close()
        except Exception: pass

if __name__ == "__main__":
    main()
