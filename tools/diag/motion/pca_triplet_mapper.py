#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — PCA9685 Triplet Mapper (Interactive)
-------------------------------------------------
Goal: empirically discover working (PWM, IN1, IN2) triplets for each wheel,
      including forward (+) and reverse (−), on quirky H-bridge boards where
      reverse may need a different PWM+IN pair than forward.

What this does
- Iterates candidate triplets (PWM in --pwm-cands; IN1/IN2 in --in-cands, IN1!=IN2).
- Skips channels listed in --reserve (e.g., 8..15 servos).
- For each triplet, pulses FWD (+) then REV (−) and asks you what moved.
- You can report multiple wheels & signs (e.g., "fl+ rl- rr+"), with strength tags:
  "?" weak, "!" strong. Use "none" if nothing clear.
- Writes CSV rows (one per reported wheel/sign). If "none", writes a single row.

End summary
- Suggests per-wheel best triplet for (+) and (−), preferring STRONG > normal > weak,
  then by frequency.
- Shows a pairing hint if FL=RL and FR=RR look consistent.

Examples
--------
Fresh sweep over all channels (0..7), 3s pulses, duty 0.35:
  python3 tools/diag/motion/pca_triplet_mapper.py \
    --pca-addr 0x40 --i2c-bus 1 --pwm-freq 800 \
    --reserve 8,9,10,11,12,13,14,15 \
    --duty 0.35 --secs 3 --pause 0.5 \
    --csv /tmp/pca_map.csv --truncate

Narrow (if you already know likely regions):
  --pwm-cands 0,2,6,7 --in-cands 0,1,3,4,5,6,7

Answering prompt
- Single:  fr+!      (FR forward strong)
- Multiple: fl+ rl-  (FL forward, RL reverse)
- With weak: rr-?    (RR reverse weak)
- None:     none
- Repeat:   r        (repeat same test)
- Next:     n        (accept & go next without logging)
- Quit:     q

NOTE: Motor-safe channels are 0..7. Reserve 8..15 to protect servo pins.
"""

import argparse, csv, os, sys, time
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
try:
    from smbus2 import SMBus
except Exception:
    print("ERROR: smbus2 is required. Install: sudo apt install -y python3-smbus || pip3 install smbus2", file=sys.stderr)
    raise

# ---------------------- PCA9685 low-level ----------------------
MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, OUTDRV, ALLCALL = 0x80, 0x10, 0x20, 0x04, 0x01

class PCA:
    def __init__(self, bus=1, addr=0x40, freq=800.0, set_freq=True):
        self.addr, self.bus = int(addr), SMBus(int(bus))
        self.w8(MODE2, OUTDRV); self.w8(MODE1, AI); time.sleep(0.003)
        if set_freq:
            f = float(max(40.0, min(1500.0, freq)))
            prescale = int(max(3, min(255, round(25_000_000.0/(4096.0*f)-1.0))))
            old = self.r8(MODE1)
            self.w8(MODE1, (old & ~RESTART)|SLEEP)
            self.w8(PRESCALE, prescale)
            self.w8(MODE1, old & ~SLEEP)
            time.sleep(0.003)
            self.w8(MODE1, (old | RESTART | AI) & ~ALLCALL)
            print(f"[PCA] freq={f:.1f}Hz prescale={prescale}")
    def close(self):
        try: self.bus.close()
        except Exception: pass
    def w8(self, r, v): self.bus.write_byte_data(self.addr, r&0xFF, v&0xFF)
    def r8(self, r):    return self.bus.read_byte_data(self.addr, r&0xFF)
    def raw(self, ch, on, off):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0, on & 0xFF); self.w8(base+1, (on>>8)&0x0F)
        self.w8(base+2, off&0xFF);  self.w8(base+3, (off>>8)&0x0F)
    def full_on(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0,0); self.w8(base+1,0x10); self.w8(base+2,0); self.w8(base+3,0)
    def full_off(self, ch):
        base = LED0_ON_L + 4*int(ch)
        self.w8(base+0,0); self.w8(base+1,0); self.w8(base+2,0); self.w8(base+3,0x10)
    def set_duty(self, ch, duty):
        duty = max(0.0, min(1.0, float(duty))); off = int(round(duty*4095))
        if off<=0: self.full_off(ch)
        elif off>=4095: self.full_on(ch)
        else: self.raw(ch, 0, off)

def dig(pca: PCA, ch: int, level: int, active_low: bool=False):
    if active_low: level ^= 1
    (pca.full_on if level else pca.full_off)(ch)

# ---------------------- CSV helper ----------------------
CSV_FIELDS = [
    "timestamp","phase","intended_sign","+1/-1","pwm","in1","in2","duty","pulse",
    "answer_wheel","answer_sign","ans(+1/-1)","strength"
]

def csv_write_rows(path: str, rows: List[Dict], truncate: bool=False):
    exists = os.path.exists(path)
    if truncate and exists:
        os.remove(path)
        exists = False
    need_header = not exists
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if need_header:
            w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[CSV] Saved: {path}")

# ---------------------- Parse answer ----------------------
# tokens: fl+, fr-, rl+!, rr-?, none, r, n, q
WHEELS = {"fl","fr","rl","rr"}
def parse_tokens(ans: str) -> Tuple[str, List[Tuple[str,int,str]]]:
    s = ans.strip().lower().replace(",", " ")
    s = " ".join(s.split())
    if s in ("q",): return "quit", []
    if s in ("r",): return "repeat", []
    if s in ("n",): return "next", []
    if s in ("none",): return "ok", []  # no movements reported
    tokens = s.split()
    out = []
    for t in tokens:
        # allow forms like fl+, fr-?, rr+!, rl-
        wheel = None; sign = 0; strength = "normal"
        if len(t) >= 3 and t[:2] in WHEELS and t[2] in ("+","-"):
            wheel = t[:2]
            sign = +1 if t[2]=="+" else -1
            if len(t) >= 4:
                if "!" in t[3:]: strength = "strong"
                elif "?" in t[3:]: strength = "weak"
            out.append((wheel, sign, strength))
        else:
            # ignore bad token but we could warn
            pass
    if not out:
        return "bad", []
    return "ok", out

# ---------------------- Scorebook ----------------------
@dataclass(frozen=True)
class Triplet:
    pwm: int; in1: int; in2: int

def score_strength(s: str) -> int:
    return {"weak": 0, "normal": 1, "strong": 2}.get(s, 1)

# scorebook[wheel]['+'][Triplet] = (count, best_strength_score)
def suggest_from_scores(scorebook):
    def best_for(wheel: str, sign: str) -> Optional[Triplet]:
        bucket = scorebook.get(wheel, {}).get(sign, {})
        if not bucket: return None
        # Prefer stronger strength, then higher count
        # bucket[t] = (count, best_strength_score)
        best = sorted(bucket.items(),
                      key=lambda kv: (kv[1][1], kv[1][0]), reverse=True)
        return best[0][0]
    return {
        "FL": {"+": best_for("fl","+"), "-": best_for("fl","-")},
        "FR": {"+": best_for("fr","+"), "-": best_for("fr","-")},
        "RL": {"+": best_for("rl","+"), "-": best_for("rl","-")},
        "RR": {"+": best_for("rr","+"), "-": best_for("rr","-")},
    }

def print_suggestions(sug):
    print("\n=== SUGGESTED TRIPLETS (with signs) ===")
    def fmt(t): 
        return "None" if t is None else f"({t.pwm},{t.in1},{t.in2})"
    print(f" FL: + {fmt(sug['FL']['+'])}    - {fmt(sug['FL']['-'])}")
    print(f" FR: + {fmt(sug['FR']['+'])}    - {fmt(sug['FR']['-'])}")
    print(f" RL: + {fmt(sug['RL']['+'])}    - {fmt(sug['RL']['-'])}")
    print(f" RR: + {fmt(sug['RR']['+'])}    - {fmt(sug['RR']['-'])}")

    # Pairing hint
    flp, rlp = sug["FL"]["+"], sug["RL"]["+"]
    flm, rlm = sug["FL"]["-"], sug["RL"]["-"]
    frp, rrp = sug["FR"]["+"], sug["RR"]["+"]
    frm, rrm = sug["FR"]["-"], sug["RR"]["-"]
    def same(a,b): 
        return (a is not None and b is not None and a == b)
    res = []
    if same(flp,rlp): res.append("FL=RL looks paired for Forward")
    if same(flm,rlm): res.append("FL=RL looks paired for Reverse")
    if same(frp,rrp): res.append("FR=RR looks paired for Forward")
    if same(frm,rrm): res.append("FR=RR looks paired for Reverse")
    print("\n=== PAIRING RESULT ===")
    print(" OK: " + "; ".join(res) if res else " Mixed/Ambiguous. Narrow search or re-test.")

    print("\n=== COPY-PASTE (if correct) ===")
    def cp(name):
        return (f" --{name}-fwd-pwm {getattr(sug[name]['+'],'pwm', 'X')} "
                f"--{name}-fwd-in1 {getattr(sug[name]['+'],'in1', 'X')} "
                f"--{name}-fwd-in2 {getattr(sug[name]['+'],'in2', 'X')} "
                f"--{name}-rev-pwm {getattr(sug[name]['-'],'pwm', 'X')} "
                f"--{name}-rev-in1 {getattr(sug[name]['-'],'in1', 'X')} "
                f"--{name}-rev-in2 {getattr(sug[name]['-'],'in2', 'X')}")
    print(cp("fl"))
    print(cp("fr"))
    print(cp("rl"))
    print(cp("rr"))
    print()

# ---------------------- Core test pulse ----------------------
def pulse_triplet(pca: PCA, trip: Triplet, sign: int, duty: float, secs: float, active_low=False, pause=0.4):
    # quench everything related to this triplet
    for ch in (trip.in1, trip.in2, trip.pwm):
        pca.full_off(ch)
    time.sleep(0.05)

    # set direction on INs (H-bridge truth table)
    # Forward: IN1=1, IN2=0; Reverse: IN1=0, IN2=1
    if sign >= 0:
        dig(pca, trip.in1, 1, active_low); dig(pca, trip.in2, 0, active_low)
        phase = "fwd"; intended = "+"
    else:
        dig(pca, trip.in1, 0, active_low); dig(pca, trip.in2, 1, active_low)
        phase = "rev"; intended = "-"

    # PWM on the triplet's PWM channel
    pca.set_duty(trip.pwm, duty)
    print(f"[TEST] {phase.upper()} ({'+' if sign>=0 else '−'})  triplet=({trip.pwm},{trip.in1},{trip.in2})  duty={duty:.2f}  for {secs:.2f}s")
    t0 = time.time()
    try:
        while (time.time() - t0) < secs:
            time.sleep(0.02)
    finally:
        # stop & short pause between phases
        pca.set_duty(trip.pwm, 0.0)
        dig(pca, trip.in1, 0, active_low); dig(pca, trip.in2, 0, active_low)
        time.sleep(pause)
    return phase, intended

# ---------------------- Argparse ----------------------
def build_args():
    ap = argparse.ArgumentParser("PCA9685 triplet mapper (interactive)")
    ap.add_argument("--i2c-bus", type=int, default=1)
    ap.add_argument("--pca-addr", type=lambda x:int(x,0), default=0x40)
    ap.add_argument("--pwm-freq", type=float, default=800.0)
    ap.add_argument("--set-freq", action="store_true", default=True)
    ap.add_argument("--active-low", action="store_true")

    # Candidate sets (defaults = full 0..7 as requested)
    ap.add_argument("--pwm-cands", type=str, default="0,1,2,3,4,5,6,7",
                    help="PWM channels to test (default: all 0..7)")
    ap.add_argument("--in-cands",  type=str, default="0,1,2,3,4,5,6,7",
                    help="IN channels to test (default: all 0..7)")

    # Safety reserves
    ap.add_argument("--reserve", type=str, default="8,9,10,11,12,13,14,15",
                    help="Comma list of channels to FULL-OFF and skip")

    # Pulse config
    ap.add_argument("--duty", type=float, default=0.35)
    ap.add_argument("--secs", type=float, default=2.0)
    ap.add_argument("--pause", type=float, default=0.5)

    # CSV logging
    ap.add_argument("--csv", type=str, default="/tmp/pca_map.csv")
    ap.add_argument("--truncate", action="store_true", help="Delete CSV first")

    return ap.parse_args()

# ---------------------- Main ----------------------
def main():
    args = build_args()

    # Parse lists
    def parse_list(s):
        return sorted({int(x) for x in str(s).split(",") if str(x).strip()!=""})
    pwm_cands = parse_list(args.pwm_cands)
    in_cands  = parse_list(args.in_cands)
    reserved  = set(parse_list(args.reserve))

    # Open PCA, reserve OFF
    p = PCA(args.i2c_bus, args.pca_addr, args.pwm_freq, args.set_freq)
    if reserved:
        for ch in reserved:
            p.full_off(ch)
        print(f"[Reserve] FULL-OFF: {sorted(reserved)}")

    # CSV init
    if args.truncate and os.path.exists(args.csv):
        os.remove(args.csv)
    # Scorebook
    score = {w: {"+": {}, "-": {}} for w in WHEELS}  # dict of dicts

    test_id = 0
    def log_rows(ts, phase, intended_sign, sign_val, trip, duty, secs, answers):
        # answers: list[(wheel, +1/-1, strength)] ; if empty => "none"
        rows = []
        if not answers:
            rows.append({
                "timestamp": ts, "phase": phase, "intended_sign": intended_sign, "+1/-1": sign_val,
                "pwm": trip.pwm, "in1": trip.in1, "in2": trip.in2, "duty": f"{duty:.3f}", "pulse": f"{secs:.3f}",
                "answer_wheel": "none", "answer_sign": "", "ans(+1/-1)": "", "strength": "normal"
            })
        else:
            for (w, sgn, strength) in answers:
                rows.append({
                    "timestamp": ts, "phase": phase, "intended_sign": intended_sign, "+1/-1": sign_val,
                    "pwm": trip.pwm, "in1": trip.in1, "in2": trip.in2, "duty": f"{duty:.3f}", "pulse": f"{secs:.3f}",
                    "answer_wheel": w, "answer_sign": "+" if sgn>0 else "-", "ans(+1/-1)": sgn, "strength": strength
                })
        csv_write_rows(args.csv, rows, truncate=False)

        # Update scorebook
        for (w, sgn, strength) in answers:
            sign_key = "+" if sgn > 0 else "-"
            bucket = score[w][sign_key]
            key = Triplet(trip.pwm, trip.in1, trip.in2)
            cnt, best = bucket.get(key, (0, -1))
            sc = score_strength(strength)
            bucket[key] = (cnt+1, max(best, sc))

    try:
        for pwm in pwm_cands:
            if pwm in reserved: continue
            for in1 in in_cands:
                if in1 in reserved or in1 == pwm: continue
                for in2 in in_cands:
                    if in2 in reserved or in2 == in1 or in2 == pwm: continue
                    trip = Triplet(pwm, in1, in2)

                    # FWD phase
                    test_id += 1
                    phase, intended = pulse_triplet(p, trip, +1, args.duty, args.secs, args.active_low, args.pause)
                    while True:
                        try:
                            ans = input("  moved? (fl+/fl-/fr+/fr-/rl+/rl-/rr+/rr- [add ?/!], or none; r=repeat, n=next, q=quit): ").strip()
                        except KeyboardInterrupt:
                            print("\n[INTERRUPT] Writing CSV and quitting...")
                            print_suggestions(suggest_from_scores(score))
                            return
                        status, tokens = parse_tokens(ans)
                        if status == "quit":
                            print_suggestions(suggest_from_scores(score))
                            return
                        if status == "repeat":
                            phase, intended = pulse_triplet(p, trip, +1, args.duty, args.secs, args.active_low, args.pause)
                            continue
                        if status == "bad":
                            print("  Please type valid tokens: fl+/fl-/fr+/fr-/rl+/rl-/rr+/rr- (add ? weak / ! strong), or none/r/n/q.")
                            continue
                        # status ok or next
                        if status == "ok":
                            ts = time.strftime("%Y-%m-%dT%H:%M:%S")
                            log_rows(ts, phase, intended, +1, trip, args.duty, args.secs, tokens)
                        break  # next phase

                    # REV phase
                    phase, intended = pulse_triplet(p, trip, -1, args.duty, args.secs, args.active_low, args.pause)
                    while True:
                        try:
                            ans = input("  moved? (fl+/fl-/fr+/fr-/rl+/rl-/rr+/rr- [add ?/!], or none; r=repeat, n=next, q=quit): ").strip()
                        except KeyboardInterrupt:
                            print("\n[INTERRUPT] Writing CSV and quitting...")
                            print_suggestions(suggest_from_scores(score))
                            return
                        status, tokens = parse_tokens(ans)
                        if status == "quit":
                            print_suggestions(suggest_from_scores(score))
                            return
                        if status == "repeat":
                            phase, intended = pulse_triplet(p, trip, -1, args.duty, args.secs, args.active_low, args.pause)
                            continue
                        if status == "bad":
                            print("  Please type valid tokens: fl+/fl-/fr+/fr-/rl+/rl-/rr+/rr- (add ? weak / ! strong), or none/r/n/q.")
                            continue
                        if status == "ok":
                            ts = time.strftime("%Y-%m-%dT%H:%M:%S")
                            log_rows(ts, phase, intended, -1, trip, args.duty, args.secs, tokens)
                        break  # next triplet

    finally:
        p.close()
        # Final suggestions
        print_suggestions(suggest_from_scores(score))

if __name__ == "__main__":
    main()
