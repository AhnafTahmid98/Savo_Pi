#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Summarize PCA9685 triplet mapping results from CSV (no retest needed).
Prints:
  - SUGGESTED TRIPLETS (best + and - per wheel)
  - PAIRING RESULT (FL=RL, FR=RR hints)
  - COPY-PASTE block for drive_manual_direct.py (direction-triplet mode)

CSV is whatever pca_triplet_mapper.py produced. Columns used (case-insensitive):
  'phase' ('fwd'/'rev'), 'pwm','in1','in2','answer_wheel' (fl/fr/rl/rr),
  'answer_sign' ('+'/'-'), 'strength' ('weak'/'normal'/'strong').

Usage:
  python3 tools/diag/motion/pca_triplet_summarize.py --csv /tmp/pca_map.csv
"""
import argparse, csv, os, sys
from collections import defaultdict, Counter
from dataclasses import dataclass

@dataclass(frozen=True)
class Triplet:
    pwm: int; in1: int; in2: int

def norm(s: str) -> str:
    return (s or "").strip().lower()

def strength_weight(s: str) -> float:
    s = norm(s)
    if s.startswith("strong"): return 1.5
    if s.startswith("weak"):   return 0.5
    return 1.0  # normal/empty

def load_rows(path: str):
    if not os.path.exists(path):
        print(f"ERROR: CSV not found: {path}", file=sys.stderr)
        sys.exit(1)
    with open(path, newline="") as f:
        rdr = csv.DictReader(f)
        rows = []
        for r in rdr:
            try:
                rows.append({
                    "phase":         norm(r.get("phase","")),
                    "pwm":           int(r.get("pwm","-1")),
                    "in1":           int(r.get("in1","-1")),
                    "in2":           int(r.get("in2","-1")),
                    "answer_wheel":  norm(r.get("answer_wheel","")),
                    "answer_sign":   r.get("answer_sign","").strip()[:1],  # '+' or '-'
                    "strength":      norm(r.get("strength","normal")),
                })
            except Exception:
                # skip malformed lines silently
                continue
    return rows

def suggest_from_rows(rows):
    # score[(wheel, sign, triplet)] = sum of weights
    score = defaultdict(float)
    count = defaultdict(int)

    for r in rows:
        w = r["answer_wheel"]   # 'fl','fr','rl','rr' (user input)
        s = r["answer_sign"]    # '+' or '-'
        if w not in ("fl","fr","rl","rr"): 
            continue
        if s not in ("+","-"):
            continue
        pwm, in1, in2 = r["pwm"], r["in1"], r["in2"]
        if min(pwm,in1,in2) < 0: 
            continue
        wt = strength_weight(r["strength"])
        key = (w.upper(), s, Triplet(pwm,in1,in2))
        score[key] += wt
        count[key] += 1

    # pick best per wheel/sign
    best = {wheel: {"+": None, "-": None} for wheel in ("FL","FR","RL","RR")}
    for wheel in ("FL","FR","RL","RR"):
        for sign in ("+","-"):
            # gather candidates for (wheel, sign)
            cands = [(t,score[(wheel,sign,t)],count[(wheel,sign,t)])
                     for (_, sgn, t) in score.keys() if _==wheel and sgn==sign]
            if not cands:
                best[wheel][sign] = None
                continue
            # sort by (score desc, count desc), then by tuple to stabilize
            cands.sort(key=lambda x: (x[1], x[2], -x[0].pwm, -x[0].in1, -x[0].in2), reverse=True)
            best[wheel][sign] = cands[0][0]
    return best

def fmt_triplet(t: Triplet | None) -> str:
    return "None" if t is None else f"({t.pwm},{t.in1},{t.in2})"

def print_suggestions(sug):
    print("\n=== SUGGESTED TRIPLETS (with signs) ===")
    print(f" FL: + {fmt_triplet(sug['FL']['+'])}    - {fmt_triplet(sug['FL']['-'])}")
    print(f" FR: + {fmt_triplet(sug['FR']['+'])}    - {fmt_triplet(sug['FR']['-'])}")
    print(f" RL: + {fmt_triplet(sug['RL']['+'])}    - {fmt_triplet(sug['RL']['-'])}")
    print(f" RR: + {fmt_triplet(sug['RR']['+'])}    - {fmt_triplet(sug['RR']['-'])}")

    # Pairing hint (FL=RL / FR=RR) comparing triplets for same sign
    def same(a,b): return (a is not None and b is not None and a == b)
    res = []
    if same(sug["FL"]["+"], sug["RL"]["+"]): res.append("FL=RL looks paired for Forward")
    if same(sug["FL"]["-"], sug["RL"]["-"]): res.append("FL=RL looks paired for Reverse")
    if same(sug["FR"]["+"], sug["RR"]["+"]): res.append("FR=RR looks paired for Forward")
    if same(sug["FR"]["-"], sug["RR"]["-"]): res.append("FR=RR looks paired for Reverse")

    print("\n=== PAIRING RESULT ===")
    print(" OK: " + "; ".join(res) if res else " Mixed/Ambiguous. Narrow search or re-test.")

    # COPY-PASTE block (direction-triplet teleop)
    def cp(name_lo):
        name = name_lo.lower()
        W = name_lo.upper()
        fwd = sug[W]["+"]
        rev = sug[W]["-"]
        def orX(v): return v if v is not None else "X"
        return (f" --{name}-fwd-pwm {orX(getattr(fwd,'pwm',None))}"
                f" --{name}-fwd-in1 {orX(getattr(fwd,'in1',None))}"
                f" --{name}-fwd-in2 {orX(getattr(fwd,'in2',None))}"
                f" --{name}-rev-pwm {orX(getattr(rev,'pwm',None))}"
                f" --{name}-rev-in1 {orX(getattr(rev,'in1',None))}"
                f" --{name}-rev-in2 {orX(getattr(rev,'in2',None))}")
    print("\n=== COPY-PASTE (if correct) ===")
    print(cp("fl"))
    print(cp("fr"))
    print(cp("rl"))
    print(cp("rr"))
    print()

def main():
    ap = argparse.ArgumentParser(description="Summarize pca_triplet_mapper CSV into wiring suggestions.")
    ap.add_argument("--csv", required=True, help="Path to CSV produced by pca_triplet_mapper.py")
    args = ap.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        print("ERROR: CSV has no usable rows.", file=sys.stderr)
        sys.exit(2)

    sug = suggest_from_rows(rows)
    print_suggestions(sug)

if __name__ == "__main__":
    main()
