#!/usr/bin/env python3
import argparse, os
import pandas as pd
from itertools import combinations

def main():
    ap = argparse.ArgumentParser(description="Summarize independent vs pair-mode movements from mapper CSV")
    ap.add_argument("--csv", default="/tmp/pca_map.csv", help="Path to CSV produced by pca_triplet_mapper.py")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print(f"ERROR: CSV not found: {args.csv}\n"
              f"• Either pass the correct path with --csv <file>\n"
              f"• Or re-run the mapper with: --csv /tmp/pca_map.csv --truncate")
        return

    df = pd.read_csv(args.csv)

    # Keep rows where a wheel was reported (not 'none')
    moves = df[df["answer_wheel"].astype(str).str.lower() != "none"].copy()
    if moves.empty:
        print("No movement rows found in CSV. Did you answer prompts with fl+/fr-/... ?")
        return

    # Normalize
    moves["answer_wheel"] = moves["answer_wheel"].str.lower()
    moves["answer_sign"]  = moves["answer_sign"].str.strip()
    moves["triplet"] = list(zip(moves.pwm, moves.in1, moves.in2))

    print("\n=== UNIQUE TRIPLETS PER WHEEL & DIRECTION ===")
    for w in ["fl","fr","rl","rr"]:
        for s in ["+","-"]:
            subset = moves[(moves.answer_wheel==w) & (moves.answer_sign==s)]
            uniq = subset["triplet"].dropna().unique()
            if len(uniq):
                print(f" {w.upper()} {s}: {len(uniq)} ways → {uniq}")
            else:
                print(f" {w.UPPER()} {s}: 0 ways")  # fallback if none

    print("\n=== PAIR-MODE MATCHING TRIPLETS (shared responses) ===")
    def overlap(a,b,sign):
        a_set = set(a[(a.answer_sign==sign)]["triplet"])
        b_set = set(b[(b.answer_sign==sign)]["triplet"])
        return a_set & b_set

    fl = moves[moves.answer_wheel=="fl"]
    rl = moves[moves.answer_wheel=="rl"]
    fr = moves[moves.answer_wheel=="fr"]
    rr = moves[moves.answer_wheel=="rr"]

    for sign in ["+","-"]:
        left_pair  = sorted(overlap(fl, rl, sign))
        right_pair = sorted(overlap(fr, rr, sign))
        print(f" Left (FL=RL) {sign}: {len(left_pair)} shared → {left_pair}")
        print(f" Right (FR=RR) {sign}: {len(right_pair)} shared → {right_pair}")

    print("\n✅ Analysis complete — this shows both independent and paired movement patterns.\n")

if __name__ == "__main__":
    main()
