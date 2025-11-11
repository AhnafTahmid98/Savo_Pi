import pandas as pd
from itertools import combinations

csv_path = "/tmp/pca_map.csv"
df = pd.read_csv(csv_path)

# Filter only actual movements
moves = df[df["answer_wheel"] != "none"].copy()
moves["triplet"] = list(zip(moves.pwm, moves.in1, moves.in2))

# --- 1. Independent wheel summary ---
print("\n=== UNIQUE TRIPLETS PER WHEEL & DIRECTION ===")
for w in ["fl","fr","rl","rr"]:
    for s in ["+","-"]:
        subset = moves[(moves.answer_wheel==w) & (moves.answer_sign==s)]
        uniq = subset["triplet"].unique()
        if len(uniq):
            print(f" {w.upper()} {s}: {len(uniq)} ways → {uniq}")
        else:
            print(f" {w.upper()} {s}: 0 ways")

# --- 2. Pair-mode check (FL=RL and FR=RR) ---
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
    left_pair = overlap(fl, rl, sign)
    right_pair = overlap(fr, rr, sign)
    print(f" Left (FL=RL) {sign}: {len(left_pair)} shared → {sorted(left_pair)}")
    print(f" Right (FR=RR) {sign}: {len(right_pair)} shared → {sorted(right_pair)}")

print("\n✅ Analysis complete — this shows both independent and paired movement patterns.\n")

