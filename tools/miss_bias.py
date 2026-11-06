import csv
import statistics
from collections import Counter


def load(p):
    return list(csv.DictReader(open(p, encoding="utf-8")))


for path, name in ((r"runs\cmp3\3_track.csv", "video3"), (r"runs\cmp4\4_track.csv", "video4")):
    rows = load(path)
    shots = [r for r in rows if r["shot"] == "1"]
    hits = [r for r in shots if r["shot_result"] == "1"]
    miss = [r for r in shots if r["shot_result"] == "0"]
    print(f"\n== {name}: {len(hits)}/{len(hits)+len(miss)} ==")
    for label, group in (("hits", hits), ("misses", miss)):
        us = [float(r["off_u"]) for r in group]
        if not us:
            continue
        bins = Counter()
        for u in us:
            bins[round(u)] += 1
        gross = sum(1 for u in us if abs(u) > 2.0)
        print(f"  {label:6s} n={len(us):4d} gross(|u|>2)={gross:3d} ({100*gross/len(us):4.1f}%)  "
              f"|u|<0.5: {100*sum(1 for u in us if abs(u)<0.5)/len(us):4.1f}%")
        print("         off_u histogram:", dict(sorted(bins.items())))
    # bias vs spin direction
    for sign, tag in ((1, "w>0"), (-1, "w<0")):
        g = [float(r["off_u"]) for r in miss if float(r["w"]) * sign > 0]
        if len(g) > 5:
            print(f"  misses {tag}: n={len(g):3d} median off_u {statistics.median(g):+.2f} mean {statistics.mean(g):+.2f}")
    # near-miss margin: how far outside the panel edge, in half-widths (|u|-1)
    margins = sorted(abs(float(r["off_u"])) - 1.0 for r in miss)
    if margins:
        near = sum(1 for m in margins if m <= 0.5)
        print(f"  misses within 0.5 half-width outside the panel edge: {near}/{len(margins)} ({100*near/len(margins):.0f}%)")
