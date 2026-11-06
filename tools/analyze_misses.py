#!/usr/bin/env python3
"""Where do the shots actually miss? Reads the per-frame CSV written by
`autoaim_video` (runs/*/ *_track.csv) and reports the miss distribution by
rotational speed, plate facing angle, flight time and time window, plus the
systematic offset of the aim point inside the panel.
"""

from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path


def load(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as fh:
        return [row for row in csv.DictReader(fh)]


def f(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def bucket_stats(name: str, values: list[tuple[float, int]]) -> None:
    """values: (bucket_key, hit 0/1)"""
    agg: dict[float, list[int]] = defaultdict(list)
    for k, hit in values:
        agg[k].append(hit)
    print(f"\n{name}")
    print(f"  {'bucket':>10s} {'shots':>6s} {'hit%':>6s}")
    for k in sorted(agg):
        v = agg[k]
        print(f"  {k:10.1f} {len(v):6d} {100.0 * sum(v) / len(v):6.1f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--label", default="")
    ap.add_argument("--top", type=int, default=10)
    args = ap.parse_args()

    rows = load(args.csv)
    # ShotResult is written as 1 = HIT, 0 = MISS, -1 = UNKNOWN
    result_of = lambda r: {1.0: "HIT", 0.0: "MISS", -1.0: "UNKNOWN"}.get(f(r, "shot_result"), "UNKNOWN")
    shots = [r for r in rows if f(r, "shot") == 1]
    hits = [r for r in shots if result_of(r) == "HIT"]
    misses = [r for r in shots if result_of(r) == "MISS"]
    unknown = [r for r in shots if result_of(r) == "UNKNOWN"]

    print(f"== {args.label or args.csv.name} ==")
    print(f"frames {len(rows)}  shots {len(shots)}  hits {len(hits)}  misses {len(misses)}  "
          f"unknown {len(unknown)}  hit rate {100.0 * len(hits) / max(1, len(hits) + len(misses)):.1f}%")

    if misses:
        mp = sorted(f(r, "miss_px") for r in misses)
        print(f"miss_px  mean {statistics.mean(mp):.1f}  median {statistics.median(mp):.1f}  "
              f"p90 {mp[int(0.9 * (len(mp) - 1))]:.1f}  max {mp[-1]:.1f}")

    # systematic offset inside the panel: 0 = centre, +-0.5 = panel edge
    for name, group in (("hits", hits), ("misses", misses)):
        if not group:
            continue
        us = [f(r, "off_u") for r in group]
        vs = [f(r, "off_v") for r in group]
        print(f"{name:6s} off_u mean {statistics.mean(us):+.3f} median {statistics.median(us):+.3f} | "
              f"off_v mean {statistics.mean(vs):+.3f} median {statistics.median(vs):+.3f}")

    bucket_stats("by |w| (rad/s)", [(abs(f(r, "w")) // 1 * 1, 1 if r in hits else 0) for r in hits + misses])
    bucket_stats("by facing (deg)", [(abs(f(r, "facing_deg")) // 10 * 10, 1 if r in hits else 0)
                                     for r in hits + misses])
    bucket_stats("by t_fly (ms)", [(f(r, "t_fly") * 1000 // 50 * 50, 1 if r in hits else 0)
                                   for r in hits + misses])
    bucket_stats("by distance (m)", [(f(r, "t_fly") * 25 // 2 * 2, 1 if r in hits else 0)
                                     for r in hits + misses])

    # time windows of 250 frames
    window: dict[int, list[int]] = defaultdict(list)
    for r in hits + misses:
        window[int(f(r, "frame")) // 250].append(1 if r in hits else 0)
    print("\nby frame window (250 frames each)")
    print(f"  {'frames':>12s} {'shots':>6s} {'hit%':>6s}")
    for k in sorted(window):
        v = window[k]
        print(f"  {k * 250:5d}-{k * 250 + 249:<6d} {len(v):6d} {100.0 * sum(v) / len(v):6.1f}")

    worst = sorted(misses, key=lambda r: -f(r, "miss_px"))[: args.top]
    if worst:
        print(f"\nlargest misses ({args.top})")
        print(f"  {'frame':>6s} {'miss_px':>8s} {'w':>7s} {'facing':>7s} {'t_fly':>6s} {'off_u':>7s} {'off_v':>7s}")
        for r in worst:
            print(f"  {int(f(r, 'frame')):6d} {f(r, 'miss_px'):8.1f} {f(r, 'w'):7.2f} "
                  f"{f(r, 'facing_deg'):7.1f} {f(r, 't_fly') * 1000:6.0f} "
                  f"{f(r, 'off_u'):+7.2f} {f(r, 'off_v'):+7.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
