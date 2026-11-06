#!/usr/bin/env python3
"""Simulator-in-the-loop parameter tuning for the autoaim tracker/aimer.

The simulator (apps/autoaim_sim.cpp) replays eight synthetic scenarios through
the same pose solver, tracker, aimer and gimbal model used on the real videos,
and judges every shot against the exact ground-truth plate. That makes it a
cheap, deterministic objective for tuning -- provided we do not overfit to a
single noise realisation, hence the multi-seed evaluation.

Search: cross-entropy method (CEM) over normalised log/linear parameter bounds,
starting from the values actually shipped in config/default.yaml (not from the
C++ struct defaults -- several of those are overridden by the YAML).

Objective: 0.6 * mean hit% + 0.4 * worst-scenario hit% + 0.05 * tracking%,
minus a penalty when a candidate buys hit rate by firing much less often. That
last term matters: `max_fire_facing` and `side_angle` can trivially raise the
hit rate by refusing to shoot.

Usage:
    python3 tools/tune_sim.py --exe build/Release/autoaim_sim.exe
    python3 tools/tune_sim.py --exe build/Release/autoaim_sim.exe --iterations 6 --evals 8

Writes tools/tune_sim_result.json with the full trace for the report.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import re
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

# name -> (low, high, log_scale). Values come from config/default.yaml.
# Keys are the --set names used by setParam(); YAML_KEYS maps them to the
# field names in the YAML file, which do not always match.
SPACE: dict[str, tuple[float, float, bool]] = {
    "q_xyz": (0.2, 20.0, True),
    "q_yaw": (5.0, 200.0, True),
    "q_adapt_max": (2.0, 60.0, True),
    "nis_alpha": (0.05, 0.60, False),
    "r_yaw": (0.003, 0.20, True),
    "max_fire_facing": (15.0, 60.0, False),
    "side_angle": (8.0, 45.0, False),
    "center_mode_omega": (5.0, 20.0, False),
}

YAML_KEYS = {
    "nis_alpha": "nis_ema_alpha",
    "max_fire_facing": "max_fire_facing_deg",
    "side_angle": "side_angle_deg",
}


@dataclass
class ScenarioResult:
    name: str
    hit: float
    hits_per_sec: float
    tracking: float
    centre_err_cm: float
    omega_err: float


def load_yaml(path: Path) -> dict[str, float]:
    """Flat scalar reader for the hand-written config; enough for our keys."""
    text = path.read_text(encoding="utf-8", errors="replace")
    out: dict[str, float] = {}
    for key in SPACE:
        yaml_key = YAML_KEYS.get(key, key)
        m = re.search(rf"^\s*{re.escape(yaml_key)}:\s*([-\d.eE+]+)", text, re.MULTILINE)
        if m:
            out[key] = float(m.group(1))
    return out


def to_unit(name: str, value: float) -> float:
    lo, hi, log_scale = SPACE[name]
    if log_scale:
        return (math.log(value) - math.log(lo)) / (math.log(hi) - math.log(lo))
    return (value - lo) / (hi - lo)


def from_unit(name: str, t: float) -> float:
    lo, hi, log_scale = SPACE[name]
    t = min(max(t, 0.0), 1.0)
    if log_scale:
        return math.exp(math.log(lo) + t * (math.log(hi) - math.log(lo)))
    return lo + t * (hi - lo)


def parse_table(stdout: str) -> list[ScenarioResult]:
    rows: list[ScenarioResult] = []
    for line in stdout.splitlines():
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 9 or cells[0] == "scenario" or set(cells[0]) <= {"-"}:
            continue
        try:
            rows.append(
                ScenarioResult(
                    name=cells[0],
                    hit=float(cells[3]),
                    hits_per_sec=float(cells[4]),
                    tracking=float(cells[8]),
                    centre_err_cm=float(cells[5]),
                    omega_err=float(cells[6]),
                )
            )
        except ValueError:
            continue
    return rows


def run_sim(exe: Path, cwd: Path, overrides: dict[str, float], seconds: float,
            noise: float, dropout: float, seed: int) -> list[ScenarioResult]:
    cmd = [str(exe), "--seconds", str(seconds), "--noise", str(noise),
           "--dropout", str(dropout), "--seed", str(seed)]
    for k, v in overrides.items():
        cmd += ["--set", f"{k}={v:.6g}"]
    proc = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"sim failed ({proc.returncode}): {' '.join(cmd)}\n{proc.stderr[-800:]}")
    rows = parse_table(proc.stdout)
    if len(rows) != 8:
        raise RuntimeError(f"expected 8 scenarios, parsed {len(rows)}\n{proc.stdout[:800]}")
    return rows


def evaluate(exe: Path, cwd: Path, overrides: dict[str, float], seeds: list[int],
             seconds: float, noise: float, dropout: float,
             base_hps: dict[str, float] | None = None) -> dict:
    per_seed = [run_sim(exe, cwd, overrides, seconds, noise, dropout, s) for s in seeds]
    names = [r.name for r in per_seed[0]]
    hits = {n: statistics.mean(ps[i].hit for ps in per_seed) for i, n in enumerate(names)}
    hps = {n: statistics.mean(ps[i].hits_per_sec for ps in per_seed) for i, n in enumerate(names)}
    tracks = {n: statistics.mean(ps[i].tracking for ps in per_seed) for i, n in enumerate(names)}
    centre = {n: statistics.mean(ps[i].centre_err_cm for ps in per_seed) for i, n in enumerate(names)}

    mean_hit = statistics.mean(hits.values())
    worst = min(hits.values())
    # Penalise "fire less to look better": each scenario that loses more than
    # 25% of its shot rate costs up to 15 points.
    rate_penalty = 0.0
    if base_hps:
        for n in names:
            if base_hps[n] > 1e-6 and hps[n] < 0.75 * base_hps[n]:
                rate_penalty += 15.0 * min(1.0, (0.75 * base_hps[n] - hps[n]) / (0.75 * base_hps[n]))
        rate_penalty /= len(names)
    score = 0.6 * mean_hit + 0.4 * worst + 0.05 * statistics.mean(tracks.values()) - rate_penalty

    return {
        "hits": hits, "hits_per_sec": hps, "tracking": tracks, "centre_err_cm": centre,
        "mean_hit": mean_hit, "worst_hit": worst,
        "mean_tracking": statistics.mean(tracks.values()),
        "mean_hps": statistics.mean(hps.values()),
        "rate_penalty": rate_penalty,
        "score": score,
    }


def fmt_overrides(o: dict[str, float]) -> str:
    return " ".join(f"--set {k}={v:.6g}" for k, v in o.items())


def video_hit_rate(exe: Path, cwd: Path, video: str, overrides: dict[str, float],
                   out_dir: str) -> float | None:
    """Run the real-video evaluator and return its robot-EKF hit rate."""
    cmd = [str(exe), video, "--out", out_dir]
    for k, v in overrides.items():
        cmd += ["--set", f"{k}={v:.6g}"]
    proc = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"  guard run failed ({proc.returncode}): {proc.stderr[-400:]}", file=sys.stderr)
        return None
    for line in proc.stdout.splitlines():
        if line.startswith("| robot EKF (whole-robot model)"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            return float(cells[4].rstrip("%"))
    return None


def print_table(base: dict, tuned: dict) -> None:
    print(f"{'scenario':52s} {'base':>6s} {'tuned':>6s} {'delta':>6s} {'hits/s':>7s}")
    for name in base["hits"]:
        b, t = base["hits"][name], tuned["hits"][name]
        print(f"{name:52s} {b:6.1f} {t:6.1f} {t - b:+6.1f} {tuned['hits_per_sec'][name]:7.2f}")
    print(f"{'mean':52s} {base['mean_hit']:6.1f} {tuned['mean_hit']:6.1f} "
          f"{tuned['mean_hit'] - base['mean_hit']:+6.1f} {tuned['mean_hps']:7.2f}")
    print(f"{'worst':52s} {base['worst_hit']:6.1f} {tuned['worst_hit']:6.1f} "
          f"{tuned['worst_hit'] - base['worst_hit']:+6.1f}")


def guard_check(args, over: dict[str, float], cwd: Path):
    """Run the real-video regression guard. Returns (ok, record)."""
    gexe = Path(args.guard_exe).resolve()
    print(f"\n== real-video guard: {args.guard_video} (+/-{args.guard_tolerance:g} pp) ==")
    base_hit = video_hit_rate(gexe, cwd, args.guard_video, {}, "runs/guard_base")
    tuned_hit = video_hit_rate(gexe, cwd, args.guard_video, over, "runs/guard_tuned")
    if base_hit is None or tuned_hit is None:
        print("  守卫视频无法评估，跳过（不算通过也不算失败）")
        return True, {"skipped": True}
    delta = tuned_hit - base_hit
    ok = delta >= -args.guard_tolerance
    print(f"  baseline {base_hit:.1f}%  tuned {tuned_hit:.1f}%  delta {delta:+.1f} pp  "
          f"-> {'ACCEPT' if ok else 'REJECT'}")
    return ok, {"video": args.guard_video, "baseline": base_hit, "tuned": tuned_hit,
                "delta": delta, "accepted": ok}


def main() -> int:
    ap = argparse.ArgumentParser(description="CEM tuning on the autoaim simulator")
    ap.add_argument("--exe", default="build/Release/autoaim_sim.exe")
    ap.add_argument("--cwd", default=".")
    ap.add_argument("--config", default="config/default.yaml")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--noise", type=float, default=0.7)
    ap.add_argument("--dropout", type=float, default=0.05)
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--iterations", type=int, default=8)
    ap.add_argument("--evals", type=int, default=8)
    ap.add_argument("--elite", type=int, default=3)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", default="tools/tune_sim_result.json")
    ap.add_argument("--apply", default="", help="只评估给定覆盖，不搜索；格式 k=v,k=v")
    ap.add_argument("--noise-grid", default="", help="鲁棒性扫描：逗号分隔的噪声")
    ap.add_argument("--dropout-grid", default="", help="鲁棒性扫描：逗号分隔的丢帧率")
    ap.add_argument("--guard-exe", default="", help="真实视频评估器（autoaim_video），用于回归守卫")
    ap.add_argument("--guard-video", default="", help="守卫用视频，例如 video/3.avi")
    ap.add_argument("--guard-tolerance", type=float, default=1.0,
                    help="允许的真实视频命中率下降（百分点），超过则拒绝")
    args = ap.parse_args()

    exe = Path(args.exe).resolve()
    cwd = Path(args.cwd).resolve()
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    if not exe.is_file():
        print(f"找不到模拟器: {exe}", file=sys.stderr)
        return 1
    base_vals = load_yaml(cwd / args.config)
    missing = [k for k in SPACE if k not in base_vals]
    if missing:
        print(f"配置里缺少参数: {missing}", file=sys.stderr)
        return 1

    if args.apply:
        over = {}
        for kv in args.apply.split(","):
            if not kv.strip():
                continue
            k, v = kv.split("=")
            over[k.strip()] = float(v)
        noise_grid = [float(x) for x in args.noise_grid.split(",") if x.strip()] or [args.noise]
        drop_grid = [float(x) for x in args.dropout_grid.split(",") if x.strip()] or [args.dropout]
        print(f"== robustness sweep: {len(seeds)} seeds, {args.seconds:g}s/scenario ==")
        print(f"{'noise':>6s} {'dropout':>8s} {'mean':>7s} {'worst':>7s} {'hits/s':>7s}")
        rows = []
        base = evaluate(exe, cwd, {}, seeds, args.seconds, noise_grid[0], drop_grid[0])
        for nz in noise_grid:
            for dp in drop_grid:
                r = evaluate(exe, cwd, over, seeds, args.seconds, nz, dp, base["hits_per_sec"])
                rows.append({"noise": nz, "dropout": dp, **{k: r[k] for k in
                             ("mean_hit", "worst_hit", "mean_hps", "mean_tracking")}})
                print(f"{nz:6.2f} {dp:8.2f} {r['mean_hit']:7.1f} {r['worst_hit']:7.1f} {r['mean_hps']:7.2f}")
        Path(args.out).write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\nwrote {args.out}")
        if args.guard_exe and args.guard_video:
            ok, _ = guard_check(args, over, cwd)
            return 0 if ok else 2
        return 0

    print(f"== baseline: config/default.yaml ({len(seeds)} seeds, {args.seconds:g}s/scenario) ==")
    for k, v in base_vals.items():
        print(f"  {k:18s} {v:g}")
    base = evaluate(exe, cwd, {}, seeds, args.seconds, args.noise, args.dropout)
    print(f"  mean {base['mean_hit']:.1f}%  worst {base['worst_hit']:.1f}%  "
          f"tracking {base['mean_tracking']:.1f}%  hits/s {base['mean_hps']:.2f}  "
          f"score {base['score']:.2f}\n")

    keys = list(SPACE)
    mu = [to_unit(k, base_vals[k]) for k in keys]
    sigma = [0.20] * len(keys)
    rng = random.Random(20261104)
    cache: dict[tuple, dict] = {}
    best: tuple[float, dict] | None = None
    trace: list[dict] = []

    for it in range(args.iterations):
        cands = [[min(max(rng.gauss(m, s), 0.0), 1.0) for m, s in zip(mu, sigma)]
                 for _ in range(args.evals)]
        overrides = [{k: from_unit(k, t) for k, t in zip(keys, c)} for c in cands]
        ckeys = [tuple(round(o[k], 4) for k in keys) for o in overrides]

        todo = [(ck, o) for ck, o in zip(ckeys, overrides) if ck not in cache]
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {
                pool.submit(evaluate, exe, cwd, o, seeds, args.seconds, args.noise,
                            args.dropout, base["hits_per_sec"]): ck
                for ck, o in todo
            }
            for fut, ck in futures.items():
                cache[ck] = fut.result()

        ranked = sorted(((cache[ck]["score"], ck) for ck in ckeys), reverse=True)
        elite = ranked[: args.elite]
        if best is None or elite[0][0] > best[0]:
            best = (elite[0][0], overrides[ckeys.index(elite[0][1])])
        elite_vecs = [cands[ckeys.index(ck)] for _, ck in elite]
        mu = [statistics.mean(v[i] for v in elite_vecs) for i in range(len(keys))]
        sigma = [max(0.03, 1.2 * statistics.pstdev(v[i] for v in elite_vecs)) for i in range(len(keys))]
        top = cache[elite[0][1]]
        trace.append({"iter": it + 1, "score": elite[0][0], "mean_hit": top["mean_hit"],
                      "worst_hit": top["worst_hit"], "mean_hps": top["mean_hps"]})
        print(f"iter {it + 1:2d}: score {elite[0][0]:6.2f}  mean {top['mean_hit']:5.1f}%  "
              f"worst {top['worst_hit']:5.1f}%  hits/s {top['mean_hps']:5.2f}")

    assert best is not None
    _, over = best
    final = evaluate(exe, cwd, over, seeds, args.seconds, args.noise, args.dropout, base["hits_per_sec"])
    print("\n== best ==")
    print("  " + fmt_overrides(over))
    print()
    print_table(base, final)

    guard = None
    guard_ok = True
    if args.guard_exe and args.guard_video:
        guard_ok, guard = guard_check(args, over, cwd)

    Path(args.out).write_text(json.dumps({
        "settings": {"seconds": args.seconds, "noise": args.noise, "dropout": args.dropout,
                     "seeds": seeds, "iterations": args.iterations, "evals": args.evals},
        "baseline_params": base_vals,
        "baseline": base, "tuned": final, "overrides": over,
        "score": best[0], "trace": trace, "evaluations": len(cache), "guard": guard,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nwrote {args.out}  ({len(cache)} unique evaluations)")
    if not guard_ok:
        print("拒绝该组参数：仿真提升但真实视频回退超过容差。")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
