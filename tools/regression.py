#!/usr/bin/env python3
"""Regression gate over the two real videos and the simulator.

Reads the numbers BASELINE.md section 10 froze, re-runs `autoaim_video` on both
recordings (reusing the detection cache) and `autoaim_sim`, and fails if any
metric drops past its tolerance. This is the manual gate that CI cannot run,
because the videos are not in the repository.

    python3 tools/regression.py                 # uses build/Release on Windows
    python3 tools/regression.py --exe build/autoaim_video

Exit code 0 = no regression, 1 = regression or missing input.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

# Frozen references (BASELINE.md section 10). Keep in sync when the baseline moves.
REFERENCE = {
    "3": {"hit": 67.6, "pred_032": 14.3, "hits_per_s": 10.6},
    "4": {"hit": 68.6, "pred_032": 13.4, "hits_per_s": 10.0},
}
SIM_MEAN_REF = 78.1
TOL_HIT_PP = 1.5
TOL_PRED_PX = 1.5
TOL_HPS = 0.5
TOL_SIM_PP = 3.0


def default_exe() -> Path:
    for p in ("build/Release/autoaim_video", "build/autoaim_video"):
        if Path(p + ".exe").is_file():
            return Path(p + ".exe")
        if Path(p).is_file():
            return Path(p)
    return Path("build/Release/autoaim_video.exe")


def default_video_dir() -> Path:
    for p in ("../template/template/video", "../video", "video"):
        if Path(p).is_dir():
            return Path(p)
    return Path("video")


def parse_report(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    out: dict[str, float] = {}
    m = re.search(r"^\| robot EKF \(whole-robot model\) \|.*?\| ([\d.]+)% \| ([\d.]+) \|",
                  text, re.MULTILINE)
    if m:
        out["hit"] = float(m.group(1))
        out["hits_per_s"] = float(m.group(2))
    m = re.search(r"^\| robot EKF \| 0\.32 s \|.*?\| ([\d.]+) \| ([\d.]+) \|",
                  text, re.MULTILINE)
    if m:
        out["pred_032"] = float(m.group(2))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="regression gate against BASELINE section 10")
    ap.add_argument("--exe", default=str(default_exe()))
    ap.add_argument("--videos", default="", help="逗号分隔；默认 3.avi,4.avi")
    ap.add_argument("--out", default="runs/regress")
    ap.add_argument("--sim-exe", default="", help="默认取 video exe 同目录的 autoaim_sim")
    args = ap.parse_args()

    exe = Path(args.exe)
    if not exe.is_file():
        print(f"找不到 autoaim_video: {exe}", file=sys.stderr)
        return 1

    vdir = default_video_dir()
    stems = [s.strip() for s in args.videos.split(",") if s.strip()] or ["3", "4"]
    videos = []
    for stem in stems:
        v = vdir / f"{stem}.avi"
        if not v.is_file():
            print(f"找不到视频: {v}", file=sys.stderr)
            return 1
        videos.append((stem, v))

    failures = 0
    print(f"{'video':10s} {'metric':12s} {'reference':>9s} {'measured':>9s} {'delta':>8s}  verdict")
    for stem, video in videos:
        proc = subprocess.run([str(exe), str(video), "--out", args.out],
                              capture_output=True, text=True)
        if proc.returncode != 0:
            print(f"{stem}: 运行失败\n{proc.stderr[-500:]}", file=sys.stderr)
            return 1
        got = parse_report(Path(args.out) / f"{stem}_report.md")
        ref = REFERENCE.get(stem)
        if ref is None or len(got) < 3:
            print(f"{stem}: 无法解析报告", file=sys.stderr)
            return 1
        for key, tol, unit in (("hit", TOL_HIT_PP, "%"), ("pred_032", TOL_PRED_PX, "px"),
                               ("hits_per_s", TOL_HPS, "/s")):
            delta = got[key] - ref[key]
            ok = delta >= -tol if key != "pred_032" else delta <= tol
            failures += 0 if ok else 1
            print(f"{stem + '.avi':10s} {key:12s} {ref[key]:9.1f} {got[key]:9.1f} "
                  f"{delta:+8.1f}  {'PASS' if ok else 'FAIL'}")

    sim_exe = Path(args.sim_exe) if args.sim_exe else exe.with_name(
        "autoaim_sim.exe" if exe.name.endswith(".exe") else "autoaim_sim")
    if sim_exe.is_file():
        proc = subprocess.run([str(sim_exe)], capture_output=True, text=True)
        hits = [float(m) for m in re.findall(r"^\| .*? \| [\d.]+ \| [\d.]+ \| ([\d.]+) \|",
                                             proc.stdout, re.MULTILINE)]
        if len(hits) == 8:
            mean = sum(hits) / 8
            ok = mean >= SIM_MEAN_REF - TOL_SIM_PP
            failures += 0 if ok else 1
            print(f"{'sim':10s} {'mean_hit':12s} {SIM_MEAN_REF:9.1f} {mean:9.1f} "
                  f"{mean - SIM_MEAN_REF:+8.1f}  {'PASS' if ok else 'FAIL'}")
        else:
            print(f"sim: 解析到 {len(hits)} 个场景（应为 8），跳过", file=sys.stderr)
    else:
        print(f"跳过仿真：找不到 {sim_exe}", file=sys.stderr)

    print("\nRESULT: PASS" if failures == 0 else f"\nRESULT: FAIL ({failures} 项)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
