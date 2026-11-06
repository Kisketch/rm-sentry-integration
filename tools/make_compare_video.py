#!/usr/bin/env python3
"""Build frame-aligned "original vs detection+aim" comparison clips.

Left half: the raw video frame. Right half: the autoaim HUD (detection boxes,
modelled plates, aim point, hit marker, top view). Both halves also get a
burned-in frame number so a still can be traced back to the source.
"""

import argparse
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def default_video_dir() -> Path:
    """Works in both layouts: autoaim/video and RM/template/template/video."""
    for candidate in (ROOT.parent / "template" / "template" / "video",
                      ROOT.parent / "video", ROOT / "video"):
        if candidate.is_dir():
            return candidate
    return ROOT.parent / "video"


def default_ffmpeg() -> str | None:
    found = shutil.which("ffmpeg")
    if found:
        return found
    # Windows: the yt-dlp build ships a usable ffmpeg next to the executable
    local = Path.home() / "AppData" / "Local" / "Programs" / "yt-dlp" / "ffmpeg.exe"
    return str(local) if local.is_file() else None

PANEL_W, PANEL_H = 960, 720
BAR_H = 34
FONT = cv2.FONT_HERSHEY_SIMPLEX
GAMMA = 1.6  # display-only lift; both panels get the same curve, detection untouched


def lift(img: np.ndarray) -> np.ndarray:
    lut = np.array([((i / 255.0) ** (1.0 / GAMMA)) * 255 for i in range(256)], np.uint8)
    return cv2.LUT(img, lut)


def compose(original: Path, overlay: Path, out_avi: Path, start_frame: int) -> int:
    cap_o = cv2.VideoCapture(str(original))
    cap_a = cv2.VideoCapture(str(overlay))
    if not cap_o.isOpened() or not cap_a.isOpened():
        raise SystemExit(f"cannot open {original} or {overlay}")
    # The overlay starts at start_frame, not at source frame zero. Without this
    # seek the short clips compare different moments while displaying the same
    # frame number in the title bar.
    cap_o.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    actual = round(cap_o.get(cv2.CAP_PROP_POS_FRAMES))
    if actual != start_frame:
        raise SystemExit(f"original seek landed at frame {actual}, expected {start_frame}")
    fps = cap_a.get(cv2.CAP_PROP_FPS) or 25.0
    writer = cv2.VideoWriter(str(out_avi), cv2.VideoWriter_fourcc(*"MJPG"), fps,
                             (PANEL_W * 2, PANEL_H + BAR_H))
    n = 0
    while True:
        ok_o, fo = cap_o.read()
        ok_a, fa = cap_a.read()
        if not ok_a:
            break
        if not ok_o:
            raise SystemExit(f"original ended before overlay at frame {start_frame + n}")
        lo = lift(cv2.resize(fo, (PANEL_W, PANEL_H), interpolation=cv2.INTER_AREA))
        ra = lift(cv2.resize(fa, (PANEL_W, PANEL_H), interpolation=cv2.INTER_AREA))
        bar = np.full((BAR_H, PANEL_W * 2, 3), 24, np.uint8)
        cv2.putText(bar, "ORIGINAL  (no overlay)", (10, 24), FONT, 0.7, (210, 210, 210), 2)
        cv2.putText(bar, "DETECTION + AIM  (green=armor, red O=aim, x=impact)", (PANEL_W + 10, 24),
                    FONT, 0.7, (120, 255, 120), 2)
        idx = start_frame + n
        stamp = f"f{idx}  t={idx / fps:5.2f}s  gamma {GAMMA:g}"
        cv2.putText(bar, stamp, (PANEL_W - 290, 24), FONT, 0.62, (90, 200, 255), 2)
        frame = np.vstack([bar, np.hstack([lo, ra])])
        # thin divider between the two panels
        cv2.line(frame, (PANEL_W, BAR_H), (PANEL_W, BAR_H + PANEL_H), (60, 60, 60), 2)
        writer.write(frame)
        n += 1
    cap_o.release()
    cap_a.release()
    writer.release()
    if n == 0:
        raise SystemExit(f"overlay contains no frames: {overlay}")
    return n


def encode(avi: Path, mp4: Path, ffmpeg: str) -> None:
    subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(avi),
                    "-c:v", "libx264", "-preset", "medium", "-crf", "22",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)


def concat(parts: list[Path], out: Path, ffmpeg: str, tmp: Path) -> None:
    lst = tmp / "concat.txt"
    lst.write_text("".join(f"file '{p.as_posix()}'\n" for p in parts), encoding="utf-8")
    subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
                    "-i", str(lst), "-c", "copy", "-movflags", "+faststart", str(out)], check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--full", action="store_true", help="render both full-length comparisons")
    ap.add_argument("--video-dir", type=Path, default=default_video_dir(),
                    help="directory containing original 3.avi and 4.avi")
    ap.add_argument("--overlay-dir", type=Path, default=ROOT / "runs",
                    help="directory containing cmp3/ and cmp4/ overlay outputs")
    ap.add_argument("--out-dir", type=Path, default=ROOT / "out_compare",
                    help="where to write comparison clips")
    ap.add_argument("--ffmpeg", default=default_ffmpeg(),
                    help="ffmpeg executable; defaults to PATH or the yt-dlp build")
    args = ap.parse_args()
    if not args.ffmpeg:
        ap.error("ffmpeg is required; install it or pass --ffmpeg PATH")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    tmp = args.out_dir / "_tmp"
    tmp.mkdir(parents=True, exist_ok=True)
    if args.full:
        jobs = [
            ("3", 0, 2208, "06_全片_3号_原画vs自瞄.mp4"),
            ("4", 0, 3415, "07_全片_4号_原画vs自瞄.mp4"),
        ]
    else:
        jobs = [
            ("3", 850, 1000, "01_3号视频_原画vs自瞄.mp4"),
            ("4", 2625, 2775, "02_4号视频_原画vs自瞄.mp4"),
        ]
    made = []
    for tag, start, end, name in jobs:
        original = args.video_dir / f"{tag}.avi"
        overlay = args.overlay_dir / f"cmp{tag}" / f"{tag}_overlay_{start}_{end}.avi"
        avi = tmp / f"cmp_{tag}.avi"
        frames = compose(original.resolve(), overlay, avi, start)
        if frames != end - start + 1:
            raise SystemExit(f"{overlay}: got {frames} frames, expected {end - start + 1}")
        mp4 = args.out_dir / name
        encode(avi, mp4, args.ffmpeg)
        size = mp4.stat().st_size / 1024
        print(f"{name}: {frames} frames, {size / 1024:.1f} MB")
        made.append(mp4)
    if args.full:
        combined = args.out_dir / "08_全片合集_3号+4号_原画vs自瞄.mp4"
        concat(made, combined, args.ffmpeg, tmp)
        print(f"{combined.name}: {combined.stat().st_size / 1024 / 1024:.1f} MB")
    else:
        combined = args.out_dir / "03_合集_3号+4号_原画vs自瞄.mp4"
        concat(made, combined, args.ffmpeg, tmp)
        print(f"{combined.name}: {combined.stat().st_size / 1024:.0f} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
