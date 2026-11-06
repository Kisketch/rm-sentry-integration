#!/usr/bin/env python3
"""Integration check: a detection cache belongs to its video and detector settings."""

import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np


def run(binary: Path, video: Path, out: Path, *args: str) -> str:
    result = subprocess.run(
        [str(binary), str(video), "--out", str(out), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    return result.stdout


def make_video(path: Path, color: tuple[int, int, int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 25, (320, 240))
    if not writer.isOpened():
        raise RuntimeError("cannot create MJPG test video")
    for _ in range(12):
        frame = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(frame, (120, 80), (200, 160), color, -1)
        writer.write(frame)
    writer.release()


def main() -> None:
    binary = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="rm_autoaim_cache_") as tmp:
        root = Path(tmp)
        a = root / "a" / "clip.avi"
        b = root / "b" / "clip.avi"
        out = root / "out"
        make_video(a, (0, 0, 0))
        make_video(b, (0, 0, 255))

        assert "detected 12 frames" in run(binary, a, out)
        assert "loaded 12 frames" in run(binary, a, out)
        assert "detected 12 frames" in run(binary, b, out)  # same stem and length
        assert "loaded 12 frames" in run(binary, b, out)
        assert "detected 12 frames" in run(binary, a, out, "--set", "refine_corners=0")
        assert "loaded 12 frames" in run(binary, a, out, "--set", "refine_corners=0")

        cache = out / "clip_detections.txt"
        cache.write_text("0 0.000 1\n" + "1 0.040 0\n" * 11, encoding="utf-8")
        assert "detected 12 frames" in run(binary, a, out, "--set", "refine_corners=0")
        invalid = subprocess.run(
            [str(binary), str(a), "--out", str(out), "--sweep", "refine_corners=0,1"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        assert invalid.returncode != 0 and "changes detections" in invalid.stderr

    print("cache provenance and corruption checks passed")


if __name__ == "__main__":
    main()
