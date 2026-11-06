#!/usr/bin/env python3
"""The first left-panel frame must match the overlay's source frame index."""

import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.make_compare_video import BAR_H, PANEL_H, PANEL_W, compose, lift


def write_video(path: Path, values: list[int]) -> None:
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 25, (160, 120))
    assert writer.isOpened()
    for value in values:
        writer.write(np.full((120, 160, 3), value, np.uint8))
    writer.release()


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="rm_compare_") as tmp:
        root = Path(tmp)
        original, overlay, output = (root / name for name in ("original.avi", "overlay.avi", "out.avi"))
        write_video(original, [10 * i for i in range(12)])
        write_video(overlay, [50, 60, 70])
        assert compose(original, overlay, output, start_frame=5) == 3
        cap = cv2.VideoCapture(str(output))
        ok, frame = cap.read()
        cap.release()
        assert ok
        left = frame[BAR_H:BAR_H + PANEL_H, :PANEL_W]
        expected = lift(np.full((PANEL_H, PANEL_W, 3), 50, np.uint8))
        wrong = lift(np.full((PANEL_H, PANEL_W, 3), 0, np.uint8))
        mae = np.abs(left.astype(np.float32) - expected).mean()
        wrong_mae = np.abs(left.astype(np.float32) - wrong).mean()
        assert mae < 5 and wrong_mae > 30, (mae, wrong_mae)
    print("comparison frame alignment passed")


if __name__ == "__main__":
    main()
