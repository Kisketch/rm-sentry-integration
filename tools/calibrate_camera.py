#!/usr/bin/env python3
"""Calibrate the actual RM camera from chessboard images, producing --camera YAML.

Example: python3 tools/calibrate_camera.py 'calibration/*.png' --cols 9 --rows 6
         --square-mm 25 --out config/my_camera.yml
"""

import argparse
import glob
import math
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", help="quoted glob of raw images from the actual camera")
    parser.add_argument("--cols", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument("--square-mm", type=float, required=True, help="measured square side in mm")
    parser.add_argument("--pitch-deg", type=float, default=0.0,
                        help="measured downward camera tilt; defaults to 0 until measured")
    parser.add_argument("--out", required=True, help="output OpenCV YAML for autoaim_video --camera")
    parser.add_argument("--min-views", type=int, default=12)
    args = parser.parse_args()
    if args.cols < 3 or args.rows < 3 or args.square_mm <= 0 or args.min_views < 6:
        parser.error("board dimensions, square side and min-views must be positive and plausible")

    paths = sorted(glob.glob(args.images))
    if not paths:
        parser.error("no images match the glob")
    grid = np.zeros((args.rows * args.cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm
    object_points, image_points, accepted = [], [], []
    size = None
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"SKIP unreadable: {path}")
            continue
        this_size = (image.shape[1], image.shape[0])
        if size is None:
            size = this_size
        if this_size != size:
            raise SystemExit(f"mixed image sizes: {path} is {this_size}, expected {size}")
        ok, corners = cv2.findChessboardCornersSB(image, (args.cols, args.rows))
        if not ok:
            print(f"SKIP no board: {path}")
            continue
        object_points.append(grid)
        image_points.append(corners.reshape(-1, 2).astype(np.float32))
        accepted.append(path)
    if len(accepted) < args.min_views:
        raise SystemExit(f"only {len(accepted)} valid views; need at least {args.min_views}")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, size, None, None)
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        delta = projected.reshape(-1, 2) - img
        per_view.append(math.sqrt(float(np.mean(np.sum(delta * delta, axis=1)))))
    if not np.isfinite(K).all() or not np.isfinite(dist).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise SystemExit("calibration returned invalid intrinsics")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(out), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise SystemExit(f"cannot write {out}")
    storage.startWriteStruct("camera", cv2.FileNode_MAP)
    for name, value in (("width", size[0]), ("height", size[1]), ("fx", K[0, 0]),
                        ("fy", K[1, 1]), ("cx", K[0, 2]), ("cy", K[1, 2])):
        storage.write(name, value)
    storage.startWriteStruct("dist", cv2.FileNode_SEQ)
    for value in dist.reshape(-1):
        storage.write("", float(value))
    storage.endWriteStruct()
    storage.write("pitch_deg", args.pitch_deg)
    storage.endWriteStruct()
    storage.release()
    print(f"saved {out}: {len(accepted)}/{len(paths)} views, {size[0]}x{size[1]}")
    print(f"RMS {rms:.3f} px, view RMS mean {np.mean(per_view):.3f}, max {max(per_view):.3f} px")
    print("fx {:.2f}, fy {:.2f}, cx {:.2f}, cy {:.2f}".format(K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    all_corners = np.concatenate(image_points, axis=0)
    span = np.ptp(all_corners, axis=0) / np.array(size)
    print(f"corner coverage: {span[0]:.1%} of image width, {span[1]:.1%} of height")
    if min(span) < 0.5:
        print("WARNING: limited frame coverage; capture tilted views near all four image corners")
    if max(per_view) > 1.0:
        print("WARNING: at least one view exceeds 1 px RMS; inspect blur, board flatness and corner detection")
    for path, error in zip(accepted, per_view):
        print(f"  {error:.3f} px  {path}")


if __name__ == "__main__":
    main()
