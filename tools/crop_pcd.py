#!/usr/bin/env python3
"""
Crop a PCD to an axis-aligned bounding box. Points outside the box are removed.

Two ways to use:

1. Edit the parameter block below, then run:
       python3 crop_pcd.py

2. Override via command line (any unset arg falls back to the block above):
       crop_pcd.py --in scans1.pcd --out scans1_crop.pcd \
                   --xrange -5 12  --yrange -3 8  --zrange 0 2.5

Notes:
    - Coordinates are in the same frame as the input PCD (meters).
    - All point fields (intensity, normal, curvature, ...) are preserved.
    - Supports DATA binary / binary_compressed / ascii (via pcl_passthrough_filter).
"""

# ============== 在这里修改参数 ==============
FILE   = "map_blue.pcd"   # 输入 PCD
OUTPUT = "map_blue1.pcd"   # 输出 PCD

X_MIN, X_MAX = -0.1, 12.5   # X 保留范围（米）
Y_MIN, Y_MAX = -6.1, 0.1    # Y 保留范围
Z_MIN, Z_MAX = -4.0,  4.0    # Z 保留范围
# ============================================

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def passthrough(in_path: Path, out_path: Path, field: str,
                lo: float, hi: float):
    cmd = ["pcl_passthrough_filter", str(in_path), str(out_path),
           "-field", field,
           "-min", f"{lo:.6f}",
           "-max", f"{hi:.6f}",
           "-inside", "1",
           "-keep", "0"]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="in_pcd", type=Path, default=Path(FILE))
    ap.add_argument("--out", dest="out_pcd", type=Path, default=Path(OUTPUT))
    ap.add_argument("--xrange", nargs=2, type=float, default=[X_MIN, X_MAX],
                    metavar=("XMIN", "XMAX"))
    ap.add_argument("--yrange", nargs=2, type=float, default=[Y_MIN, Y_MAX],
                    metavar=("YMIN", "YMAX"))
    ap.add_argument("--zrange", nargs=2, type=float, default=[Z_MIN, Z_MAX],
                    metavar=("ZMIN", "ZMAX"))
    args = ap.parse_args()

    if not args.in_pcd.exists():
        sys.exit(f"input not found: {args.in_pcd}")
    if shutil.which("pcl_passthrough_filter") is None:
        sys.exit("pcl_passthrough_filter not found; install libpcl-tools.")

    lo = [args.xrange[0], args.yrange[0], args.zrange[0]]
    hi = [args.xrange[1], args.yrange[1], args.zrange[1]]
    for axis, a, b in zip("xyz", lo, hi):
        if a >= b:
            sys.exit(f"--{axis}range: min ({a}) must be < max ({b})")

    print(f"Input : {args.in_pcd}")
    print(f"Output: {args.out_pcd}")
    print(f"Box   : x{args.xrange}  y{args.yrange}  z{args.zrange}")
    args.out_pcd.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        stages = [
            ("x", lo[0], hi[0], tmp / "stage_x.pcd"),
            ("y", lo[1], hi[1], tmp / "stage_y.pcd"),
            ("z", lo[2], hi[2], args.out_pcd),
        ]
        src = args.in_pcd
        for field, mn, mx, dst in stages:
            print(f"  passthrough {field}: [{mn:.3f}, {mx:.3f}] -> {dst.name}")
            passthrough(src, dst, field, mn, mx)
            src = dst

    print(f"Wrote -> {args.out_pcd}")


if __name__ == "__main__":
    main()
