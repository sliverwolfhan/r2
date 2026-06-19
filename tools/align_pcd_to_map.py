#!/usr/bin/env python3
"""
Align a Point-LIO PCD to the map frame using surveyed anchor points.

Usage:
    align_pcd_to_map.py --anchors anchors.yaml \
                        --in  scans1.pcd \
                        --out scans1_map.pcd \
                        [--matrix-out T_map_lio.yaml] \
                        [--allow-scale]

Workflow:
    1. Place permanent landmarks in the scene and survey their coordinates
       in the target map frame (write to anchors.yaml under "map").
    2. Run Point-LIO and produce a PCD.
    3. Open the PCD in CloudCompare (or pcl_viewer), pick each landmark,
       fill its LIO-frame coordinate in anchors.yaml under "lio".
    4. Run this script. It computes T_map_lio via SVD (Umeyama),
       prints residuals, then calls pcl_transform_point_cloud to write
       the aligned PCD.
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = False):
    """Estimate T (4x4) such that dst ≈ T * src. Both arrays Nx3."""
    assert src.shape == dst.shape and src.shape[1] == 3
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d

    cov = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(cov)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt

    if with_scale:
        var_s = (src_c ** 2).sum() / n
        scale = (S * np.diag(D)).sum() / var_s
    else:
        scale = 1.0

    t = mu_d - scale * R @ mu_s
    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = t
    return T, scale


def load_anchors(path: Path):
    with path.open() as f:
        data = yaml.safe_load(f)
    src, dst, names = [], [], []
    for a in data["anchors"]:
        src.append(a["lio"])
        dst.append(a["map"])
        names.append(a.get("name", f"#{len(names)}"))
    return np.asarray(src, float), np.asarray(dst, float), names


def report_residuals(T, src, dst, names):
    src_h = np.hstack([src, np.ones((len(src), 1))])
    pred = (T @ src_h.T).T[:, :3]
    err = np.linalg.norm(pred - dst, axis=1)
    print(f"\nPer-anchor residuals (m):")
    for n, e in zip(names, err):
        print(f"  {n:<12s} {e:.4f}")
    print(f"RMS: {np.sqrt((err ** 2).mean()):.4f}    max: {err.max():.4f}\n")
    return err


def save_matrix_yaml(path: Path, T: np.ndarray):
    path.write_text(
        "# T_map_lio: map_point = T * lio_point (homogeneous)\n"
        "T_map_lio:\n"
        + "\n".join("  - [" + ", ".join(f"{v: .8f}" for v in row) + "]" for row in T)
        + "\n"
    )


def apply_transform(in_pcd: Path, out_pcd: Path, T: np.ndarray):
    if shutil.which("pcl_transform_point_cloud") is None:
        sys.exit("pcl_transform_point_cloud not found; install libpcl-tools or "
                 "apply the matrix yourself.")
    matrix_arg = ",".join(f"{v:.8f}" for v in T.flatten())
    cmd = ["pcl_transform_point_cloud", str(in_pcd), str(out_pcd),
           "-matrix", matrix_arg]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--anchors", required=True, type=Path)
    ap.add_argument("--in", dest="in_pcd", type=Path,
                    help="input PCD in LIO frame (optional — omit to only compute T)")
    ap.add_argument("--out", dest="out_pcd", type=Path,
                    help="output PCD in map frame")
    ap.add_argument("--matrix-out", type=Path, default=Path("T_map_lio.yaml"))
    ap.add_argument("--allow-scale", action="store_true",
                    help="solve similarity (rigid + uniform scale); default is rigid")
    args = ap.parse_args()

    src, dst, names = load_anchors(args.anchors)
    if len(src) < 3:
        sys.exit("Need at least 3 anchors.")

    T, scale = umeyama(src, dst, with_scale=args.allow_scale)
    print(f"Solved T_map_lio (scale = {scale:.6f}):")
    print(np.array2string(T, precision=6, suppress_small=True))
    report_residuals(T, src, dst, names)

    save_matrix_yaml(args.matrix_out, T)
    print(f"Wrote matrix -> {args.matrix_out}")

    if args.in_pcd:
        if not args.out_pcd:
            sys.exit("--in given but --out missing.")
        apply_transform(args.in_pcd, args.out_pcd, T)
        print(f"Wrote aligned PCD -> {args.out_pcd}")


if __name__ == "__main__":
    main()
