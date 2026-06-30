#!/usr/bin/env python3
"""
Auto-align a freshly built (LIO-frame) PCD to the map frame using the field's
RECTANGULAR walls. Fallback for when the meilin stones are heavily occluded
(prefer auto_align_blocks.py — it's more accurate and pins the origin).

Pipeline:
  1. Filter points in the wall_z band (LIO frame; Point-LIO is gravity-aligned
     so LIO z ~= map z modulo a small ground offset).
  2. Project to XY, fit the minimum-area bounding rectangle.
  3. Match against the target rectangle from field config (length / width /
     center / yaw).
  4. A rectangle has 4 90deg symmetries → 4 candidate transforms.
     Break the tie with start_pose_map: the LIO origin (0,0,+x) should land
     near the rough start pose after the transform.
  5. Lift z so the LIO ground aligns with ground_z_map.

Warns if the fitted rectangle deviates >15% from config (walls under-scanned),
and if the best/second-best candidate scores are too close (start pose did not
break symmetry — likely positioned too symmetrically in the field).

Usage:
    auto_align_walls.py --field field.yaml --in scans1.pcd \
                        [--out scans1_map.pcd] [--zone red|blue] \
                        [--matrix-out T_map_lio.yaml]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

from pcd_io import read_pcd_xyz, write_pcd_xyz


# --------------------------------------------------------------------------- #
def load_config(path: Path):
    with path.open() as f:
        cfg = yaml.safe_load(f)
    fld = cfg["field"]
    return {
        "length": float(fld["length"]),
        "width": float(fld["width"]),
        "center_map": np.asarray(fld["center_map"], float),
        "yaw_map": float(fld["yaw_map"]),
        "wall_z": [float(v) for v in cfg["wall_z"]],
        "ground_z_map": float(cfg.get("ground_z_map", 0.0)),
        "start_pose_map": np.asarray(cfg["start_pose_map"], float),
        "default_zone": str(cfg.get("default_zone", "red")),
    }


def mirror_for_blue(cfg):
    """Red/blue zones mirror about the map x-axis. Only the map-side targets
    move (length/width unchanged); rectangle shape is identical."""
    c = cfg["center_map"].copy()
    c[1] *= -1
    sp = cfg["start_pose_map"].copy()
    sp[1] *= -1
    sp[2] *= -1
    return {**cfg, "center_map": c, "yaw_map": -cfg["yaw_map"],
            "start_pose_map": sp}


# --------------------------------------------------------------------------- #
# minimum-area bounding rectangle by brute angle search
# --------------------------------------------------------------------------- #
def fit_min_area_rect(pts_xy):
    """Sweep angle in [0, 90deg) at 0.5deg, axis-align by rotation, take the
    inner-percentile bbox (robust to a few wall-overshoot points / spurious
    obstacles inside the field). Returns center, dimensions, and orientation
    of the long edge in the original frame."""
    best = None
    for ang in np.arange(0.0, np.pi / 2, np.radians(0.5)):
        c, s = np.cos(ang), np.sin(ang)
        # rotate points by +ang
        xr = c * pts_xy[:, 0] - s * pts_xy[:, 1]
        yr = s * pts_xy[:, 0] + c * pts_xy[:, 1]
        xmin, xmax = np.percentile(xr, [1, 99])
        ymin, ymax = np.percentile(yr, [1, 99])
        w = xmax - xmin
        h = ymax - ymin
        area = w * h
        if best is None or area < best[0]:
            cxr = (xmin + xmax) / 2
            cyr = (ymin + ymax) / 2
            # rotate center back by -ang
            cx = c * cxr + s * cyr
            cy = -s * cxr + c * cyr
            if w >= h:
                length, width = w, h
                long_yaw = -ang
            else:
                length, width = h, w
                long_yaw = -ang + np.pi / 2
            best = (area, cx, cy, length, width, long_yaw)
    return best[1:]


# --------------------------------------------------------------------------- #
def build_T(theta, src_cx, src_cy, dst_cx, dst_cy):
    """Rigid 4x4: rotate by theta about origin, then translate so the rotated
    (src_cx, src_cy) lands on (dst_cx, dst_cy)."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([dst_cx, dst_cy]) - R @ np.array([src_cx, src_cy])
    T = np.eye(4)
    T[:2, :2] = R
    T[0, 3], T[1, 3] = t
    return T


def candidate_Ts(cx, cy, long_yaw, cfg):
    """4 candidates from the rectangle's 90deg symmetry."""
    return [build_T(cfg["yaw_map"] - long_yaw + k * np.pi / 2,
                    cx, cy, cfg["center_map"][0], cfg["center_map"][1])
            for k in range(4)]


def score_candidate(T, sp):
    """Lower is better. Where does the LIO origin (0,0,facing +x) land?
    Compare against the rough start pose."""
    landed_xy = T[:2, 3]
    yaw = np.arctan2(T[1, 0], T[0, 0])
    pos_err = np.linalg.norm(landed_xy - sp[:2])
    yaw_err = (yaw - sp[2] + np.pi) % (2 * np.pi) - np.pi
    return pos_err + 0.5 * abs(yaw_err)   # 0.5 m per rad weighting


# --------------------------------------------------------------------------- #
def save_matrix_yaml(path: Path, T: np.ndarray):
    path.write_text(
        "# T_map_lio: map_point = T * lio_point (homogeneous)\n"
        "T_map_lio:\n"
        + "\n".join("  - [" + ", ".join(f"{v: .8f}" for v in row) + "]"
                    for row in T)
        + "\n"
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--field", required=True, type=Path)
    ap.add_argument("--in", dest="in_pcd", required=True, type=Path)
    ap.add_argument("--out", dest="out_pcd", type=Path)
    ap.add_argument("--matrix-out", type=Path, default=Path("T_map_lio.yaml"))
    ap.add_argument("--zone", choices=["red", "blue"], default=None)
    args = ap.parse_args()

    cfg = load_config(args.field)
    zone = args.zone or cfg["default_zone"]
    if zone == "blue":
        cfg = mirror_for_blue(cfg)
    print(f"Zone: {zone}")
    print(f"Target rect: L={cfg['length']:.2f} W={cfg['width']:.2f}  "
          f"center=[{cfg['center_map'][0]:.2f}, {cfg['center_map'][1]:.2f}]  "
          f"yaw={np.degrees(cfg['yaw_map']):.1f}deg")
    sp = cfg["start_pose_map"]
    print(f"Start pose hint: x={sp[0]:.2f} y={sp[1]:.2f} "
          f"yaw={np.degrees(sp[2]):.0f}deg (only used to break 90deg symmetry)")

    print(f"Reading {args.in_pcd} ...")
    xyz = read_pcd_xyz(str(args.in_pcd))
    print(f"  {len(xyz)} points")

    zlo, zhi = cfg["wall_z"]
    wall = xyz[(xyz[:, 2] > zlo) & (xyz[:, 2] < zhi)]
    if len(wall) < 1000:
        sys.exit(f"\nToo few points in wall band z=[{zlo}, {zhi}]: "
                 f"{len(wall)} (need >=1000). Adjust wall_z to bracket the "
                 f"actual wall heights in the LIO cloud.")
    print(f"Wall band z=[{zlo}, {zhi}]: {len(wall)} points")

    cx, cy, length, width, long_yaw = fit_min_area_rect(wall[:, :2])
    print(f"Fitted rectangle: L={length:.2f} W={width:.2f}  "
          f"center=[{cx:.2f}, {cy:.2f}]  "
          f"long_yaw={np.degrees(long_yaw):.1f}deg")

    L_err = abs(length - cfg["length"]) / cfg["length"]
    W_err = abs(width - cfg["width"]) / cfg["width"]
    if L_err > 0.15 or W_err > 0.15:
        print(f"  WARN: fitted L/W differ from config by "
              f"{L_err*100:.0f}% / {W_err*100:.0f}% (>15%). "
              f"Likely cause: walls not fully scanned, or wall_z bracketed "
              f"a non-wall layer (obstacles, ground, ceiling).")

    cands = candidate_Ts(cx, cy, long_yaw, cfg)
    scores = [score_candidate(T, sp) for T in cands]
    order = np.argsort(scores)
    T_best = cands[order[0]]
    print("Candidate scores (sorted): "
          + ", ".join(f"{scores[i]:.3f}" for i in order))
    if scores[order[1]] - scores[order[0]] < 0.5:
        print(f"  WARN: best/second-best scores too close "
              f"({scores[order[0]]:.3f} vs {scores[order[1]]:.3f}). "
              f"start_pose_map did not clearly break the 90deg symmetry — "
              f"reboot with the robot in a more asymmetric corner.")

    # lift z so the LIO ground (low-percentile of z) lands on ground_z_map
    z_ground_lio = float(np.percentile(xyz[:, 2], 5))
    T_best[2, 3] = cfg["ground_z_map"] - z_ground_lio

    yaw_final = np.arctan2(T_best[1, 0], T_best[0, 0])
    print(f"\nT_map_lio: translation=[{T_best[0,3]:.3f}, {T_best[1,3]:.3f}, "
          f"{T_best[2,3]:.3f}]  yaw={np.degrees(yaw_final):.3f}deg")
    print(np.array2string(T_best, precision=6, suppress_small=True))

    save_matrix_yaml(args.matrix_out, T_best)
    print(f"\nWrote matrix -> {args.matrix_out}")

    if args.out_pcd:
        print(f"Transforming cloud -> {args.out_pcd} ...")
        out = (T_best @ np.hstack([xyz, np.ones((len(xyz), 1))]).T).T[:, :3]
        write_pcd_xyz(str(args.out_pcd), out)
        print(f"Wrote aligned PCD -> {args.out_pcd}")


if __name__ == "__main__":
    main()
