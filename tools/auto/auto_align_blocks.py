#!/usr/bin/env python3
"""
Auto-align a freshly built (LIO-frame) PCD to the map frame using the 12
meilin stepping stones (梅花桩) — no manual anchor picking in CloudCompare.

The stones are 1.2 m blocks butted edge-to-edge into a stepped terrain whose
4x3 height pattern is known (block_red.yaml / block_blue.yaml). Their map
coordinates ARE the map frame (defined by the original manual-anchor
alignment), and the asymmetric height pattern pins orientation uniquely
(no 90deg ambiguity). Two stages:

  Stage 1 — height-pattern coarse align. Starting from the rough mapping
    start pose (T0), grid-search (dx, dy, dyaw): sample the median height at
    each of the 12 expected cell centres and match against the known heights.
    This fixes yaw and which-cell-is-which, robust to ~10deg / 0.3 m start
    error, and validates the pattern (mismatch => wrong zone or 90deg off).

  Stage 2 — step-edge refinement. Between every pair of adjacent cells of
    DIFFERENT height, the riser is a sharp z-step whose map position is known
    (midway between the two cell centres). We locate the real z-step in the
    cloud (crisp to ~1.5 cm) and apply a robust median translation correction
    per axis with a shrinking search window (yaw stays locked from Stage 1).

A convergence guard refuses to emit a map if the fit does not converge
(usually means start_pose_map / --zone is wrong).

Validated against the aligned rc_2026_blue.pcd: ±0.2m/±10deg perturbation
recovery → t_err ~1.3 cm, yaw_err ~0deg.

Usage:
    auto_align_blocks.py --map map.yaml --in scans1.pcd \
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
# geometry helpers
# --------------------------------------------------------------------------- #
def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    return m


def rigid(dyaw, dx, dy):
    T = rot_z(dyaw)
    T[0, 3], T[1, 3] = dx, dy
    return T


def apply(T, xyz):
    return (T @ np.hstack([xyz, np.ones((len(xyz), 1))]).T).T[:, :3]


def start_pose_to_T0(start_pose):
    """Nominal T_map_lio from rough start pose [x, y, yaw]: the LIO origin
    (robot boot, facing +x) sits at start_pose in the map frame."""
    T0 = rot_z(start_pose[2])
    T0[0, 3], T0[1, 3] = start_pose[0], start_pose[1]
    return T0


# --------------------------------------------------------------------------- #
# config / block table
# --------------------------------------------------------------------------- #
def load_config(path: Path):
    with path.open() as f:
        cfg = yaml.safe_load(f)
    return {
        "blocks_red": cfg["blocks_red"],
        "blocks_blue": cfg["blocks_blue"],
        "default_zone": str(cfg.get("default_zone", "red")),
        "start_pose": np.asarray(cfg["start_pose_map"], float),
        "cell_radius": float(cfg.get("cell_radius", 0.25)),
        "edge_halfwin": float(cfg.get("edge_halfwin", 0.15)),
        "edge_band": float(cfg.get("edge_band", 0.30)),
        "height_tol": float(cfg.get("height_tol", 0.12)),
        "converge_rms": float(cfg.get("converge_rms", 0.03)),
        "search_xy": float(cfg.get("search_xy", 0.4)),
        "search_yaw_deg": float(cfg.get("search_yaw_deg", 15.0)),
    }


def load_blocks(yaml_path: Path):
    """12 stones (nodes 1..12) as a dict {(x,y): h} on the 4x3 grid,
    plus the sorted unique x (rows) and y (cols)."""
    with yaml_path.open() as f:
        data = yaml.safe_load(f)
    stones = []
    for k, v in data["blocks"].items():
        if 1 <= int(k) <= 12:
            stones.append((float(v["x"]), float(v["y"]), float(v["height"])))
    if len(stones) != 12:
        sys.exit(f"Expected 12 stones in {yaml_path}, found {len(stones)}.")
    xs = sorted({round(s[0], 3) for s in stones})
    ys = sorted({round(s[1], 3) for s in stones})
    if len(xs) != 4 or len(ys) != 3:
        sys.exit(f"Expected a 4x3 grid, got {len(xs)}x{len(ys)} in {yaml_path}.")
    grid = {(round(x, 3), round(y, 3)): h for x, y, h in stones}
    return grid, xs, ys


# --------------------------------------------------------------------------- #
# stage 1: height-pattern coarse alignment
# --------------------------------------------------------------------------- #
def sample_height(near, cx, cy, r):
    nx, ny, nz = near[:, 0], near[:, 1], near[:, 2]
    m = ((nx - cx) ** 2 + (ny - cy) ** 2 < r * r) & (nz > 0.05)
    zz = nz[m]
    return np.median(zz) if len(zz) >= 10 else np.nan


def coarse_align(xyz, grid, xs, ys, cfg):
    """Grid-search (dx,dy,dyaw) around T0 to best match the 12 known heights.

    Speed: only points in/near the block region matter, so after applying T0
    we crop to the block bounding box (+ a margin covering the search range)
    and subsample. The grid search then runs on ~tens of thousands of points,
    not the full cloud — turning minutes into a second or two."""
    T0 = start_pose_to_T0(cfg["start_pose"])
    base = apply(T0, xyz)
    centres = [(x, y, grid[(x, y)]) for x in xs for y in ys]
    r = cfg["cell_radius"]

    margin = cfg["search_xy"] + r + 0.3
    bx = (min(xs) - margin, max(xs) + margin)
    by = (min(ys) - margin, max(ys) + margin)
    inbox = ((base[:, 0] > bx[0]) & (base[:, 0] < bx[1]) &
             (base[:, 1] > by[0]) & (base[:, 1] < by[1]) & (base[:, 2] > 0.05))
    base = base[inbox]
    if len(base) > 120000:
        step = len(base) // 120000 + 1
        base = base[::step]

    def cost(T):
        near = apply(T, base)
        miss = 0
        d = 0.0
        for cx, cy, h in centres:
            hh = sample_height(near, cx, cy, r)
            if np.isnan(hh):
                miss += 1
            else:
                d += abs(hh - h)
        if miss > 2:
            return 1e9
        return d / (12 - miss)

    def search(yaws, xyspan, seed):
        best = (1e18, seed)
        for dyaw in yaws:
            for dx in xyspan:
                for dy in xyspan:
                    T = rigid(dyaw, dx, dy)
                    c = cost(T)
                    if c < best[0]:
                        best = (c, T)
        return best

    yaws = np.radians(np.arange(-cfg["search_yaw_deg"], cfg["search_yaw_deg"] + 0.1, 2))
    xyspan = np.arange(-cfg["search_xy"], cfg["search_xy"] + 0.01, 0.1)
    c0, T0c = search(yaws, xyspan, np.eye(4))
    yaw0 = np.arctan2(T0c[1, 0], T0c[0, 0])
    fyaw = yaw0 + np.radians(np.arange(-2, 2.01, 0.25))
    fxy0, fxy1 = T0c[0, 3], T0c[1, 3]
    best = (1e18, T0c)
    for dyaw in fyaw:
        for dx in fxy0 + np.arange(-0.12, 0.121, 0.03):
            for dy in fxy1 + np.arange(-0.12, 0.121, 0.03):
                T = rigid(dyaw, dx, dy)
                c = cost(T)
                if c < best[0]:
                    best = (c, T)
    return best[1] @ T0, best[0]


# --------------------------------------------------------------------------- #
# stage 2: step-edge refinement
# --------------------------------------------------------------------------- #
def find_step_along(near, axis, edge_coord, fixed_coord, halfwin, band):
    """Locate a z-step crossing along `axis` (0=x,1=y) at expected `edge_coord`,
    sampling a strip at the other coordinate = fixed_coord +/- band.
    Returns measured edge position, or None."""
    a = near[:, axis]
    o = near[:, 1 - axis]
    z = near[:, 2]
    m = (np.abs(o - fixed_coord) < band) & (a > edge_coord - halfwin) \
        & (a < edge_coord + halfwin) & (z > 0.05)
    if m.sum() < 30:
        return None
    av, zv = a[m], z[m]
    bins = np.arange(edge_coord - halfwin, edge_coord + halfwin, 0.03)
    bc, bz = [], []
    for i in range(len(bins) - 1):
        s = (av >= bins[i]) & (av < bins[i + 1])
        if s.sum() >= 3:
            bc.append((bins[i] + bins[i + 1]) / 2)
            bz.append(np.median(zv[s]))
    if len(bz) < 5:
        return None
    bc, bz = np.array(bc), np.array(bz)
    zmid = (bz.max() + bz.min()) / 2
    if bz.max() - bz.min() < 0.08:
        return None
    cross = None
    for i in range(len(bz) - 1):
        if (bz[i] - zmid) * (bz[i + 1] - zmid) < 0:
            t = (zmid - bz[i]) / (bz[i + 1] - bz[i])
            xc = bc[i] + t * (bc[i + 1] - bc[i])
            if cross is None or abs(xc - edge_coord) < abs(cross - edge_coord):
                cross = xc
    return cross


def edge_constraints(grid, xs, ys):
    """List edges between adjacent DIFFERENT-height cells.
    Each: (axis, edge_map_coord, fixed_map_coord)."""
    cons = []
    for i in range(len(xs) - 1):
        ec = (xs[i] + xs[i + 1]) / 2
        for y in ys:
            if grid[(xs[i], y)] != grid[(xs[i + 1], y)]:
                cons.append((0, ec, y))
    for j in range(len(ys) - 1):
        ec = (ys[j] + ys[j + 1]) / 2
        for x in xs:
            if grid[(x, ys[j])] != grid[(x, ys[j + 1])]:
                cons.append((1, ec, x))
    return cons


def refine(xyz, T_coarse, grid, xs, ys, cfg):
    """Refine TRANSLATION only against the step edges; yaw stays as fixed by
    Stage 1 (whose fine grid resolves it to <0.5deg).

    Why translation-only, single-shot, median:
      · If yaw is left free, the small per-edge offset folds into a spurious
        rotation that injects cm-scale cross-axis error — so yaw is locked.
      · Each edge is localized by a z-midpoint crossing carrying ~1.3cm of
        systematic offset plus ~1.3cm scatter. Iterating the correction against
        a biased measurement oscillates around that bias (limit cycle), so we
        apply ONE robust median correction per axis. Median rejects the
        occasional edge that locked onto noise.

    Stage 1 fixes yaw well but barely constrains position (heights are flat
    inside a tile), so Stage 2 pulls in translation: it sweeps the edge search
    window from wide to narrow, applying a median correction at each width.
    The wide first window tolerates a coarse position error up to ~0.4 m; the
    narrowing windows then home onto the correct risers (not the neighbouring
    one 1.2 m away). Returns (T, med_residual, n_edges)."""
    T = T_coarse.copy()
    cons = edge_constraints(grid, xs, ys)
    band = cfg["edge_band"]
    schedule = [0.55, 0.35, 0.20, cfg["edge_halfwin"]]
    med, n = np.inf, 0
    for half in schedule:
        near = apply(T, xyz)
        dxs, dys, res = [], [], []
        for axis, ec, fc in cons:
            pos = find_step_along(near, axis, ec, fc, half, band)
            if pos is None:
                continue
            (dxs if axis == 0 else dys).append(ec - pos)
            res.append(abs(ec - pos))
        n = len(res)
        if n < 4 or not dxs or not dys:
            return T, np.inf, n
        corr = np.eye(4)
        corr[0, 3] = np.median(dxs)
        corr[1, 3] = np.median(dys)
        T = corr @ T
        med = float(np.median(res))
    return T, med, n


# --------------------------------------------------------------------------- #
def save_matrix_yaml(path: Path, T: np.ndarray):
    path.write_text(
        "# T_map_lio: map_point = T * lio_point (homogeneous)\n"
        "T_map_lio:\n"
        + "\n".join("  - [" + ", ".join(f"{v: .8f}" for v in row) + "]" for row in T)
        + "\n"
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, type=Path)
    ap.add_argument("--in", dest="in_pcd", required=True, type=Path)
    ap.add_argument("--out", dest="out_pcd", type=Path)
    ap.add_argument("--matrix-out", type=Path, default=Path("T_map_lio.yaml"))
    ap.add_argument("--zone", choices=["red", "blue"], default=None)
    args = ap.parse_args()

    cfg = load_config(args.map)
    zone = args.zone or cfg["default_zone"]
    bpath = Path(cfg["blocks_blue"] if zone == "blue" else cfg["blocks_red"])
    if not bpath.is_absolute():
        bpath = (args.map.parent / bpath).resolve()
    grid, xs, ys = load_blocks(bpath)
    print(f"Zone: {zone}   block table: {bpath}")
    sp = cfg["start_pose"]
    print(f"Start pose (T0): x={sp[0]:.2f} y={sp[1]:.2f} yaw={np.degrees(sp[2]):.0f}deg")

    print(f"Reading {args.in_pcd} ...")
    xyz = read_pcd_xyz(str(args.in_pcd))
    print(f"  {len(xyz)} points")

    print("\nStage 1: height-pattern coarse align ...")
    T_coarse, hcost = coarse_align(xyz, grid, xs, ys, cfg)
    print(f"  mean height residual: {hcost:.3f} m  "
          f"(yaw={np.degrees(np.arctan2(T_coarse[1,0],T_coarse[0,0])):.2f}deg, "
          f"t=[{T_coarse[0,3]:.2f}, {T_coarse[1,3]:.2f}])")
    if hcost > cfg["height_tol"]:
        sys.exit(f"\nHeight pattern does not match (residual {hcost:.3f} m > "
                 f"{cfg['height_tol']} m). Wrong --zone, 90deg off, or "
                 f"start_pose_map too far from the real boot pose.")

    print("Stage 2: step-edge refinement (translation, yaw locked) ...")
    T, rms, nedges = refine(xyz, T_coarse, grid, xs, ys, cfg)
    print(f"  edges used: {nedges}   median edge residual: {rms:.4f} m")
    print(f"T_map_lio: translation=[{T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}]  "
          f"yaw={np.degrees(np.arctan2(T[1,0],T[0,0])):.3f} deg")
    print(np.array2string(T, precision=6, suppress_small=True))

    if rms > cfg["converge_rms"] or nedges < 4:
        sys.exit(f"\nNOT CONVERGED (edge RMS {rms:.3f} m, {nedges} edges). "
                 f"Fix start_pose_map / --zone; refusing to write a bad map.")

    save_matrix_yaml(args.matrix_out, T)
    print(f"\nWrote matrix -> {args.matrix_out}")

    if args.out_pcd:
        print(f"Transforming cloud -> {args.out_pcd} ...")
        write_pcd_xyz(str(args.out_pcd), apply(T, xyz))
        print(f"Wrote aligned PCD -> {args.out_pcd}")


if __name__ == "__main__":
    main()
