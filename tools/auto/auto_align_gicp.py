#!/usr/bin/env python3
"""
Auto-align a freshly built (LIO-frame) PCD to the map frame by GICP-registering
it onto an IDEAL point-cloud model that is already drawn in the map frame
(origin at the red/blue corner). The registration result IS T_map_lio — no
CloudCompare, no hand-picked anchors.

General method (works on non-rectangular / stone-less new fields too), as long
as you have:
  1. an ideal model PCD authored in the target map frame, and
  2. a rough start pose to seed GICP and break symmetry.

Pipeline:
  1. init_pose_map [x, y, z, roll, pitch, yaw] -> nominal transform T0 (seeds
     GICP; a local optimizer NEEDS a decent initial guess, and on a symmetric
     field it is also what breaks the 90/180deg ambiguity). Same 6-DoF layout
     and ZYX rotation order as the online small_gicp_relocalization node's
     `init_pose` parameter.
  2. Feed source (LIO cloud) + target (ideal model) + T0 to small_gicp via the
     small_gicp_align shim. It returns the refined T_target_source = T_map_lio.
  3. Always write T_map_lio.yaml and (if an output path is set) the aligned
     map-frame PCD — pure-Python transform, no PCL CLI. Fit quality (inlier
     ratio, drift from the seed) is printed as a non-blocking WARNING only; the
     small_gicp result is never withheld.

Build the shim once before first use:
    cmake -S tools/auto/gicp -B tools/auto/gicp/build
    cmake --build tools/auto/gicp/build

Usage:
    # paths (prior_model / input_pcd / output_pcd / matrix_out) live in the yaml:
    auto_align_gicp.py --config gicp.yaml [--zone red|blue]

    # or override any path on the CLI:
    auto_align_gicp.py --config gicp.yaml --in scans1.pcd --out scans1_map.pcd
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

from pcd_io import read_pcd_xyz, write_pcd_xyz

SHIM = Path(__file__).resolve().parent / "gicp" / "build" / "small_gicp_align"


# --------------------------------------------------------------------------- #
def load_config(path: Path):
    with path.open() as f:
        cfg = yaml.safe_load(f)
    base = path.resolve().parent

    def resolve(p):
        return None if p is None else (base / p).resolve()

    return {
        "base": base,
        "prior_model": resolve(cfg["prior_model"]),
        "prior_model_blue": resolve(cfg.get("prior_model_blue")),
        "default_zone": str(cfg.get("default_zone", "red")),
        "input_pcd": resolve(cfg.get("input_pcd")),
        "output_pcd": resolve(cfg.get("output_pcd")),
        "matrix_out": resolve(cfg.get("matrix_out")),
        "init_pose_map": np.asarray(cfg["init_pose_map"], float),
        "downsampling_resolution": float(cfg.get("downsampling_resolution", 0.25)),
        "max_correspondence_distance": float(cfg.get("max_correspondence_distance", 1.0)),
        "max_iterations": int(cfg.get("max_iterations", 20)),
        "num_threads": int(cfg.get("num_threads", 4)),
        "inlier_ratio_min": float(cfg.get("inlier_ratio_min", 0.5)),
        "max_shift_m": float(cfg.get("max_shift_m", 1.0)),
        "max_yaw_deg": float(cfg.get("max_yaw_deg", 20.0)),
        "passthrough": cfg.get("passthrough") or {},
    }


def rot_zyx(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll) — matches the online
    small_gicp_relocalization node's init_pose convention (ZYX intrinsic)."""
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return Rz @ Ry @ Rx


def T0_from_init_pose(pose):
    """Nominal LIO->map transform from init_pose_map [x, y, z, roll, pitch, yaw]."""
    x, y, z, roll, pitch, yaw = (float(v) for v in pose)
    T = np.eye(4)
    T[:3, :3] = rot_zyx(roll, pitch, yaw)
    T[:3, 3] = [x, y, z]
    return T


def apply_passthrough(xyz, pt_cfg):
    """直通滤波:按 x/y/z 各轴的 [min, max] 区间裁剪点云,过滤掉干扰配准的点
    (天花板、远处场外、地面等)。

    pt_cfg 形如:
        {enabled: true, frame: lio,
         x: [min, max], y: [min, max], z: [min, max]}
    每个轴可选,只写想限制的轴;min/max 任一可为 null(表示该侧不限)。
    返回 (滤波后的 xyz, 保留数, 原始数)。
    """
    n0 = len(xyz)
    if not pt_cfg or not pt_cfg.get("enabled", False):
        return xyz, n0, n0
    mask = np.ones(n0, dtype=bool)
    for ax, col in (("x", 0), ("y", 1), ("z", 2)):
        rng = pt_cfg.get(ax)
        if rng is None:
            continue
        lo, hi = rng
        if lo is not None:
            mask &= xyz[:, col] >= float(lo)
        if hi is not None:
            mask &= xyz[:, col] <= float(hi)
    return xyz[mask], int(mask.sum()), n0


def mirror_pose_blue(pose):
    """Red->blue mirror about the map x-z plane (y -> -y). Under that reflection
    a rigid pose maps as: y, roll, yaw negate; x, z, pitch unchanged."""
    p = pose.copy()
    p[1] *= -1.0   # y
    p[3] *= -1.0   # roll
    p[5] *= -1.0   # yaw
    return p


def save_matrix_yaml(path: Path, T: np.ndarray):
    # Same format as align_pcd_to_map.py — downstream reads T_map_lio unchanged.
    path.write_text(
        "# T_map_lio: map_point = T * lio_point (homogeneous)\n"
        "T_map_lio:\n"
        + "\n".join("  - [" + ", ".join(f"{v: .8f}" for v in row) + "]"
                    for row in T)
        + "\n"
    )


def yaw_of(T):
    return np.arctan2(T[1, 0], T[0, 0])


def run_shim(target_xyz, source_xyz, T0, cfg):
    """Write clouds to temp raw files, call the shim, parse its output."""
    if not SHIM.exists():
        sys.exit(f"\nShim not built: {SHIM}\nBuild it once:\n"
                 f"  cmake -S tools/auto/gicp -B tools/auto/gicp/build\n"
                 f"  cmake --build tools/auto/gicp/build")
    with tempfile.TemporaryDirectory() as td:
        tp = Path(td) / "target.bin"
        sp = Path(td) / "source.bin"
        np.ascontiguousarray(target_xyz, dtype=np.float64).tofile(tp)
        np.ascontiguousarray(source_xyz, dtype=np.float64).tofile(sp)
        cmd = [str(SHIM), str(tp), str(sp),
               "--init", *[f"{v:.10f}" for v in T0.flatten()],
               "--downsample", f"{cfg['downsampling_resolution']:.6f}",
               "--max-dist", f"{cfg['max_correspondence_distance']:.6f}",
               "--max-iter", str(cfg["max_iterations"]),
               "--threads", str(cfg["num_threads"])]
        proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f"\nsmall_gicp_align failed (rc={proc.returncode}):\n{proc.stderr}")
    res = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "T":
            res["T"] = np.array([float(v) for v in parts[1:17]]).reshape(4, 4)
        elif len(parts) == 2:
            res[parts[0]] = parts[1]
    return res


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, type=Path)
    ap.add_argument("--in", dest="in_pcd", type=Path, default=None,
                    help="LIO-frame PCD to align (the GICP source); "
                         "overrides input_pcd in the config")
    ap.add_argument("--out", dest="out_pcd", type=Path, default=None,
                    help="output map-frame PCD; overrides output_pcd in the config")
    ap.add_argument("--matrix-out", type=Path, default=None,
                    help="4x4 T_map_lio yaml; overrides matrix_out in the config")
    ap.add_argument("--zone", choices=["red", "blue"], default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    zone = args.zone or cfg["default_zone"]

    # CLI overrides config; config paths are resolved relative to the config dir.
    in_pcd = args.in_pcd or cfg["input_pcd"]
    out_pcd = args.out_pcd or cfg["output_pcd"]
    matrix_out = args.matrix_out or cfg["matrix_out"] or Path("T_map_lio.yaml")
    if in_pcd is None:
        sys.exit("No input cloud: pass --in or set input_pcd in the config.")

    pose = cfg["init_pose_map"]
    if pose.shape != (6,):
        sys.exit("init_pose_map must be 6 numbers [x, y, z, roll, pitch, yaw] "
                 f"(got {pose.shape[0]}).")

    # --- pick / derive the ideal model target for this zone ------------------
    mirror_model = False
    if zone == "blue":
        if cfg["prior_model_blue"] is not None:
            model_path = cfg["prior_model_blue"]
        else:
            model_path = cfg["prior_model"]      # red-frame model, mirror y below
            mirror_model = True
        pose = mirror_pose_blue(pose)            # red/blue mirror about map x-axis
    else:
        model_path = cfg["prior_model"]

    print(f"Zone: {zone}")
    print(f"Ideal model (target): {model_path}"
          + ("  [y-mirrored from red frame]" if mirror_model else ""))
    print(f"Init pose (T0 seed): "
          f"xyz=[{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]  "
          f"rpy=[{np.degrees(pose[3]):.1f}, {np.degrees(pose[4]):.1f}, "
          f"{np.degrees(pose[5]):.1f}]deg")

    if not Path(model_path).exists():
        sys.exit(f"Ideal model not found: {model_path}")

    print(f"Reading target model {model_path} ...")
    target = read_pcd_xyz(str(model_path))
    if mirror_model:
        target = target.copy()
        target[:, 1] *= -1
    print(f"  {len(target)} points")

    print(f"Reading source (LIO) {in_pcd} ...")
    source = read_pcd_xyz(str(in_pcd))
    print(f"  {len(source)} points")

    # --- 直通滤波:裁掉干扰 GICP 配准的点(在 LIO 系里裁,配准前) ----------
    pt_cfg = cfg["passthrough"]
    if pt_cfg.get("enabled", False):
        bounds = ", ".join(f"{ax}∈{pt_cfg[ax]}" for ax in ("x", "y", "z")
                           if pt_cfg.get(ax) is not None) or "(无区间)"
        source, kept, n0 = apply_passthrough(source, pt_cfg)
        print(f"  直通滤波(LIO系) {bounds}: 保留 {kept}/{n0} 点 "
              f"(裁掉 {n0 - kept})")
        if kept == 0:
            sys.exit("直通滤波后源点云为空 —— 检查 passthrough 区间是否写反/单位错误。")

    T0 = T0_from_init_pose(pose)

    print("Running small_gicp ...")
    res = run_shim(target, source, T0, cfg)
    T = res["T"]                                  # T_map_lio
    converged = res.get("converged") == "1"
    iterations = int(res.get("iterations", 0))
    num_inliers = int(res.get("num_inliers", 0))
    error = float(res.get("error", 0.0))
    source_points = int(res.get("source_points", 0))

    # --- diagnostics ---------------------------------------------------------
    inlier_ratio = num_inliers / source_points if source_points else 0.0
    mean_res = (error / num_inliers) if num_inliers else float("inf")
    delta = np.linalg.inv(T0) @ T                 # how far GICP moved from T0
    shift = float(np.linalg.norm(delta[:3, 3]))
    dyaw = np.degrees((yaw_of(delta) + np.pi) % (2 * np.pi) - np.pi)

    print(f"\nGICP: converged={converged} iters={iterations}  "
          f"inliers={num_inliers}/{source_points} ({inlier_ratio*100:.0f}%)  "
          f"mean_res={mean_res:.4f} m")
    print(f"Refinement moved {shift:.3f} m / {dyaw:+.1f}deg from the init-pose seed")
    print(f"T_map_lio: translation=[{T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}]  "
          f"yaw={np.degrees(yaw_of(T)):.3f}deg")
    print(np.array2string(T, precision=6, suppress_small=True))

    # --- non-blocking quality warnings (output is written regardless) --------
    # These are hints, not gates: small_gicp always runs and we always write its
    # result. A low inlier ratio / large drift just flags that the fit may be
    # off (bad init pose, wrong zone, or LIO cloud extends far beyond the model).
    warnings = []
    if not converged:
        warnings.append("GICP did not converge (result may be unreliable)")
    if inlier_ratio < cfg["inlier_ratio_min"]:
        warnings.append(f"low inlier ratio {inlier_ratio*100:.0f}% "
                        f"(< {cfg['inlier_ratio_min']*100:.0f}%): clouds overlap "
                        f"little — check init_pose_map / zone, or crop the LIO "
                        f"cloud to the model's footprint")
    if shift > cfg["max_shift_m"] or abs(dyaw) > cfg["max_yaw_deg"]:
        warnings.append(f"large drift from seed {shift:.2f} m / {dyaw:+.1f}deg "
                        f"(> {cfg['max_shift_m']} m / {cfg['max_yaw_deg']}deg): "
                        f"possible symmetry flip / wrong basin")
    if warnings:
        print("\nWARNING (writing output anyway):")
        for w in warnings:
            print(f"  - {w}")

    save_matrix_yaml(matrix_out, T)
    print(f"\nWrote matrix -> {matrix_out}")

    if out_pcd:
        print(f"Transforming cloud -> {out_pcd} ...")
        out = (T @ np.hstack([source, np.ones((len(source), 1))]).T).T[:, :3]
        write_pcd_xyz(str(out_pcd), out)
        print(f"Wrote aligned PCD -> {out_pcd}")
    else:
        print("(no output_pcd / --out — only computed T_map_lio)")


if __name__ == "__main__":
    main()
