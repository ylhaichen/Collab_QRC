#!/usr/bin/env python3
"""Offline 3D Gaussian Splatting trainer (proposal §10.2).

Two operating modes:

A. **Init-only** (default; CPU-only):
   Convert `accumulated_cloud.ply` (geometry-only XYZ from
   accumulated_pointcloud_node) into a 3DGS-compatible PLY by
   assigning per-point Gaussian attributes. Each accumulated voxel
   becomes one Gaussian with:
       - position    : voxel centroid
       - scale       : isotropic, proportional to local point density
       - rotation    : identity (quat)
       - opacity     : 0.5
       - sh_dc       : grey (no RGB) or coloured by elevation
   The output PLY follows the Inria gaussian-splatting conventions
   (vertex fields x y z nx ny nz f_dc_0 f_dc_1 f_dc_2 opacity scale_0
   scale_1 scale_2 rot_0 rot_1 rot_2 rot_3) and is loadable by
   SuperSplat / WebGL viewers / gsplat. This is a non-trained
   scaffold; visual quality is bounded by the geometry.

B. **Full training** (`--train`):
   If `gsplat` is importable AND a CUDA GPU is available AND
   keyframes/ exists with ≥ 8 frames, run the standard 3DGS
   optimisation loop on the keyframe set, using the init from step A
   as the starting Gaussian distribution. Saves the optimised model
   to <out>/3dgs_optimized.ply.

This separation lets us produce a viewable 3DGS deliverable on the
laptop without GPU-bound training, and upgrade to full optimisation
when keyframes are captured.

Usage:
    python3 train_3dgs.py <trial_dir>                 # init only
    python3 train_3dgs.py <trial_dir> --train         # also train
    python3 train_3dgs.py <trial_dir> --voxel 0.10    # downsample input
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


# ──────────────────────────────────────────────────────────────────────
# PLY parsing — tiny ASCII reader for accumulated_cloud.ply.
# ──────────────────────────────────────────────────────────────────────


def _read_ply_xyz(path: Path) -> np.ndarray:
    """Read x, y, z columns from an ASCII PLY file. Returns (N, 3)."""
    with path.open("r", encoding="utf-8") as fh:
        header_lines: list[str] = []
        n_vertex = 0
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"{path}: PLY header ended unexpectedly")
            header_lines.append(line.rstrip("\n"))
            if line.startswith("element vertex"):
                n_vertex = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        rows = []
        for _ in range(n_vertex):
            tok = fh.readline().split()
            if len(tok) < 3:
                continue
            rows.append((float(tok[0]), float(tok[1]), float(tok[2])))
    return np.array(rows, dtype=np.float32)


def _voxel_downsample(xyz: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0.0:
        return xyz
    keys = np.floor(xyz / voxel).astype(np.int32)
    # Use lexsort to group identical keys, then take the first per group.
    flat = (
        keys[:, 0].astype(np.int64) * 7919 * 7919
        + keys[:, 1].astype(np.int64) * 7919
        + keys[:, 2].astype(np.int64)
    )
    _, idx = np.unique(flat, return_index=True)
    return xyz[idx]


# ──────────────────────────────────────────────────────────────────────
# Gaussian attribute initialisation.
# ──────────────────────────────────────────────────────────────────────


def _local_density_scale(xyz: np.ndarray, k: int = 8) -> np.ndarray:
    """For each point, compute mean distance to k nearest neighbours.
    Returns a (N,) array of suggested isotropic Gaussian σ values
    (clipped to a sane range)."""
    n = len(xyz)
    if n == 0:
        return np.empty((0,), dtype=np.float32)
    if n <= k + 1:
        return np.full((n,), 0.10, dtype=np.float32)
    # KD-tree via scipy if available, else brute-force tile-and-mean.
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(xyz)
        d, _ = tree.query(xyz, k=k + 1)
        mean_nn = d[:, 1:].mean(axis=1)
    except ImportError:
        # O(N^2 * k) fallback; OK for small clouds.
        mean_nn = np.zeros((n,), dtype=np.float32)
        for i in range(n):
            diff = xyz - xyz[i]
            d2 = np.einsum("ij,ij->i", diff, diff)
            d2[i] = np.inf
            sub = np.partition(d2, k)[:k]
            mean_nn[i] = float(np.sqrt(sub.mean()))
    # Map mean-NN distance → Gaussian σ. Tight cluster → smaller σ.
    sigma = np.clip(mean_nn * 0.5, 0.02, 0.30).astype(np.float32)
    return sigma


def _elevation_color(xyz: np.ndarray) -> np.ndarray:
    """Map point z value to an RGB triple in [0, 1] via a viridis-like
    ramp. Returns (N, 3)."""
    n = len(xyz)
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    z = xyz[:, 2]
    z_lo, z_hi = float(np.percentile(z, 5)), float(np.percentile(z, 95))
    z_lo = z_lo if z_hi > z_lo + 1e-3 else (z_lo - 0.05)
    t = np.clip((z - z_lo) / max(1e-3, (z_hi - z_lo)), 0.0, 1.0)
    # Simple 3-stop viridis approximation.
    r = 0.20 + 0.65 * t ** 1.5
    g = 0.20 + 0.55 * t
    b = 0.55 - 0.45 * t
    rgb = np.stack([r, g, b], axis=1).astype(np.float32)
    return rgb


def _sh0_from_rgb(rgb: np.ndarray) -> np.ndarray:
    """Convert RGB ∈ [0,1] to spherical-harmonics DC coefficient that
    Inria gaussian-splatting consumers expect (rgb - 0.5) / 0.28209)."""
    return ((rgb - 0.5) / 0.28209479177387814).astype(np.float32)


def _logit_opacity(o: float) -> float:
    o = float(min(0.99, max(0.01, o)))
    return float(math.log(o / (1.0 - o)))


# ──────────────────────────────────────────────────────────────────────
# 3DGS PLY writer (Inria gaussian-splatting layout).
# ──────────────────────────────────────────────────────────────────────


def write_3dgs_ply(
    out_path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray,
    sigma: np.ndarray,
    opacity: float = 0.5,
) -> None:
    n = len(xyz)
    f_dc = _sh0_from_rgb(rgb)
    log_sigma = np.log(np.maximum(1e-3, sigma)).astype(np.float32)
    log_op = np.full((n,), _logit_opacity(opacity), dtype=np.float32)
    # Identity rotation as unit quaternion (w, x, y, z).
    rot = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (n, 1))
    nz = np.zeros((n, 3), dtype=np.float32)  # placeholder normals
    with out_path.open("w", encoding="utf-8") as fh:
        fh.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {n}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property float nx\nproperty float ny\nproperty float nz\n"
            "property float f_dc_0\nproperty float f_dc_1\nproperty float f_dc_2\n"
            "property float opacity\n"
            "property float scale_0\nproperty float scale_1\nproperty float scale_2\n"
            "property float rot_0\nproperty float rot_1\nproperty float rot_2\nproperty float rot_3\n"
            "end_header\n"
        )
        for i in range(n):
            x, y, z = xyz[i]
            nxv, nyv, nzv = nz[i]
            d0, d1, d2 = f_dc[i]
            fh.write(
                f"{x:.4f} {y:.4f} {z:.4f} "
                f"{nxv:.3f} {nyv:.3f} {nzv:.3f} "
                f"{d0:.4f} {d1:.4f} {d2:.4f} "
                f"{log_op[i]:.4f} "
                f"{log_sigma[i]:.4f} {log_sigma[i]:.4f} {log_sigma[i]:.4f} "
                f"{rot[i,0]:.4f} {rot[i,1]:.4f} {rot[i,2]:.4f} {rot[i,3]:.4f}\n"
            )


# ──────────────────────────────────────────────────────────────────────
# Optional: full gsplat training. Stays a stub when keyframes/GPU
# absent; the upgrade path is documented in PORTING_3DGS.md.
# ──────────────────────────────────────────────────────────────────────


def _maybe_train_gsplat(
    init_ply: Path,
    keyframes_dir: Path,
    cameras_json: Path,
    out_path: Path,
) -> bool:
    """Attempt full 3DGS optimisation. Returns True if training ran.

    Requires:
      - `gsplat` Python package importable
      - CUDA GPU available
      - keyframes/ contains ≥ 8 PNG + .pose.json pairs
      - cameras.json parseable
    """
    try:
        import gsplat  # noqa: F401
    except ImportError:
        print("[train_3dgs] gsplat not installed; skipping full training.")
        print("            Install: pip install gsplat (requires CUDA)")
        return False
    try:
        import torch
    except ImportError:
        print("[train_3dgs] pytorch not importable; skipping full training.")
        return False
    if not torch.cuda.is_available():
        print("[train_3dgs] no CUDA device; skipping full training.")
        return False
    kf_pngs = sorted(keyframes_dir.glob("*.png")) if keyframes_dir.exists() else []
    if len(kf_pngs) < 8:
        print(f"[train_3dgs] only {len(kf_pngs)} keyframes; need ≥ 8.")
        return False
    # The actual gsplat optimisation loop is intentionally out of
    # scope here — see PORTING_3DGS.md. For now, log the readiness
    # and exit; the init_ply is already a viewable scaffold.
    print(
        f"[train_3dgs] all prerequisites met ({len(kf_pngs)} keyframes, "
        f"GPU {torch.cuda.get_device_name(0)}). Implement the gsplat "
        "training loop here per PORTING_3DGS.md when ready."
    )
    return False  # stays False until the loop is wired


# ──────────────────────────────────────────────────────────────────────
# Driver.
# ──────────────────────────────────────────────────────────────────────


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("trial_dir", type=Path)
    ap.add_argument("--cloud", type=Path, default=None,
                    help="override accumulated_cloud.ply path")
    ap.add_argument("--keyframes", type=Path, default=None,
                    help="override keyframes/ directory")
    ap.add_argument("--cameras", type=Path, default=None,
                    help="override cameras.json path")
    ap.add_argument("--out", type=Path, default=None,
                    help="output 3dgs_init.ply path")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="optional voxel-downsample on input cloud")
    ap.add_argument("--opacity", type=float, default=0.5,
                    help="initial Gaussian opacity")
    ap.add_argument("--color", type=str, choices=("elevation", "grey"), default="elevation")
    ap.add_argument("--train", action="store_true",
                    help="run gsplat optimisation if prerequisites met")
    args = ap.parse_args()

    trial = args.trial_dir.resolve()
    if not trial.is_dir():
        print(f"not a directory: {trial}", file=sys.stderr)
        return 2
    cloud = args.cloud or (trial / "accumulated_cloud.ply")
    keyframes = args.keyframes or (trial / "keyframes")
    cameras = args.cameras or (trial / "cameras.json")
    out_path = args.out or (trial / "3dgs_init.ply")

    if not cloud.exists():
        print(f"[train_3dgs] missing {cloud}; cannot init.", file=sys.stderr)
        return 1
    t0 = time.time()
    xyz = _read_ply_xyz(cloud)
    if xyz.size == 0:
        print(f"[train_3dgs] {cloud} has 0 points.")
        return 1
    if args.voxel > 0:
        xyz = _voxel_downsample(xyz, float(args.voxel))
    n = len(xyz)
    sigma = _local_density_scale(xyz)
    if args.color == "elevation":
        rgb = _elevation_color(xyz)
    else:
        rgb = np.full((n, 3), 0.5, dtype=np.float32)
    write_3dgs_ply(out_path, xyz, rgb, sigma, opacity=float(args.opacity))
    elapsed = time.time() - t0
    summary = {
        "schema": "3dgs_init/v1",
        "input_cloud": str(cloud),
        "init_ply": str(out_path),
        "gaussian_count": int(n),
        "voxel_downsample_m": float(args.voxel),
        "color_mode": args.color,
        "opacity_init": float(args.opacity),
        "elapsed_sec": round(elapsed, 3),
    }
    if args.train:
        trained = _maybe_train_gsplat(out_path, keyframes, cameras, trial / "3dgs_optimized.ply")
        summary["train_attempted"] = True
        summary["train_succeeded"] = bool(trained)
    summary_path = trial / "3dgs_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(
        f"[train_3dgs] init done: {n} Gaussians → {out_path} ({elapsed:.2f}s); "
        f"summary → {summary_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
