# 3D Gaussian Splatting Integration

This file documents the integration points for proposal §10.2 (offline
3DGS) and §10.3 (online 3DGS planning). The geometry-first stack
(Stage 4 + 7 — `accumulated_pointcloud_node`,
`reconstruction_quality_node`) already produces all the inputs that
both 3DGS layers consume; this doc records what wires up where so
future work can swap in real training without rebuilding the data
pipeline.

---

## §10.2 — Offline 3DGS

### Status

**Scaffold complete.** `scripts/offline/train_3dgs.py` consumes a
finished trial directory and produces a viewable `3dgs_init.ply`
without GPU training. Each accumulated voxel is mapped to one
Gaussian with isotropic σ derived from local k-NN density (k=8) and
RGB DC coefficient driven by either elevation (default) or grey
(`--color grey`). The output PLY follows the Inria
gaussian-splatting field layout (vertex props `x y z nx ny nz f_dc_0
f_dc_1 f_dc_2 opacity scale_0 scale_1 scale_2 rot_0 rot_1 rot_2
rot_3`) so SuperSplat / WebGL renderers / `gsplat` load it directly.

### Inputs (already produced by the live stack)

| File | Producer | Purpose for 3DGS |
|---|---|---|
| `accumulated_cloud.ply` | `accumulated_pointcloud_node` (Stage 7) | Gaussian centre seeds |
| `reconstruction_quality_summary.json` | `reconstruction_quality_node` (Stage 7) | per-voxel `low_quality` mask → can prioritise revisits |
| `keyframes/<ns>_NNNN.png` + `.pose.json` | `keyframe_logger_node` (Stage 9, optional) | RGB supervision for full training |
| `cameras.json` | `keyframe_logger_node` | intrinsics + per-frame extrinsics |

### Run modes

```bash
# Init only — CPU, ~0.2 s for 15 k Gaussians, output viewable in any 3DGS viewer.
python3 scripts/offline/train_3dgs.py results/<run>/trial_1

# With voxel downsample + grey colour
python3 scripts/offline/train_3dgs.py results/<run>/trial_1 \
    --voxel 0.20 --color grey

# Full training (placeholder — requires implementing the gsplat loop;
# see "Upgrading to full training" below)
python3 scripts/offline/train_3dgs.py results/<run>/trial_1 --train
```

The init scaffold's visual quality is bounded by the geometry — no
view-fidelity refinement, no learned colours. Suitable for paper
figures of the geometric coverage; not yet for novel-view-synthesis
metrics (PSNR / SSIM / LPIPS in §14.3).

### Upgrading to full training

1. Install gsplat: `pip install --user gsplat` (1.5.3 ships
   wheels — no CUDA toolkit needed for `pip install`).
2. **MuJoCo RGBD camera is already on** as of Stage 10. The plugin
   auto-instantiates from `<camera>` tags in `demo3_mixed.xml` and
   publishes `/front_camera/color/image_raw` (robot_a) and
   `/b_front_camera/color/image_raw` (robot_b) at 25 Hz, encoding
   `8UC3`. `keyframe_logger_node` spawns automatically when
   `reconstruction_quality_enabled=true` and saves to
   `<trial>/keyframes/<ns>_NNNN.png` plus `cameras.json`.
3. Stage-10b verified: a 90 s `loop_risk_recon_graph_mppi` run
   captured **115 keyframes (56 + 59)** with full per-frame pose +
   intrinsics — well above the ≥ 8 threshold.
4. The optimisation loop body is wired in
   `_maybe_train_gsplat()` in `scripts/offline/train_3dgs.py`:
   - Loads init PLY → `means / log_scales / quats / opacities /
     sh0` (all `requires_grad=True`)
   - Per step: pick keyframe, build `viewmat = inv(world-from-cam)`,
     call `gsplat.rasterization(...)` with `(C=1, N, 3)` colors,
     compute L1 loss, Adam step
   - Saves `3dgs_optimized.ply` in Inria layout
5. **Runtime blocker on this dev machine**: gsplat's CUDA backend
   needs JIT compilation at first call (no prebuilt `.so` ships
   with the wheel). Compilation fails because:
     * apt-shipped CUDA toolkit on Ubuntu 22.04 is 11.5
       (`/usr/bin/nvcc`), too old for `sm_89` (RTX 4070 Ada).
     * pip-shipped `nvidia-cuda-nvcc-cu12` 12.9.86 only contains
       `ptxas`, not `nvcc`.
     * pip-shipped `nvidia-cuda-runtime-cu13` provides the headers
       (`cuda_runtime.h`) and `libcudart.so.13` but no compiler.
   To unblock training on this machine, install a CUDA toolkit
   ≥ 12.0 with sudo:
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda_12.4.1_550.54.15_linux.run
   sudo sh cuda_12.4.1_550.54.15_linux.run --toolkit --silent
   export CUDA_HOME=/usr/local/cuda-12.4
   export PATH=$CUDA_HOME/bin:$PATH
   ```
   On the compute cluster (the 4×A100 / 2×4070 noted in the
   proposal) this is already the default install.
6. After the first successful run, `3dgs_optimized.ply` lands next
   to `3dgs_init.ply` and a follow-up commit can add a
   `reconstruction_3dgs` claim type to `mission_summary_generator.py`
   citing PSNR / SSIM against held-out keyframes.

### Outputs

Per trial directory, after `train_3dgs.py`:
- `3dgs_init.ply` — viewable Gaussian splatting model
- `3dgs_summary.json` — gaussian count, voxel down-sample, mode, elapsed
- (`3dgs_optimized.ply` — only after full training is wired)

Mission-summary integration: a future claim type
`reconstruction_3dgs` cites `3dgs_summary.json` evidence id; the
verifier marks it `unverified` when the file is absent.

---

## §10.3 — Online 3DGS Planning

### Status

**Out of scope for the current paper** per proposal §10.3 ("optional
for a later paper"). This file documents the integration interface
so a follow-up project can plug in without touching the live stack.

### Interface contract

A future `online_3dgs_planning_node` would:

1. Subscribe to:
   - `/cfpa2/accumulated_cloud` (Gaussian seed updates)
   - `/<ns>/front_camera/image_raw` + pose (training frames)
2. Maintain an in-memory gsplat model on GPU.
3. After every K training iterations, evaluate per-voxel rendered-
   view fidelity loss → publish a Gaussian uncertainty map on
   `/cfpa2/reconstruction_uncertainty`.
4. CFPA2 already supports `role=reconstruct` candidates; the online
   uncertainty becomes a richer `recon_gain` than the geometry proxy.

### Why deferred

- Memory budget: full 3DGS training on a 384 m² scene with Mid-360
  + ~2 M raw points needs ≥ 16 GB VRAM during densification. The
  laptop RTX 4070 (8 GB) cannot host it; A100s would.
- Latency budget: 100 ms / iteration × 10 iterations / second × per
  robot = ≥ 1 s of GPU work per pose update. Multi-robot real-time
  is borderline.
- Research scope: §10.3 explicitly states the contribution is the
  collaborative quadruped system + grounded reports, not online
  3DGS itself. An offline 3DGS visualisation already supports the
  paper figures.

### Engineering hooks already in place

- `accumulated_pointcloud_node` publishes a stable PointCloud2 stream
  of voxel centres (5 Hz, capped at 1 M voxels), suitable as the
  Gaussian densification source.
- `keyframe_logger_node` saves trigger-based (translation /
  rotation / period) keyframes; the same trigger logic transfers
  to online mode by replacing disk-write with GPU upload.
- `reconstruction_quality_node` already publishes per-voxel
  `recon_gain`; an online layer would multiply this by a learned
  Gaussian uncertainty before CFPA2 ingests it.

So the data pipeline is ready; only the GPU optimiser node + a
launch arg + a training-loop body are missing.

---

## File layout reference

```
src/collaborative_exploration/reconstruction_awareness/
    reconstruction_awareness/
        accumulated_pointcloud_node.py     ✓ Stage 7
        reconstruction_quality_node.py     ✓ Stage 7
        scene_graph_builder_node.py        ✓ Stage 6
        keyframe_logger_node.py            ✓ Stage 9
        # online_3dgs_planning_node.py     □ deferred (§10.3)

scripts/offline/
    train_3dgs.py                          ✓ Stage 9 (init scaffold)

docs/
    3DGS_INTEGRATION.md                    ← this file
```
