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

**End-to-end working** as of Stage 10b on this dev laptop (RTX 4070,
local CUDA 12.4 toolkit in `$HOME/cuda-12.4`). A 90 s
`loop_risk_recon_graph_mppi` trial captures 115 keyframes and
`scripts/offline/train_3dgs.py --train` runs gsplat optimisation
end-to-end:

```
[train_3dgs] training on 115 keyframes, GPU NVIDIA GeForce RTX 4070 Laptop GPU
gsplat: CUDA extension has been set up successfully in 45.72 seconds.
[train_3dgs] step    0/1000  loss=0.3168
[train_3dgs] step  200/1000  loss=0.2904
[train_3dgs] step  600/1000  loss=0.1326
[train_3dgs] step  800/1000  loss=0.1104
[train_3dgs] training done; final L1 loss = 0.1433
```

Outputs per trial: `3dgs_init.ply` (CPU init scaffold, ~0.13 s) +
`3dgs_optimized.ply` (gsplat-trained, ~30 s for 1k iterations after
first JIT compile). Both follow Inria layout (vertex props `x y z nx
ny nz f_dc_0 f_dc_1 f_dc_2 opacity scale_0 scale_1 scale_2 rot_0
rot_1 rot_2 rot_3`) and load directly in SuperSplat / WebGL viewers
/ gsplat consumers.

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

### How to run on a fresh machine

1. **Install gsplat**: `pip install --user gsplat` (1.5.3 ships
   wheels — no CUDA toolkit needed for `pip install`).
2. **Install a CUDA 12.x toolkit** with `sm_89` support
   (the apt-shipped 11.5 on Ubuntu 22.04 is too old for Ada GPUs).
   No-sudo runfile:
   ```bash
   cd /tmp
   wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda_12.4.1_550.54.15_linux.run
   sh cuda_12.4.1_550.54.15_linux.run --silent --toolkit \
      --installpath=$HOME/cuda-12.4 --override --no-opengl-libs
   ```
   `env.sh` then automatically activates `$HOME/cuda-12.4` on every
   `source env.sh` (sets `CUDA_HOME`, prepends `$CUDA_HOME/bin` to
   PATH, prepends `$CUDA_HOME/lib64` to LD_LIBRARY_PATH, and exports
   `TORCH_CUDA_ARCH_LIST=8.9` for RTX 4070 Ada).
3. **MuJoCo RGBD camera** is already on. The plugin auto-instantiates
   from `<camera>` tags in `demo3_mixed.xml` and publishes
   `/front_camera/color/image_raw` (robot_a) +
   `/b_front_camera/color/image_raw` (robot_b) at 25 Hz, encoding
   `8UC3`. `keyframe_logger_node` spawns automatically when
   `reconstruction_quality_enabled=true` and saves to
   `<trial>/keyframes/<ns>_NNNN.png` plus `cameras.json`. A 90 s
   `loop_risk_recon_graph_mppi` trial captures ~110 keyframes.
4. **Run the trial** (Stage 1-10 launch path, unchanged):
   ```bash
   NUM_TRIALS=1 DURATION_SEC=180 \
     OUT_DIR=results/3dgs_run/$(date +%Y%m%d_%H%M%S) \
     GUI=true RVIZ=true NAV_A=nav2_mppi NAV_B=nav2_mppi \
     ./scripts/bench/benchmark_loop_risk_allocator.sh loop_risk_recon_graph_mppi
   ```
5. **Train**:
   ```bash
   source env.sh   # activates CUDA_HOME if $HOME/cuda-12.4 exists
   TRIAL=$(ls -td results/3dgs_run/*/trial_1/ | head -1)
   python3 scripts/offline/train_3dgs.py "$TRIAL" --train --train-iters 3000
   ```
   First run takes ~45 s extra for gsplat JIT compile; subsequent
   runs reuse the cached `.so` and start in milliseconds.
6. **Optimisation loop body** (in `_maybe_train_gsplat()`):
   - Loads init PLY → trainable tensors (means / log_scales /
     quats=identity / opacities=logit(0.5) / sh0=zeros).
   - Per step: pick keyframe, build `viewmat = inv(world-from-cam)`
     from pose JSON, call `gsplat.rasterization(...)` with the
     gsplat-1.5+ shape conventions: colors `(C, N, 3)`, viewmats
     `(C, 4, 4)`, Ks `(C, 3, 3)`, sh_degree=None.
   - Loss = L1(rendered, target).
   - Optimiser = Adam(lr=1e-2).
   - Final `3dgs_optimized.ply` written in Inria layout.

### Stage-10b end-to-end run (this laptop, recorded)

```
keyframes captured  : 115           (robot_a 56 + robot_b 59)
init scaffold       : 16 662 Gaussians, 0.14 s
gsplat compile      : 45.7 s        (first call only, cached after)
training 1000 iter  : ~30 s
final L1 loss       : 0.1433        (down from 0.3168 at step 0)
3dgs_optimized.ply  : 1.8 MB        (Inria layout, viewable)
```

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
