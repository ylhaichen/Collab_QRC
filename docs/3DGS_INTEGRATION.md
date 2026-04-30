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

1. Install gsplat: `pip install gsplat` (requires PyTorch + CUDA).
2. Capture keyframes during a trial by enabling
   `keyframe_logger_enabled:=true` in the launch (TODO: launch arg
   not yet wired; spawn the node manually for now).
3. Confirm `keyframes/` has ≥ 8 well-separated frames.
4. Implement the gsplat optimisation loop inside
   `_maybe_train_gsplat()` in `train_3dgs.py`:
   - Load init PLY → `Splat3D` model
   - Render each keyframe pose → image, compare to ground truth
   - Standard 3DGS loss (L1 + D-SSIM) + densification schedule
   - Save trained model to `3dgs_optimized.ply`
5. Add `gsplat_metrics` field to `mission_summary.json` claims so the
   report cites PSNR / SSIM against held-out keyframes.

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
