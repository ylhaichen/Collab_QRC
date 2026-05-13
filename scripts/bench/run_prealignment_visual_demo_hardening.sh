#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
DURATION_SEC="${DURATION_SEC:-90}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${WS_DIR}/logs/prealignment_visual_demo_runtime_${STAMP}"
mkdir -p "${OUT_DIR}"

safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

timeout "$((DURATION_SEC + 90))s" ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=false \
  rviz:=false \
  cleanup_stale:=false \
  loop_closure:=true \
  loop_closure_backend:=ros1_bridge \
  relative_pose_source:=discovered \
  inter_robot_loop_closure:=true \
  local_slam_backend:=point_lio \
  registration_backend:=icp_2d \
  team_pose_graph_backend:=gtsam_cpp \
  team_alignment_allow_export_only_gate:=false \
  no_overlap_rejection_passed:=true \
  use_dynamic_filter:=true \
  map_merge:=true \
  prealignment_exploration_enabled:=true \
  occupancy_grid_visualization_enabled:=true \
  frontier_trust_enabled:=false \
  prealign_min_goal_distance:=2.0 \
  prealign_min_start_displacement:=3.0 \
  prealign_dwell_timeout_sec:=20.0 \
  prealign_stuck_replan_limit:=3 \
  prealign_goal_blacklist_radius:=1.0 \
  prealign_overlap_timeout_sec:=60.0 \
  prealign_min_keyframes_before_alignment:=5 \
  prealign_exploration_radius_growth:=1.5 \
  prealign_far_frontier_bonus:=1.0 \
  prealign_corridor_frontier_bonus:=0.5 \
  prealign_keyframe_gain_bonus:=0.5 \
  prealign_goal_hold_sec:=5.0 \
  mujoco_cameras:=false \
  enable_gt_drift_metrics:=false \
  session_duration_sec:="${DURATION_SEC}" \
  session_output_dir:="${OUT_DIR}" \
  loop_risk_output_dir:="${OUT_DIR}/loop_risk" \
  "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml" \
  2>&1 | tee "${OUT_DIR}/launch.log"

printf 'Prealignment hardening artifacts: %s\n' "${OUT_DIR}"
