#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WS_DIR}/logs/manual"
ARTIFACT_DIR="${LOG_DIR}/visualized_pointlio_disco_demo_artifacts_${STAMP}"
LOG_FILE="${LOG_DIR}/visualized_pointlio_disco_demo_${STAMP}.log"
mkdir -p "${LOG_DIR}" "${ARTIFACT_DIR}" "${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
PREALIGN_SCRIPTED_OVERLAP_DEMO="${PREALIGN_SCRIPTED_OVERLAP_DEMO:-false}"
PREALIGN_ROBUST_ACCEPTANCE_MIN_INLIERS="${PREALIGN_ROBUST_ACCEPTANCE_MIN_INLIERS:-7}"

safe_source() { set +u; source "$1"; set -u; }
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

cat <<'EOF'
Point-LIO DiSCo-style visual simulation demo

Expected visible state:
- MuJoCo GUI opens with robot_a and robot_b in the overlap scene.
- RViz opens with robot poses, local goals, keyframe clouds, individual occupancy grids, and the gated merged occupancy grid after robust alignment.
- /merged_map must stay closed until /team_slam/alignment_status reports status=aligned.
- Runtime alignment uses discovered DiSCo-style loop closure only; GT runtime alignment is disabled.
- Pre-alignment exploration is local-only: no peer-frame goals, no GT, and no /merged_map dependency.
- Optional scripted overlap demo uses local-frame primitives only and still requires robust loop closure before merge.

Useful topic checks in another terminal:
  ros2 topic hz /robot_a/Odometry
  ros2 topic hz /robot_b/Odometry
  ros2 topic echo --once /robot_a/way_point_coord
  ros2 topic echo --once /robot_b/way_point_coord
  ros2 topic echo --once /team_slam/alignment_status
  ros2 topic hz /team_slam/keyframes
  ros2 topic hz /team_slam/cross_robot_candidates
  ros2 topic hz /team_slam/robust_loop_inliers
  ros2 topic hz /robot_a/local_occupancy_grid
  ros2 topic hz /robot_b/local_occupancy_grid
  ros2 topic hz /team_slam/merged_occupancy_grid
  ros2 topic list | grep merged_map
  ros2 topic hz /robot_a/cloud_static
  ros2 topic hz /robot_a/cloud_dynamic
  ros2 topic echo --once /robot_a/prealignment_exploration_status
  ros2 topic echo --once /robot_b/prealignment_exploration_status

Success looks like:
- /robot_a/Odometry and /robot_b/Odometry are nonzero-rate.
- /team_slam/keyframes grows for both robots.
- /team_slam/robust_loop_inliers reaches the configured threshold.
- /team_slam/alignment_status becomes aligned.
- /merged_map appears only after robust evidence and accepted pose-graph factors.
- /team_slam/merged_occupancy_grid appears only after robust alignment; /robot_a/local_occupancy_grid and /robot_b/local_occupancy_grid publish before alignment.
- /robot_a/prealignment_exploration_status and /robot_b/prealignment_exploration_status show distance_from_start growth beyond the configured minimum.
- CFPA2 logs ASSIGN [cfpa2] repeatedly and /robot_a/way_point_coord plus /robot_b/way_point_coord move away from the start area.
- If /merged_map is occupied-heavy, CFPA2 logs a shared-map quality-gate warning and falls back to per-robot maps for frontier extraction.

Failure looks like:
- Point-LIO odometry stays silent.
- alignment_status stays unknown/rejected in the overlap scene.
- /merged_map appears before robust inliers or in no-overlap mode.
- CFPA2 repeatedly logs NO_GOAL [no_frontiers_after_extract] without later ASSIGN [cfpa2].
- Raw LiDAR, dense maps, or full costmaps are continuously exchanged as peer traffic.

Press Ctrl-C to stop. Log file:
EOF
printf '  %s\n\n' "${LOG_FILE}"
printf 'Prealignment scripted overlap demo: %s\n' "${PREALIGN_SCRIPTED_OVERLAP_DEMO}"
printf 'Robust inlier acceptance target for exploration policy: %s\n\n' "${PREALIGN_ROBUST_ACCEPTANCE_MIN_INLIERS}"

launch_cmd=(
  ./scripts/launch/nav_test_demo3_mixed.sh
  gui:=true
  rviz:=true
  loop_closure:=true
  loop_closure_backend:=ros1_bridge
  relative_pose_source:=discovered
  inter_robot_loop_closure:=true
  local_slam_backend:=point_lio
  registration_backend:=icp_2d
  team_pose_graph_backend:=gtsam_cpp
  team_alignment_allow_export_only_gate:=false
  no_overlap_rejection_passed:=true
  use_dynamic_filter:=true
  map_merge:=true
  prealignment_exploration_enabled:=true
  occupancy_grid_visualization_enabled:=true
  prealign_min_goal_distance:=2.0
  prealign_min_start_displacement:=3.0
  prealign_dwell_timeout_sec:=20.0
  prealign_stuck_replan_limit:=3
  prealign_goal_blacklist_radius:=1.0
  prealign_overlap_timeout_sec:=60.0
  prealign_min_keyframes_before_alignment:=5
  prealign_exploration_radius_growth:=1.5
  prealign_far_frontier_bonus:=1.0
  prealign_corridor_frontier_bonus:=0.5
  prealign_keyframe_gain_bonus:=0.5
  prealign_goal_hold_sec:=5.0
  prealign_robust_acceptance_min_inliers:="${PREALIGN_ROBUST_ACCEPTANCE_MIN_INLIERS}"
  prealign_scripted_overlap_demo:="${PREALIGN_SCRIPTED_OVERLAP_DEMO}"
  mujoco_cameras:=false
  enable_gt_drift_metrics:=false
  loop_risk_output_dir:="${ARTIFACT_DIR}"
  "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml"
)

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -TERM "-${LAUNCH_PID}" >/dev/null 2>&1 || kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" >/dev/null 2>&1 || kill -KILL "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  docker rm -f collab_qrc_point_lio_robot_a collab_qrc_point_lio_robot_b >/dev/null 2>&1 || true
}
trap cleanup INT TERM EXIT

setsid "${launch_cmd[@]}" > >(tee -a "${LOG_FILE}") 2>&1 &
LAUNCH_PID="$!"
wait "${LAUNCH_PID}"
