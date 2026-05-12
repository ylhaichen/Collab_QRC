#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

HARDENING_TRIALS="${HARDENING_TRIALS:-3}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-180}"
OVERLAP_DURATION_SEC="${OVERLAP_DURATION_SEC:-180}"
NO_OVERLAP_DURATION_SEC="${NO_OVERLAP_DURATION_SEC:-90}"
HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC:-12}"
PROFILE="${PROFILE:-robust}"
TEAM_POSE_GRAPH_BACKEND="${TEAM_POSE_GRAPH_BACKEND:-gtsam_cpp}"
TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE="${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE:-false}"
USE_DYNAMIC_FILTER="${USE_DYNAMIC_FILTER:-true}"
LOCAL_SLAM_BACKEND="${LOCAL_SLAM_BACKEND:-point_lio}"
REGISTRATION_BACKEND="${REGISTRATION_BACKEND:-icp_2d}"
ROBUST_SELECTION_BACKEND="${ROBUST_SELECTION_BACKEND:-greedy_consistency_fallback}"
RUN_POINT_LIO_VALIDATIONS="${RUN_POINT_LIO_VALIDATIONS:-true}"
RUN_DYNAMIC_VALIDATION="${RUN_DYNAMIC_VALIDATION:-true}"
RUN_COMM_VALIDATION="${RUN_COMM_VALIDATION:-true}"
RUN_FAST_LIO_REGRESSION="${RUN_FAST_LIO_REGRESSION:-true}"
HARDENING_ROOT="${HARDENING_ROOT:-logs/simulation_hardening}"

rm -rf "${HARDENING_ROOT}"
mkdir -p "${HARDENING_ROOT}/overlap" "${HARDENING_ROOT}/no_overlap"

run_optional() {
  local name="$1"
  shift
  echo "[simulation-hardening] ${name}: $*"
  set +e
  "$@"
  local rc=$?
  set -e
  if [[ "${rc}" -ne 0 ]]; then
    echo "[simulation-hardening] ${name} exited rc=${rc}; continuing to aggregate exact blocker."
  fi
  return 0
}

if [[ "${RUN_POINT_LIO_VALIDATIONS}" == "true" ]]; then
  run_optional point_lio_shadow bash scripts/bench/run_point_lio_shadow_validation.sh
  run_optional point_lio_primary bash scripts/bench/run_point_lio_primary_validation.sh
fi

for trial in $(seq 1 "${HARDENING_TRIALS}"); do
  no_dir="${HARDENING_ROOT}/no_overlap/trial_${trial}"
  mkdir -p "${no_dir}"
  echo "[simulation-hardening] no-overlap trial ${trial}/${HARDENING_TRIALS}"
  set +e
  OUT_DIR="${no_dir}" \
  DURATION_SEC="${NO_OVERLAP_DURATION_SEC}" \
  TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC}" \
  HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC}" \
  PROFILE="${PROFILE}" \
  TRIAL_ID="${trial}" \
  SCENE_NAME=no_overlap_dual_scene.xml \
  SCENE_HAS_OVERLAP=false \
  scripts/bench/benchmark_cross_loop_closure.sh \
    "team_pose_graph_backend:=${TEAM_POSE_GRAPH_BACKEND}" \
    "team_alignment_allow_export_only_gate:=${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE}" \
    "local_slam_backend:=${LOCAL_SLAM_BACKEND}" \
    "registration_backend:=${REGISTRATION_BACKEND}" \
    "robust_selection_backend:=${ROBUST_SELECTION_BACKEND}" \
    "no_overlap_rejection_passed:=false" \
    "use_dynamic_filter:=${USE_DYNAMIC_FILTER}" \
    "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/no_overlap_dual_scene.xml"
  no_rc=$?
  set -e
  if [[ "${no_rc}" -ne 0 ]]; then
    echo "[simulation-hardening] no-overlap trial ${trial} exited rc=${no_rc}"
  fi

  no_overlap_rejection_passed="$(python3 - "${no_dir}/cross_loop_closure_summary.json" <<'PY'
from __future__ import annotations
import json
import sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.exists():
    print("false")
    raise SystemExit(0)
row = json.loads(path.read_text())
passed = (
    row.get("runtime_valid") is True
    and row.get("gt_used_runtime") is False
    and row.get("alignment_status") in {"rejected", "tentative"}
    and row.get("false_alignment") is False
    and row.get("merged_map_enabled_time_sec") is None
    and int(row.get("pose_graph_inter_robot_factors") or 0) == 0
)
print("true" if passed else "false")
PY
)"

  overlap_dir="${HARDENING_ROOT}/overlap/trial_${trial}"
  mkdir -p "${overlap_dir}"
  echo "[simulation-hardening] overlap trial ${trial}/${HARDENING_TRIALS} no_overlap_rejection_passed=${no_overlap_rejection_passed}"
  set +e
  OUT_DIR="${overlap_dir}" \
  DURATION_SEC="${OVERLAP_DURATION_SEC}" \
  TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC}" \
  HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC}" \
  PROFILE="${PROFILE}" \
  TRIAL_ID="${trial}" \
  SCENE_NAME=demo3_mixed.xml \
  SCENE_HAS_OVERLAP=true \
  scripts/bench/benchmark_cross_loop_closure.sh \
    "team_pose_graph_backend:=${TEAM_POSE_GRAPH_BACKEND}" \
    "team_alignment_allow_export_only_gate:=${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE}" \
    "local_slam_backend:=${LOCAL_SLAM_BACKEND}" \
    "registration_backend:=${REGISTRATION_BACKEND}" \
    "robust_selection_backend:=${ROBUST_SELECTION_BACKEND}" \
    "no_overlap_rejection_passed:=${no_overlap_rejection_passed}" \
    "use_dynamic_filter:=${USE_DYNAMIC_FILTER}" \
    "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml"
  overlap_rc=$?
  set -e
  if [[ "${overlap_rc}" -ne 0 ]]; then
    echo "[simulation-hardening] overlap trial ${trial} exited rc=${overlap_rc}"
  fi
done

if [[ "${RUN_DYNAMIC_VALIDATION}" == "true" ]]; then
  run_optional dynamic_filter bash scripts/bench/run_dynamic_filter_validation.sh
fi
if [[ "${RUN_COMM_VALIDATION}" == "true" ]]; then
  run_optional decentralized_comm bash scripts/bench/run_decentralized_comm_validation.sh
fi
if [[ "${RUN_FAST_LIO_REGRESSION}" == "true" ]]; then
  run_optional fast_lio_fallback bash scripts/bench/run_fast_lio_fallback_regression.sh
fi

python3 scripts/bench/simulation_hardening_reporter.py \
  --repo-root "${WS_DIR}" \
  --hardening-root "${HARDENING_ROOT}" \
  --required-trials "${HARDENING_TRIALS}"
