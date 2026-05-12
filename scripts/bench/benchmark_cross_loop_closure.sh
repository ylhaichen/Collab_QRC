#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
DURATION_SEC="${DURATION_SEC:-180}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-180}"
RUN_FINAL_EVAL="${RUN_FINAL_EVAL:-false}"
TRIALS="${TRIALS:-3}"
PROFILE="${PROFILE:-robust}"
TRIAL_ID="${TRIAL_ID:-1}"
SCENE_NAME="${SCENE_NAME:-demo3_mixed.xml}"
SCENE_HAS_OVERLAP="${SCENE_HAS_OVERLAP:-true}"
OUT_DIR="${OUT_DIR:-${WS_DIR}/logs/cross_loop_closure_$(date +%Y%m%d_%H%M%S)}"

safe_source() { set +u; source "$1"; set -u; }

profile_args() {
  case "$1" in
    normal)
      printf '%s\n' \
        "team_alignment_min_matches:=2" \
        "robust_min_inliers:=4" \
        "robust_min_inlier_ratio:=0.25" \
        "robust_max_median_rmse:=0.50" \
        "robust_max_translation_spread_m:=1.25" \
        "robust_max_yaw_spread_deg:=12.0" \
        "robust_prefilter_max_rmse:=0.50" \
        "robust_prefilter_min_inlier_ratio:=0.30" \
        "robust_prefilter_min_correspondences:=0" \
        "robust_prefilter_max_descriptor_distance:=0.50"
      ;;
    cautious)
      printf '%s\n' \
        "team_alignment_min_matches:=4" \
        "robust_min_inliers:=6" \
        "robust_min_inlier_ratio:=0.30" \
        "robust_max_median_rmse:=0.45" \
        "robust_max_translation_spread_m:=1.0" \
        "robust_max_yaw_spread_deg:=12.0" \
        "robust_prefilter_max_rmse:=0.45" \
        "robust_prefilter_min_inlier_ratio:=0.45" \
        "robust_prefilter_min_correspondences:=0" \
        "robust_prefilter_max_descriptor_distance:=0.45"
      ;;
    robust)
      printf '%s\n' \
        "team_alignment_min_matches:=7" \
        "robust_min_inliers:=7" \
        "robust_min_inlier_ratio:=0.25" \
        "robust_max_median_rmse:=0.45" \
        "robust_max_translation_spread_m:=1.5" \
        "robust_max_yaw_spread_deg:=18.0" \
        "robust_prefilter_max_rmse:=0.45" \
        "robust_prefilter_min_inlier_ratio:=0.35" \
        "robust_prefilter_min_correspondences:=0" \
        "robust_prefilter_max_descriptor_distance:=0.45"
      ;;
    *)
      echo "Unknown PROFILE='${1}' (expected normal|cautious|robust)" >&2
      return 2
      ;;
  esac
}

scene_has_overlap() {
  case "$1" in
    demo3_mixed.xml|demo3_dual.xml)
      echo "true"
      ;;
    *)
      echo "false"
      ;;
  esac
}

if [[ "${RUN_FINAL_EVAL}" == "true" ]]; then
  ROOT_OUT="${OUT_DIR}"
  mkdir -p "${ROOT_OUT}"
  SCENES_CSV="${SCENES:-demo3_mixed.xml,demo3_dual.xml,two_rooms_door_scene.xml,no_overlap_dual_scene.xml}"
  PROFILES_CSV="${PROFILES:-normal,cautious,robust}"
  IFS=',' read -r -a scenes <<< "${SCENES_CSV}"
  IFS=',' read -r -a profiles <<< "${PROFILES_CSV}"
  for scene in "${scenes[@]}"; do
    for profile in "${profiles[@]}"; do
      for trial in $(seq 1 "${TRIALS}"); do
        run_dir="${ROOT_OUT}/${scene%.xml}/${profile}/trial_${trial}"
        mkdir -p "${run_dir}"
        echo "[cross-loop] scene=${scene} profile=${profile} trial=${trial} out=${run_dir}"
        OUT_DIR="${run_dir}" \
        PROFILE="${profile}" \
        TRIAL_ID="${trial}" \
        SCENE_NAME="${scene}" \
        SCENE_HAS_OVERLAP="$(scene_has_overlap "${scene}")" \
        RUN_FINAL_EVAL=false \
        "$0" "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/${scene}"
      done
    done
  done
  safe_source "${ROS2_SETUP_BASH}"
  safe_source "${WS_DIR}/install/setup.bash"
  python3 "${WS_DIR}/scripts/bench/cross_loop_closure_reporter.py" \
    --log-dir "${ROOT_OUT}" \
    --aggregate-root
  mkdir -p "${WS_DIR}/logs"
  cp "${ROOT_OUT}/cross_loop_closure_final_eval.json" "${WS_DIR}/logs/cross_loop_closure_final_eval.json"
  cp "${ROOT_OUT}/cross_loop_closure_final_eval.md" "${WS_DIR}/logs/cross_loop_closure_final_eval.md"
  latest_metrics="$(find "${ROOT_OUT}" -path '*/artifacts/team_pose_graph_metrics.json' -type f | sort | tail -n 1 || true)"
  if [[ -n "${latest_metrics}" ]]; then
    cp "${latest_metrics}" "${WS_DIR}/logs/team_pose_graph_metrics.json"
  fi
  echo "cross_loop_closure_final_eval.json: ${ROOT_OUT}/cross_loop_closure_final_eval.json"
  echo "cross_loop_closure_final_eval.md  : ${ROOT_OUT}/cross_loop_closure_final_eval.md"
  exit 0
fi

mkdir -p "${OUT_DIR}"
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

pids=()
launch_pids=()
have_setsid() {
  command -v setsid >/dev/null 2>&1
}
process_group_alive() {
  local pid="$1"
  if have_setsid; then
    kill -0 "-${pid}" >/dev/null 2>&1
  else
    kill -0 "${pid}" >/dev/null 2>&1
  fi
}
kill_process_group() {
  local signal="$1"
  local pid="$2"
  if have_setsid; then
    kill "-${signal}" "-${pid}" >/dev/null 2>&1 || true
  else
    kill "-${signal}" "${pid}" >/dev/null 2>&1 || true
    pkill "-${signal}" -P "${pid}" >/dev/null 2>&1 || true
  fi
}
cleanup() {
  for pid in "${launch_pids[@]:-}"; do
    if process_group_alive "${pid}"; then
      kill_process_group TERM "${pid}"
    fi
  done
  for pid in "${pids[@]:-}"; do
    if process_group_alive "${pid}"; then
      kill_process_group TERM "${pid}"
    fi
  done
  sleep 2
  for pid in "${launch_pids[@]:-}"; do
    if process_group_alive "${pid}"; then
      kill_process_group KILL "${pid}"
    fi
  done
  for pid in "${pids[@]:-}"; do
    if process_group_alive "${pid}"; then
      kill_process_group KILL "${pid}"
    fi
  done
  for pid in "${launch_pids[@]:-}" "${pids[@]:-}"; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

start_recorder() {
  local output="$1"
  local script="$2"
  shift 2
  if have_setsid; then
    setsid bash -c "${script}" bash "$@" > "${output}" 2>"${output}.err" &
  else
    bash -c "${script}" bash "$@" > "${output}" 2>"${output}.err" &
  fi
  pids+=("$!")
}

record_string_topic() {
  local topic="$1"
  local output="$2"
  start_recorder "${output}" '
    set -euo pipefail
    exec python3 "$1" --topic "$2" --field data --wait-sec "$3"
  ' "${WS_DIR}/scripts/bench/record_topic_field.py" "${topic}" "${TOPIC_WAIT_SEC}"
}

record_topic_field() {
  local topic="$1"
  local field="$2"
  local output="$3"
  start_recorder "${output}" '
    set -euo pipefail
    exec python3 "$1" --topic "$2" --field "$3" --wait-sec "$4"
  ' "${WS_DIR}/scripts/bench/record_topic_field.py" "${topic}" "${field}" "${TOPIC_WAIT_SEC}"
}

mapfile -t profile_launch_args < <(profile_args "${PROFILE}")
scene_overlap_arg=()
if [[ "${SCENE_HAS_OVERLAP}" == "true" ]]; then
  scene_overlap_arg=(--scene-has-overlap)
fi

launch_cmd=(
  ./scripts/launch/nav_test_demo3_mixed.sh
  gui:=false
  rviz:=false
  loop_closure:=true
  loop_closure_backend:=ros1_bridge
  relative_pose_source:=discovered
  inter_robot_loop_closure:=true
  mujoco_cameras:=false
  enable_gt_drift_metrics:=false
  loop_risk_output_dir:="${OUT_DIR}/artifacts"
  "${profile_launch_args[@]}"
  "$@"
)
if have_setsid; then
  setsid "${launch_cmd[@]}" > "${OUT_DIR}/launch.log" 2>&1 &
else
  "${launch_cmd[@]}" > "${OUT_DIR}/launch.log" 2>&1 &
fi
launch_pid="$!"
launch_pids+=("${launch_pid}")

sleep 8
record_string_topic /team_slam/keyframes "${OUT_DIR}/keyframes.jsonl"
record_string_topic /team_slam/cross_robot_candidates "${OUT_DIR}/cross_robot_candidates.jsonl"
record_string_topic /team_slam/cross_robot_matches "${OUT_DIR}/cross_robot_matches.jsonl"
record_string_topic /team_slam/robust_loop_inliers "${OUT_DIR}/robust_loop_inliers.jsonl"
record_string_topic /team_slam/pose_graph_metrics "${OUT_DIR}/pose_graph_metrics.jsonl"
record_string_topic /team_slam/team_pose_graph_factors "${OUT_DIR}/team_pose_graph_factors.jsonl"
record_string_topic /team_slam/alignment_status "${OUT_DIR}/alignment_status.jsonl"
record_string_topic /team_slam/dynamic_filter_metrics "${OUT_DIR}/dynamic_filter_metrics.jsonl"
record_string_topic /team_slam/peer/status "${OUT_DIR}/peer_status.jsonl"
record_string_topic /team_slam/peer/envelopes "${OUT_DIR}/peer_envelopes.jsonl"
record_string_topic /cfpa2/loop_candidates "${OUT_DIR}/loop_candidates.jsonl"
record_topic_field /merged_map header.stamp.sec "${OUT_DIR}/merged_map_stamps.txt"

sleep "${DURATION_SEC}"
cleanup

python3 "${WS_DIR}/scripts/bench/cross_loop_closure_reporter.py" \
  --log-dir "${OUT_DIR}" \
  --relative-pose-source discovered \
  --scene-name "${SCENE_NAME}" \
  --trial-id "${TRIAL_ID}" \
  --profile "${PROFILE}" \
  "${scene_overlap_arg[@]}"

mkdir -p "${WS_DIR}/logs"
cp "${OUT_DIR}/cross_loop_closure_summary.json" "${WS_DIR}/logs/cross_loop_closure_summary.json"
cp "${OUT_DIR}/cross_loop_closure_summary.md" "${WS_DIR}/logs/cross_loop_closure_summary.md"
cp "${OUT_DIR}/cross_loop_closure_final_eval.json" "${WS_DIR}/logs/cross_loop_closure_final_eval.json"
cp "${OUT_DIR}/cross_loop_closure_final_eval.md" "${WS_DIR}/logs/cross_loop_closure_final_eval.md"
if [[ -f "${OUT_DIR}/artifacts/team_pose_graph_metrics.json" ]]; then
  cp "${OUT_DIR}/artifacts/team_pose_graph_metrics.json" "${WS_DIR}/logs/team_pose_graph_metrics.json"
fi

echo "cross_loop_closure_summary.json   : ${OUT_DIR}/cross_loop_closure_summary.json"
echo "cross_loop_closure_final_eval.json: ${OUT_DIR}/cross_loop_closure_final_eval.json"
echo "latest final eval copy            : ${WS_DIR}/logs/cross_loop_closure_final_eval.json"
