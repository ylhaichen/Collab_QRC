#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push
manual_run_logged real_hybrid_host_preflight bash "${ROOT}/scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh" --host
if [[ "${CONFIRM_REAL_ROBOT:-0}" != "1" ]]; then
  echo "ERROR: refusing to launch real robot shadow validation without CONFIRM_REAL_ROBOT=1." >&2
  echo "Set CONFIRM_REAL_ROBOT=1 only on the robot/Jetson or validated field computer." >&2
  exit 3
fi
manual_run_logged real_robot_shadow_runtime timeout "${REAL_RUNTIME_TIMEOUT_SEC:-180s}" "${ROOT}/scripts/real/real_autonomy.sh" deployment_mode=real_hybrid_ros1_slam_ros2_nav slam_backend=swarm_lio2_shadow hybrid_preflight=false
manual_run_logged real_robot_shadow_summary bash "${ROOT}/scripts/bench/run_real_hybrid_ros1_slam_ros2_nav_validation.sh"
echo "Real robot shadow validation command sequence completed. Inspect logs before any pass claim."
