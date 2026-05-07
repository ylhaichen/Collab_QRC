#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push
manual_run_logged real_hybrid_host_preflight bash "${ROOT}/scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh" --host
if [[ "${CONFIRM_REAL_ROBOT:-0}" != "1" ]]; then
  echo "ERROR: refusing to launch real robot primary validation without CONFIRM_REAL_ROBOT=1." >&2
  echo "Primary mode is not valid until shadow, Nav2, dynamic object, ERASOR, overlap/no-overlap, and agreement gate validations pass." >&2
  exit 3
fi
manual_run_logged real_robot_primary_runtime timeout "${REAL_RUNTIME_TIMEOUT_SEC:-180s}" "${ROOT}/scripts/real/real_autonomy.sh" deployment_mode=real_hybrid_ros1_slam_ros2_nav slam_backend=swarm_lio2_primary hybrid_preflight=false
manual_run_logged real_robot_primary_summary bash "${ROOT}/scripts/bench/run_real_hybrid_ros1_slam_ros2_nav_validation.sh"
echo "Real robot primary validation command sequence completed. Inspect logs before any replacement claim."
