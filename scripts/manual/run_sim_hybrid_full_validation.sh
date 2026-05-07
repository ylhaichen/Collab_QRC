#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push
manual_run_logged sim_hybrid_swarm_check bash "${ROOT}/scripts/setup/check_swarm_lio2.sh" --docker
manual_run_logged sim_hybrid_dynamic_check bash "${ROOT}/scripts/setup/check_dynamic_lio.sh" --docker
manual_run_logged sim_hybrid_erasor_check bash "${ROOT}/scripts/setup/check_erasor.sh" --docker
manual_run_logged sim_hybrid_docker_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" build ros1_hybrid_slam
manual_run_logged sim_hybrid_catkin_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" run --rm ros1_hybrid_slam build
manual_run_logged sim_hybrid_static_validation bash "${ROOT}/scripts/bench/run_sim_hybrid_ros1_slam_ros2_nav_validation.sh"
manual_run_logged sim_hybrid_comparison python3 "${ROOT}/scripts/bench/hybrid_slam_validation.py" comparison
echo "Sim hybrid validation scripts completed. Inspect logs before making any replacement claim."
