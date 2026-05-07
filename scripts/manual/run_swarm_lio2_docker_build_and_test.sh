#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push
manual_run_logged swarm_lio2_docker_precheck bash "${ROOT}/scripts/setup/check_swarm_lio2.sh" --docker
manual_run_logged ros1_hybrid_docker_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" build ros1_hybrid_slam
manual_run_logged swarm_lio2_catkin_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" run --rm ros1_hybrid_slam build
manual_run_logged_allow_codes swarm_lio2_launch_smoke "124" \
  timeout "${SWARM_LIO2_LAUNCH_SMOKE_TIMEOUT_SEC:-45s}" \
  docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" run --rm ros1_hybrid_slam run
manual_run_logged swarm_lio2_docker_postcheck bash "${ROOT}/scripts/setup/check_swarm_lio2.sh" --docker
echo "Swarm-LIO2 Docker build/test completed only if build commands exited 0 and launch smoke timed out while roslaunch stayed alive."
