#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push
manual_run_logged erasor_docker_precheck bash "${ROOT}/scripts/setup/check_erasor.sh" --docker
manual_run_logged ros1_hybrid_docker_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" build ros1_hybrid_slam
manual_run_logged erasor_catkin_build docker compose -f "${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml" run --rm ros1_hybrid_slam build
manual_run_logged erasor_docker_postcheck bash "${ROOT}/scripts/setup/check_erasor.sh" --docker
echo "ERASOR Docker build/test completed only if all commands above exited 0."
