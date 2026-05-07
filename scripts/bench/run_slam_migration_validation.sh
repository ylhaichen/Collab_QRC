#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

bash "${ROOT}/scripts/bench/run_sim_hybrid_ros1_slam_ros2_nav_validation.sh" >/dev/null
bash "${ROOT}/scripts/bench/run_real_hybrid_ros1_slam_ros2_nav_validation.sh" >/dev/null || true
python3 "${ROOT}/scripts/bench/hybrid_slam_validation.py" comparison
