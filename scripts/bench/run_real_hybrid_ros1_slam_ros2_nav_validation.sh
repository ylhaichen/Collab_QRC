#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bash "${ROOT}/scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh" || true
python3 "${ROOT}/scripts/bench/hybrid_slam_validation.py" real
python3 "${ROOT}/scripts/bench/hybrid_slam_validation.py" comparison >/dev/null
