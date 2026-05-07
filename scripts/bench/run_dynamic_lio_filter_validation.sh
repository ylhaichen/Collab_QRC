#!/usr/bin/env bash
set -euo pipefail
DEPLOYMENT_MODE="${DEPLOYMENT_MODE:-sim_hybrid_ros1_slam_ros2_nav}"
python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/scripts/bench/hybrid_slam_validation.py" dynamic --deployment-mode "${DEPLOYMENT_MODE}"
