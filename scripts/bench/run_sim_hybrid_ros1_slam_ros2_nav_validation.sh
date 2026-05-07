#!/usr/bin/env bash
set -euo pipefail
python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/scripts/bench/hybrid_slam_validation.py" sim
