#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
PREALIGN_SCRIPTED_OVERLAP_DEMO="${PREALIGN_SCRIPTED_OVERLAP_DEMO:-false}"
export PREALIGN_SCRIPTED_OVERLAP_DEMO

cat <<'EOF'
Point-LIO + DiSCo-style occupancy-map visualization

This wrapper runs the discovered-pose visual demo with:
  /robot_a/local_occupancy_grid
  /robot_b/local_occupancy_grid
  /team_slam/merged_occupancy_grid

The merged grid is safety-gated by /team_slam/alignment_status and
/team_slam/relative_transform. No GT runtime alignment is enabled.

Checks:
  ros2 topic echo --once /team_slam/alignment_status
  ros2 topic echo --once /robot_a/prealignment_exploration_status
  ros2 topic echo --once /robot_b/prealignment_exploration_status
  ros2 topic hz /robot_a/local_occupancy_grid
  ros2 topic hz /robot_b/local_occupancy_grid
  ros2 topic hz /team_slam/merged_occupancy_grid
  ros2 topic list | grep merged

EOF
printf 'prealign_scripted_overlap_demo:=%s\n' "${PREALIGN_SCRIPTED_OVERLAP_DEMO}"
printf 'Enable demo primitives with:\n  PREALIGN_SCRIPTED_OVERLAP_DEMO=true bash scripts/manual/run_visualized_pointlio_disco_maps.sh\n\n'

exec bash "${WS_DIR}/scripts/manual/run_visualized_pointlio_disco_demo.sh"
