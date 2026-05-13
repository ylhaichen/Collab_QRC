#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
PREALIGN_SCRIPTED_OVERLAP_DEMO="${PREALIGN_SCRIPTED_OVERLAP_DEMO:-false}"
OCCUPANCY_RVIZ_VIEW="${OCCUPANCY_RVIZ_VIEW:-robot_a}"
RVIZ_LAYOUT="${RVIZ_LAYOUT:-single}"
export PREALIGN_SCRIPTED_OVERLAP_DEMO
export OCCUPANCY_RVIZ_VIEW
export RVIZ_LAYOUT

cat <<'EOF'
Point-LIO + DiSCo-style occupancy-map visualization

This wrapper runs the discovered-pose visual demo with:
  /robot_a/local_occupancy_grid
  /robot_b/local_occupancy_grid
  /team_slam/merged_occupancy_grid

The merged grid is safety-gated by /team_slam/alignment_status and
/team_slam/relative_transform. No GT runtime alignment is enabled.

Pre-alignment RViz frame note:
  robot_a/map and robot_b/map are intentionally separate until robust alignment.
  Use OCCUPANCY_RVIZ_VIEW=robot_b to open the robot_b local-map RViz config.
  Use RVIZ_LAYOUT=multi to open robot_a local, robot_b local, and team-map windows.

Checks:
  ros2 topic echo --once /team_slam/alignment_status
  ros2 topic echo --once /robot_a/prealignment_exploration_status
  ros2 topic echo --once /robot_b/prealignment_exploration_status
  ros2 topic echo --once --full-length /robot_b/nav_start_cell_diagnostics
  ros2 topic hz /robot_a/local_occupancy_grid
  ros2 topic hz /robot_b/local_occupancy_grid
  ros2 topic hz /team_slam/merged_occupancy_grid
  ros2 topic echo --once --full-length /team_slam/merged_occupancy_grid_status
  ros2 topic list | grep merged

EOF
printf 'prealign_scripted_overlap_demo:=%s\n' "${PREALIGN_SCRIPTED_OVERLAP_DEMO}"
printf 'occupancy_rviz_view:=%s\n' "${OCCUPANCY_RVIZ_VIEW}"
printf 'rviz_layout:=%s\n' "${RVIZ_LAYOUT}"
printf 'Enable demo primitives with:\n  PREALIGN_SCRIPTED_OVERLAP_DEMO=true bash scripts/manual/run_visualized_pointlio_disco_maps.sh\n\n'
printf 'Open robot_b local map view with:\n  OCCUPANCY_RVIZ_VIEW=robot_b bash scripts/manual/run_visualized_pointlio_disco_maps.sh\n\n'
printf 'Open all map views with:\n  RVIZ_LAYOUT=multi bash scripts/manual/run_visualized_pointlio_disco_maps.sh\n\n'

exec bash "${WS_DIR}/scripts/manual/run_visualized_pointlio_disco_demo.sh"
