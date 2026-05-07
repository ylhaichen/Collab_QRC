#!/usr/bin/env bash
# Launch heterogeneous dual-robot nav test on demo3_mixed.
#   robot_a = Go2W (wheeled-legged, at (4, 2))
#   robot_b = Go2  (non-wheeled,    at (4, -6))
# Fast-LIO2 SLAM per robot. CFPA2 coordinator partitions frontiers.
# Inter-robot collision monitor reports any A↔B contacts.
#
# Defaults (post 2026-05-02): both robots on Nav2 MPPI + SE2-holonomic
# overlay (SmacPlannerLattice w/ diff primitives + forward/pivot DiffDrive
# MPPI, no lateral strafe). Mirrors the real-Go2W profile and fits both
# Go2W and Go2 walking kinematics best.
#
# Usage:
#   ./scripts/launch/nav_test_demo3_mixed.sh                        # both = nav2_mppi + se2_holonomic
#   ./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
#   ./scripts/launch/nav_test_demo3_mixed.sh explore:=false         # manual goals only
#
# Go2 (robot_b) SE2-holonomic, isolated:
#   # Go2 alone on SE2 lattice; Go2W on baseline diff-drive Hybrid for A/B comparison
#   ./scripts/launch/nav_test_demo3_mixed.sh \
#       holonomic_profile_a:=off holonomic_profile_b:=se2_holonomic
#   # or with Go2W disabled / running other backend:
#   ./scripts/launch/nav_test_demo3_mixed.sh \
#       nav_backend_a:=astar nav_backend_b:=nav2_mppi \
#       holonomic_profile_b:=se2_holonomic
#
# Legacy backends:
#   ./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far    # both FAR
#   ./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=astar nav_backend_b:=astar # both A*
#   ./scripts/launch/nav_test_demo3_mixed.sh nav_backend_b:=far                       # mixed
#
# Opt out of SE2 (back to SmacPlannerHybrid + diff MPPI):
#   ./scripts/launch/nav_test_demo3_mixed.sh holonomic_profile_a:=off holonomic_profile_b:=off
#   ./scripts/launch/nav_test_demo3_mixed.sh debug:=true            # nav-only diagnostic terminal:
#       silences mujoco / fast_lio / octomap / champ / ekf / sensor
#       bridges / terrain_analysis / rviz / map_merge / session_reporter
#       / RSP / spawners. Keeps on stdout the planner+controller pipeline
#       and the safety monitor:
#           astar_nav_node (INFO, throttled), twist_bridge (quiet),
#           go2w_hybrid_cmd_router (quiet), far_status_adapter (INFO),
#           far_planner / localPlanner / pathFollower (downgraded to WARN
#               in debug mode — kills 5-10 Hz per-cycle INFO floods),
#           cfpa2_coordinator (downgraded to WARN — silent unless
#               allocation actually fails),
#           dual_robot_collision_monitor (INFO).
#
#       Output budget the agent sees (idle steady state, both robots
#       navigating cleanly):
#           ~12 lines/min from monitor periodic summary (1 line/robot
#               every 10 s — pose, yaw, v, ω, nav_state, goal, d2g,
#               walls, tilt+peak, tip/stuck flags, coverage % vs scene),
#           ~5 lines/min from astar_nav throttled INFO,
#           ~3 lines/min for new goal allocations,
#         = ~20 lines/min nominal; verbose nodes are quiet unless they
#           hit a real warning.
#
#       On every WALL CONTACT / TIP-OVER / PLANNER STUCK event the
#       monitor emits a single banner line + a 6-line context block
#       (header + 5 evenly-spaced samples of the last 2 s of pose/yaw/
#       tilt/v/ω/d2g/nav_state) — enough for an LLM agent to reconstruct
#       what the planner+controller were doing at the moment of failure.
#
#       Silenced nodes still log to ~/.ros/log/<session>/<name>*.log
#       if you need them — `tail -f ~/.ros/log/latest/<node>*.log`.
#
# Collision report written to /tmp/dual_robot_collision_report.json on exit.
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

prepend_path() {
  local var_name="$1"
  local value="$2"
  if [[ -d "${value}" ]]; then
    if [[ -n "${!var_name:-}" ]]; then
      export "${var_name}=${value}:${!var_name}"
    else
      export "${var_name}=${value}"
    fi
  fi
}

if ! /usr/bin/python3 -c 'import xacro' >/dev/null 2>&1; then
  cat >&2 <<'EOF'
ERROR: Python module 'xacro' is not available in the current ROS 2 environment.

Ubuntu 22.04 / ROS 2 Humble fix:
  sudo apt-get update
  sudo apt-get install -y ros-humble-xacro

Then retry this launch command.
EOF
  exit 127
fi

if ! /usr/bin/python3 -c 'import nav2_common' >/dev/null 2>&1; then
  cat >&2 <<'EOF'
ERROR: Python module 'nav2_common' is not available in the current ROS 2 environment.

Ubuntu 22.04 / ROS 2 Humble fix:
  sudo apt-get update
  sudo apt-get install -y ros-humble-navigation2

Then retry this launch command.
EOF
  exit 127
fi

LOCAL_ROBOT_LOCALIZATION_PREFIX="${LOCAL_ROBOT_LOCALIZATION_PREFIX:-/tmp/collab_qrc_ros_overlay_robot_localization/opt/ros/humble}"
LOCAL_ROBOT_LOCALIZATION_ROOT="${LOCAL_ROBOT_LOCALIZATION_PREFIX%/opt/ros/humble}"
USING_LOCAL_ROBOT_LOCALIZATION=false
if ! ros2 pkg prefix robot_localization >/dev/null 2>&1 && [[ -d "${LOCAL_ROBOT_LOCALIZATION_PREFIX}/share/robot_localization" ]]; then
  prepend_path AMENT_PREFIX_PATH "${LOCAL_ROBOT_LOCALIZATION_PREFIX}"
  prepend_path CMAKE_PREFIX_PATH "${LOCAL_ROBOT_LOCALIZATION_PREFIX}"
  prepend_path LD_LIBRARY_PATH "${LOCAL_ROBOT_LOCALIZATION_PREFIX}/lib"
  prepend_path LD_LIBRARY_PATH "${LOCAL_ROBOT_LOCALIZATION_ROOT}/usr/lib/x86_64-linux-gnu"
  prepend_path PYTHONPATH "${LOCAL_ROBOT_LOCALIZATION_PREFIX}/local/lib/python3.10/dist-packages"
  USING_LOCAL_ROBOT_LOCALIZATION=true
fi

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  cat >&2 <<'EOF'
ERROR: ROS 2 package 'robot_localization' is not available in the current ROS 2 environment.

Ubuntu 22.04 / ROS 2 Humble fix:
  sudo apt-get update
  sudo apt-get install -y ros-humble-robot-localization

Non-root temporary fallback:
  apt-get download ros-humble-robot-localization
  apt-get download libgeographic19
  mkdir -p /tmp/collab_qrc_ros_overlay_robot_localization
  dpkg-deb -x ros-humble-robot-localization_*_amd64.deb /tmp/collab_qrc_ros_overlay_robot_localization
  dpkg-deb -x libgeographic19_*_amd64.deb /tmp/collab_qrc_ros_overlay_robot_localization

Then retry this launch command.
EOF
  exit 127
fi

if [[ "${USING_LOCAL_ROBOT_LOCALIZATION}" == "true" ]] \
  && ! ldconfig -p 2>/dev/null | grep -q 'libGeographic\.so\.19' \
  && [[ ! -e "${LOCAL_ROBOT_LOCALIZATION_ROOT}/usr/lib/x86_64-linux-gnu/libGeographic.so.19" ]]; then
  cat >&2 <<'EOF'
ERROR: local robot_localization overlay is present, but libGeographic.so.19 is missing.

Ubuntu 22.04 system fix:
  sudo apt-get install -y libgeographic19

Non-root temporary fallback:
  cd /tmp
  apt-get download libgeographic19
  dpkg-deb -x libgeographic19_*_amd64.deb /tmp/collab_qrc_ros_overlay_robot_localization

Then retry this launch command.
EOF
  exit 127
fi

LOCAL_GTSAM_PREFIX="${LOCAL_GTSAM_PREFIX:-${WS_DIR}/.local_deps/gtsam_humble/extract/opt/ros/humble}"
if [[ -f "${LOCAL_GTSAM_PREFIX}/include/gtsam/slam/BetweenFactor.h" ]]; then
  prepend_path CMAKE_PREFIX_PATH "${LOCAL_GTSAM_PREFIX}"
  prepend_path LD_LIBRARY_PATH "${LOCAL_GTSAM_PREFIX}/lib/x86_64-linux-gnu"
  prepend_path LD_LIBRARY_PATH "${LOCAL_GTSAM_PREFIX}/lib"
fi

SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}/share/sc_pgo" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${SC_PGO_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export PATH="${SC_PGO_PREFIX}/bin:${PATH}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# Defaults are inherited from nav_test_mujoco_fastlio_mixed.launch.py:
#   nav_backend_a / nav_backend_b   → nav2_mppi
#   holonomic_profile_a / _b        → se2_holonomic
# (the SE2 lattice + forward/pivot MPPI profile fits both Go2W and Go2
#  walking kinematics best). To revert per-robot:
#   nav_backend_a:=astar / =far                 # legacy A* / CMU FAR
#   holonomic_profile_a:=off                    # SmacPlannerHybrid + diff-MPPI baseline
exec ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=true \
  rviz:=true \
  "$@"
