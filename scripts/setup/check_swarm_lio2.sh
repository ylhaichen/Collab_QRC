#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC_DIR="${SWARM_LIO2_SOURCE_DIR:-${WS_DIR}/external/Swarm-LIO2}"

json_escape() {
  printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'
}

available=false
buildable=false
runtime_ready=false
blocker=""

if [[ ! -d "${SRC_DIR}/.git" && ! -d "${SRC_DIR}" ]]; then
  blocker="Swarm-LIO2 source not found at ${SRC_DIR}; run scripts/setup/fetch_slam_backends.sh or clone https://github.com/hku-mars/Swarm-LIO2"
else
  available=true
  if find "${SRC_DIR}" -maxdepth 3 \( -name package.xml -o -name CMakeLists.txt \) | grep -q .; then
    if command -v catkin_make >/dev/null 2>&1 && command -v rospack >/dev/null 2>&1; then
      buildable=true
    else
      blocker="Swarm-LIO2 source exists, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack"
    fi
  else
    blocker="Swarm-LIO2 source exists, but no package.xml/CMakeLists.txt was found within depth 3"
  fi
  if [[ -d "${WS_DIR}/install/swarm_lio" || -d "${WS_DIR}/install/swarm_msgs" ||
        -d "${SRC_DIR}/devel" || -d "${SRC_DIR}/install" ||
        -x "${SRC_DIR}/devel/lib/swarm_lio/swarm_lio_node" ]]; then
    runtime_ready=true
  elif [[ -z "${blocker}" ]]; then
    blocker="Swarm-LIO2 source is present and appears buildable, but no installed runtime artifact was found under install/"
  fi
fi

cat <<EOF
{
  "backend": "Swarm-LIO2",
  "source_dir": "$(json_escape "${SRC_DIR}")",
  "available": ${available},
  "buildable": ${buildable},
  "runtime_ready": ${runtime_ready},
  "blocker": "$(json_escape "${blocker}")"
}
EOF
