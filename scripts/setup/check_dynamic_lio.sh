#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC_DIR="${DYNAMIC_LIO_SOURCE_DIR:-${WS_DIR}/external/dynamic_lio}"

json_escape() {
  printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'
}

available=false
buildable=false
runtime_ready=false
blocker=""

if [[ ! -d "${SRC_DIR}/.git" && ! -d "${SRC_DIR}" ]]; then
  blocker="Dynamic-LIO source not found at ${SRC_DIR}; run scripts/setup/fetch_slam_backends.sh or clone https://github.com/ZikangYuan/dynamic_lio"
else
  available=true
  if find "${SRC_DIR}" -maxdepth 3 \( -name package.xml -o -name CMakeLists.txt \) | grep -q .; then
    buildable=true
  else
    blocker="Dynamic-LIO source exists, but no package.xml/CMakeLists.txt was found within depth 3"
  fi
  if [[ -d "${WS_DIR}/install/dynamic_lio" ||
        -d "${SRC_DIR}/devel" || -d "${SRC_DIR}/install" ||
        -x "${SRC_DIR}/devel/lib/dynamic_lio/dynamic_lio_node" ]]; then
    runtime_ready=true
  elif [[ -z "${blocker}" ]]; then
    blocker="Dynamic-LIO source is present and appears buildable, but no installed filtering/runtime artifact was found under install/"
  fi
fi

cat <<EOF
{
  "backend": "Dynamic-LIO",
  "source_dir": "$(json_escape "${SRC_DIR}")",
  "available": ${available},
  "buildable": ${buildable},
  "runtime_ready": ${runtime_ready},
  "blocker": "$(json_escape "${blocker}")"
}
EOF
