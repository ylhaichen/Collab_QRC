#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/noetic/setup.bash
if [[ -f /point_lio_ws/devel/setup.bash ]]; then
  source /point_lio_ws/devel/setup.bash
fi
set -u

exec "$@"
