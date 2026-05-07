#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

arch="$(uname -m)"
ram_kb="$(awk '/MemTotal/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
ros2_status="missing"
livox_status="missing"
dds_status="unknown"
peer_ping_status="not_configured"

if command -v ros2 >/dev/null 2>&1; then
  ros2_status="available"
  if ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1 || ros2 pkg prefix livox_ros_driver >/dev/null 2>&1; then
    livox_status="available"
  fi
fi

if [[ -n "${PEER_IP:-}" ]]; then
  if ping -c 1 -W 1 "${PEER_IP}" >/dev/null 2>&1; then
    peer_ping_status="ok"
  else
    peer_ping_status="failed"
  fi
fi

if [[ "${ros2_status}" == "available" ]]; then
  dds_status="ros2_available_not_runtime_tested"
fi

cat > logs/jetson_readiness_report.md <<EOF
# Jetson Readiness Report

- cpu_arch: \`${arch}\`
- ram_mb: \`$((ram_kb / 1024))\`
- ros2: \`${ros2_status}\`
- livox_driver: \`${livox_status}\`
- dds_discovery: \`${dds_status}\`
- peer_ping: \`${peer_ping_status}\`
- gtsam_check: run \`scripts/setup/check_gtsam_backend.sh\`

This script is a readiness gate only. Real two-Jetson validation still requires both Jetsons on the target DDS network.
EOF

cat logs/jetson_readiness_report.md
