#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

POINT_LIO_REPO="${POINT_LIO_REPO:-https://github.com/hku-mars/Point-LIO.git}"
POINT_LIO_SRC="${POINT_LIO_SRC:-src/vendor/Point-LIO}"
STATUS_JSON="logs/point_lio_validation.json"
STATUS_MD="logs/point_lio_validation.md"

docker_status="missing"
if command -v docker >/dev/null 2>&1; then
  docker_status="available"
fi

source_status="missing"
if [[ -d "${POINT_LIO_SRC}" ]]; then
  source_status="available:${POINT_LIO_SRC}"
fi

remote_status="not_checked"
remote_error=""
if command -v git >/dev/null 2>&1; then
  tmp_err="$(mktemp)"
  if git ls-remote "${POINT_LIO_REPO}" HEAD >/dev/null 2>"${tmp_err}"; then
    remote_status="reachable"
  else
    remote_status="unreachable"
    remote_error="$(tr '\n' ' ' < "${tmp_err}")"
  fi
  rm -f "${tmp_err}"
fi

runtime_ready=false
dependency_blocker=""
if [[ "${docker_status}" != "available" ]]; then
  dependency_blocker="docker_not_found"
elif [[ "${source_status}" == "missing" && "${remote_status}" != "reachable" ]]; then
  dependency_blocker="point_lio_source_unavailable:${remote_error}"
else
  runtime_ready=true
fi

cat > "${STATUS_JSON}" <<JSON
{
  "schema": "point_lio_validation/v1",
  "validation_name": "point_lio_backend_availability_check",
  "backend": "point_lio",
  "docker_status": "${docker_status}",
  "source_status": "${source_status}",
  "remote_status": "${remote_status}",
  "runtime_ready": ${runtime_ready},
  "dependency_blocker": "${dependency_blocker}",
  "gt_used_runtime": false
}
JSON

cat > "${STATUS_MD}" <<MD
# Point-LIO Availability Check

- docker_status: \`${docker_status}\`
- source_status: \`${source_status}\`
- remote_status: \`${remote_status}\`
- runtime_ready: \`${runtime_ready}\`
- dependency_blocker: \`${dependency_blocker}\`
- gt_used_runtime: \`false\`
MD

cat "${STATUS_JSON}"
if [[ "${runtime_ready}" != "true" ]]; then
  exit 2
fi
