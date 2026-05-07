#!/usr/bin/env bash

MANUAL_EXPECTED_BRANCH="feature/swarm-lio2-primary-dynamiclio-erasor-clean"
MANUAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MANUAL_LOG_DIR="${MANUAL_ROOT}/logs/manual"
mkdir -p "${MANUAL_LOG_DIR}"

manual_require_branch() {
  local branch
  branch="$(git -C "${MANUAL_ROOT}" branch --show-current)"
  echo "current_branch=${branch}"
  if [[ "${branch}" != "${MANUAL_EXPECTED_BRANCH}" && "${FORCE:-0}" != "1" ]]; then
    echo "ERROR: refusing to run on branch ${branch}; expected ${MANUAL_EXPECTED_BRANCH}. Set FORCE=1 to override." >&2
    exit 2
  fi
}

manual_log_name() {
  local stem="$1"
  local stamp
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  printf '%s/%s_%s.log' "${MANUAL_LOG_DIR}" "${stem}" "${stamp}"
}

manual_run_logged() {
  local stem="$1"
  shift
  manual_run_logged_allow_codes "${stem}" "0" "$@"
}

manual_run_logged_allow_codes() {
  local stem="$1"
  local allowed_codes="$2"
  shift 2
  local log_file
  log_file="$(manual_log_name "${stem}")"
  {
    echo "command=$*"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } | tee "${log_file}"
  set +e
  "$@" 2>&1 | tee -a "${log_file}"
  local status=${PIPESTATUS[0]}
  set -e
  echo "exit_code=${status}" | tee -a "${log_file}"
  if [[ ",${allowed_codes}," != *",${status},"* ]]; then
    echo "ERROR: command failed; see ${log_file}" >&2
    exit "${status}"
  fi
}

manual_refuse_origin_push() {
  local origin_push
  origin_push="$(git -C "${MANUAL_ROOT}" remote get-url --push origin 2>/dev/null || true)"
  if [[ "${origin_push}" == *"HanshangZhu/Collab_QRC"* ]]; then
    echo "origin push intentionally skipped: origin=${origin_push}"
  fi
}
