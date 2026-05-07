#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="${WS_DIR}/docker/ros1_scpgo/docker-compose.yml"

if ! command -v docker >/dev/null 2>&1; then
  cat >&2 <<'EOF'
ERROR: docker is not installed or not on PATH.

Ubuntu 22.04 quick fix:
  sudo apt-get update
  sudo apt-get install -y docker.io docker-compose-v2
  sudo systemctl enable --now docker
  sudo usermod -aG docker "$USER"

Then open a new shell, or run:
  newgrp docker

Verify:
  docker version
  docker compose version

Then retry:
  bash scripts/launch/scpgo_ros1_bridge.sh
EOF
  exit 127
fi

if ! docker info >/dev/null 2>&1; then
  cat >&2 <<'EOF'
ERROR: docker is installed, but this shell cannot access the Docker daemon.

Most likely your docker group membership has not reached this shell yet.
Run:
  newgrp docker

Or close this terminal and open a new one, then verify:
  id
  docker version

If docker is still inactive:
  sudo systemctl enable --now docker
EOF
  exit 126
fi

if docker compose version >/dev/null 2>&1; then
  exec docker compose -f "${COMPOSE_FILE}" up --build "$@"
fi

if command -v docker-compose >/dev/null 2>&1; then
  exec docker-compose -f "${COMPOSE_FILE}" up --build "$@"
fi

cat >&2 <<'EOF'
ERROR: docker is installed, but neither 'docker compose' nor 'docker-compose' is available.

Ubuntu 22.04 quick fix:
  sudo apt-get install -y docker-compose-v2

Legacy fallback if needed:
  sudo apt-get install -y docker-compose
EOF
exit 127
