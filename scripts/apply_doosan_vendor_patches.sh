#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOOSAN_ROOT="${1:-${HOME}/ros2_ws/src/doosan-robot2}"

if [[ ! -d "${DOOSAN_ROOT}/.git" ]]; then
  echo "Doosan repo not found: ${DOOSAN_ROOT}" >&2
  exit 1
fi

git -C "${DOOSAN_ROOT}" apply "${REPO_ROOT}/vendor/doosan-robot2/0001-dsr-controller2-state-topics.patch"
git -C "${DOOSAN_ROOT}" apply "${REPO_ROOT}/vendor/doosan-robot2/0002-gazebo-startup-and-update-rate.patch"

echo "Applied vendor patches to ${DOOSAN_ROOT}"
