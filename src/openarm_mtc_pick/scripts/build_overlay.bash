#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Deactivate Conda before building this ROS workspace." >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OPENARM_UNDERLAY="${OPENARM_UNDERLAY:-}"

if [[ -n "${OPENARM_CUSTOM_STACK_ROOT:-}" ]]; then
  REPO_ROOT="$(cd -- "${OPENARM_CUSTOM_STACK_ROOT}" && pwd)"
else
  REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
fi
if [[ ! -f "${REPO_ROOT}/upstream.repos" ]]; then
  echo "Run this script from the source checkout, or set" >&2
  echo "OPENARM_CUSTOM_STACK_ROOT to that checkout." >&2
  exit 2
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -n "${OPENARM_UNDERLAY}" ]]; then
  source "${OPENARM_UNDERLAY}/setup.bash"
fi

cd "${REPO_ROOT}"
colcon build \
  --symlink-install \
  --packages-up-to openarm_mtc_pick \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
