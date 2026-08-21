#!/usr/bin/env bash

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Refusing to mix Conda (${CONDA_PREFIX}) with the ROS overlay." >&2
  echo "Run 'conda deactivate' and source this file again." >&2
  return 2 2>/dev/null || exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OPENARM_UNDERLAY="${OPENARM_UNDERLAY:-}"

if [[ -n "${OPENARM_CUSTOM_STACK_ROOT:-}" ]]; then
  OVERLAY_PREFIX="${OPENARM_CUSTOM_STACK_ROOT}/install"
elif [[ -f "${SCRIPT_DIR}/../../../upstream.repos" ]]; then
  OVERLAY_PREFIX="${SCRIPT_DIR}/../../../install"
else
  # Installed layout: <prefix>/lib/openarm_mtc_pick/setup_overlay.bash
  OVERLAY_PREFIX="${SCRIPT_DIR}/../.."
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -n "${OPENARM_UNDERLAY}" ]]; then
  source "${OPENARM_UNDERLAY}/setup.bash"
fi
source "${OVERLAY_PREFIX}/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
unset ROS_LOCALHOST_ONLY

echo "Loaded: ROS Jazzy -> openarm_ws -> openarm_mtc_ws"
