#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WORKSPACE}/install/setup.bash"
PORT=18088
DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "[ERROR] workspace is not built: ${WS_SETUP}" >&2
  exit 1
fi
if command -v ss >/dev/null 2>&1; then
  if ss -ltnH | awk '{print $4}' | grep -Eq '(^|:)18088$'; then
    echo "[ERROR] TCP port 18088 is already in use." >&2
    exit 1
  fi
fi

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" && -f /opt/mapping_ws/cyclonedds_remote.xml ]]; then
  export CYCLONEDDS_URI="file:///opt/mapping_ws/cyclonedds_remote.xml"
fi

# ROS setup scripts are not safe under nounset on this machine.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

LAN_IPS="$(hostname -I 2>/dev/null || true)"
echo "[INFO] starting Route3D Web Console (motion is not started automatically)"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, port=${PORT}"
echo "[INFO] local: http://127.0.0.1:${PORT}"
for LAN_IP in ${LAN_IPS}; do
  echo "[INFO] LAN:   http://${LAN_IP}:${PORT}"
done

exec ros2 launch route3d_web_console route3d_web_console.launch.py \
  port:="${PORT}" \
  ros_domain_id:="${ROS_DOMAIN_ID}"
