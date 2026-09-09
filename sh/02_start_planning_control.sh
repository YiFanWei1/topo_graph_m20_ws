#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="/home/langyi/workspace/wyf/topo_graph_ws"
readonly DEFAULT_GRAPH="${WORKSPACE}/data/ceshi_1/topoGraph_data.json"
readonly GRAPH_FILE="${1:-${DEFAULT_GRAPH}}"
readonly NETWORK_INTERFACE="${2:-enp2s0}"
readonly ENABLE_MOTION="${3:-true}"
readonly LAUNCH_RVIZ="${4:-true}"

if [[ ! -f "${GRAPH_FILE}" ]]; then
  echo "错误：拓扑图不存在：${GRAPH_FILE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi
python3 -m json.tool "${GRAPH_FILE}" >/dev/null

cd "${WORKSPACE}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "启动规控："
echo "  graph=${GRAPH_FILE}"
echo "  network_interface=${NETWORK_INTERFACE}"
echo "  enable_motion=${ENABLE_MOTION}"
echo "  launch_rviz=${LAUNCH_RVIZ}"
exec ros2 launch route3d_go2_adapter go2_pid_route.launch.xml \
  graph_file:="${GRAPH_FILE}" \
  network_interface:="${NETWORK_INTERFACE}" \
  enable_motion:="${ENABLE_MOTION}" \
  use_sim_time:=false \
  launch_rviz:="${LAUNCH_RVIZ}"
