#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEFAULT_GRAPH="${WORKSPACE}/data/shinei/topoGraph_data.json"
readonly GRAPH_FILE="${1:-${DEFAULT_GRAPH}}"
readonly ENABLE_MOTION="${2:-true}"
# readonly ENABLE_MOTION="${2:-false}"
readonly LAUNCH_RVIZ="${3:-true}"

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
echo "  enable_motion=${ENABLE_MOTION}"
echo "  start_m20_bridge=${ENABLE_MOTION}（false 时不接管底盘，保留遥控器控制）"
echo "  launch_rviz=${LAUNCH_RVIZ}"
exec ros2 launch route3d_m20_adapter m20_pid_route.launch.xml \
  graph_file:="${GRAPH_FILE}" \
  enable_motion:="${ENABLE_MOTION}" \
  start_m20_bridge:="${ENABLE_MOTION}" \
  use_sim_time:=false \
  launch_rviz:="${LAUNCH_RVIZ}"
