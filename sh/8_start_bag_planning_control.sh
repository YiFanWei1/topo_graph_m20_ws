#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEFAULT_GRAPH="${WORKSPACE}/data/79792/topoGraph_data.json"
readonly GRAPH_FILE="${1:-${DEFAULT_GRAPH}}"
readonly LAUNCH_RVIZ="${2:-true}"
readonly RECORDING_BODY_HEIGHT_M="${3:-0.57}"

if [[ ! -f "${GRAPH_FILE}" ]]; then
  echo "错误：拓扑图不存在：${GRAPH_FILE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi
if [[ "${LAUNCH_RVIZ}" != "true" && "${LAUNCH_RVIZ}" != "false" ]]; then
  echo "错误：launch_rviz 只能是 true 或 false。" >&2
  exit 2
fi
if ! python3 -c 'import math, sys; value=float(sys.argv[1]); sys.exit(0 if math.isfinite(value) and value >= 0.0 else 1)' \
    "${RECORDING_BODY_HEIGHT_M}"; then
  echo "错误：recording_body_height_m 必须是非负有限数。" >&2
  exit 2
fi
python3 -m json.tool "${GRAPH_FILE}" >/dev/null

cd "${WORKSPACE}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "启动 Bag 回放规控栈（不会连接实机）："
echo "  graph=${GRAPH_FILE}"
echo "  launch_rviz=${LAUNCH_RVIZ}"
echo "  recording_body_height_m=${RECORDING_BODY_HEIGHT_M}"
echo "  enable_motion=false"
echo "  start_m20_bridge=false"
echo "  publish_odometry_tf=true"
echo "  use_sim_time=true"
exec ros2 launch route3d_m20_adapter m20_pid_route.launch.xml \
  graph_file:="${GRAPH_FILE}" \
  enable_motion:=false \
  start_m20_bridge:=false \
  publish_odometry_tf:=true \
  use_sim_time:=true \
  odometry_body_height_m:="${RECORDING_BODY_HEIGHT_M}" \
  launch_rviz:="${LAUNCH_RVIZ}"
