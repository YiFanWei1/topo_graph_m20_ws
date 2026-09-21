#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 默认编辑实时打点结果；需要编辑其他地图时修改这里。
readonly GRAPH_FILE="${WORKSPACE}/data/shan66/topoGraph_data.json"
readonly FRAME_ID="camera_init"

if [[ $# -ne 0 ]]; then
  echo "用法：./sh/19_edit_topology.sh" >&2
  exit 2
fi
if [[ ! -f "${GRAPH_FILE}" ]]; then
  echo "错误：拓扑文件不存在：${GRAPH_FILE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 pkg prefix route3d_topology_editor >/dev/null 2>&1; then
  echo "错误：未找到 route3d_topology_editor，请先编译该功能包。" >&2
  exit 1
fi

echo "启动 RViz 拓扑属性编辑器：${GRAPH_FILE}"
echo "请先停止实时记录节点，避免文件被定时快照覆盖。"

exec ros2 launch route3d_topology_editor editor.launch.py \
  graph_file:="${GRAPH_FILE}" \
  frame_id:="${FRAME_ID}"

