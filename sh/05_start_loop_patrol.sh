#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="/home/langyi/workspace/wyf/topo_graph_ws"
readonly DEFAULT_CONFIG="${WORKSPACE}/sh/loop_patrol.yaml"
readonly CONFIG_FILE="${1:-${DEFAULT_CONFIG}}"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "错误：循环巡检配置不存在：${CONFIG_FILE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi

cd "${WORKSPACE}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "启动到达信号门控的循环巡检，配置：${CONFIG_FILE}"
exec ros2 launch route3d_loop_patrol loop_patrol.launch.py \
  config_file:="${CONFIG_FILE}"
