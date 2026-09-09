#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="/home/langyi/workspace/wyf/topo_graph_ws"
readonly DEFAULT_CONFIG="${WORKSPACE}/src/route3d_product_demo/config/live_route_product.yaml"
readonly CONFIG_FILE="${1:-${DEFAULT_CONFIG}}"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "错误：自动打点配置不存在：${CONFIG_FILE}" >&2
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

echo "启动自动打点，配置：${CONFIG_FILE}"
echo "键盘操作：1 开始，2 停止并保存，q 退出。"
exec ros2 launch route3d_product_demo live_route_product.launch.py \
  config_file:="${CONFIG_FILE}"
