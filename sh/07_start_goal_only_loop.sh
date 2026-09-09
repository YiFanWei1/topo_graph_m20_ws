#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEFAULT_CONFIG="${WORKSPACE}/sh/goal_only_loop_patrol.yaml"
readonly CONFIG_FILE="${1:-${DEFAULT_CONFIG}}"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "错误：仅终点循环配置不存在：${CONFIG_FILE}" >&2
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

echo "启动仅终点循环巡检，配置：${CONFIG_FILE}"
exec ros2 launch route3d_loop_patrol loop_patrol.launch.py \
  config_file:="${CONFIG_FILE}"
