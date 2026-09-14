#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (($# != 1)); then
  echo "用法：$0 <终点编号>" >&2
  echo "示例：$0 11" >&2
  exit 2
fi
if [[ ! "$1" =~ ^[0-9]+$ ]]; then
  echo "错误：终点编号必须是非负整数。" >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "发送仅终点规划请求：当前位置最近拓扑点 -> $1"
exec ros2 topic pub --once \
  /route3d_dijkstra/goal_request \
  std_msgs/msg/Int32 \
  "{data: $1}"
