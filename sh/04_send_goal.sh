#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (($# != 2)); then
  echo "用法：$0 <起点编号> <终点编号>" >&2
  echo "示例：$0 1 11" >&2
  exit 2
fi
if [[ ! "$1" =~ ^[0-9]+$ || ! "$2" =~ ^[0-9]+$ ]]; then
  echo "错误：起点编号和终点编号必须是非负整数。" >&2
  exit 2
fi
if [[ "$1" == "$2" ]]; then
  echo "错误：起点编号和终点编号不能相同。" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "发送规划请求：$1 -> $2"
exec ros2 topic pub --once \
  /route3d_dijkstra/plan_request \
  std_msgs/msg/Int32MultiArray \
  "{data: [$1, $2]}"
