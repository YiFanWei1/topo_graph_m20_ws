#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 实时模式配置：需要切换保存位置时直接修改 OUTPUT_FILE。
readonly CONFIG_FILE="${WORKSPACE}/src/route3d_odom_waypoint/config/odom_waypoint.yaml"
readonly OUTPUT_FILE="${WORKSPACE}/data/test4/topoGraph_data.json"

if [[ $# -ne 0 ]]; then
  echo "用法：./sh/18_start_live_odom_waypoint.sh" >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "错误：没有找到 ROS 2 Jazzy：/opt/ros/jazzy/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "错误：配置文件不存在：${CONFIG_FILE}" >&2
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_FILE}")"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 pkg prefix route3d_odom_waypoint >/dev/null 2>&1; then
  echo "错误：未找到 route3d_odom_waypoint，请先编译该功能包。" >&2
  exit 1
fi

echo "启动实时无点云拓扑打点："
echo "  输入话题：/lio_odom"
echo "  配置文件：${CONFIG_FILE}"
echo "  输出文件：${OUTPUT_FILE}"
echo "节点默认自动记录；按 Ctrl+C 或调用 stop_and_save 服务时完成并保存。"

cd "${WORKSPACE}"
exec ros2 launch route3d_odom_waypoint odom_waypoint.launch.py \
  config_file:="${CONFIG_FILE}" \
  output_file:="${OUTPUT_FILE}" \
  frame_id:=camera_init \
  launch_rviz:=true
