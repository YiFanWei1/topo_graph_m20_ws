#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="/home/langyi/workspace/wyf/topo_graph_ws"
readonly ROS_PROCESS_PATTERN='(/opt/ros/[^/]+/(bin/ros2|lib/)|/install/[^ ]+/lib/[^ ]+)'

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  source "${WORKSPACE}/install/setup.bash"
fi
set -u

echo "先请求停止循环巡检和当前运动任务……"
timeout 3 ros2 service call /route3d_loop_patrol/stop std_srvs/srv/Trigger '{}' \
  >/dev/null 2>&1 || true
timeout 3 ros2 service call /route3d_pid_controller/cancel std_srvs/srv/Trigger '{}' \
  >/dev/null 2>&1 || true
sleep 1

find_ros_pids() {
  pgrep -u "$(id -u)" -f "${ROS_PROCESS_PATTERN}" 2>/dev/null || true
}

mapfile -t ros_pids < <(find_ros_pids)
if ((${#ros_pids[@]} == 0)); then
  echo "没有发现当前用户的 ROS 2 进程。"
  exit 0
fi

echo "向 ${#ros_pids[@]} 个 ROS 2 进程发送 SIGINT……"
kill -INT "${ros_pids[@]}" 2>/dev/null || true

for _ in {1..10}; do
  sleep 0.5
  mapfile -t ros_pids < <(find_ros_pids)
  ((${#ros_pids[@]} == 0)) && break
done

if ((${#ros_pids[@]} > 0)); then
  echo "仍有 ${#ros_pids[@]} 个进程未退出，发送 SIGTERM……"
  kill -TERM "${ros_pids[@]}" 2>/dev/null || true
  sleep 1
fi

mapfile -t ros_pids < <(find_ros_pids)
if ((${#ros_pids[@]} > 0)); then
  echo "警告：仍有 ROS 2 进程未退出：" >&2
  ps -o pid=,args= -p "$(IFS=,; echo "${ros_pids[*]}")" >&2 || true
  exit 1
fi

echo "ROS 2 节点已全部关闭。"
