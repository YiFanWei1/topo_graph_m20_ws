#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (($# != 2)); then
  echo "用法：$0 <拓扑 JSON> <点号>" >&2
  echo "示例：$0 ./data/full_2_stop/topoGraph_data.json 68" >&2
  exit 2
fi

GRAPH_FILE="$1"
readonly VERTEX_ID="$2"
if [[ "${GRAPH_FILE}" != /* ]]; then
  GRAPH_FILE="${WORKSPACE}/${GRAPH_FILE#./}"
fi
if [[ ! -f "${GRAPH_FILE}" ]]; then
  echo "错误：拓扑文件不存在：${GRAPH_FILE}" >&2
  exit 1
fi
GRAPH_FILE="$(realpath "${GRAPH_FILE}")"
readonly GRAPH_FILE
if [[ ! "${VERTEX_ID}" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误：点号必须是大于 0 的整数。" >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" && -f /opt/mapping_ws/cyclonedds_remote.xml ]]; then
  export CYCLONEDDS_URI="file:///opt/mapping_ws/cyclonedds_remote.xml"
fi

# ROS setup 脚本在本机不兼容 nounset，加载期间临时关闭。
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

TEMP_INITIALIZER_PID=""
cleanup() {
  if [[ -n "${TEMP_INITIALIZER_PID}" ]] && kill -0 "${TEMP_INITIALIZER_PID}" 2>/dev/null; then
    kill "${TEMP_INITIALIZER_PID}" 2>/dev/null || true
    wait "${TEMP_INITIALIZER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ros2 node list 2>/dev/null | grep -qx '/route3d_vertex_initializer'; then
  echo "检测到初始化节点，切换拓扑：${GRAPH_FILE}"
  ros2 topic pub --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /route3d_initial_pose/graph_file \
    std_msgs/msg/String \
    "{data: '${GRAPH_FILE}'}"
  sleep 0.5
else
  readonly PARAMS_FILE="${WORKSPACE}/install/route3d_vertex_initializer/share/route3d_vertex_initializer/config/vertex_initializer.yaml"
  if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "错误：找不到初始化节点配置：${PARAMS_FILE}" >&2
    exit 1
  fi
  echo "未检测到初始化节点，临时启动并加载拓扑：${GRAPH_FILE}"
  ros2 run route3d_vertex_initializer vertex_initializer_node \
    --ros-args \
    --params-file "${PARAMS_FILE}" \
    -p "fallback_graph_file:=${GRAPH_FILE}" \
    -p prefer_graph_file_topic:=false &
  TEMP_INITIALIZER_PID=$!
  sleep 1.0
  if ! kill -0 "${TEMP_INITIALIZER_PID}" 2>/dev/null; then
    echo "错误：初始化节点启动失败。" >&2
    wait "${TEMP_INITIALIZER_PID}"
  fi
fi

echo "使用拓扑点 ${VERTEX_ID} 发布 /initialpose，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
ros2 run route3d_vertex_initializer set_initial_pose_by_vertex "${VERTEX_ID}" --wait 3.0

# 给 DDS 和定位节点留出接收时间，再结束临时初始化节点。
sleep 1.0
echo "初始化位姿已发送。"
