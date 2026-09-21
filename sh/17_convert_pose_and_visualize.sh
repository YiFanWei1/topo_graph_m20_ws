#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 固定输入与输出路径；需要切换地图时直接修改这里。
readonly INPUT_FILE="/home/langyi/workspace/map/shinei/map/slam_data/trajectory/pose.json"
readonly OUTPUT_FILE="/home/langyi/workspace/wyf/topo_graph_m20_ws/data/shinei/topoGraph_data.json"

usage() {
  cat <<'EOF'
用法：
  ./sh/17_convert_pose_and_visualize.sh

输入、输出路径在脚本顶部的 INPUT_FILE 和 OUTPUT_FILE 中配置。

可选环境变量：
  ROUTE3D_CONFIG_FILE          默认读取已安装的 route3d_odom_waypoint 配置
  ROUTE3D_FRAME_ID             默认 camera_init
  其余 ROUTE3D_* 变量仅在显式设置时覆盖 YAML 中的对应值：
  ROUTE3D_BODY_HEIGHT、ROUTE3D_TARGET_SPACING、ROUTE3D_RELOCATION_DISTANCE
  ROUTE3D_OBSTACLE_MODE、ROUTE3D_GEOMETRIC_LOOP_CLOSURE
  ROUTE3D_LOOP_XY_TOLERANCE、ROUTE3D_LOOP_EXIT_XY_TOLERANCE
  ROUTE3D_LOOP_Z_TOLERANCE、ROUTE3D_LOOP_HEADING_DEGREES
  ROUTE3D_LOOP_CONFIRM_DISTANCE、ROUTE3D_LOOP_EXIT_CONFIRM_DISTANCE
  ROUTE3D_LOOP_MIN_GRAPH_DISTANCE、ROUTE3D_LOOP_MIN_TIME
  ROUTE3D_LOOP_VERTEX_SNAP、ROUTE3D_LOOP_REJECTION_COOLDOWN
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "错误：此脚本不再接收输入、输出路径参数，请修改脚本顶部配置。" >&2
  usage >&2
  exit 2
fi

readonly FRAME_ID="${ROUTE3D_FRAME_ID:-camera_init}"
if [[ -f "${WORKSPACE}/install/share/route3d_odom_waypoint/config/odom_waypoint.yaml" ]]; then
  readonly DEFAULT_CONFIG="${WORKSPACE}/install/share/route3d_odom_waypoint/config/odom_waypoint.yaml"
else
  readonly DEFAULT_CONFIG="${WORKSPACE}/src/route3d_odom_waypoint/config/odom_waypoint.yaml"
fi
readonly CONFIG_FILE="${ROUTE3D_CONFIG_FILE:-${DEFAULT_CONFIG}}"

if [[ ! -f "${INPUT_FILE}" ]]; then
  echo "错误：姿态文件不存在：${INPUT_FILE}" >&2
  exit 1
fi
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "错误：配置文件不存在：${CONFIG_FILE}" >&2
  exit 1
fi
if [[ "${INPUT_FILE}" == "${OUTPUT_FILE}" ]]; then
  echo "错误：输出文件不能与输入姿态文件相同，以免覆盖原始数据。" >&2
  exit 1
fi
if [[ "${OUTPUT_FILE}" != *.json ]]; then
  echo "错误：输出路径必须是一个 .json 文件：${OUTPUT_FILE}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_FILE}")"

cd "${WORKSPACE}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 pkg prefix route3d_odom_waypoint >/dev/null 2>&1; then
  echo "错误：未找到 route3d_odom_waypoint，请先编译新功能包。" >&2
  exit 1
fi
echo "开始转换无点云姿态轨迹："
echo "  输入：${INPUT_FILE}"
echo "  输出：${OUTPUT_FILE}"
echo "  配置：${CONFIG_FILE}"
echo "  frame_id=${FRAME_ID}"

CONVERT_ARGUMENTS=(
  "${INPUT_FILE}"
  --output "${OUTPUT_FILE}"
  --frame-id "${FRAME_ID}"
  --config-file "${CONFIG_FILE}"
)

append_override() {
  local variable_name="$1"
  local option_name="$2"
  if [[ -n "${!variable_name+x}" ]]; then
    CONVERT_ARGUMENTS+=("${option_name}" "${!variable_name}")
  fi
}

append_override ROUTE3D_BODY_HEIGHT --body-height
append_override ROUTE3D_TARGET_SPACING --target-spacing
append_override ROUTE3D_RELOCATION_DISTANCE --relocation-distance
append_override ROUTE3D_OBSTACLE_MODE --obstacle-mode
append_override ROUTE3D_LOOP_XY_TOLERANCE --loop-corridor-xy-tolerance
append_override ROUTE3D_LOOP_EXIT_XY_TOLERANCE --loop-corridor-exit-xy-tolerance
append_override ROUTE3D_LOOP_Z_TOLERANCE --loop-z-tolerance
append_override ROUTE3D_LOOP_HEADING_DEGREES --loop-heading-tolerance-degrees
append_override ROUTE3D_LOOP_CONFIRM_DISTANCE --loop-confirmation-distance
append_override ROUTE3D_LOOP_EXIT_CONFIRM_DISTANCE --loop-exit-confirmation-distance
append_override ROUTE3D_LOOP_MIN_GRAPH_DISTANCE --loop-minimum-graph-separation
append_override ROUTE3D_LOOP_MIN_TIME --loop-minimum-time-separation
append_override ROUTE3D_LOOP_VERTEX_SNAP --loop-vertex-snap-distance
append_override ROUTE3D_LOOP_REJECTION_COOLDOWN --loop-rejection-cooldown-distance

if [[ -n "${ROUTE3D_GEOMETRIC_LOOP_CLOSURE+x}" ]]; then
  case "${ROUTE3D_GEOMETRIC_LOOP_CLOSURE,,}" in
    true|1|yes|on) CONVERT_ARGUMENTS+=(--geometric-loop-closure) ;;
    false|0|no|off) CONVERT_ARGUMENTS+=(--no-geometric-loop-closure) ;;
    *)
      echo "错误：ROUTE3D_GEOMETRIC_LOOP_CLOSURE 必须是 true 或 false。" >&2
      exit 1
      ;;
  esac
fi

ros2 run route3d_odom_waypoint pose_file_to_topology "${CONVERT_ARGUMENTS[@]}"

echo "转换成功，启动 RViz：${OUTPUT_FILE}"
exec ros2 launch route3d_odom_waypoint file_result_visualization.launch.py \
  pose_file:="${INPUT_FILE}" \
  topology_file:="${OUTPUT_FILE}" \
  frame_id:="${FRAME_ID}" \
  body_height:="${ROUTE3D_BODY_HEIGHT:-0.57}" \
  show_vertex_labels:=true \
  show_edge_labels:=true \
  launch_rviz:=true
