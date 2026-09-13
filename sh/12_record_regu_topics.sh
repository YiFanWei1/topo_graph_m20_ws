#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly WS_SETUP="${WORKSPACE}/install/setup.bash"
readonly DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

if (($# > 2)); then
  echo "用法：$0 [输出父目录] [bag名称]" >&2
  echo "示例：$0 /home/langyi/bag m20_regu_001" >&2
  exit 2
fi

readonly OUTPUT_ROOT="${1:-${WORKSPACE}/bags}"
readonly BAG_NAME="${2:-m20_regu_$(date +%Y%m%d_%H%M%S)}"

if [[ ! "${BAG_NAME}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "错误：bag 名称只能包含字母、数字、点、下划线和横线。" >&2
  exit 2
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "错误：ROS 环境不存在：${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "错误：工作空间尚未编译：${WS_SETUP}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"
readonly OUTPUT_PATH="$(realpath -m "${OUTPUT_ROOT}/${BAG_NAME}")"
if [[ -e "${OUTPUT_PATH}" ]]; then
  echo "错误：输出路径已经存在：${OUTPUT_PATH}" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" && -f /opt/mapping_ws/cyclonedds_remote.xml ]]; then
  export CYCLONEDDS_URI="file:///opt/mapping_ws/cyclonedds_remote.xml"
fi

# 本机 ROS setup 在 nounset 模式下可能引用未定义变量，因此 source 时临时关闭 set -u。
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

# 与 /home/wei/bag/regu 完全一致的四个传感器/坐标话题。
readonly -a TOPICS=(
  /cloud_registered_body
  /lio_odom
  /lio_odom_hf
  /tf
)

mapfile -t AVAILABLE_TOPICS < <(ros2 topic list 2>/dev/null || true)
missing_topics=()
for topic in "${TOPICS[@]}"; do
  if ! printf "%s\n" "${AVAILABLE_TOPICS[@]}" | grep -Fqx "${topic}"; then
    missing_topics+=("${topic}")
  fi
done

echo "录制目录：${OUTPUT_PATH}"
echo "存储格式：MCAP"
echo "录制话题："
printf "  %s\n" "${TOPICS[@]}"
echo "使用单个 MCAP 文件持续录制，不进行分片。"
if ((${#missing_topics[@]} > 0)); then
  echo "警告：以下话题当前尚未发现：" >&2
  printf "  %s\n" "${missing_topics[@]}" >&2
  echo "录制仍会启动，并等待这些话题出现。" >&2
fi
echo "按 Ctrl+C 正常停止并写入 metadata.yaml。"

exec ros2 bag record \
  --storage mcap \
  --output "${OUTPUT_PATH}" \
  --topics \
  "${TOPICS[@]}"
