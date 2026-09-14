#!/usr/bin/env bash
set -Eeuo pipefail

readonly WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if (($# < 1 || $# > 3)); then
  echo "用法：$0 <bag目录> [回放倍率] [是否循环]" >&2
  echo "示例：$0 /home/wei/bag/m20_route_001 1.0 false" >&2
  exit 2
fi

readonly BAG_PATH="$(realpath -e "$1")"
readonly PLAYBACK_RATE="${2:-1.0}"
readonly LOOP_PLAYBACK="${3:-false}"

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "错误：不是有效的 rosbag2 目录（缺少 metadata.yaml）：${BAG_PATH}" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译：${WORKSPACE}" >&2
  exit 1
fi
if ! python3 -c 'import math, sys; value=float(sys.argv[1]); sys.exit(0 if math.isfinite(value) and value > 0.0 else 1)' \
    "${PLAYBACK_RATE}"; then
  echo "错误：回放倍率必须是大于 0 的有限数。" >&2
  exit 2
fi
if [[ "${LOOP_PLAYBACK}" != "true" && "${LOOP_PLAYBACK}" != "false" ]]; then
  echo "错误：是否循环只能是 true 或 false。" >&2
  exit 2
fi

cd "${WORKSPACE}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

play_args=(
  "${BAG_PATH}"
  --clock
  --rate "${PLAYBACK_RATE}"
)
if [[ "${LOOP_PLAYBACK}" == "true" ]]; then
  play_args+=(--loop)
fi
play_args+=(
  --topics
  /lio_odom
  /lio_odom_hf
  /cloud_registered_body
)

echo "回放传感器输入：${BAG_PATH}"
echo "倍率：${PLAYBACK_RATE}，循环：${LOOP_PLAYBACK}"
echo "仅发布 /lio_odom、/lio_odom_hf、/cloud_registered_body。"
echo "旧 Bag 的冲突 TF 不回放；规控栈从 /lio_odom_hf 生成 camera_init -> base_link。"
echo "不会重放历史速度命令、ready 状态或 M20 控制反馈。"
exec ros2 bag play "${play_args[@]}"
