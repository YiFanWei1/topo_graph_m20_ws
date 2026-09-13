#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENABLE_MOTION="${1:-false}"

if [[ "${ENABLE_MOTION}" != "true" && "${ENABLE_MOTION}" != "false" ]]; then
  echo "用法: $0 [true|false]"
  echo "  false: 连接 bridge 但不准备或放行运动（默认）"
  echo "  true : 实机准备完成后放行 /m20/manual/cmd_vel"
  exit 2
fi

if pgrep -u "$(id -u)" -f 'route3d_m20_adapter|basic_server_bridge_node' >/dev/null; then
  echo "拒绝启动：检测到 Route3D M20 adapter 或 basic_server_bridge 正在运行。"
  echo "请先在原 launch 终端按 Ctrl+C，确保同一时间只有一套 M20 控制栈。"
  exit 3
fi

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "未找到 ${WORKSPACE_DIR}/install/setup.bash，请先执行 colcon build。"
  exit 4
fi
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "启动 M20 独立手动速度控制："
echo "  input=/m20/manual/cmd_vel"
echo "  enable_motion=${ENABLE_MOTION}"
echo "  watchdog=0.30s"
echo "  limits=|vx|<=0.8, vy=0, |wz|<=0.5"

exec ros2 launch m20_velocity_control m20_manual_control.launch.py \
  enable_motion:="${ENABLE_MOTION}"
