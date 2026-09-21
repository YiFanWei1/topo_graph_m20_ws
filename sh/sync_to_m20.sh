#!/usr/bin/env bash
set -Eeuo pipefail

readonly REMOTE_HOST="192.168.121.1"
readonly REMOTE_USER="langyi"
readonly REMOTE_PASSWORD="langyi"
readonly REMOTE_WORKSPACE="/home/langyi/workspace/wyf/topo_graph_m20_ws"
readonly LOCAL_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly -a SSH_OPTIONS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
)

usage() {
  cat <<'EOF'
用法：
  ./sh/sync_to_m20.sh [--build] <目录> [目录...]

说明：
  从本机增量同步所选目录到 M20 主机，不删除远端额外文件。
  --build  同步成功后在 M20 主机执行 colcon build --symlink-install

示例：
  ./sh/sync_to_m20.sh src sh
  ./sh/sync_to_m20.sh --build src sh
  ./sh/sync_to_m20.sh src/route3d_product_demo
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

validate_relative_directory() {
  local path="$1"
  local segment

  [[ "${path}" =~ ^[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)*$ ]] ||
    die "目录必须是安全的相对路径：${path}"

  IFS='/' read -r -a segments <<< "${path}"
  for segment in "${segments[@]}"; do
    case "${segment}" in
      .|..|.git|build|install|log)
        die "禁止同步目录段 '${segment}'：${path}"
        ;;
    esac
  done
}

build_after_sync=false
directories=()
parse_options=true
for argument in "$@"; do
  if [[ "${parse_options}" == true ]]; then
    case "${argument}" in
      --build)
        build_after_sync=true
        continue
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        parse_options=false
        continue
        ;;
      -*)
        echo "错误：未知选项：${argument}" >&2
        usage >&2
        exit 2
        ;;
    esac
  fi
  directories+=("${argument}")
done

if ((${#directories[@]} == 0)); then
  usage >&2
  exit 2
fi

for directory in "${directories[@]}"; do
  validate_relative_directory "${directory}"
  [[ -d "${LOCAL_WORKSPACE}/${directory}" ]] ||
    die "本机源目录不存在：${LOCAL_WORKSPACE}/${directory}"
done

command -v sshpass >/dev/null 2>&1 || die "本机未安装 sshpass"
command -v ssh >/dev/null 2>&1 || die "本机未安装 ssh"
command -v rsync >/dev/null 2>&1 || die "本机未安装 rsync"

export SSHPASS="${REMOTE_PASSWORD}"
ssh_command=(
  sshpass -e ssh
  "${SSH_OPTIONS[@]}"
  "${REMOTE_USER}@${REMOTE_HOST}"
)
readonly RSYNC_RSH="sshpass -e ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new"

echo "检查远端工作空间和工具……"
if [[ "${build_after_sync}" == true ]]; then
  "${ssh_command[@]}" bash -s <<'REMOTE_CHECK_BUILD'
set -eu
workspace='/home/langyi/workspace/wyf/topo_graph_m20_ws'
command -v rsync >/dev/null 2>&1 || {
  echo '错误：远端未安装 rsync' >&2
  exit 1
}
test -d "${workspace}" || {
  echo "错误：远端工作空间不存在：${workspace}" >&2
  exit 1
}
test -f /opt/ros/jazzy/setup.bash || {
  echo '错误：远端缺少 /opt/ros/jazzy/setup.bash' >&2
  exit 1
}
set +u
. /opt/ros/jazzy/setup.bash
set -u
command -v colcon >/dev/null 2>&1 || {
  echo '错误：远端未安装 colcon' >&2
  exit 1
}
REMOTE_CHECK_BUILD
else
  "${ssh_command[@]}" bash -s <<'REMOTE_CHECK'
set -eu
workspace='/home/langyi/workspace/wyf/topo_graph_m20_ws'
command -v rsync >/dev/null 2>&1 || {
  echo '错误：远端未安装 rsync' >&2
  exit 1
}
test -d "${workspace}" || {
  echo "错误：远端工作空间不存在：${workspace}" >&2
  exit 1
}
REMOTE_CHECK
fi

"${ssh_command[@]}" bash -s -- "${directories[@]}" <<'REMOTE_MKDIR'
set -eu
workspace='/home/langyi/workspace/wyf/topo_graph_m20_ws'
for directory in "$@"; do
  mkdir -p "${workspace}/${directory}"
done
REMOTE_MKDIR

for directory in "${directories[@]}"; do
  source_directory="${LOCAL_WORKSPACE}/${directory}"
  echo "推送：${source_directory} -> ${REMOTE_WORKSPACE}/${directory}"
  rsync -a --human-readable --info=progress2 \
    --exclude='.git/' \
    -e "${RSYNC_RSH}" \
    "${source_directory}/" \
    "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_WORKSPACE}/${directory}/"
done

if [[ "${build_after_sync}" == true ]]; then
  echo "在 M20 主机编译：${REMOTE_WORKSPACE}"
  "${ssh_command[@]}" bash -s <<'REMOTE_BUILD'
set -eo pipefail
cd /home/langyi/workspace/wyf/topo_graph_m20_ws
set +u
. /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install
REMOTE_BUILD
fi

echo "完成：已向 M20 同步 ${#directories[@]} 个目录。"
