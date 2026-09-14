#!/usr/bin/env bash
set -Eeuo pipefail

readonly MAPPING_WORKSPACE="/opt/mapping_ws"
readonly MAPPING_SCRIPT="${MAPPING_WORKSPACE}/run_mapping_nodes_2.sh"

if [[ ! -x "${MAPPING_SCRIPT}" ]]; then
  echo "错误：找不到可执行建图脚本：${MAPPING_SCRIPT}" >&2
  exit 1
fi

cd "${MAPPING_WORKSPACE}"

echo "启动 Robosense LIO 建图："
echo "  ${MAPPING_SCRIPT} mode:=mapping config:=robosense_lio"
echo "建图完成前请保持本终端运行；使用 Ctrl+C 停止。"

exec "${MAPPING_SCRIPT}" \
  mode:=mapping \
  config:=robosense_lio
