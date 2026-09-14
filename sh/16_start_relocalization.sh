#!/usr/bin/env bash

set -u

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：没有找到 gnome-terminal，请先安装或在 GNOME 桌面终端中运行。" >&2
  exit 1
fi

gnome-terminal \
  --title="Route3D Relocalization" \
  -- bash -lc '
    cd /opt/mapping_ws || {
      echo "无法进入 /opt/mapping_ws"
      exec bash
    }
    if [[ -f install/setup.bash ]]; then
      source install/setup.bash
    else
      echo "警告：未找到 /opt/mapping_ws/install/setup.bash，将使用当前 ROS 环境"
    fi
    echo "[mapping] ./run_mapping_nodes_2.sh mode:=localization config:=robosense_loc_2"
    ./run_mapping_nodes_2.sh mode:=localization config:=robosense_loc_2 
    exit_code=$?
    echo "[mapping] 进程已退出，状态码：${exit_code}"
    exec bash
  ' &

wait
