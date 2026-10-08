#!/usr/bin/env bash
# 在 compose 启动的 sim_host 与 ecu 两个容器之间拉一条虚拟 CAN 线（vxcan 隧道），
# 两端在各自容器里都叫 can0 —— ECU 的 SocketCAN 驱动代码可以原样测试（接口层级 I2）。
#
# 前提：Linux 宿主；容器已启动（docker compose up -d）；宿主内核有 vxcan 模块。
# 用法（在仓库根目录）：sudo scripts/vcan_link.sh [SIM_SERVICE=sim_host] [ECU_SERVICE=ecu]
# 然后两个容器都用 can_overlay.yaml，并把其中的 vcan0 改为 can0（或 -p io.can.interface:=can0）。
# 注意：容器重启后 netns 会变，需要重新执行本脚本。
set -euo pipefail

SIM=${1:-sim_host}
ECU=${2:-ecu}
if [[ $EUID -ne 0 ]]; then exec sudo "$0" "$@"; fi
cd "$(dirname "${BASH_SOURCE[0]}")/.."

modprobe -a can can_raw vxcan
pid() {
  local id
  id=$(docker compose ps -q "$1")
  [[ -n "$id" ]] || { echo "service '$1' is not running (docker compose up -d first)" >&2; exit 1; }
  docker inspect -f '{{.State.Pid}}' "$id"
}
SIM_PID=$(pid "$SIM")
ECU_PID=$(pid "$ECU")

# 先清理上一次留下的同名接口（容器里的 can0 会随 netns 一起消失）
ip link del vxcan_ecu 2>/dev/null || true
ip link add vxcan_ecu type vxcan peer name vxcan_sim
ip link set vxcan_ecu netns "$ECU_PID"
ip link set vxcan_sim netns "$SIM_PID"
nsenter -t "$ECU_PID" -n ip link set vxcan_ecu name can0
nsenter -t "$SIM_PID" -n ip link set vxcan_sim name can0
nsenter -t "$ECU_PID" -n ip link set can0 up
nsenter -t "$SIM_PID" -n ip link set can0 up
echo "vxcan link up: $SIM:can0 <-> $ECU:can0"
echo "check: docker compose exec $ECU candump can0   |   docker compose exec $SIM cansend can0 123#DEADBEEF"
