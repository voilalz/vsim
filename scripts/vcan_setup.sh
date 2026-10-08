#!/usr/bin/env bash
# 在 Linux 主机（或带 NET_ADMIN 的特权容器）上创建虚拟 CAN 接口（同机多进程测试用）。
#
# 用法：sudo scripts/vcan_setup.sh [IFACE] [--fd]      默认 vcan0；--fd 允许 CAN FD 帧（MTU 72）
# 之后：
#   vsim_can_node --role sim --iface vcan0 & vsim_can_node --role ecu --iface vcan0 --duration 10
#   或 ROS：ros2 launch vsim_ros vsim.launch.py io:=can
#   抓包：candump -td vcan0
set -euo pipefail

IFACE=vcan0
MTU=16
for arg in "$@"; do
  case "$arg" in
    --fd) MTU=72 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) IFACE="$arg" ;;
  esac
done

if [[ $EUID -ne 0 ]]; then exec sudo "$0" "$@"; fi

modprobe -a can can_raw vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  ip link add dev "$IFACE" type vcan
fi
ip link set "$IFACE" down
ip link set "$IFACE" mtu "$MTU"
ip link set "$IFACE" up
ip -details link show "$IFACE"
