#!/bin/bash
# 容器入口：加载 ROS 与 vsim 工作空间环境后执行命令（compose 里的 command）。
set -e
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${VSIM_PREFIX:-/ws/install}/setup.bash"

# io=can：跨容器的 vxcan 链路由宿主在容器启动之后创建（scripts/vcan_link.sh），先等接口出现
if [[ "${VSIM_IO:-ros}" == "can" && -n "${VSIM_CAN_IFACE:-}" ]]; then
  echo "[vsim] waiting for CAN interface '${VSIM_CAN_IFACE}' (run: sudo scripts/vcan_link.sh on the host) ..."
  until ip link show "${VSIM_CAN_IFACE}" >/dev/null 2>&1; do sleep 0.5; done
  echo "[vsim] ${VSIM_CAN_IFACE} is up"
fi
exec "$@"
