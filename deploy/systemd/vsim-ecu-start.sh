#!/bin/bash
# vsim-ecu.service 的启动包装：加载 ROS 与 vsim 安装环境后运行 ECU 节点。
set -eo pipefail
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
# shellcheck disable=SC1091
source "${VSIM_PREFIX:-/opt/vsim/install}/setup.bash"
exec ros2 run vsim_ros ecu_node --ros-args --params-file "${VSIM_ECU_PARAMS:-/etc/vsim/ecu.yaml}"
