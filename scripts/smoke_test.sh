#!/usr/bin/env bash
# ROS 2 端到端冒烟测试：同机启动三个单元（lockstep）+ monitor_node，按跟踪误差给出 PASS/FAIL（退出码 0/1）。
#
# 前提：已 source ROS 2 与工作空间（source install/setup.bash）。
# 用法：scripts/smoke_test.sh [pendulum|two_link] [观测时长/秒，默认 10] [RMS 上限/rad，默认 0.01]
# 没有 ros2 命令行工具的环境：设置 VSIM_ROS_PREFIX=<工作空间>/install（或 vsim_ros 的安装前缀）。
set -euo pipefail

MODEL=${1:-pendulum}
DURATION=${2:-10}
MAX_RMS=${3:-0.01}
PREFIX=${VSIM_ROS_PREFIX:-$(ros2 pkg prefix vsim_ros)}
BIN="$PREFIX/lib/vsim_ros"
PARAMS="$PREFIX/share/vsim_ros/config/$MODEL.yaml"
[[ -f "$PARAMS" ]] || { echo "no such model config: $PARAMS" >&2; exit 2; }

LOG_DIR=$(mktemp -d -t vsim_smoke.XXXXXX)
PIDS=()
cleanup() {
  [[ ${#PIDS[@]} -gt 0 ]] || return 0
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 1
  kill -KILL "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT

for unit in sim_host ecu planner; do
  "$BIN/${unit}_node" --ros-args --params-file "$PARAMS" >"$LOG_DIR/$unit.log" 2>&1 &
  PIDS+=($!)
done

set +e
"$BIN/monitor_node" --ros-args -p duration:="$DURATION" -p max_rms:="$MAX_RMS" \
  -p timeout:=$((DURATION * 3 + 30))
rc=$?
set -e
if [[ $rc -ne 0 ]]; then
  for unit in sim_host ecu planner; do
    echo "---- $unit ($LOG_DIR/$unit.log)"
    tail -n 20 "$LOG_DIR/$unit.log"
  done
fi
exit $rc
