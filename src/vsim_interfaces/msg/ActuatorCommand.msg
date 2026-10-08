# 一个控制周期的执行器输出（= 真实 ECU 的输出 I/O）。
# header.stamp 回显它所响应的 SensorFrame.header.stamp —— sim_host 据此做 lockstep 握手。

uint8 MODE_DISABLED=0
uint8 MODE_EFFORT=1
uint8 MODE_VELOCITY=2
uint8 MODE_POSITION=3

std_msgs/Header header
uint64 seq
uint8 mode

float64[] effort      # N·m
float64[] velocity    # rad/s
float64[] position    # rad

bool reset_faults     # 本周期请求驱动器故障复位（操作者复位时置位一个周期；驱动器故障绝不自动复位）
