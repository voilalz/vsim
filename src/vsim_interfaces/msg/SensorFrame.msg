# 一个控制周期的传感器输入（= 真实 ECU 的输入 I/O）。物理量一律 SI 单位。
# header.stamp = 采样时刻（仿真时间或硬件同步时刻），不是发送时刻。

uint32 FLAG_ENCODER_FAULT=1
uint32 FLAG_DRIVE_FAULT=2
uint32 FLAG_STALE=4
uint32 FLAG_PARTIAL=8

std_msgs/Header header
uint64 seq

string[] name
float64[] position    # rad
float64[] velocity    # rad/s
float64[] effort      # N·m

bool imu_valid
geometry_msgs/Quaternion imu_orientation
geometry_msgs/Vector3 imu_angular_velocity       # rad/s
geometry_msgs/Vector3 imu_linear_acceleration    # m/s^2

uint32 flags
