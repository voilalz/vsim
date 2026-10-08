# ECU 控制器内部状态（可视化 / 录制 / 调参用）。header.stamp = 对应传感器帧的采样时刻。

std_msgs/Header header
string[] name
float64[] position_reference
float64[] velocity_reference
float64[] position
float64[] velocity
float64[] effort
float64[] error
uint8 state
uint32 faults
