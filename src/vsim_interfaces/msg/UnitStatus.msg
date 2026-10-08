# 单元运行状态（ECU / sim_host / planner 都发布），供其他单元与 HMI 使用。

uint8 STATE_INIT=0
uint8 STATE_STANDBY=1
uint8 STATE_RUNNING=2
uint8 STATE_FAULT=3
uint8 STATE_ESTOP=4

std_msgs/Header header
string unit
uint8 state
string state_name
uint32 faults
string[] fault_names
uint64 cycles
uint64 overruns
float64 cycle_time_avg_us
float64 cycle_time_max_us
string message
