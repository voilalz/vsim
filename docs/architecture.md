# 架构说明

## 1. 设计原则

1. **核心与中间件分离。** 模型、仿真、控制、规划、ECU 逻辑、硬件抽象全部放在纯 C++17 的 `vsim_core` 里，不依赖 ROS。这样它可以单测、做蒙特卡洛批量仿真、交叉编译到板子上。ROS 2 层（`vsim_ros`）只负责消息翻译、参数装载和进程托管。
2. **时间由外部注入。** 所有时间都是 `int64` 纳秒（`vsim::TimeNs`）。算法代码从不读系统时钟，时间都来自传感器帧的时间戳：仿真中是仿真时间，实机上是 ECU 的单调时钟。需要墙钟的地方（节拍、超时）通过 `IClock` 注入，测试时换成 `ManualClock`。
3. **可复现。** 默认 lockstep：仿真侧收到 ECU 对当前帧的回应后才推进一步，结果与算力、网络延迟、进程调度无关。固定随机种子时逐位一致。
4. **实时安全。** ECU 的控制线程不分配内存、不阻塞在锁上。与非实时线程交换数据有三种方式：
   - 轨迹：`try_lock` 邮箱，实时侧取不到锁就下个周期再取；
   - 请求、急停：原子量；
   - 遥测：SPSC 无锁队列。
5. **单一配置来源。** 所有参数的默认值只在核心库里定义一次。独立程序读 INI，ROS 节点读 YAML 参数，两者映射到同一套点分键名（见 [configuration.md](configuration.md)）。
6. **仿真与实机走同一条代码路径。** 同一个驱动器协议编解码器既用于 ECU（主站），也用于仿真侧的"虚拟驱动器"（从站）。所以 ECU 的 CAN 驱动在接真实驱动器之前，就已经在虚拟总线上以同样的帧格式跑过了。

## 2. 分层

```text
进程 / 托管   vsim_ros：sim_host_node · ecu_node · planner_node · monitor_node
              apps：vsim_standalone · vsim_can_node
────────────────────────────────────────────────────────────────────────────────
装配          system/LocalSystem（单进程三单元） · *_factory（按配置创建）
────────────────────────────────────────────────────────────────────────────────
单元          sim/       SimEngine · SimRunner · SensorModel · FaultInjector
              ecu/       EcuApp · EcuRuntime · SafetyMonitor · EcuStateMachine
              planning/  IPlanner · PlannerScheduler
────────────────────────────────────────────────────────────────────────────────
构件          model/     IJointDynamics（M(q)、h(q, q̇)、g(q)）；sim/ IPlant
              control/   IController · TrajectorySampler
              hal/       IoBackend · ICanBus · IMotorCodec · IDigitalInput · IWatchdog
              rt/        RtThreadConfig · PeriodicTimer
────────────────────────────────────────────────────────────────────────────────
基础          common/    time · status · config · logging · spsc_queue · stats
              msg/       数据类型（SensorFrame · ActuatorCommand · Trajectory …）
```

依赖只能自上而下。`msg/types.hpp` 与 `vsim_interfaces` 的消息一一对应，翻译集中在 `vsim_ros/conversions.hpp`。

## 3. 单元之间的接口契约

| 话题 | 类型 | 方向 | 频率 | 说明 |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim_host → 全部 | 每帧 | 仿真时间；ecu、planner 用 `use_sim_time: true` |
| `/sim/sensor_frame` | `vsim_interfaces/SensorFrame` | sim_host → ecu | 1 kHz | 一个控制周期的全部输入；`header.stamp` = 采样时刻 |
| `/ecu/actuator_command` | `vsim_interfaces/ActuatorCommand` | ecu → sim_host | 1 kHz | `header.stamp` 回显所响应的帧时刻（lockstep 握手） |
| `/planner/trajectory` | `trajectory_msgs/JointTrajectory` | planner → ecu | 5 Hz | 覆盖未来 1 s，点对齐到 50 ms 网格；`header.stamp` = 轨迹起点（绝对时间） |
| `/ecu/status` | `vsim_interfaces/UnitStatus` | ecu → planner/HMI | 10 Hz | 状态、故障名、周期数、超时次数、周期耗时 |
| `/ecu/controller_state` | `vsim_interfaces/ControllerState` | ecu → 可视化/录制 | 控制频率 / `telemetry.decimation` | 参考、实际、误差、力矩 |
| `/sim/status` | `vsim_interfaces/UnitStatus` | sim_host → HMI | 2 Hz | 仿真时间、重发、排程重锚、被拒指令、活动故障数 |
| `/joint_states` | `sensor_msgs/JointState` | sim_host → 可视化 | 控制频率 / `viz.joint_state_decimation` | 仿真真值（不含传感器噪声与故障） |

| 服务 | 类型 | 作用 |
|---|---|---|
| `/ecu/set_mode` | `vsim_interfaces/SetEcuMode` | 1 使能、2 停用、3 复位故障、4 软件急停、5 释放急停；在下一个控制周期生效 |
| `/sim_host/inject_fault` | `vsim_interfaces/InjectFault` | 注入故障：类型、关节、数值、延迟、持续时间 |
| `/sim_host/clear_faults` | `std_srvs/Trigger` | 清除所有注入的故障 |
| `/sim_host/reset` | `std_srvs/Trigger` | 仿真回到 t = 0（ECU 与 planner 会检测到时间回退） |

## 4. 时间与同步

### 4.1 lockstep 握手

```text
sim_host                                   ecu
  │ SensorFrame(stamp = t) ─────────────────▶ │ step()：采样轨迹 → 控制 → 安全 → 状态机
  │ ◀──────────────── ActuatorCommand(stamp = t)
  │ 接受（stamp == t 且本步尚未收到回应）
  │ 积分 [t, t+dt] → SensorFrame(t+dt) ─────▶ │ …
```

各种异常情况的处理：

- **sim_host 收到坏指令：** 时间戳不等于当前时刻，或本步已收到过回应，这类指令直接拒绝，计入 `rejected`。
- **超时未回应：** 超过 `sim.resend_timeout` 没收到回应，sim_host 重发当前帧。
  - ROS 话题：ECU 收到与上一帧同一时刻的帧，原样重发上一条指令（幂等），丢包、ECU 迟启动都能自动恢复。
  - CAN + `simple` 协议：指令帧带 SYNC 标签（所回应帧时刻的微秒数低 16 位）。ECU 照常回应重发，仿真侧按标签丢弃对旧帧的回应。所以丢帧（回应丢失）与重发堆积（ECU 一时太慢）都能自动恢复，lockstep 不会错位。
  - CAN + `cia402`（RPDO 没有空位放标签）：ECU 不回应重发，否则重复回应会被当成对下一帧的回应。代价是：仿真模式下一旦丢帧，lockstep 会停住。实机用 `sync_master: ecu`，不受影响。
  - 仿真侧重发同一帧时，保留该帧已经收到的各关节指令。
- **帧时间回退：** 只可能是仿真复位，因为传输层保证单写者、按序送达。ECU 按"虚拟上电"处理：
  1. 状态机回到 INIT，锁存故障清除；
  2. 按配置重新自动使能；
  3. 以当前位置为保持点。

  所以复位后的场景与冷启动完全一致。planner 检测到时间回退后重新开始调度。

### 4.2 节拍（`SimRunner`）

- **`realtime_factor > 0`：** 按墙钟排程推进，仿真时刻 t 的计划到达时间为 `anchor_wall + (t − anchor_sim) / realtime_factor`：
  - 回应在到期前到达，就等到期的 tick 再推进；
  - 已经落后于排程，收到回应立即推进来追赶；
  - 落后超过 `sim.max_lag`（例如等待 ECU 启动、进程被调试器暂停），则重新锚定排程，不做突发追赶，并计入 `schedule_resets`。

  因此定时器抖动、偶尔漏掉的 tick 不会让实时倍率系统性偏低。
- **`realtime_factor <= 0`（仅 lockstep）：** 收到回应立即推进，尽可能快，用于批量回归。
- **自由运行（`lockstep: false`）：** 每个 tick 补齐排程上已到期的步数（单次最多 10 步），指令零阶保持。

### 4.3 实机时间基准

仿真后端的帧时间就是仿真时间，与 `/clock` 一致。实机后端的帧时间是 ECU 本机的 `CLOCK_MONOTONIC`，由周期定时器产生，不受 NTP 跳变影响。实机后端包括：CAN 且 `sync_master: ecu`、EtherCAT、实时 loopback。

规划器在实机上按系统时间打时间戳（多机之间用 NTP/PTP 对齐）。`IoBackend::time_base()` 告诉上层帧时间属于哪种基准，`ecu_node` 据此换算：

- 收到的轨迹：`header.stamp` 从系统时间换算到单调时钟；实机上 `stamp = 0` 按 ROS 惯例表示"立即开始"。
- 发出的状态与遥测：时间戳换算回系统时间。

## 5. ECU 内部

### 5.1 线程

| 线程 | 做什么 | 约束 |
|---|---|---|
| 控制线程（`EcuRuntime`） | `read → step → write` 循环；节拍由 I/O 后端决定 | 不分配、不阻塞在锁上；可选 SCHED_FIFO + CPU 绑定 + mlockall |
| ROS 执行器线程 | 轨迹订阅 → `post_trajectory()`（在非实时侧完成预处理）；服务 → `post_request()`；急停输入轮询；看门狗喂狗；遥测出队发布；状态发布 | 可以分配、可以阻塞 |
| `RosSimIo` 私有执行器线程 | 只处理传感器帧订阅，放入条件变量邮箱，唤醒阻塞在 `read()` 上的控制线程 | 仅仿真后端 |

看门狗按控制循环的迭代计数喂，I/O 超时的迭代也算。所以进程活着但控制线程卡死时，硬件看门狗会让板子复位；而总线断开、驱动器还在上电时，控制线程仍在正常输出安全态，不会被误复位。

控制线程不写日志：状态切换、虚拟上电、无效请求都作为事件放进无锁队列，由非实时线程取出后格式化并记录（`EcuApp::log_events()`）。周期耗时统计同样只在控制线程里累加，它用 `try_lock` 发布快照，从不等待读者。

### 5.2 一个控制周期（`EcuApp::step`）

1. **帧检查：** 与上一帧同一时刻，说明是重发，返回上次输出；时间回退则虚拟上电。
2. **取新轨迹与请求：**
   - 轨迹：`try_lock` 邮箱，O(1) 交换。新轨迹的起点若还在未来、而当前轨迹仍有效，就先沿当前轨迹走，到起点再切换，避免规划器时钟略快时参考被反复冻结。
   - 操作请求：按到达顺序排队，每个周期处理一个，所以"急停之后紧跟着使能"不会冲掉急停。ECU 还在 INIT（没处理过任何一帧）时拒绝请求，免得积压的旧请求在通信建立后突然生效。
3. **采样与参考生成：** 轨迹采样（线性或三次 Hermite 插值），然后生成**控制器实际跟踪的参考**：
   - 没有轨迹、轨迹起点在未来、或轨迹已用尽：跟踪保持点（进入运行时的实测位置，或轨迹中断时冻结的参考）；
   - 从保持点切入轨迹：在 `ecu.ref_blend_time` 内用五次多项式平滑过渡，位置、速度、加速度都连续。以下情况即使轨迹离当前位置很远，也不会出现参考跳变：
     - 复位后重新使能；
     - 规划器中断后又发来新轨迹。
4. **安全检查：**
   - 帧有效性：尺寸、非有限值、时间戳递增、编码器故障位；
   - 位置、速度限位；
   - 跟踪误差，针对第 3 步的参考计算，所以过渡段不会误报；
   - 驱动器故障、急停输入、周期超时告警。
5. **状态机：**

   ```text
   INIT ──I/O 就绪──▶ STANDBY ──使能──▶ RUNNING ──停用──▶ STANDBY
     任何状态 ──严重故障──▶ FAULT ──复位（且故障已消失）──▶ STANDBY
     任何状态 ──急停──▶ ESTOP ──释放──▶ FAULT（急停锁存）──复位──▶ STANDBY
   ```

   严重故障（低 16 位）锁存，直到复位；警告（高 16 位）只反映当前情况，不改变状态。`auto_enable` 在 I/O 就绪后只自动使能一次，之后的停用与复位由操作者决定。
6. **输出：**
   - RUNNING：控制器输出（PD/PID + 重力前馈或逆动力学前馈，带力矩限幅）。输出里若出现 NaN/Inf（算法缺陷），立即锁存 `controller_output` 故障，本周期就改发安全输出；
   - 其他状态：安全输出，二选一——主动阻尼 `τ = −c·q̇`，或断开力矩；
   - 驱动器故障复位：操作者复位（RESET_FAULT）后的 0.5 s 窗口内，指令里带 `reset_faults`，只有这时才向驱动器发故障复位（CiA 402 控制字 bit7 上升沿）；驱动器报故障后绝不自动复位。窗口内状态机持续尝试复位，驱动器几个周期后清掉故障位时，按一次复位就够了。
7. **遥测：** 按 `telemetry.decimation` 写入 SPSC 队列；队列满了就丢弃并计数，绝不阻塞。

### 5.3 故障位

| 位 | 名称 | 含义 |
|---|---|---|
| 0 | `sensor_timeout` | 通信建立后，连续 `runtime.max_consecutive_timeouts` 个周期没有传感器帧；或某个关节连续 `safety.max_partial_frames` 帧缺反馈。收到第一帧之前的超时不算：此时状态保持 INIT，等对端启动 |
| 1 | `sensor_invalid` | 帧尺寸不符、非有限值、时间戳不递增、编码器故障位 |
| 2 | `position_limit` | 超出 `safety.position_min/max` |
| 3 | `velocity_limit` | 超出 `safety.velocity_max` |
| 4 | `tracking_error` | \|参考 − 实际\| > `safety.tracking_error_max`（仅 RUNNING） |
| 5 | `estop` | 急停输入或软件急停（锁存） |
| 6 | `io_error` | I/O 后端报错（总线错误等） |
| 7 | `drive_fault` | 驱动器报故障 |
| 8 | `controller_output` | 控制器输出非有限值（NaN/Inf） |
| 16 | `trajectory_stale`（警告） | 轨迹已用尽，保持末端位置 |
| 17 | `cycle_overrun`（警告） | 1 s 内出现过"计算 + 写出"超过 `runtime.cycle_budget` 的周期 |
| 18 | `sensor_stale`（警告） | 传感器数据冻结 |

## 6. 被控对象仿真

- **`ArticulatedPlant`：** 求解 `M(q) q̈ + h(q, q̇) = τ_motor + τ_ext`，积分器可选 RK4 或半隐式 Euler。驱动器模型包括：
  - 力矩限幅；
  - 一阶力矩滞后；
  - 位置、速度模式的内环；
  - 执行器故障：出力衰减、卡死。

  模型有两种：单摆，以及两连杆平面臂（角度自竖直向下量起）。能量守恒有单元测试。
- **`SensorModel`：** 高斯噪声、量化、整数步延迟、固定种子。
- **`FaultInjector`：** 六类故障：`sensor_freeze`、`sensor_bias`、`sensor_dropout`、`actuator_weak`、`actuator_stuck`、`torque_disturbance`。可以在配置里预置，按仿真时间发生、可复现；也可以运行中经服务注入。

## 7. 规划

- `IPlanner::plan(t0, horizon, step)` 生成对齐网格的轨迹点，含位置、速度、加速度。
- `PlannerScheduler` 决定何时重规划：`period`、`horizon`、`step`。其中 `t0 = ⌊now/step⌋·step`，所以相邻两次规划的轨迹点落在同一网格上，切换时参考连续。
- 已实现两种规划器：
  - `sine`：`q = offset + A·sin(ω t + φ)`，ω 单位 rad/s；
  - `waypoints`：各路径点之间用最小加加速度五次多项式连接，端点速度、加速度为 0，可循环。

## 8. 接口层级与 I/O 后端

| 层级 | ECU 与"世界"之间 | 后端（`io.backend`） | 节拍主站 |
|---|---|---|---|
| I0 | 进程内函数调用 | `sim_link`（独立程序） | 仿真 |
| I1 | ROS 2 话题 | `ros_sim` | sim_host（lockstep） |
| I2 | CAN 帧：进程内虚拟总线、`vcan`、跨容器 `vxcan` | `can` + `sync_master: bus` | sim_host 发带时间的 SYNC |
| I3 | 真实总线与驱动器：`can0`、EtherCAT | `can` + `sync_master: ecu`，或 `ethercat` | ECU 周期定时器 |
| — | 无被控对象的回环假硬件 | `loopback` | ECU 周期定时器（或虚拟时间） |

CAN 协议：

- **`simple`**（vsim 简单协议 v1，仿真与自研驱动器用），均为 8 字节小端：
  - SYNC `0x080`：int64 时间；
  - 指令 `0x200+node`：模式、标志（使能、故障复位）、定点值、SYNC 标签；
  - 反馈 A `0x180+node`：位置、速度；
  - 反馈 B `0x280+node`：力矩、状态。
- **`cia402`**（CANopen CiA 402 伺服）：
  - PDO：RPDO1 = 控制字 + 目标力矩 + 目标位置；TPDO1 = 状态字 + 实际位置 + 实际力矩；TPDO2 = 实际速度；
  - 启动：NMT Start，SDO 写 `0x6060` 运行模式；
  - 主站按 CiA 402 状态机逐步使能驱动器：Shutdown → Switch On → Enable Operation，出现故障时发 Fault Reset；
  - 仿真时 SYNC 可以携带时间（`sync_with_time`，非标准）以支持 lockstep。

## 9. 测试策略

| 层次 | 内容 | 位置 |
|---|---|---|
| 单元 | 配置解析、SPSC 队列、线性代数、动力学（能量守恒、逆动力学一致性）、轨迹采样边界、规划器、调度网格、传感器与故障模型、仿真推进与重发、节拍排程、状态机、安全监控、参考过渡、编解码往返、CiA 402 状态序列、虚拟总线、实时配置 | `src/vsim_core/test` |
| 闭环 | 单进程三单元：跟踪精度、逐位确定性、CAN 路径与直连路径一致、CiA 402 闭环、跟踪误差故障触发 | `test_closed_loop.cpp` |
| 并发 | ASan/UBSan、TSan 下跑全部测试 | `scripts/build_core.sh --asan / --tsan` |
| 端到端 | 三个 ROS 节点 + `monitor_node` 判据（跟踪误差、ECU 状态） | `scripts/smoke_test.sh`、compose 的 `check` |
| 场景 | 预置故障、模型失配、噪声与延迟、蒙特卡洛（改随机种子） | `config/robustness.ini` + `vsim_standalone --max-rms` |
