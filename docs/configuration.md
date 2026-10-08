# 配置参考

## 1. 来源与优先级

所有参数的默认值只在 `vsim_core` 中定义一次。配置是一张"点分键 → 字符串值"的表（`vsim::Config`），列表用逗号分隔，矩阵的行用分号分隔。

**独立程序**（`vsim_standalone`、`vsim_can_node`）按以下顺序叠加，后者优先：

1. 核心库默认值（`vsim_standalone --print-defaults` 可导出带注释的完整版本，即 `config/defaults.ini`）；
2. `--model` 预设（`pendulum` / `two_link`）；
3. `--config FILE`（可多次，INI 的 `[a.b]` 段 = 键前缀 `a.b.`）；
4. `--set key=value`（可多次）。

三个单元的键分别在 `sim_host.*`、`ecu.*`、`planner.*` 之下。

**ROS 节点**：YAML 参数文件中每个节点的 `ros__parameters` 就是该单元的配置，键名去掉单元前缀，嵌套即点分层级。例如独立程序里的 `ecu.controller.kp` 对应 `ecu` 节点的参数 `controller.kp`。未写的键使用核心库默认值。launch 时后给出的参数文件覆盖先给出的。

```yaml
ecu:                       # 节点名
  ros__parameters:
    controller:
      kp: [60.0]           # → 配置键 controller.kp = "60"
```

命令行覆盖：`--ros-args -p controller.kp:=[80.0]`。

## 2. sim_host（上位机）

| 键 | 默认 | 说明 |
|---|---|---|
| `plant.model.type` | `pendulum` | `pendulum` \| `two_link` |
| `plant.model.mass / com / inertia / damping / gravity` | 1.0 / 0.5 / 0.01 / 0.05 / 9.81 | 单摆：质量 kg、质心距 m、附加惯量 kg·m²、粘滞摩擦 N·m·s/rad |
| `plant.model.m1 m2 l1 lc1 lc2 i1 i2 b1 b2 gravity` | 1.0 0.8 0.5 0.25 0.2 0.02 0.012 0.05 0.05 9.81 | 两连杆参数（角度自竖直向下量起） |
| `plant.model.joint_names` | `joint1[,joint2]` | 关节名 |
| `plant.drive.effort_limit` | 不限 | 执行器力矩上限 N·m（1 个值广播到所有关节） |
| `plant.drive.torque_time_constant` | 0 | 力矩一阶滞后时间常数 s |
| `plant.drive.position_kp / position_kd / velocity_kp` | 100 / 5 / 5 | 驱动器位置、速度模式的内环增益 |
| `plant.integrator` | `rk4` | `rk4` \| `euler`（半隐式） |
| `sensor.position_noise_std / velocity_noise_std / effort_noise_std` | 1e-4 / 0 / 0 | 高斯噪声标准差 |
| `sensor.position_resolution` | 0 | 编码器分辨率 rad（0 = 不量化） |
| `sensor.delay_steps` | 0 | 测量延迟（仿真步数） |
| `sensor.seed` | 42 | 随机种子（固定则结果逐位可复现） |
| `sim.dt` | 0.001 | 仿真步长 s（= 控制周期） |
| `sim.lockstep` | true | 收到 ECU 回应后才推进 |
| `sim.realtime_factor` | 1.0（`defaults.ini` 中为 0） | 实时倍率；lockstep 下 ≤ 0 = 尽可能快 |
| `sim.resend_timeout` | 0.1 | lockstep：多久没回应就重发当前帧 s |
| `sim.max_lag` | 0.1 | 落后墙钟排程超过该值则重新锚定 s |
| `sim.stop_time` | 0 | 仿真时间到达后停止推进（0 = 不限） |
| `sim.initial_position / initial_velocity` | 0 | 初始状态（每关节） |
| `faults` | 空 | 预置故障场景，见第 6 节 |

仅 `sim_host_node`（ROS）使用的键：

| 键 | 默认 | 说明 |
|---|---|---|
| `io.mode` | `ros` | `ros`：I1，经话题与 ECU 交换；`can`：I2，作为虚拟驱动器挂在 CAN 上 |
| `io.can.interface / codec / node_ids / fd` | `vcan0` / `simple` / `1..n` / false | 与 ECU 侧一致（见第 3 节） |
| `io.can.cia402.*` | | 同 ECU 侧 |
| `ros.sensor_topic / ros.command_topic` | `/sim/sensor_frame` / `/ecu/actuator_command` | 话题名 |
| `viz.joint_state_decimation` | 10 | `/joint_states` 每 N 步发布一次 |

## 3. ecu（嵌入式控制单元）

| 键 | 默认 | 说明 |
|---|---|---|
| `ecu.auto_enable` | true | I/O 就绪后自动使能一次（实机建议 false） |
| `ecu.safe_output` | `damping` | 故障、停用时的输出：`damping`（主动阻尼）\| `disable`（断开力矩） |
| `ecu.safe_damping` | 1.0 | 主动阻尼系数 N·m·s/rad |
| `ecu.ref_blend_time` | 1.0 | 由保持点切入轨迹的五次多项式过渡时间 s（0 = 直接跳到轨迹） |
| `ecu.nominal_dt` | 0.001 | 名义控制周期 s（第一帧与异常 dt 时使用） |
| `ecu.joint_names` | 模型的关节名 | 与轨迹按名字匹配 |
| `model.*` | 同 `plant.model.*` | ECU 对被控对象的"认知"，用于前馈；可以故意与仿真侧不同，用来测鲁棒性 |
| `controller.type` | `joint_pd` | 控制器（在 `create_controller()` 注册新算法） |
| `controller.kp / kd / ki` | 60 / 4 / 0 | 每关节增益（1 个值广播） |
| `controller.integral_limit` | 2.0 | 积分项限幅 N·m |
| `controller.effort_limit` | 8.0 | 输出力矩限幅 N·m |
| `controller.feedforward` | `gravity` | `none` \| `gravity`（g(q)）\| `inverse_dynamics`（M q̈_ref + h） |
| `safety.position_min / position_max` | 不检查 | 位置限位 rad |
| `safety.velocity_max` | 不检查 | 速度限位 rad/s |
| `safety.tracking_error_max` | 0（不检查） | 跟踪误差限 rad，仅在 RUNNING 时检查，针对实际跟踪的参考 |
| `safety.max_partial_frames` | 3 | 某关节连续缺反馈的帧数上限，超出即 `sensor_timeout` |
| `telemetry.decimation` | 1 | 每 N 个周期记录一次遥测 |
| `telemetry.capacity` | 8192 | 遥测队列容量（满了就丢弃并计数） |
| `runtime.read_timeout` | 0.1 | 等传感器帧的超时 s |
| `runtime.max_consecutive_timeouts` | 5 | 通信建立后的连续超时次数 → `sensor_timeout` 故障（第一帧之前只等待） |
| `runtime.cycle_budget` | 0（不检查） | 每周期"计算 + 写出"耗时预算 s |
| `rt.enable` | false | 控制线程实时化（SCHED_FIFO、绑核、mlockall） |
| `rt.required` | false | 实时配置失败时拒绝启动 |
| `rt.priority / cpu / lock_memory / prefault_stack` | 80 / -1 / true / 262144 | |
| `io.backend` | 独立程序 `sim_link`；ROS `ros_sim` | `sim_link` \| `ros_sim` \| `can` \| `ethercat` \| `loopback` |

I/O 后端的键（都在 `io.` 之下）：

| 后端 | 键 |
|---|---|
| `can` | `can.interface`（`vcan0` / `can0` / `virtual:<名字>`）、`can.fd`、`can.codec`（`simple` \| `cia402`）、`can.node_ids`、`can.sync_master`（`bus` \| `ecu`）、`can.period`（ecu 主站周期 s）、`can.feedback_timeout`（默认：bus 主站 0.005 s，ecu 主站半个周期） |
| `can` + `cia402` | `can.cia402.counts_per_rad`（20860.756 = 131072/2π）、`velocity_counts_per_rad_s`、`rated_torque`（8.0 N·m）、`op_mode`（10 = CST）、`sync_with_time`（false；仿真 lockstep 时设 true） |
| `ethercat` | `ethercat.interface`、`slaves`、`period`、`op_mode`、`counts_per_rad`、`rated_torque`、`dc_sync`（预留，见 hardware_integration.md） |
| `loopback` | `period`（0.001）、`realtime`（true = 本机实时节拍）、`damping`、`initial_position` |
| `ros_sim` | `ros.sensor_topic`、`ros.command_topic`、`ros.qos_depth` |

仅 `ecu_node`（ROS）使用的键：

| 键 | 默认 | 说明 |
|---|---|---|
| `safety.estop.type` | `sim` | `none` \| `sim`（软件输入）\| `gpiod`（预留） |
| `safety.estop.chip / line / active_low` | `/dev/gpiochip0` / 0 / true | GPIO 急停输入 |
| `watchdog.type / device / timeout` | `none` / `/dev/watchdog` / 1.0 | 硬件看门狗 |
| `ros.trajectory_topic` | `/planner/trajectory` | |

## 4. planner（高级规划单元）

| 键 | 默认 | 说明 |
|---|---|---|
| `type` | `sine` | `sine` \| `waypoints`（在 `create_planner()` 注册新算法） |
| `joint_names` | `joint1` | 轨迹中的关节名（ECU 按名字匹配） |
| `amplitude / frequency / phase / offset` | 1 / 1 / 0 / 0 | sine：`q = offset + amplitude·sin(frequency·t + phase)`，frequency 单位 rad/s |
| `waypoints` | — | waypoints：`"0.0, 0.0; 0.8, -0.6; -0.5, 0.9"`（行 = 路径点，列 = 关节） |
| `segment_time` | 2.0 | 每段时长 s（1 个值广播） |
| `loop` | true | 走完后回到第一个点循环 |
| `scheduler.period / horizon / step` | 0.2 / 1.0 / 0.05 | 重规划周期、规划时域、轨迹点网格 s |
| `ros.trajectory_topic`（仅 ROS） | `/planner/trajectory` | |

## 5. monitor_node 与 launch 参数

`monitor_node`（无界面健康检查）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `warmup` | 2.0 | 从收到第一帧起跳过的仿真时间 s（与 `vsim_standalone --settle` 一致） |
| `duration` | 0 | 观测窗口 s；> 0 时观测满后输出 PASS/FAIL 并以 0/1 退出 |
| `max_rms` | 0 | 各关节 RMS 误差上限 rad（0 = 只要求 ECU 始终 RUNNING） |
| `timeout` | 60 | 墙钟超时 s |
| `report_period` | 1.0 | 打印间隔 s |

`vsim.launch.py`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `model` | `pendulum` | 加载 `config/<model>.yaml` |
| `io` | `ros` | `ros` \| `can`（叠加 `can_overlay.yaml`） |
| `can_iface` | 空 | io=can 时覆盖 CAN 接口名（跨容器 vxcan 用 `can0`） |
| `units` | `sim_host,ecu,planner` | 要启动的单元；可加 `monitor` |
| `params_file` | 空 | 额外的覆盖文件 |

## 6. 预置故障场景

`sim_host.faults`（独立程序）或 `sim_host` 节点的参数 `faults`，是一个字符串：

```text
类型 键=值 ...; 类型 键=值 ...
```

| 键 | 默认 | 说明 |
|---|---|---|
| `joint` | -1 | 关节序号，-1 = 全部关节 |
| `value` | 0 | 故障数值（见下表） |
| `start` | 0 | 起始仿真时间 s |
| `duration` | 0 | 持续时间 s，0 = 一直持续 |

| 类型 | `value` 的含义 |
|---|---|
| `sensor_freeze` | —（保持故障前读数，置 `stale` 标志） |
| `sensor_bias` | 位置读数偏置 rad |
| `sensor_dropout` | —（读数为 NaN，置编码器故障位） |
| `actuator_weak` | 出力缩放系数（0.7 = 剩 70%） |
| `actuator_stuck` | —（力矩卡在故障发生时的值） |
| `torque_disturbance` | 外加扰动力矩 N·m |

示例（`config/robustness.ini`）：

```ini
[sim_host]
faults = torque_disturbance joint=0 value=1.5 start=4 duration=0.3; actuator_weak joint=0 value=0.7 start=7 duration=2
```

运行中也可以经 `/sim_host/inject_fault` 服务注入（`start_delay` 相对当前仿真时间）。
