# 接入真实硬件

原则：算法（控制器、规划器）、状态机、安全监控都不改动，只替换 ECU 的 I/O 后端（`io.backend`），再调整配置。每向真实世界迈一步，都先在上一级的虚拟环境里把同一份代码跑通。

## 1. 推进路线

| 阶段 | 真实的部分 | 仍是仿真的部分 | 在本工程中怎么做 |
|---|---|---|---|
| S0 算法（MIL/SIL） | 无 | 全部 | `vsim_standalone`；ROS 三节点（I1） |
| S1 总线级 | ECU 的 CAN 驱动与协议编解码 | 驱动器与被控对象（虚拟驱动器） | 同机 `vcan0`，或跨容器 `vxcan`：`io.backend: can` + `sync_master: bus` |
| S2 处理器在环（PIL） | 目标板 CPU、操作系统与实时性、交叉编译的 ECU 程序 | 被控对象在上位机 | 板子上跑 `ecu_node`，经以太网（I1）或真实 CAN 线（I2）连上位机的 `sim_host_node` |
| S3 硬件在环（HIL） | 真实驱动器与电机（空载 / 机械固定） | 负载、场景 | `ecu_hardware.yaml`：`sync_master: ecu`、手动使能、小力矩限幅 |
| S4 整机 | 全部 | 无 | 同 S3，并打开完整的安全配置 |

S2 经真实 CAN 线的接法：上位机插 USB-CAN 适配器（SocketCAN 驱动，比如 `can0`），跑 `sim_host_node`，设置 `io.mode: can`、`io.can.interface: can0`；板子的 `can0` 用双绞线接过来，跑 `ecu_node`，设置 `io.backend: can`、`sync_master: bus`。

这样，上位机的"虚拟驱动器"就成了板子眼中的真实驱动器：帧格式、节点号、波特率都与实机一致，而且仍然 lockstep、可复现。

- 两端都要设置相同的 `bitrate`，并在总线两端接 120 Ω 终端电阻。
- 这种接法建议用 `simple` 协议：它的指令帧带 SYNC 标签，偶发丢帧、重发都能自动恢复。
- `cia402` 在这种 lockstep 接法下，一旦丢帧就会停住（见 architecture.md 第 4.1 节）。

> 仿真侧的虚拟驱动器目前只支持"仿真侧发 SYNC"（`sync_master: bus`）。如果要让 ECU 用自己的定时器做主站、对着虚拟驱动器测实时行为，需要给 `CanMotorEmulator` 增加"收到 SYNC 即回反馈"的模式，这部分尚未实现。

## 2. CAN（SocketCAN）——已实现

### 2.1 接口

- 接口名直接写在 `io.can.interface`：`can0`、`vcan0`、`vxcan*`，或者 `virtual:<name>`（进程内总线，只用于测试）。
- 波特率与总线恢复由系统配置，不在程序里：
  - 开发时：`ip link set can0 type can bitrate 1000000 restart-ms 100 && ip link set can0 up`；
  - 部署时：`deploy/systemd/80-can0.network`，由 systemd-networkd 在开机时拉起。
- 诊断：`candump -td can0`、`ip -details -statistics link show can0`（错误计数、bus-off 次数）。

### 2.2 驱动器协议

- **`simple`：** vsim 自定义协议，适合自研驱动器，或者作为定制协议的模板。
- **`cia402`：** CANopen 伺服，是接商用驱动器的起点。默认 PDO 映射见 `cia402_codec.hpp` 文件头。接入时逐项核对：
  1. **PDO 映射**必须与驱动器实际配置一致。不一致时，要么改编解码器，要么在 `startup_frames()` 里用 SDO 重新映射（写 `0x1600/0x1A00…`），并按厂商手册设置 `0x1400/0x1800` 的通信参数（传输类型 1 = 每个 SYNC 同步一次）；
  2. **单位**：`counts_per_rad`（编码器每弧度计数）、`velocity_counts_per_rad_s`、`rated_torque`（对象 `0x6076`，力矩按其千分比传输）；
  3. **运行模式** `op_mode`：10 = CST 周期同步力矩（默认），8 = CSP 周期同步位置。对应地，ECU 控制器输出力矩或位置；
  4. **节点号** `node_ids` 与关节顺序一一对应；
  5. **急停链路**：驱动器的 STO 必须由硬件急停回路直接切断，软件急停只是补充；
  6. **通信丢失反应**：在驱动器侧配置心跳或 SYNC 监控（`0x1016` / `0x1006` 等，按厂商手册），以及 `0x6007`（通信中断时的动作）。这样 ECU 崩溃、断电、线缆断开时，驱动器会自己停机。

  驱动器故障不会被自动复位：只有操作者经 `/ecu/set_mode` 复位 ECU 故障之后的 0.5 s 窗口内，才会发出 CiA 402 故障复位（一次上升沿）。故障原因仍在时驱动器保持故障，不会反复复位。

  ECU 正常停止时（SIGINT、`systemctl stop`），`deactivate()` 会向所有节点发两轮"禁用"指令。ECU 做 SYNC 主站时，每轮后面补发一个 SYNC，确保同步 PDO 生效。
- **新协议：** 实现 `IMotorCodec`，在 `create_motor_codec()` 注册即可。建议按下面的顺序验证：
  1. 编码 / 解码往返单元测试（参照 `test_can.cpp`）；
  2. 在进程内虚拟总线上与 `CanMotorEmulator`（同一个编解码器）跑闭环（参照 `ClosedLoop.Cia402CodecOverVirtualCan`）；
  3. 在 vcan 上跑；
  4. 最后接真实驱动器。

### 2.3 周期主站与时间

- **`sync_master: ecu`**（实机）：ECU 按 `can.period` 用绝对时间定时器（`CLOCK_MONOTONIC`）发 SYNC，并在 `can.feedback_timeout`（默认半个周期，纳秒级等待）内收齐各节点反馈。
  - 发 SYNC 之前，先丢弃上一周期迟到的帧，计入诊断的 `late_frames`，避免一次迟到造成永久一拍延迟；
  - 某个关节缺反馈时置 `kSensorPartial`，连续 `safety.max_partial_frames` 帧缺失即 `sensor_timeout` 故障；
  - 通信建立后，连续 `runtime.max_consecutive_timeouts` 个周期一帧都没有收到，同样是 `sensor_timeout`；
  - 上电时驱动器还没回应，ECU 保持 INIT 等待，不报故障。
- 帧时间是本机单调时钟，`ecu_node` 负责与系统时间互相换算（见 architecture.md 第 4.3 节）。多台机器之间用 NTP，要求高时用 PTP（`linuxptp`）对齐系统时间，规划器与 ECU 的时间才一致。

## 3. EtherCAT（SOEM）——接口已预留

`EthercatIo`（`hal/ethercat/`）已经有：

- 配置解析：`ethercat.interface`、`slaves`、`period`、`op_mode`、`counts_per_rad`、`rated_torque`、`dc_sync`；
- 按字节打包的 CiA 402 PDO 结构；
- 周期定时与 CiA 402 使能状态机（与 CAN 共用 `cia402.hpp`）；
- 单位换算。

还缺的只是 SOEM 调用。补全步骤：

1. 安装 SOEM（源码或发行版包），用 `-DVSIM_WITH_SOEM=ON` 构建，在 `CMakeLists.txt` 中打开 `find_package(soem)` / `target_link_libraries` 两行；
2. 补全 `activate()`：`ec_init` → `ec_config_init` → PO2SO 钩子里用 SDO 选择 PDO、写 `0x6060` → `ec_config_map` → 分布式时钟（`ec_configdc`、`ec_dcsync0`）→ 切到 OP 并确认；
3. `read()`：`ec_send_processdata` / `ec_receive_processdata`，检查 working counter，把 IOmap 拷贝到 `tx_`；
4. `write()`：`rx_` 拷贝到 IOmap；
5. `deactivate()`：发 Shutdown，回到 INIT，`ec_close`。

每一步的调用顺序都以注释形式写在 `ethercat_io.cpp` 里。EtherCAT 主站需要原始套接字权限（`CAP_NET_RAW`）与实时内核。

## 4. 急停输入（GPIO，libgpiod v2）——接口已预留

`GpiodDigitalInput` 的配置已经就位：`safety.estop.type: gpiod`、`chip`、`line`、`active_low`。补全步骤：

1. 构建时加 `-DVSIM_WITH_GPIOD=ON`，在 `CMakeLists.txt` 中打开 `pkg_check_modules(GPIOD libgpiod>=2)`；
2. 按 `digital_io.cpp` 里的注释补全：
   - `open()`：`gpiod_chip_open` → `gpiod_line_settings_new` 并设为输入 → `gpiod_line_config_add_line_settings` → `gpiod_chip_request_lines`；
   - `read()`：`gpiod_line_request_get_value`。

`ecu_node` 每 20 ms 轮询一次急停输入，按下即锁存 `estop`。急停输入属于"补充"，驱动器 STO 硬件回路才是主链路。

## 5. 硬件看门狗与实时性——已实现

- **看门狗：** 配置 `watchdog.type: linux`（`/dev/watchdog`）。控制循环不再迭代（线程卡死）时停止喂狗，板子复位。I/O 超时的迭代也会喂狗，所以总线断开时 ECU 输出安全态而不是反复重启。正常退出时写 magic close，不触发复位。
- **实时线程：** `rt.enable: true` 后，控制线程会：
  - `mlockall` 锁定内存，预触栈；
  - 绑定到 `rt.cpu`；
  - 设为 `SCHED_FIFO` 优先级 `rt.priority`。

  设置 `rt.required: true` 时，实时配置一旦失败就拒绝启动，而不是降级运行。
- **目标板内核与系统：**
  - 内核：PREEMPT_RT；内核参数 `isolcpus=3 nohz_full=3 rcu_nocbs=3`，与 `rt.cpu` 对应；
  - 关闭 CPU 频率调节（`performance` governor）；
  - 用 `cyclictest` 先确认最坏延迟。
- **诊断：** 用 `runtime.cycle_budget` 设定每周期"计算 + 写出"的耗时预算。超出会计入 `/ecu/status` 的 `overruns`，并在 1 s 内置 `cycle_overrun` 警告位。

## 6. 目标板部署

### 6.1 交叉编译（不带 ROS 的核心库与独立程序）

```bash
sudo apt install g++-aarch64-linux-gnu qemu-user     # qemu 可选：用于在宿主上跑交叉编译的单元测试
scripts/cross_build_aarch64.sh --cpu cortex-a53      # 产物在 build/aarch64/
# 板卡厂商工具链：--prefix aarch64-xxx-linux-gnu- --sysroot /path/to/sysroot
```

Yocto：`deploy/yocto/vsim-core_0.1.0.bb` 是核心库的配方骨架。ROS 2 部分（`vsim_interfaces`、`vsim_ros`）用 meta-ros 层构建；也可以在板子上装 ROS 2 二进制包（Ubuntu arm64）后直接 `colcon build`。

### 6.2 ECU 作为系统服务

```bash
sudo install -D -m 644 deploy/systemd/vsim-ecu.service /etc/systemd/system/vsim-ecu.service
sudo install -D -m 755 deploy/systemd/vsim-ecu-start.sh /opt/vsim/bin/vsim-ecu-start.sh
sudo install -D -m 644 deploy/systemd/ecu.env /etc/vsim/ecu.env                       # 改 Zenoh 路由器地址
sudo install -D -m 644 src/vsim_ros/config/ecu_hardware.yaml /etc/vsim/ecu.yaml       # 按实机修改
sudo install -D -m 644 deploy/systemd/80-can0.network /etc/systemd/network/80-can0.network
sudo useradd --system --groups dialout vsim
sudo systemctl daemon-reload && sudo systemctl enable --now vsim-ecu
journalctl -u vsim-ecu -f
```

服务配置了 `LimitRTPRIO`、`LimitMEMLOCK`、`CAP_SYS_NICE` / `CAP_IPC_LOCK`，并绑定 `can0` 设备：`can0` 消失时服务停止。关停时发 SIGINT，节点会输出安全态并正常关闭看门狗。

### 6.3 第一次上电

1. **机械与急停：** 机械固定或空载；驱动器 STO / 急停回路接好并实测一次。
2. **起点配置：** 用 `ecu_hardware.yaml` 作起点，并确认以下几点：
   - `auto_enable: false`、`safe_output: disable`；
   - `controller.effort_limit` 取额定力矩的一小部分；
   - `safety.position_min/max` 小于机械行程；
   - `tracking_error_max` 收紧。
3. **不接驱动器空跑：** 设 `io.backend: loopback`、`watchdog.type: none`。检查 `/ecu/status` 中的周期耗时与 `overruns`，并确认 `rt` 配置生效（日志里不能出现 real-time configuration failed）。
4. **接上驱动器，不使能：** 切回 `can`。确认状态字、位置读数方向与零点、力矩符号：手动推动关节，看 `/ecu/controller_state`。
5. **使能并验证：** `ros2 service call /ecu/set_mode vsim_interfaces/srv/SetEcuMode "{command: 1}"`。ECU 以当前位置为保持点，并在 `ref_blend_time` 内平滑切入轨迹。先用小幅正弦验证，再逐步放开限幅。

## 7. 新硬件的扩展点速查

| 硬件 | 实现 | 注册 / 配置 |
|---|---|---|
| 新总线 / 新 I/O 板卡 | `IoBackend`：`configure / activate / read / write / deactivate / time_base` | `IoBackendRegistry::instance().register_backend("name", …)` → `io.backend: name` |
| 新驱动器协议（CAN） | `IMotorCodec` | `create_motor_codec()` → `io.can.codec` |
| 新 CAN 适配器（非 SocketCAN） | `ICanBus` | `create_can_bus()` 按接口名前缀分派 |
| 新数字输入 | `IDigitalInput` | `create_estop_input()` → `safety.estop.type` |
| 新看门狗 | `IWatchdog` | `create_watchdog()` → `watchdog.type` |

`IoBackend` 的时序约定：

- `read()` 阻塞到本周期数据就绪，返回的 `SensorFrame.stamp` 是本周期采样时刻；
- `write()` 下发本周期指令；
- 谁决定节拍（周期主站）由后端决定，`time_base()` 声明帧时间属于哪种时钟。
