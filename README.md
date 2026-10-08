# vsim —— 通用算法仿真虚拟平台（C++）

在还没有硬件的阶段，用软件把一台"整机"的三个单元跑起来，并且让将来接入真实硬件时只替换 I/O 后端、不改算法代码：

| 单元 | 角色 | 本工程中的实现 |
|---|---|---|
| **sim_host**（上位机） | 被控对象仿真、仿真时钟、故障注入、可视化数据 | `SimEngine` + `SimRunner`，ROS 节点 `sim_host_node` |
| **ecu**（嵌入式 Linux 控制单元） | 周期控制循环、状态机、安全监控、硬件 I/O | `EcuApp` + `EcuRuntime` + HAL 后端，ROS 节点 `ecu_node` |
| **planner**（高级规划单元） | 按仿真时间周期重规划，发布时间参数化轨迹 | `IPlanner` + `PlannerScheduler`，ROS 节点 `planner_node` |

核心库 `vsim_core` 是纯 C++17，不依赖 ROS。它可以单独编译、单元测试、交叉编译到 aarch64 板子上。ROS 2 层只做"翻译 + 托管"。

```text
planner    IPlanner（sine / waypoints / …）+ PlannerScheduler（按仿真时间周期重规划）
   │
   │   /planner/trajectory（绝对时间戳，轨迹点对齐时间网格）
   ▼
ecu        EcuApp：轨迹采样 → 参考生成 → 控制器 → 安全监控 → 状态机
           EcuRuntime：read → step → write（实时线程）
           IoBackend（硬件抽象层）
  ▲  │
  │  │     I1：/sim/sensor_frame ⇄ /ecu/actuator_command（ROS 2 话题）
  │  │     I2：CAN 帧（vcan / 跨容器 vxcan；simple 或 CiA 402 协议）
  │  ▼     I3：真实驱动器（can0 / EtherCAT）取代 sim_host
sim_host   ArticulatedPlant（M(q)q̈ + h = τ）· 传感器模型 · 故障注入 · SimRunner（lockstep、实时倍率）
```

## 目录

```text
vsim/
├── src/
│   ├── vsim_core/          纯 C++17 核心库（plain CMake 包，colcon 也能构建）
│   │   ├── include/vsim/   common · msg · math · model · sim · control · planning · ecu · hal · rt · system
│   │   ├── apps/           vsim_standalone（单进程三单元）、vsim_can_node（经 SocketCAN 分进程）
│   │   └── test/           GoogleTest 单元测试与闭环测试
│   ├── vsim_interfaces/    接口契约：SensorFrame / ActuatorCommand / UnitStatus / ControllerState + 2 个服务
│   └── vsim_ros/           ROS 2 节点：sim_host_node · ecu_node · planner_node · monitor_node，launch 与参数
├── config/                 独立程序的 INI 配置：defaults.ini（全部键与注释）、鲁棒性场景、CiA 402 示例
├── docker/ + compose.yaml  每个单元一个容器，rmw_zenoh 路由；profile "check" 做无界面回归
├── scripts/                build_core.sh · smoke_test.sh · vcan_setup.sh · vcan_link.sh · cross_build_aarch64.sh
├── cmake/toolchains/       aarch64 交叉编译工具链文件
├── deploy/                 systemd 服务（目标板 ECU）、CAN 网络配置、Yocto 配方骨架
└── docs/                   architecture.md · hardware_integration.md · configuration.md
```

## 快速开始

### 1. 不装 ROS：核心库 + 单进程闭环（最快）

需要 CMake ≥ 3.16、GCC ≥ 9 或 Clang ≥ 10（Linux）。首次构建会从 GitHub 拉取 GoogleTest，系统已装 `libgtest-dev` 时直接使用系统版本。

```bash
scripts/build_core.sh                      # 编译 → 单元测试 → 两个闭环冒烟用例
scripts/build_core.sh --asan               # 或 --tsan：带 sanitizer 构建并测试

B=build/core-release
$B/vsim_standalone --duration 10                               # 单摆 + 正弦轨迹，lockstep，尽可能快
$B/vsim_standalone --model two_link --io can --csv run.csv     # 两连杆，ECU 经虚拟 CAN 与"虚拟驱动器"通信
$B/vsim_standalone --config config/robustness.ini --duration 12 --max-rms 0.02   # 模型失配 + 噪声 + 故障场景
$B/vsim_standalone --model two_link --config config/two_link_cia402.ini           # CiA 402 协议路径
$B/vsim_standalone --print-defaults > my.ini                   # 导出带注释的全部配置项
```

`--max-rms X` 让程序在任一关节跟踪误差 RMS 超过 X，或 ECU 未处于 RUNNING 时返回非零，可以直接用作 CI 判据。

### 2. ROS 2（三个节点，同机）

以 ROS 2 Lyrical 为例（Jazzy 同样可用）：

```bash
cd vsim                                    # 仓库根目录本身就是 colcon 工作空间
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch vsim_ros vsim.launch.py                    # 单摆
ros2 launch vsim_ros vsim.launch.py model:=two_link    # 两连杆 + 路径点规划
scripts/smoke_test.sh two_link 10 0.01                 # 无界面端到端检查（退出码 0/1）
```

常用交互：

```bash
ros2 topic echo /ecu/status
ros2 service call /ecu/set_mode vsim_interfaces/srv/SetEcuMode "{command: 4}"   # 软件急停（1 使能 2 停用 3 复位 4 急停 5 释放）
ros2 service call /sim_host/inject_fault vsim_interfaces/srv/InjectFault \
  "{type: sensor_bias, joint: 0, value: 0.8, start_delay: 0.0, duration: 0.0}"
ros2 service call /sim_host/clear_faults std_srvs/srv/Trigger
ros2 service call /sim_host/reset std_srvs/srv/Trigger          # 仿真回到 t=0，ECU 视为虚拟上电
```

可视化：`/joint_states`（仿真真值）、`/ecu/controller_state`（参考、实际、误差、力矩）可以直接在 Foxglove / Lichtblick / PlotJuggler / RViz 中查看。

### 3. Docker Compose（每个单元一个容器）

```bash
docker compose up --build                  # zenoh 路由器 + sim_host + ecu + planner；Foxglove 连 ws://localhost:8765
MODEL=two_link docker compose up
docker compose --profile check up --build --abort-on-container-exit --exit-code-from check   # CI 回归
```

### 4. 虚拟 CAN（接口层级 I2）

```bash
sudo scripts/vcan_setup.sh                                       # 创建 vcan0
build/core-release/vsim_can_node --role sim --iface vcan0 &      # 被控对象 + 虚拟驱动器
build/core-release/vsim_can_node --role ecu --iface vcan0 --duration 10
ros2 launch vsim_ros vsim.launch.py io:=can                      # 或者：ROS 节点之间改走 vcan0
VSIM_IO=can docker compose up -d && sudo scripts/vcan_link.sh    # 或者：跨容器 vxcan 链路
```

## 接入真实硬件

算法、状态机与安全逻辑不需要修改，要做的是换 I/O 后端并调整配置。详见 [docs/hardware_integration.md](docs/hardware_integration.md)。

| 接口 | 状态 |
|---|---|
| SocketCAN（`can0` / `vcan` / `vxcan`）+ 驱动器协议编解码（simple、CiA 402：状态机、PDO、SDO、NMT） | **已实现**；CiA 402 在进程内虚拟总线上闭环验证过 |
| ECU 作为 SYNC 主站（实机节拍）、帧时间与系统时间换算 | **已实现**，用 loopback 后端验证过 |
| Linux 硬件看门狗（`/dev/watchdog`）、实时线程（SCHED_FIFO、CPU 绑定、mlockall、绝对时间定时器） | **已实现** |
| EtherCAT 主站（SOEM） | **接口已预留**：配置、PDO 结构、周期交换骨架都已就位，`activate()` 返回 NotImplemented |
| 急停输入 GPIO（libgpiod v2） | **接口已预留**：`GpiodDigitalInput` 骨架与调用步骤已写好 |
| 目标板部署 | aarch64 工具链文件、systemd 服务、`ecu_hardware.yaml` 起点模板、Yocto 配方骨架 |

## 验证状态

交付前在构建环境（x86_64、Ubuntu 24.04、GCC 13）中实际跑过：

- **核心库：** 以 `-Werror` 零警告编译；93 个单元测试中 91 个通过，2 个 SocketCAN 用例因环境没有 AF_CAN 而自动跳过。ASan/UBSan 与 TSan 下全部干净。
- **闭环精度：** 单摆跟踪误差 RMS 3.0e-3 rad，两连杆 1.0e-4 rad。进程内直连与虚拟 CAN 两条路径都通过，CiA 402 协议路径也通过。lockstep 结果逐位可复现。
- **ROS 2 闭环：** 在源码编译的 ROS 2 Lyrical（rmw_cyclonedds）上运行三个节点，同样的精度复现，实时倍率 1.00。以下 27 项服务检查全部符合预期：故障注入、复位（故障仍在时拒绝复位）、急停、释放急停、仿真复位（虚拟上电）。仿真侧迟启动 2.5 s，ECU 在 INIT 等待后正常进入运行。
- **实机时间基准：** 用 loopback 后端模拟实机节拍，ECU 能按系统时间戳的轨迹运动。
- **独立评审：** 另起一个评审代理审查了代码并用小程序复现问题，共 14 项。已修复 14 项。评审发现的修复副作用（带标签的 CAN 指令、积压请求、两次复位）也已修正，全部配有回归测试。
- **Compose 与交叉编译：** `docker compose config` 解析通过；交叉编译工具链的 CMake 逻辑用替身编译器检查过。

**尚未验证**（构建环境没有相应条件）：

- Docker 镜像构建与容器内运行，包括 rmw_zenoh 通信；
- 真实 aarch64 编译器与目标板运行；
- 内核 SocketCAN（vcan/vxcan）上的收发；
- `ros2 launch` 启动文件，只做了语法检查；
- 真实驱动器、EtherCAT、GPIO。

这些路径第一次运行时如遇问题，按 docs 中的说明排查。

## 扩展

| 想做什么 | 实现的接口 | 注册位置 |
|---|---|---|
| 新控制算法 | `vsim::IController`（`control/controller.hpp`） | `create_controller()` |
| 新规划算法 | `vsim::IPlanner`（`planning/planner.hpp`） | `create_planner()` |
| 新被控对象 | `vsim::IJointDynamics` 或 `vsim::IPlant` | `create_joint_dynamics()` / `create_plant()` |
| 新硬件 / 总线 | `vsim::IoBackend`（`hal/io_backend.hpp`） | `IoBackendRegistry` |
| 新驱动器协议 | `vsim::IMotorCodec`（`hal/can/motor_codec.hpp`） | `create_motor_codec()` |

仿真侧和 ECU 侧共用同一个协议编解码器：ECU 的 CAN 驱动代码在接入真实驱动器之前，就能在虚拟总线上对着"虚拟驱动器"原样测试。

## 许可

`package.xml` 中的 `Apache-2.0` 与维护者信息都是占位值，请按项目实际情况修改。
