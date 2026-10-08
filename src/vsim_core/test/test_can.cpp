#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <thread>

#include "vsim/hal/can/can_frame.hpp"
#include "vsim/hal/can/can_motor_emulator.hpp"
#include "vsim/hal/can/can_motor_io.hpp"
#include "vsim/hal/can/cia402.hpp"
#include "vsim/hal/can/cia402_codec.hpp"
#include "vsim/hal/can/simple_motor_codec.hpp"
#include "vsim/hal/can/socketcan_bus.hpp"
#include "vsim/hal/can/virtual_can_bus.hpp"
#include "vsim/hal/loopback_io.hpp"
#include "vsim/ecu/ecu_runtime.hpp"
#include "vsim/control/joint_pd_controller.hpp"

using namespace vsim;
using cia402::DriveState;

TEST(CanFrame, LittleEndianHelpers) {
  std::uint8_t b[8];
  le::put_i32(b, -123456);
  EXPECT_EQ(le::get_i32(b), -123456);
  le::put_u16(b, 0xBEEF);
  EXPECT_EQ(b[0], 0xEF);
  EXPECT_EQ(b[1], 0xBE);
  le::put_i64(b, -5'000'000'000LL);
  EXPECT_EQ(le::get_i64(b), -5'000'000'000LL);
}

TEST(SimpleMotorCodec, CommandAndFeedbackRoundTrip) {
  SimpleMotorCodec c;
  ASSERT_TRUE(c.configure(Config(), {3}).ok());
  std::vector<CanFrame> frames;
  for (auto mode : {ControlMode::kEffort, ControlMode::kVelocity, ControlMode::kPosition}) {
    frames.clear();
    JointCommandSample in;
    in.mode = mode;
    in.enable = true;
    in.effort = 1.23456;
    in.velocity = -2.5;
    in.position = 0.7853981;
    c.encode_command(3, in, frames);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].id, 0x203u);
    std::uint8_t node = 0;
    JointCommandSample out;
    ASSERT_TRUE(c.decode_command(frames[0], node, out));
    EXPECT_EQ(node, 3);
    EXPECT_EQ(out.mode, mode);
    EXPECT_TRUE(out.enable);
    if (mode == ControlMode::kEffort) {
      EXPECT_NEAR(out.effort, 1.23456, 1e-4);
    } else if (mode == ControlMode::kVelocity) {
      EXPECT_NEAR(out.velocity, -2.5, 1e-5);
    } else {
      EXPECT_NEAR(out.position, 0.7853981, 1e-6);
    }
  }
  frames.clear();
  JointFeedbackSample fb;
  fb.position = -1.234567;
  fb.velocity = 3.3;
  fb.effort = -7.5;
  fb.enabled = true;
  c.encode_feedback(3, fb, frames);
  ASSERT_EQ(frames.size(), 2u);
  JointFeedbackSample got;
  std::uint32_t parts = 0;
  for (const auto& f : frames) {
    std::uint8_t node = 0;
    ASSERT_TRUE(c.identify_feedback(f, node));
    EXPECT_EQ(node, 3);
    parts |= c.decode_feedback(f, got);
  }
  EXPECT_EQ(parts, c.feedback_complete_mask());
  EXPECT_NEAR(got.position, -1.234567, 1e-6);
  EXPECT_NEAR(got.velocity, 3.3, 1e-6);
  EXPECT_NEAR(got.effort, -7.5, 1e-4);
  EXPECT_TRUE(got.enabled);
  frames.clear();
  c.encode_sync(1'234'567'890'123LL, frames);
  TimeNs t = 0;
  bool has_time = false;
  ASSERT_TRUE(c.decode_sync(frames[0], t, has_time));
  EXPECT_TRUE(has_time);
  EXPECT_EQ(t, 1'234'567'890'123LL);
  EXPECT_FALSE(c.configure(Config(), {0}).ok());
}

TEST(Cia402, StatuswordRoundTripForAllStates) {
  for (auto s : {DriveState::kSwitchOnDisabled, DriveState::kReadyToSwitchOn, DriveState::kSwitchedOn,
                 DriveState::kOperationEnabled, DriveState::kQuickStopActive,
                 DriveState::kFaultReactionActive, DriveState::kFault, DriveState::kNotReadyToSwitchOn}) {
    EXPECT_EQ(cia402::decode_statusword(cia402::encode_statusword(s)), s) << cia402::to_string(s);
  }
  // 真实驱动器常见的状态字（带厂商位）
  EXPECT_EQ(cia402::decode_statusword(0x1637), DriveState::kOperationEnabled);
  EXPECT_EQ(cia402::decode_statusword(0x0250), DriveState::kSwitchOnDisabled);
  EXPECT_EQ(cia402::decode_statusword(0x0218), DriveState::kFault);
}

TEST(Cia402, MasterSequenceEnablesDriveAndResetsFaultOnlyOnRequest) {
  DriveState slave = DriveState::kNotReadyToSwitchOn;
  std::uint16_t prev = 0;
  int cycles = 0;
  auto cycle = [&](bool want, bool reset = false) {
    const auto master_view = cia402::decode_statusword(cia402::encode_statusword(slave));
    const std::uint16_t cw = cia402::next_controlword(master_view, want, reset);
    slave = cia402::apply_controlword(slave, cw, prev);
    prev = cw;
    ++cycles;
  };
  while (slave != DriveState::kOperationEnabled && cycles < 10) cycle(true);
  EXPECT_EQ(slave, DriveState::kOperationEnabled);
  EXPECT_LE(cycles, 5);
  cycle(false);
  EXPECT_NE(slave, DriveState::kOperationEnabled);
  slave = DriveState::kFault;
  for (int k = 0; k < 20; ++k) cycle(true);  // 主站想使能也不会自动复位驱动器故障
  EXPECT_EQ(slave, DriveState::kFault);
  cycle(true, true);  // 操作者复位：一次上升沿
  EXPECT_EQ(slave, DriveState::kSwitchOnDisabled);
  cycles = 0;
  while (slave != DriveState::kOperationEnabled && cycles < 10) cycle(true);
  EXPECT_EQ(slave, DriveState::kOperationEnabled);  // 复位后重新使能
}

TEST(Cia402, CanopenHelperFrames) {
  const CanFrame sdo = cia402::sdo_download(5, 0x6060, 0, 10, 1);
  EXPECT_EQ(sdo.id, 0x605u);
  EXPECT_EQ(sdo.data[0], 0x2F);
  EXPECT_EQ(sdo.data[1], 0x60);
  EXPECT_EQ(sdo.data[2], 0x60);
  EXPECT_EQ(sdo.data[3], 0x00);
  EXPECT_EQ(sdo.data[4], 10);
  EXPECT_EQ(cia402::sdo_download(5, 0x6071, 0, 0, 2).data[0], 0x2B);
  EXPECT_EQ(cia402::sdo_download(5, 0x607A, 0, 0, 4).data[0], 0x23);
  const CanFrame nmt = cia402::nmt_frame(0x01, 5);
  EXPECT_EQ(nmt.id, 0x000u);
  EXPECT_EQ(nmt.len, 2);
  EXPECT_EQ(nmt.data[1], 5);
}

TEST(VirtualCanBus, BroadcastExcludesSenderAndTimesOut) {
  VirtualCanBus a("test_bus_1"), b("test_bus_1"), c("test_bus_1"), other("test_bus_2");
  for (auto* bus : {&a, &b, &c, &other}) ASSERT_TRUE(bus->open().ok());
  CanFrame f;
  f.id = 0x123;
  f.len = 1;
  f.data[0] = 42;
  ASSERT_TRUE(a.send(f).ok());
  CanFrame r;
  EXPECT_EQ(b.receive(r, kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(r.id, 0x123u);
  EXPECT_EQ(c.receive(r, kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(a.receive(r, kNsPerMs), ReadResult::kTimeout);      // 发送者收不到自己的帧
  EXPECT_EQ(other.receive(r, kNsPerMs), ReadResult::kTimeout);  // 不同总线
  std::thread t([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    b.interrupt();
  });
  EXPECT_EQ(b.receive(r, 2 * kNsPerSec), ReadResult::kClosed);
  t.join();
}

// ECU 侧 CanMotorIo 与仿真侧 CanMotorEmulator 在虚拟总线上交换一个周期
TEST(CanMotorIo, OneLockstepCycleOverVirtualBus) {
  for (const std::string codec : {"simple", "cia402"}) {
    Config io;
    io.set("can.interface", "virtual:cycle_" + codec);
    io.set("can.codec", codec);
    io.set("can.node_ids", "1,2");
    io.set("can.sync_master", "bus");
    io.set("can.cia402.sync_with_time", "true");
    CanMotorIo ecu_io;
    ASSERT_TRUE(ecu_io.configure(io, 2).ok());
    ASSERT_TRUE(ecu_io.activate().ok());
    CanMotorEmulator emu(create_can_bus("virtual:cycle_" + codec), create_motor_codec(codec), {1, 2});
    ASSERT_TRUE(emu.open(io).ok());

    SensorFrame truth;
    truth.resize(2);
    truth.stamp = 42 * kNsPerMs;
    truth.position = {0.1, -0.2};
    truth.velocity = {1.0, -1.0};
    for (int k = 0; k < 6; ++k) {  // CiA 402 需要几个周期完成使能序列
      truth.stamp += kNsPerMs;
      ASSERT_TRUE(emu.publish(truth).ok());
      SensorFrame got;
      ASSERT_EQ(ecu_io.read(got, 100 * kNsPerMs), ReadResult::kOk) << codec;
      EXPECT_EQ(got.stamp, truth.stamp);
      EXPECT_NEAR(got.position[1], -0.2, 1e-4) << codec;
      EXPECT_NEAR(got.velocity[0], 1.0, 1e-3) << codec;
      ActuatorCommand cmd;
      cmd.resize(2);
      cmd.mode = ControlMode::kEffort;
      cmd.effort = {1.5, -0.75};
      cmd.stamp = got.stamp;  // 回显所响应的帧时刻（simple 协议据此打 SYNC 标签）
      ASSERT_TRUE(ecu_io.write(cmd).ok());
      ActuatorCommand rx;
      ASSERT_EQ(emu.poll_command(rx, 100 * kNsPerMs), ReadResult::kOk);
      EXPECT_EQ(rx.stamp, truth.stamp);  // 仿真侧给指令打上本周期时间 → lockstep 握手
      if (k == 5) {
        EXPECT_EQ(rx.mode, ControlMode::kEffort) << codec;  // CiA 402 已进入 OPERATION_ENABLED
        EXPECT_NEAR(rx.effort[0], 1.5, 0.01) << codec;       // CiA 402 力矩是额定值的千分比
        EXPECT_NEAR(rx.effort[1], -0.75, 0.01) << codec;
      }
    }
    ecu_io.deactivate();
    emu.close();
  }
}

TEST(IoBackend, TimeBaseTellsWhetherFrameStampsAreSimulationTime) {
  Config io;
  io.set("can.interface", "virtual:time_base");
  io.set("can.node_ids", "1");
  io.set("can.codec", "simple");
  io.set("can.sync_master", "bus");
  CanMotorIo sim_bus;
  ASSERT_TRUE(sim_bus.configure(io, 1).ok());
  EXPECT_EQ(sim_bus.time_base(), TimeBase::kExternal);  // SYNC 携带仿真时间
  io.set("can.sync_master", "ecu");
  CanMotorIo ecu_master;
  ASSERT_TRUE(ecu_master.configure(io, 1).ok());
  EXPECT_EQ(ecu_master.time_base(), TimeBase::kMonotonic);  // 实机：ECU 自己的周期定时器
  io.set("can.sync_master", "bus");
  io.set("can.codec", "cia402");
  CanMotorIo plain_canopen;
  ASSERT_TRUE(plain_canopen.configure(io, 1).ok());
  EXPECT_EQ(plain_canopen.time_base(), TimeBase::kMonotonic);  // 标准 CANopen SYNC 不带时间
  io.set("can.cia402.sync_with_time", "true");
  CanMotorIo sim_canopen;
  ASSERT_TRUE(sim_canopen.configure(io, 1).ok());
  EXPECT_EQ(sim_canopen.time_base(), TimeBase::kExternal);

  LoopbackIo realtime_loop;
  ASSERT_TRUE(realtime_loop.configure(Config(), 1).ok());
  EXPECT_EQ(realtime_loop.time_base(), TimeBase::kMonotonic);
  Config lc;
  lc.set("realtime", "false");
  LoopbackIo virtual_loop;
  ASSERT_TRUE(virtual_loop.configure(lc, 1).ok());
  EXPECT_EQ(virtual_loop.time_base(), TimeBase::kExternal);
}

// ---------------------------------------------------------------- 评审后补充的回归用例
namespace {
std::unique_ptr<EcuApp> plain_ecu(std::size_t dof) {
  JointPdParams jp = JointPdParams::from_config(Config(), dof);
  jp.feedforward = JointPdParams::Feedforward::kNone;
  EcuAppParams p = EcuAppParams::from_config(Config(), dof);
  auto app = std::make_unique<EcuApp>(p, std::make_unique<JointPdController>(jp, nullptr), dof);
  EXPECT_TRUE(app->configure().ok());
  return app;
}
Config can_cfg(const std::string& bus, const std::string& master, const std::string& nodes = "1") {
  Config io;
  io.set("can.interface", "virtual:" + bus);
  io.set("can.codec", "simple");
  io.set("can.node_ids", nodes);
  io.set("can.sync_master", master);
  return io;
}
}  // namespace

TEST(CanMotorIo, StoppingTheRuntimeLeavesDrivesDisabled) {
  auto app = plain_ecu(1);
  CanMotorIo io;
  ASSERT_TRUE(io.configure(can_cfg("stop_test", "ecu"), 1).ok());
  VirtualCanBus sniffer("stop_test");
  ASSERT_TRUE(sniffer.open().ok());
  EcuRuntimeParams rp;
  rp.read_timeout = 5 * kNsPerMs;
  EcuRuntime rt(*app, io, rp);
  ASSERT_TRUE(rt.start().ok());
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  rt.stop();  // 与 SIGINT / systemctl stop 走同一条路径：interrupt → join → deactivate
  CanFrame f;
  int disabled_commands = 0;
  bool sync_after_disable = false;
  while (sniffer.receive(f, 0) == ReadResult::kOk) {
    if (f.id == SimpleMotorCodec::kCommandBase + 1) {
      if (f.data[0] == 0 && (f.data[1] & 0x1) == 0) ++disabled_commands;
    } else if (f.id == SimpleMotorCodec::kSyncId && disabled_commands > 0) {
      sync_after_disable = true;  // ECU 做主站：禁用指令之后补发 SYNC，同步 PDO 才会生效
    }
  }
  EXPECT_GE(disabled_commands, 1);
  EXPECT_TRUE(sync_after_disable);
}

TEST(CanMotorIo, EcuMasterDiscardsLateFeedbackFromPreviousCycle) {
  CanMotorIo io;
  Config cfg = can_cfg("late_test", "ecu");
  cfg.set("can.period", "0.002");
  ASSERT_TRUE(io.configure(cfg, 1).ok());
  ASSERT_TRUE(io.activate().ok());
  VirtualCanBus drive("late_test");
  ASSERT_TRUE(drive.open().ok());
  SimpleMotorCodec codec;
  ASSERT_TRUE(codec.configure(Config(), {1}).ok());
  auto feedback = [&](double q) {
    std::vector<CanFrame> tx;
    JointFeedbackSample fb;
    fb.position = q;
    codec.encode_feedback(1, fb, tx);
    for (const auto& fr : tx) ASSERT_TRUE(drive.send(fr).ok());
  };
  feedback(0.111);  // 上一周期迟到的反馈，已经在 ECU 的接收队列里
  SensorFrame frame;
  // 本周期驱动器没有回：迟到的帧必须被丢弃，而不是当成本周期的数据
  EXPECT_EQ(io.read(frame, 50 * kNsPerMs), ReadResult::kTimeout);
  EXPECT_GE(io.diagnostics().late_frames, 2u);  // 两帧（A/B）都被丢弃
  io.deactivate();
}

TEST(CanMotorEmulator, ResendKeepsCommandsAlreadyReceivedForThatFrame) {
  Config io = can_cfg("resend_test", "bus", "1,2");
  CanMotorEmulator emu(create_can_bus("virtual:resend_test"), create_motor_codec("simple"), {1, 2});
  ASSERT_TRUE(emu.open(io).ok());
  VirtualCanBus ecu_side("resend_test");
  ASSERT_TRUE(ecu_side.open().ok());
  SimpleMotorCodec codec;
  ASSERT_TRUE(codec.configure(Config(), {1, 2}).ok());
  auto send_cmd = [&](std::uint8_t node) {
    std::vector<CanFrame> tx;
    JointCommandSample c;
    c.mode = ControlMode::kEffort;
    c.enable = true;
    c.effort = 0.5 * node;
    c.tag = sync_tag(7 * kNsPerMs);  // 回应的是 t = 7 ms 的帧
    codec.encode_command(node, c, tx);
    for (const auto& fr : tx) ASSERT_TRUE(ecu_side.send(fr).ok());
  };
  SensorFrame f;
  f.resize(2);
  f.stamp = 7 * kNsPerMs;
  ASSERT_TRUE(emu.publish(f).ok());
  send_cmd(1);
  ActuatorCommand cmd;
  EXPECT_EQ(emu.poll_command(cmd, 10 * kNsPerMs), ReadResult::kTimeout);  // 只到了 1 个节点
  ASSERT_TRUE(emu.publish(f).ok());  // 重发同一帧：已收到的节点 1 不能被清掉
  send_cmd(2);
  ASSERT_EQ(emu.poll_command(cmd, 50 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(cmd.stamp, f.stamp);
  EXPECT_NEAR(cmd.effort[0], 0.5, 1e-3);
  EXPECT_NEAR(cmd.effort[1], 1.0, 1e-3);
  emu.close();
}

TEST(CanLockstep, TaggedRepliesMakeResendsSafeInBothDirections) {
  auto app = plain_ecu(1);
  Config io = can_cfg("tag_test", "bus");
  CanMotorIo can_io;
  ASSERT_TRUE(can_io.configure(io, 1).ok());
  EXPECT_TRUE(can_io.resend_on_duplicate());  // simple 协议带标签：可以回应重发
  CanMotorEmulator emu(create_can_bus("virtual:tag_test"), create_motor_codec("simple"), {1});
  ASSERT_TRUE(emu.open(io).ok());
  EcuRuntimeParams rp;
  rp.read_timeout = 50 * kNsPerMs;
  EcuRuntime rt(*app, can_io, rp);
  ASSERT_TRUE(can_io.activate().ok());
  app->set_io_ready(true);
  SensorFrame f;
  f.resize(1);
  ActuatorCommand cmd;

  // (1) ECU 太慢，仿真侧重发堆积：两个 SYNC(t1) 都会得到回应，第二个回应带着旧标签被丢弃
  f.stamp = kNsPerMs;
  ASSERT_TRUE(emu.publish(f).ok());
  ASSERT_TRUE(emu.publish(f).ok());  // 重发
  ASSERT_TRUE(rt.run_once());        // 回应 SYNC(t1)
  ASSERT_TRUE(rt.run_once());        // 重复帧：再次回应（同一标签）
  ASSERT_EQ(emu.poll_command(cmd, 50 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(cmd.stamp, kNsPerMs);
  f.stamp = 2 * kNsPerMs;  // 仿真推进到下一帧
  ASSERT_TRUE(emu.publish(f).ok());
  ASSERT_TRUE(rt.run_once());
  ASSERT_EQ(emu.poll_command(cmd, 50 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(cmd.stamp, 2 * kNsPerMs);
  EXPECT_EQ(emu.stale_commands(), 1u);  // 对 t1 的重复回应没有被当成对 t2 的回应

  // (2) 回应丢了：重发的 SYNC 会再次得到回应，lockstep 自动恢复
  f.stamp = 3 * kNsPerMs;
  ASSERT_TRUE(emu.publish(f).ok());
  ASSERT_TRUE(rt.run_once());  // 回应（假设在总线上丢了：仿真侧不去读）
  ASSERT_TRUE(emu.publish(f).ok());  // 超时重发
  ASSERT_TRUE(rt.run_once());
  ASSERT_EQ(emu.poll_command(cmd, 50 * kNsPerMs), ReadResult::kOk);  // 至少一个回应可用
  EXPECT_EQ(cmd.stamp, 3 * kNsPerMs);
  can_io.deactivate();
  emu.close();
}

TEST(CanLockstep, UntaggedProtocolDoesNotAnswerDuplicates) {
  Config io = can_cfg("untagged_test", "bus");
  io.set("can.codec", "cia402");
  io.set("can.cia402.sync_with_time", "true");
  CanMotorIo can_io;
  ASSERT_TRUE(can_io.configure(io, 1).ok());
  EXPECT_FALSE(can_io.resend_on_duplicate());  // 不带标签：重复回应会被当成对下一帧的回应
}

// ---------------------------------------------------------------- SocketCAN（真实内核协议栈）
// 需要 vcan 内核模块与接口（scripts/vcan_setup.sh 创建 vcan0；VSIM_TEST_CAN_IFACE 可改接口名）。
// 环境不具备时（容器没有 AF_CAN、没建 vcan0）跳过而不是失败。
namespace {
std::string test_can_iface() {
  const char* env = std::getenv("VSIM_TEST_CAN_IFACE");
  return env && *env ? env : "vcan0";
}
}  // namespace

TEST(SocketCan, FramesLoopBackOnVirtualInterfaceOrSkip) {
  const std::string iface = test_can_iface();
  SocketCanBus rx(iface, false), tx(iface, false);
  const Status st = rx.open();
  if (!st.ok()) GTEST_SKIP() << "SocketCAN unavailable: " << st.to_string();
  ASSERT_TRUE(tx.open().ok());

  CanFrame f;
  f.id = 0x123;
  f.len = 3;
  f.data[0] = 0xDE;
  f.data[1] = 0xAD;
  f.data[2] = 0x01;
  ASSERT_TRUE(tx.send(f).ok());
  CanFrame got;
  ASSERT_EQ(rx.receive(got, 200 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(got.id, 0x123u);
  EXPECT_FALSE(got.extended);
  EXPECT_EQ(got.len, 3);
  EXPECT_EQ(got.data[1], 0xAD);

  f.id = 0x1ABCDEF;  // 29 位扩展帧
  f.extended = true;
  ASSERT_TRUE(tx.send(f).ok());
  ASSERT_EQ(rx.receive(got, 200 * kNsPerMs), ReadResult::kOk);
  EXPECT_TRUE(got.extended);
  EXPECT_EQ(got.id, 0x1ABCDEFu);

  EXPECT_EQ(rx.receive(got, 20 * kNsPerMs), ReadResult::kTimeout);
  std::thread waker([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    rx.interrupt();
  });
  EXPECT_EQ(rx.receive(got, 5 * kNsPerSec), ReadResult::kClosed);  // interrupt() 解除阻塞
  waker.join();
}

TEST(SocketCan, MotorIoCycleOverVirtualInterfaceOrSkip) {
  const std::string iface = test_can_iface();
  {
    SocketCanBus probe(iface, false);
    const Status st = probe.open();
    if (!st.ok()) GTEST_SKIP() << "SocketCAN unavailable: " << st.to_string();
  }
  Config io;
  io.set("backend", "can");
  io.set("can.interface", iface);
  io.set("can.codec", "simple");
  io.set("can.node_ids", "1");
  io.set("can.sync_master", "bus");
  CanMotorIo ecu_io;
  ASSERT_TRUE(ecu_io.configure(io, 1).ok());
  ASSERT_TRUE(ecu_io.activate().ok());
  CanMotorEmulator emu(create_can_bus(iface), create_motor_codec("simple"), {1});
  ASSERT_TRUE(emu.open(io).ok());
  SensorFrame truth;
  truth.resize(1);
  truth.stamp = 7 * kNsPerMs;
  truth.position = {0.25};
  ASSERT_TRUE(emu.publish(truth).ok());
  SensorFrame got;
  ASSERT_EQ(ecu_io.read(got, 200 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(got.stamp, truth.stamp);
  EXPECT_NEAR(got.position[0], 0.25, 1e-4);
  ActuatorCommand cmd;
  cmd.resize(1);
  cmd.mode = ControlMode::kEffort;
  cmd.effort = {2.0};
  ASSERT_TRUE(ecu_io.write(cmd).ok());
  ActuatorCommand rx;
  ASSERT_EQ(emu.poll_command(rx, 200 * kNsPerMs), ReadResult::kOk);
  EXPECT_EQ(rx.stamp, truth.stamp);
  EXPECT_NEAR(rx.effort[0], 2.0, 1e-3);
  ecu_io.deactivate();
  emu.close();
}
