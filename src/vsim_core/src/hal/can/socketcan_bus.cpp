#include "vsim/hal/can/socketcan_bus.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace vsim {

Status SocketCanBus::open() {
  close();
  sock_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (sock_ < 0) return Status::IoError(std::string("socket(PF_CAN): ") + std::strerror(errno));
  if (fd_mode_) {
    int enable = 1;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) != 0) {
      const std::string err = std::strerror(errno);
      close();
      return Status::IoError("CAN_RAW_FD_FRAMES: " + err);
    }
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) != 0) {
    const std::string err = std::strerror(errno);
    close();
    return Status::IoError("interface '" + ifname_ + "': " + err +
                           " (创建虚拟总线：ip link add dev " + ifname_ + " type vcan && ip link set up " +
                           ifname_ + ")");
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    const std::string err = std::strerror(errno);
    close();
    return Status::IoError("bind: " + err);
  }
  event_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  return Status::Ok();
}

void SocketCanBus::close() {
  if (sock_ >= 0) ::close(sock_);
  if (event_fd_ >= 0) ::close(event_fd_);
  sock_ = -1;
  event_fd_ = -1;
}

void SocketCanBus::interrupt() {
  if (event_fd_ >= 0) {
    const std::uint64_t one = 1;
    (void)!::write(event_fd_, &one, sizeof(one));
  }
}

Status SocketCanBus::send(const CanFrame& f) {
  if (sock_ < 0) return Status::Error(StatusCode::kUnavailable, "socketcan: not open");
  const canid_t id = f.id | (f.extended ? CAN_EFF_FLAG : 0u);
  ssize_t n = 0;
  if (f.fd) {
    if (!fd_mode_) return Status::InvalidArgument("socketcan: FD frame on classic socket");
    canfd_frame fr{};
    fr.can_id = id;
    fr.len = f.len;
    std::memcpy(fr.data, f.data.data(), f.len);
    n = ::write(sock_, &fr, CANFD_MTU);
    if (n != static_cast<ssize_t>(CANFD_MTU)) return Status::IoError(std::string("write: ") + std::strerror(errno));
  } else {
    can_frame fr{};
    fr.can_id = id;
    fr.len = f.len > 8 ? 8 : f.len;
    std::memcpy(fr.data, f.data.data(), fr.len);
    n = ::write(sock_, &fr, CAN_MTU);
    if (n != static_cast<ssize_t>(CAN_MTU)) return Status::IoError(std::string("write: ") + std::strerror(errno));
  }
  return Status::Ok();
}

ReadResult SocketCanBus::receive(CanFrame& f, TimeNs timeout) {
  if (sock_ < 0) return ReadResult::kClosed;
  pollfd fds[2] = {{sock_, POLLIN, 0}, {event_fd_, POLLIN, 0}};
  // ppoll：纳秒分辨率（poll 只有毫秒，亚毫秒的反馈等待会被放大到 1 ms）
  const TimeNs t = timeout > 0 ? timeout : 0;
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(t / kNsPerSec);
  ts.tv_nsec = static_cast<long>(t % kNsPerSec);
  const int rc = ::ppoll(fds, event_fd_ >= 0 ? 2 : 1, &ts, nullptr);
  if (rc == 0) return ReadResult::kTimeout;
  if (rc < 0) return errno == EINTR ? ReadResult::kTimeout : ReadResult::kError;
  if (event_fd_ >= 0 && (fds[1].revents & POLLIN)) {
    std::uint64_t v = 0;
    (void)!::read(event_fd_, &v, sizeof(v));
    return ReadResult::kClosed;
  }
  canfd_frame fr{};
  const ssize_t n = ::read(sock_, &fr, sizeof(fr));
  if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) return ReadResult::kError;
  f.extended = (fr.can_id & CAN_EFF_FLAG) != 0;
  f.id = fr.can_id & (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  f.fd = n == static_cast<ssize_t>(CANFD_MTU);
  f.len = fr.len;
  std::memcpy(f.data.data(), fr.data, fr.len);
  return ReadResult::kOk;
}

}  // namespace vsim
