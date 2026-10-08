// vsim/hal/can/can_frame.hpp —— 与平台无关的 CAN / CAN FD 帧及小端编解码工具
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace vsim {

struct CanFrame {
  std::uint32_t id{0};
  bool extended{false};  ///< 29 位扩展帧
  bool fd{false};        ///< CAN FD 帧
  std::uint8_t len{0};   ///< 数据长度 0..8（经典）/ 0..64（FD）
  std::array<std::uint8_t, 64> data{};
};

namespace le {
inline void put_u16(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v);
  p[1] = static_cast<std::uint8_t>(v >> 8);
}
inline void put_u32(std::uint8_t* p, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}
inline void put_u64(std::uint8_t* p, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}
inline void put_i16(std::uint8_t* p, std::int16_t v) { put_u16(p, static_cast<std::uint16_t>(v)); }
inline void put_i32(std::uint8_t* p, std::int32_t v) { put_u32(p, static_cast<std::uint32_t>(v)); }
inline void put_i64(std::uint8_t* p, std::int64_t v) { put_u64(p, static_cast<std::uint64_t>(v)); }

inline std::uint16_t get_u16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
inline std::uint32_t get_u32(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (int i = 3; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}
inline std::uint64_t get_u64(const std::uint8_t* p) {
  std::uint64_t v = 0;
  for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}
inline std::int16_t get_i16(const std::uint8_t* p) { return static_cast<std::int16_t>(get_u16(p)); }
inline std::int32_t get_i32(const std::uint8_t* p) { return static_cast<std::int32_t>(get_u32(p)); }
inline std::int64_t get_i64(const std::uint8_t* p) { return static_cast<std::int64_t>(get_u64(p)); }
}  // namespace le

}  // namespace vsim
