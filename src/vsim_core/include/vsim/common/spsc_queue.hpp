// vsim/common/spsc_queue.hpp —— 单生产者/单消费者无锁环形队列
//
// 用于把实时线程（ECU 控制循环）产生的遥测数据交给非实时线程发布：
// 生产者 try_push 不阻塞、不分配内存（槽位预先构造，拷贝赋值进去；
// 若元素含 vector，请用 init() 预先把每个槽位的 vector 调整到最终大小）。
#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <vector>

namespace vsim {

template <typename T>
class SpscQueue {
 public:
  explicit SpscQueue(std::size_t capacity = 1024) : slots_(round_up(capacity + 1)) {
    mask_ = slots_.size() - 1;
  }

  /// 预初始化每个槽位（例如把 vector 调整到关节数），避免实时侧分配内存。
  void init(const std::function<void(T&)>& fn) {
    for (auto& s : slots_) fn(s);
  }

  /// 生产者调用。队列满时返回 false（丢弃，计数由调用方负责）。
  bool try_push(const T& value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & mask_;
    if (next == tail_.load(std::memory_order_acquire)) return false;
    slots_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  /// 消费者调用。
  bool try_pop(T& out) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = slots_[tail];
    tail_.store((tail + 1) & mask_, std::memory_order_release);
    return true;
  }

  std::size_t capacity() const { return slots_.size() - 1; }
  bool empty() const {
    return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
  }

 private:
  static std::size_t round_up(std::size_t n) {
    std::size_t p = 2;
    while (p < n) p <<= 1;
    return p;
  }

  std::vector<T> slots_;
  std::size_t mask_{0};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace vsim
