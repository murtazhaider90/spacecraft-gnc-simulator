#include <array>
#include <cstddef>
#include <cstdint>

namespace kernel {

constexpr std::size_t kBlockSize = 64;
constexpr std::size_t kBlockCount = 128;

class FixedBlockAllocator {
 public:
  FixedBlockAllocator() { used_.fill(false); }

  void* allocate() {
    for (std::size_t i = 0; i < kBlockCount; ++i) {
      if (!used_[i]) {
        used_[i] = true;
        return storage_.data() + i * kBlockSize;
      }
    }
    return nullptr;
  }

  bool deallocate(void* ptr) {
    auto* p = static_cast<std::uint8_t*>(ptr);
    auto* begin = storage_.data();
    auto* end = begin + storage_.size();
    if (p < begin || p >= end) return false;
    const auto offset = static_cast<std::size_t>(p - begin);
    if (offset % kBlockSize != 0) return false;
    const std::size_t index = offset / kBlockSize;
    if (!used_[index]) return false;
    used_[index] = false;
    return true;
  }

 private:
  std::array<std::uint8_t, kBlockSize * kBlockCount> storage_{};
  std::array<bool, kBlockCount> used_{};
};

using TaskFn = void (*)();

struct Task {
  TaskFn fn{nullptr};
  bool runnable{false};
};

class CooperativeScheduler {
 public:
  bool add(TaskFn fn) {
    for (auto& task : tasks_) {
      if (!task.runnable) {
        task = Task{fn, true};
        return true;
      }
    }
    return false;
  }

  void run_once() {
    for (auto& task : tasks_) {
      if (task.runnable && task.fn) task.fn();
    }
  }

 private:
  std::array<Task, 16> tasks_{};
};

}  // namespace kernel
