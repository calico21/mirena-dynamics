#ifndef MIRENA_CONTROL__SEQLOCK_HPP_
#define MIRENA_CONTROL__SEQLOCK_HPP_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace mirena
{

/// Single-writer / many-reader hand-off for a trivially copyable payload.
///
/// The writer never blocks and never waits on a reader, which is what we need
/// between the MPC solver (one write per solve, tens of microseconds apart from
/// the deadline) and the 100 Hz command loop: the command loop must never be
/// stalled behind a solver that is holding a mutex.  Readers retry instead of
/// blocking, so a torn read is detected and repeated rather than published.
template<typename T>
class Seqlock
{
  static_assert(std::is_trivially_copyable<T>::value,
    "Seqlock payloads are copied bytewise and must be trivially copyable");

public:
  Seqlock() = default;

  explicit Seqlock(const T & initial)
  : value_(initial) {}

  /// Publish a new value.  Must only ever be called from one thread.
  void store(const T & value) noexcept
  {
    const uint32_t seq = seq_.load(std::memory_order_relaxed);
    seq_.store(seq + 1, std::memory_order_relaxed);   // odd: write in flight
    std::atomic_thread_fence(std::memory_order_release);

    std::memcpy(&value_, &value, sizeof(T));

    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(seq + 2, std::memory_order_relaxed);   // even: readable again
  }

  /// Read the most recently published value.  Safe from any number of threads.
  T load() const noexcept
  {
    T out;
    for (;;) {
      const uint32_t before = seq_.load(std::memory_order_acquire);
      if (before & 1u) {         // a write is in flight, spin
        continue;
      }
      std::memcpy(&out, &value_, sizeof(T));
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == before) {
        return out;             // no write started or finished mid-copy
      }
    }
  }

private:
  std::atomic<uint32_t> seq_{0};
  T value_{};
};

}  // namespace mirena

#endif  // MIRENA_CONTROL__SEQLOCK_HPP_
