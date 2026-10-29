#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mirena
{

/// Records raw CAN traffic to disk without putting any file I/O on the thread
/// that drains the socket.
///
/// A session is one file. start() opens one named for the moment it opened;
/// stop() flushes and closes it. Both are idempotent, so a caller can drive
/// them straight from a state signal that repeats on every broadcast without
/// having to track edges itself.
///
/// WHY THE THREAD. The socket is drained on a 5 ms timer shared with the paths
/// that publish /as_status and /sensors/wheel_speeds. Formatting a frame and
/// writing it there would put a disk stall -- an SD card garbage-collecting, a
/// USB stick flushing -- directly in front of the vehicle state the rest of the
/// stack is waiting on. So the rx thread only appends a fixed 24-byte record to
/// a preallocated buffer under a short mutex; a writer thread swaps that buffer
/// out and does the formatting and the write() calls.
///
/// The queue is capped. If the writer cannot keep up, frames are dropped and
/// counted rather than allowed to grow the buffer without bound: a log with a
/// gap in it is recoverable, a node that exhausts memory on the car is not.
///
/// Output is candump-compatible ASCII, one frame per line:
///
///     (1786097620.362741) can0 025#0300000000000000
///
/// which can-utils (canplayer, canlogserver), python-can and cantools all read
/// with no converter in between. Remote-transmission frames render as `025#R`.
class CanLogger
{
public:
    struct Stats
    {
        bool active;
        uint64_t frames_written;
        uint64_t frames_dropped;
        std::string file;      ///< current session's path, empty when inactive
    };

    /// @param directory  folder the session files are written into; created if
    ///                   it does not exist
    /// @param interface_name  name written into each line, e.g. "can0"
    CanLogger(std::string directory, std::string interface_name);
    ~CanLogger();

    CanLogger(const CanLogger &) = delete;
    CanLogger & operator=(const CanLogger &) = delete;

    /// Open a new session. Returns true if a session is open afterwards, so
    /// calling it while one already is succeeds without disturbing it. On
    /// failure the logger stays inactive and last_error() says why.
    bool start();

    /// Flush and close the current session, blocking until the file is on
    /// disk. No-op if none is open.
    void stop();

    bool active() const { return active_.load(std::memory_order_acquire); }

    /// Append one frame. Cheap and non-blocking; does nothing when inactive, so
    /// the caller need not test active() first. Called from the rx thread.
    ///
    /// @param can_id raw SocketCAN id, flag bits (EFF/RTR/ERR) still set
    void write(uint64_t stamp_ns, uint32_t can_id, uint8_t dlc, const uint8_t * data);

    Stats stats() const;
    std::string last_error() const;

private:
    /// One frame, fixed size so the producer never allocates or branches on
    /// length. 8 data bytes is the whole of classic CAN.
    struct Record
    {
        uint64_t stamp_ns;
        uint32_t can_id;
        uint8_t dlc;
        uint8_t data[8];
    };

    void writerLoop();
    void setError(std::string message);

    const std::string directory_;
    const std::string interface_;

    std::atomic<bool> active_{false};
    std::atomic<bool> stopping_{false};

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Record> queue_;   ///< producer appends here
    std::vector<Record> drain_;   ///< writer swaps it out to here

    std::thread writer_;
    /// Owned by start() until the writer thread takes it over and closes it.
    std::FILE * file_handle_{nullptr};

    std::atomic<uint64_t> written_{0};
    std::atomic<uint64_t> dropped_{0};

    /// Guards the two pieces of state a caller can read at any time while
    /// start()/stop() are rewriting them.
    mutable std::mutex meta_mutex_;
    std::string file_;
    std::string last_error_;
};

}  // namespace mirena
