#include "can_logger.hpp"

#include <linux/can.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

namespace mirena
{

namespace
{

/// Above this the writer is not keeping up and further frames are dropped.
/// 65536 records is ~1.5 MB, which is seconds of a saturated 1 Mbit/s bus --
/// long enough to ride out an SD-card stall, small enough to be a fixed cost.
constexpr size_t kMaxQueued = 65536;

/// One write() per megabyte rather than one per line.
constexpr size_t kFileBuffer = 1u << 20;

/// Formatted output is flushed to the FILE* once it passes this.
constexpr size_t kFlushThreshold = 1u << 16;

constexpr char kHex[] = "0123456789ABCDEF";

void appendByte(std::string & out, uint8_t v)
{
    out.push_back(kHex[v >> 4]);
    out.push_back(kHex[v & 0x0F]);
}

/// Ids exactly as can-utils' snprintf_canframe writes them: 3 hex digits for a
/// standard frame, 8 for an extended one, and 8 for an error frame with the
/// error flag left in place -- that flag is what tells a reader the line is a
/// bus event rather than a message with that id.
void appendId(std::string & out, uint32_t raw)
{
    const bool wide = (raw & (CAN_EFF_FLAG | CAN_ERR_FLAG)) != 0u;
    if (!wide) {
        const uint32_t id = raw & CAN_SFF_MASK;
        out.push_back(kHex[(id >> 8) & 0x0F]);
        out.push_back(kHex[(id >> 4) & 0x0F]);
        out.push_back(kHex[id & 0x0F]);
        return;
    }

    const uint32_t id = (raw & CAN_ERR_FLAG) != 0u ?
        (raw & (CAN_ERR_MASK | CAN_ERR_FLAG)) : (raw & CAN_EFF_MASK);
    for (int shift = 28; shift >= 0; shift -= 4) {
        out.push_back(kHex[(id >> shift) & 0x0F]);
    }
}

/// "(1786097620.362741) " -- seconds and microseconds, as candump writes it.
void appendStamp(std::string & out, uint64_t stamp_ns)
{
    const uint64_t sec = stamp_ns / 1000000000ull;
    const uint64_t usec = (stamp_ns % 1000000000ull) / 1000ull;

    out.push_back('(');
    char digits[24];
    int n = 0;
    uint64_t s = sec;
    do {
        digits[n++] = static_cast<char>('0' + (s % 10u));
        s /= 10u;
    } while (s != 0u);
    while (n > 0) {
        out.push_back(digits[--n]);
    }

    out.push_back('.');
    for (int shift = 100000; shift > 0; shift /= 10) {
        out.push_back(static_cast<char>('0' + ((usec / static_cast<uint64_t>(shift)) % 10u)));
    }
    out.push_back(')');
    out.push_back(' ');
}

/// ISO 8601 basic format with milliseconds, UTC: 20260807T081550.123Z
///
/// Basic rather than extended (2026-08-07T08:15:50Z) because the colons in the
/// extended form are a path separator on Windows and on the FAT/exFAT sticks
/// these logs get carried off the car on, and they need escaping in every shell
/// that later touches the file. Both are ISO 8601; only one survives the trip.
///
/// Milliseconds are included because the name has to be unique, and seconds are
/// not enough to guarantee that: a precharge that drops out and comes back
/// inside one second would otherwise resolve to a name that already exists.
std::string sessionStamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    std::tm utc{};
    gmtime_r(&seconds, &utc);

    char date[32];
    if (std::strftime(date, sizeof(date), "%Y%m%dT%H%M%S", &utc) == 0) {
        return "unknown-time";
    }
    char out[48];
    std::snprintf(out, sizeof(out), "%s.%03dZ", date, static_cast<int>(millis));
    return out;
}

}  // namespace

CanLogger::CanLogger(std::string directory, std::string interface_name)
: directory_(std::move(directory)), interface_(std::move(interface_name))
{
    queue_.reserve(kMaxQueued);
    drain_.reserve(kMaxQueued);
}

CanLogger::~CanLogger()
{
    stop();
}

void CanLogger::setError(std::string message)
{
    std::lock_guard<std::mutex> lock(meta_mutex_);
    last_error_ = std::move(message);
}

std::string CanLogger::last_error() const
{
    std::lock_guard<std::mutex> lock(meta_mutex_);
    return last_error_;
}

bool CanLogger::start()
{
    if (active_.load(std::memory_order_acquire)) {
        return true;   // idempotent: a repeated PRECHARGED must not cut the file
    }

    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec && !std::filesystem::is_directory(directory_)) {
        setError("could not create '" + directory_ + "': " + ec.message());
        return false;
    }

    // Opened here rather than on the writer thread so a bad path is a failure
    // start() can report, instead of a thread that quietly dies a moment later.
    // The handle is then owned by that thread, which closes it.
    //
    // "wx" is an exclusive create: it fails rather than truncating. A session
    // must never be able to destroy an earlier one, and a timestamp -- however
    // fine-grained -- is not on its own a guarantee of uniqueness, so a name
    // that is somehow already taken is answered by picking another rather than
    // by overwriting whatever is there.
    std::string path;
    std::FILE * file = nullptr;
    for (int attempt = 0; attempt < 100; ++attempt) {
        std::string name = sessionStamp();
        if (attempt > 0) {
            name += "-" + std::to_string(attempt);
        }
        path = (std::filesystem::path(directory_) / (name + ".log")).string();

        errno = 0;
        file = std::fopen(path.c_str(), "wx");
        if (file != nullptr) {
            break;
        }
        if (errno != EEXIST) {
            break;   // a real failure: no permission, no space, bad path
        }
    }
    if (file == nullptr) {
        setError("could not open '" + path + "' for writing: " + std::strerror(errno));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        drain_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(meta_mutex_);
        file_ = path;
    }
    written_.store(0, std::memory_order_relaxed);
    dropped_.store(0, std::memory_order_relaxed);
    stopping_.store(false, std::memory_order_relaxed);
    file_handle_ = file;

    active_.store(true, std::memory_order_release);
    writer_ = std::thread(&CanLogger::writerLoop, this);
    return true;
}

void CanLogger::stop()
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    // Cleared before the thread is woken: the loop drains whatever is queued
    // and only then sees that it should leave, so nothing already accepted is
    // lost on shutdown.
    active_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_.store(true, std::memory_order_relaxed);
    }
    cv_.notify_one();

    if (writer_.joinable()) {
        writer_.join();
    }
    file_handle_ = nullptr;   // the writer closed it
    {
        std::lock_guard<std::mutex> lock(meta_mutex_);
        file_.clear();
    }
}

void CanLogger::write(uint64_t stamp_ns, uint32_t can_id, uint8_t dlc, const uint8_t * data)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    Record record;
    record.stamp_ns = stamp_ns;
    record.can_id = can_id;
    record.dlc = dlc > 8u ? 8u : dlc;
    for (uint8_t i = 0; i < record.dlc; ++i) {
        record.data[i] = data[i];
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kMaxQueued) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        queue_.push_back(record);
    }
    cv_.notify_one();
}

void CanLogger::writerLoop()
{
    std::FILE * file = file_handle_;   // handed over by start(), closed below

    std::vector<char> io_buffer(kFileBuffer);
    std::setvbuf(file, io_buffer.data(), _IOFBF, io_buffer.size());

    std::string out;
    out.reserve(kFlushThreshold * 2);

    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] {
                return !queue_.empty() || stopping_.load(std::memory_order_relaxed);
            });

            const bool leaving = stopping_.load(std::memory_order_relaxed) && queue_.empty();
            // Swapped rather than copied, and the outgoing vector is cleared
            // first, so the producer gets this buffer's capacity back and never
            // allocates in steady state.
            drain_.clear();
            drain_.swap(queue_);
            if (leaving) {
                break;
            }
        }

        for (const Record & record : drain_) {
            appendStamp(out, record.stamp_ns);
            out.append(interface_);
            out.push_back(' ');
            appendId(out, record.can_id);
            out.push_back('#');
            if ((record.can_id & CAN_RTR_FLAG) != 0u) {
                // A remote frame carries no data, but its requested length is
                // meaningful, so can-utils writes it as "R<dlc>" and drops the
                // digit only when the length is zero.
                out.push_back('R');
                if (record.dlc > 0u && record.dlc <= 8u) {
                    out.push_back(kHex[record.dlc]);
                }
            } else {
                for (uint8_t i = 0; i < record.dlc; ++i) {
                    appendByte(out, record.data[i]);
                }
            }
            out.push_back('\n');
        }
        written_.fetch_add(drain_.size(), std::memory_order_relaxed);

        if (out.size() >= kFlushThreshold) {
            std::fwrite(out.data(), 1, out.size(), file);
            out.clear();
        }
    }

    if (!out.empty()) {
        std::fwrite(out.data(), 1, out.size(), file);
    }
    std::fflush(file);
    std::fclose(file);
}

CanLogger::Stats CanLogger::stats() const
{
    Stats s;
    s.active = active_.load(std::memory_order_acquire);
    s.frames_written = written_.load(std::memory_order_relaxed);
    s.frames_dropped = dropped_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(meta_mutex_);
        s.file = file_;
    }
    return s;
}

}  // namespace mirena
