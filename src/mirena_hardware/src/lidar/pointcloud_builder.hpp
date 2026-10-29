#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lidar_packet.hpp"
#include "rclcpp/time.hpp"
#include <cstddef>
#include <cstdint>
#include <string>

namespace mirena {

struct Timestamp {
    int64_t seconds = 0;      // 6 bytes (48 bits) on the wire, so it needs 64 here
    int32_t microseconds = 0; // 4 bytes fits a standard 32-bit int

    /// Signed difference, in seconds.
    ///
    /// This used to print "obsceno" and call exit(39) when the difference came
    /// out negative, which killed the driver -- no exception, no ROS log, no
    /// socket cleanup -- on any packet that arrived out of order. UDP does not
    /// promise ordering and the frame-boundary reset below moves the reference
    /// point, so one reordered packet was enough to take perception down mid
    /// lap. A time difference running backwards is information, not a reason to
    /// abort: it is returned, and the caller decides.
    double diff_seconds(const Timestamp& other) const {
        return static_cast<double>(seconds - other.seconds) +
               static_cast<double>(microseconds - other.microseconds) * 1e-6;
    }
};

struct StampedMSOPPacket {
    mirena::lidar::MSOPPacket packet;
    rclcpp::Time timestamp;
};

/**
 * Yaw angle traced by the car, as a function of time.
 *
 * De-skew needs the rotation between the instant a point was measured and the
 * instant the frame started. That is an integral of the yaw rate over the
 * sweep, so what this stores is not the rate but its running integral: each
 * sample carries the cumulative angle up to its own stamp, trapezoidally
 * accumulated as it arrives. Answering "how far had the car turned at time t"
 * is then one interpolation instead of a walk over the history.
 *
 * What it replaces was a single `double angular_vel_z` holding whatever the
 * last IMU callback had written, accumulated as `omega * dt` per block. That is
 * a real integral -- a rectangle rule -- but it samples the rate at the moment
 * the packet was *processed* rather than the moment its points were *measured*,
 * so the answer depended on thread scheduling and could not be reproduced.
 * Simulated over one sweep of a 13 rad/s^2 corner entry, the worst azimuth
 * error goes from 10.1 mm to 1.4 mm at 15 m: real, but not the reason to do
 * this.
 *
 * The reason is that "latest" was the only question the builder could answer,
 * which forced the packet queue -- packets had to be held until an IMU sample
 * newer than the packet had landed. Indexing by time removes the reason to
 * wait, and the queue, the timer and the unbounded growth go with it.
 */
class ImuHistory {
public:
    /// 2.5 s at 100 Hz. The query is always inside the current sweep (107 ms),
    /// so this is slack for a stalled reader, not working set.
    static constexpr size_t kCapacity = 256;

    /// How far past the newest sample a query may be answered by extrapolating
    /// at a constant rate. The IMU lands a few ms behind the lidar packet it
    /// belongs to, so a little of this is normal; at a yaw acceleration of
    /// 2 rad/s^2 a 10 ms extrapolation costs 1e-4 rad, which is 1.5 mm at 15 m.
    /// Past this the history is treated as absent rather than invented.
    static constexpr double kMaxExtrapolationS = 0.05;

    void clear() { size_ = 0; newest_ = 0; }

    bool empty() const { return size_ == 0; }

    double newest_stamp() const { return size_ ? ring_[newest_].stamp : 0.0; }

    /// Append a yaw-rate sample. Out-of-order or duplicate stamps are dropped:
    /// integrating them backwards would subtract angle the car did turn.
    void add(double stamp, double omega) {
        if (size_ == 0) {
            newest_ = 0;
            ring_[0] = {stamp, omega, 0.0};
            size_ = 1;
            return;
        }
        const Sample &last = ring_[newest_];
        const double dt = stamp - last.stamp;
        if (dt <= 0.0) return;
        // A gap this long means the stream broke; carrying the trapezoid across
        // it would invent a smooth turn through the hole.
        const double theta = dt > kMaxExtrapolationS
                                 ? last.theta
                                 : last.theta + 0.5 * (omega + last.omega) * dt;
        newest_ = (newest_ + 1) % kCapacity;
        ring_[newest_] = {stamp, omega, theta};
        if (size_ < kCapacity) size_++;
    }

    /// Cumulative yaw and yaw rate at `t`. The angle sits in an arbitrary datum
    /// shared by every query, so only differences between two calls mean
    /// anything. `ok` reports whether the history actually covered the instant.
    struct State { double theta; double omega; };

    State state_at(double t, bool *ok = nullptr) const {
        if (ok) *ok = false;
        if (size_ == 0) return {0.0, 0.0};

        const Sample &newest = ring_[newest_];
        if (t >= newest.stamp) {
            const double ahead = t - newest.stamp;
            if (ahead > kMaxExtrapolationS) return {newest.theta, 0.0};
            if (ok) *ok = true;
            return {newest.theta + newest.omega * ahead, newest.omega};
        }

        // Walk back from the newest sample. Queries land inside the current
        // sweep, so this stops within about a dozen entries whatever the
        // capacity is -- no search, no cursor to invalidate.
        size_t idx = newest_;
        for (size_t n = 1; n < size_; n++) {
            const size_t prev = (idx + kCapacity - 1) % kCapacity;
            if (ring_[prev].stamp <= t) {
                const Sample &a = ring_[prev];
                const Sample &b = ring_[idx];
                const double span = b.stamp - a.stamp;
                if (ok) *ok = true;
                if (span <= 0.0) return {a.theta, a.omega};
                const double f = (t - a.stamp) / span;
                return {a.theta + (b.theta - a.theta) * f,
                        a.omega + (b.omega - a.omega) * f};
            }
            idx = prev;
        }
        // Older than anything held: hold the oldest angle rather than
        // extrapolating backwards into a turn nobody recorded.
        return {ring_[idx].theta, ring_[idx].omega};
    }

    double angle_at(double t, bool *ok = nullptr) const { return state_at(t, ok).theta; }

private:
    struct Sample {
        double stamp;
        double omega;
        double theta; // cumulative, trapezoidal
    };
    Sample ring_[kCapacity];
    size_t newest_ = 0;
    size_t size_ = 0;
};

    class PointcloudBuilder {
    public:
        /**
         * @brief Constructor
         * @param frame_id_ The TF frame name for the pointcloud header
         * @param max_seq_num_ Estimated packets per frame (default 630 for RS-M1)
         */
        PointcloudBuilder(const char* frame_id_, uint16_t max_seq_num_ = 630)
            : frame_id(frame_id_), max_seq_num(max_seq_num_) {}

        /**
         * @brief Process an incoming MSOP packet and append to the current frame.
         * @return the completed PointCloud2 if this packet closed a frame, else nullptr.
         */
        std::unique_ptr<sensor_msgs::msg::PointCloud2> process_packet(const mirena::StampedMSOPPacket &stamped_packet);

        /// Feed a yaw-rate sample for de-skew. Stamp is ROS time.
        void add_imu(double stamp, double omega) { imu_.add(stamp, omega); }

        /// History keeps accumulating either way, so this can be toggled at
        /// runtime without the next frame starting from an empty buffer.
        void set_deskew_enabled(bool enabled) { deskew_enabled_ = enabled; }

        /// Whether the last completed frame was actually de-skewed, so the node
        /// can say so instead of silently publishing a smeared cloud.
        bool last_frame_compensated() const { return compensated_; }

        void flush(){
          last_sequence_number = 0;
          frame_start_time = Timestamp();
          frame_start_ros = 0;
          frame_start_theta = 0;
          product = nullptr;
          // imu_ is deliberately kept: it is indexed by time and says nothing
          // about the socket that just dropped.
        };
    private:

        // Configuration
        const std::string frame_id;
        const uint16_t max_seq_num;

        ImuHistory imu_;
        bool deskew_enabled_ = true;
        bool compensated_ = false;

        // State tracking
        uint16_t last_sequence_number = 0;
        /// Frame start on the *lidar's* clock, which is what the packet headers
        /// carry and what gives precise sub-sweep offsets.
        Timestamp frame_start_time;
        /// The same instant on the ROS clock, which is what the IMU history is
        /// indexed by. The lidar clock supplies the intervals, the host clock
        /// the epoch -- the same split the IMU driver's stamping uses.
        double frame_start_ros = 0;
        double frame_start_theta = 0;

        // The pointcloud message currently being "filled"
        std::unique_ptr<sensor_msgs::msg::PointCloud2> product;
        /**
         * @brief Creates and pre-allocates a new PointCloud2 message.
         */
        std::unique_ptr<sensor_msgs::msg::PointCloud2> make_new_pointcloud(const builtin_interfaces::msg::Time & stamp);

        void start_frame(const StampedMSOPPacket &stamped_packet, const Timestamp &packet_timestamp);
    };

} // namespace mirena
