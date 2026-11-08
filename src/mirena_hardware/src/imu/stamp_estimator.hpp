#pragma once

#include <cstdint>
#include <cstdlib>

namespace mirena::IMU
{
    /// Maps the sensor's own clock onto ROS time.
    ///
    /// Why not just stamp with now() on arrival: measured on
    /// rosbag2_2026_08_05-20_26_23, the delay from sample to publish varies by
    /// 10.4 ms p1-p99 -- a whole 100 Hz period. Only 0.15 ms of that is DDS and
    /// the recorder (measured by pairing the two Imu topics, which leave the
    /// same loop iteration microseconds apart); the rest is the serial link and
    /// the SDK's reader thread, and arrival-time stamping folds all of it into
    /// the header. At 20 m/s that is +-10 cm of lidar deskew.
    ///
    /// Why not just publish the sensor's GPS time: it is sub-microsecond
    /// against UTC and in a different clock domain from the lidar, the camera,
    /// TF and rosbag2's arrival stamps -- 2.026 s away on that same bag, which
    /// is past every staleness gate in the stack.
    ///
    /// So: the sensor supplies the interval, the node clock supplies the epoch.
    ///
    /// `arrival - device` is the true offset plus that sample's latency, and
    /// latency is never negative -- so the smallest value seen is the best
    /// estimate, biased high only by the fastest achievable transport. Hence:
    /// take any lower reading immediately, and let the estimate creep back up
    /// slowly enough that only genuine oscillator drift can move it. The creep
    /// rate is chrony's default maximum slew: fast enough to follow a
    /// disciplined host clock correcting itself as hard as it ever will, and no
    /// faster, since everything above that is jitter let through into the
    /// stamps. Simulated against the latency distribution measured on
    /// rosbag2_2026_08_05-20_26_23, that lands the sample instant within
    /// 0.50 ms median / 1.04 ms worst case, against 5.3 ms / 46.7 ms for
    /// stamping on arrival.
    ///
    /// Why the minimum and not an average: dawonn/vectornav (ros2 branch,
    /// Vectornav::getTimeStamp) does the same timeStartup-plus-offset trick but
    /// takes an EWMA of the *mean* of arrival - device, which converges on the
    /// true offset plus the mean latency. Simulated against the measured
    /// distribution, that is a systematic +5.96 ms lag, versus +0.47 ms here,
    /// bought for 0.26 ms less jitter. Bias is the term that hurts: the lidar
    /// stamps its own packets nearly unbiased, so a 6 ms offset on the IMU is
    /// 6 ms of *relative* error in the deskew that consumes its angular rate.
    /// Their version also has no answer for a sensor power-cycle (timeStartup
    /// restarts, the average crawls to the new epoch over ~10 000 samples --
    /// simulated worst error 273 s) or for a host clock step (~10 s), which is
    /// what the two snap conditions below are for.
    ///
    /// This does not discipline anything. If the host clock is 2 s off UTC so
    /// are these stamps -- consistently with every other node, which is the
    /// point. That is the host's to fix (gpsd + chrony); see tools/retime_bag.py.
    class StampEstimator
    {
    public:
        /// Forget the estimate. The device clock's epoch is arbitrary, so an
        /// offset only means anything for the clock it was measured against.
        void reset() { have_offset_ = false; }

        /// @param device_ns  the sensor's own monotonic time *of the sample*
        /// @param arrival_ns ROS time, read as close to arrival as possible
        /// @return the sample instant, in ROS time
        int64_t stamp(int64_t device_ns, int64_t arrival_ns)
        {
            const int64_t raw_ns = arrival_ns - device_ns;

            if (!have_offset_ ||
                // The device clock restarted: the sensor was power-cycled.
                device_ns < last_device_ns_ ||
                // Too far out to be drift -- a host clock step, or a stall long
                // enough that following it beats waiting to creep there.
                std::llabs(raw_ns - offset_ns_) > kResyncNs)
            {
                offset_ns_ = raw_ns;
                have_offset_ = true;
            }
            else if (raw_ns < offset_ns_)
            {
                offset_ns_ = raw_ns; // a faster packet is strictly better evidence
            }
            else
            {
                const int64_t elapsed_ns = device_ns - last_device_ns_;
                offset_ns_ += elapsed_ns > 0 ? elapsed_ns * kMaxSlewPpm / 1'000'000 + 1 : 0;
            }

            last_device_ns_ = device_ns;
            return device_ns + offset_ns_;
        }

    private:
        static constexpr int64_t kMaxSlewPpm = 500;
        static constexpr int64_t kResyncNs = 500'000'000;

        bool have_offset_ = false;
        int64_t offset_ns_ = 0;
        int64_t last_device_ns_ = 0;
    };
} // namespace mirena::IMU
